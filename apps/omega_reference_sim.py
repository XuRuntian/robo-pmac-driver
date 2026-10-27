"""Independent reference IK for MuJoCo, observing raw LeRobot Omega samples.

No Omega SDK, PMAC controller, Cartesian robot commands or PVT conversion.
"""
import argparse
from contextlib import ExitStack
from pathlib import Path
import sys
import time

import numpy as np
import zmq

from continuum_sdk.control.reference_teleop import (
    load_reference,
    real_branch_translation_map,
    ReferenceTeleop,
)
from continuum_sdk.core.interface_config import load_robot_interface_config
from continuum_sdk.core.omega_config import load_omega_teleop_config
from continuum_sdk.control.axis_mapper import ContinuumAxisMapper
from pmac_sdk.core.config_model import PMACConfig
from pmac_sdk.core.waiting import WindowsHighResolutionWait


CONFIG_DIR = Path(__file__).resolve().parents[1]/'config'


def initial_simulation_joint(reference, interface, axis5_pulses):
    """Apply the driver's insertion reference to a supplied startup encoder count."""
    cfg = PMACConfig()
    mapper = ContinuumAxisMapper(cfg.pulses_per_rad, cfg.pulses_per_meter,
                                cfg.axis_order, cfg.axis_signs)
    index = mapper.axis_order[4]
    current = [0] * 5
    current[index] = axis5_pulses
    base = current.copy()
    zero = interface.initial_position.insertion_zero_pulses
    if zero is not None:
        base[index] = zero
    joint = np.zeros(5)
    joint[0] = mapper.pulses_to_logical(base, current)[4]
    low, high = reference.kinematics.ContinuumKinematics(
        reference.config.KinematicsConfig()).insertion_bounds
    if not low <= joint[0] <= high:
        raise ValueError(f'Simulation startup insertion {joint[0]*1000:.3f} mm is outside '
                         f'[{low*1000:.3f}, {high*1000:.3f}] mm; '
                         'check insertion_zero_pulses and --initial-axis5-pulses.')
    return joint


def run(endpoint, visual_endpoint, reference_root, duration=0, timeout=.2, rotation_map='swap-xz',
        translation_source='robot-branch', omega_config=None, interface_config=None,
        initial_axis5_pulses=0):
    reference = load_reference(reference_root)
    interface = load_robot_interface_config(
        interface_config or CONFIG_DIR/'robot_interface_stop_priority_trial.yaml')
    initial_joint = initial_simulation_joint(reference, interface, initial_axis5_pulses)
    translation = None
    if translation_source == 'robot-branch':
        translation = real_branch_translation_map(
            load_omega_teleop_config(omega_config or CONFIG_DIR/'omega_teleop.yaml'),
            interface.frame)
    shared = ReferenceTeleop(reference, initial_joint, swap_rotation_xz=rotation_map == 'swap-xz',
                             translation_map=translation)
    interval = 1.0 / reference.config.SimulationConfig().teleop_hz
    with ExitStack() as cleanup, zmq.Context() as context:
        source = context.socket(zmq.SUB)
        cleanup.callback(source.close)
        source.setsockopt(zmq.LINGER, 0)
        source.setsockopt(zmq.SUBSCRIBE, b'')
        source.setsockopt(zmq.CONFLATE, 1)
        source.connect(endpoint)
        visual = context.socket(zmq.PUB)
        cleanup.callback(visual.close)
        visual.setsockopt(zmq.LINGER, 0)
        visual.setsockopt(zmq.SNDHWM, 1)
        visual.bind(visual_endpoint)
        sleep = time.sleep
        if sys.platform == 'win32':
            timer = WindowsHighResolutionWait()
            cleanup.callback(timer.close)
            sleep = timer.sleep
        print(f'Independent simulation IK | raw={endpoint} | visual={visual_endpoint}', flush=True)
        print(f'Simulation startup insertion d={initial_joint[0]*1000:.3f} mm | '
              f'axis-5 zero={interface.initial_position.insertion_zero_pulses} pulses | '
              f'supplied encoder={initial_axis5_pulses} pulses (not read from hardware)', flush=True)
        print(f'Simulation Omega rotation mapping: {rotation_map} (before reference mapping)', flush=True)
        print(f'Translation mapping: {translation_source}'
              + (' (matches the robot branch direction and gain)' if translation is not None
                 else ' (reference project default; differs from the robot branch)'), flush=True)
        print('Rotation still differs by construction: the robot branch uses Omega wrist '
              'joint deltas, this branch uses the reported rotation matrix.', flush=True)
        start = time.monotonic()
        last_received = None
        sequence = 0
        while not duration or time.monotonic() - start < duration:
            now = time.monotonic()
            try:
                message = source.recv_json(flags=zmq.NOBLOCK)
                shared.receive(message)
                last_received = now
            except zmq.Again:
                pass
            except (KeyError, TypeError, ValueError) as exc:
                print(f'Rejected raw sample: {exc}', flush=True)
            if last_received is None or now-last_received > timeout or not shared.connected:
                # Stop publishing so the viewer also reports STALE, then reanchor
                # on reconnect without accumulating unobserved master displacement.
                shared.pause()
            else:
                joint = shared.step()
                visual.send_json(dict(kind='continuum_visual', version=1,
                    sequence=sequence, source_sequence=shared.sequence,
                    source_session=shared.session, ideal_joint=joint.tolist(),
                    ideal_source='independent_reference_ik', feedback_age_s=0,
                    source='lerobot_raw_pose', clutch=shared.clutch), flags=zmq.NOBLOCK)
                sequence += 1
            remaining = interval - (time.monotonic()-now)
            if remaining > 0:
                sleep(remaining)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--endpoint', default='tcp://127.0.0.1:5558')
    parser.add_argument('--visual-endpoint', default='tcp://127.0.0.1:5557')
    parser.add_argument('--reference-root', type=Path, default=Path(__file__).resolve().parents[2]/
                        'surgical_continuum_robot/remote_control_ex')
    parser.add_argument('--duration', type=float, default=0, help='0 runs until Ctrl+C.')
    parser.add_argument('--timeout', type=float, default=.2)
    parser.add_argument('--initial-axis5-pulses', type=int, default=0,
                        help='Actual axis-5 encoder count at simulation startup; default assumes 0. '
                             'Combined with insertion_zero_pulses in --interface-config.')
    parser.add_argument('--rotation-map', choices=['swap-xz', 'original'], default='swap-xz',
                        help='Calibrated simulation local increments: [Rx,Ry,Rz] -> [Rz,Ry,Rx]. '
                             'original bypasses this rotation adjustment.')
    parser.add_argument('--translation-map', choices=['robot-branch', 'reference'],
                        default='robot-branch', dest='translation_source',
                        help='robot-branch matches the real branch direction and gain; '
                             'reference restores the reference project default.')
    # File-relative like --reference-root, so the entry point runs from any cwd.
    parser.add_argument('--omega-config', type=Path, default=CONFIG_DIR/'omega_teleop.yaml',
                        help='Read only for the robot-branch translation mapping.')
    parser.add_argument('--interface-config', type=Path,
                        default=CONFIG_DIR/'robot_interface_stop_priority_trial.yaml',
                        help='Insertion zero reference and robot-branch translation frame.')
    args = parser.parse_args()
    if not np.isfinite(args.duration) or args.duration < 0:
        parser.error('duration must be finite and nonnegative')
    if not np.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('timeout must be finite and positive')
    run(args.endpoint, args.visual_endpoint, args.reference_root, args.duration, args.timeout,
        args.rotation_map, args.translation_source, args.omega_config, args.interface_config,
        args.initial_axis5_pulses)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
