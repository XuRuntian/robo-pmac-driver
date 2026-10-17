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

from continuum_sdk.control.reference_teleop import load_reference, ReferenceTeleop
from pmac_sdk.core.waiting import WindowsHighResolutionWait


def run(endpoint, visual_endpoint, reference_root, duration=0, timeout=.2, rotation_map='swap-xz'):
    reference = load_reference(reference_root)
    shared = ReferenceTeleop(reference, np.zeros(5), swap_rotation_xz=rotation_map == 'swap-xz')
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
        print('Model zero initial pose | no hardware connection | LeRobot robot branch unchanged', flush=True)
        print(f'Simulation Omega rotation mapping: {rotation_map} (before reference mapping)', flush=True)
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
    parser.add_argument('--rotation-map', choices=['swap-xz', 'original'], default='swap-xz',
                        help='Simulation only: [Rx,Ry,Rz] -> [Rz,Ry,-Rx]; reverse Z after exchange.')
    args = parser.parse_args()
    if not np.isfinite(args.duration) or args.duration < 0:
        parser.error('duration must be finite and nonnegative')
    if not np.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('timeout must be finite and positive')
    run(args.endpoint, args.visual_endpoint, args.reference_root, args.duration, args.timeout,
        args.rotation_map)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
