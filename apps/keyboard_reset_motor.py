"""Hold-to-move motor jog, for recovering a tip that did not start straight.

Why motor level and not joint level: safe_boot_and_home() records the CURRENT
encoder reading as the reference, so when the robot powers up already bent the
model reports theta_a = theta_c = 0 and believes it is straight. Joint-space
and Cartesian commands are therefore meaningless until the real shape is back
to straight, and a joint-space straighten ramp would correctly conclude there
is nothing to do. The only trustworthy quantity in that state is motor pulses,
so recovery has to happen here.

Each tendon motor couples into both segments, so the display shows the shape
the model would infer from the pulses moved so far. Treat it as a relative
indication of which way the shape is going, not as absolute truth: the startup
reference itself is unknown.

This drives real hardware with --execute. Without it the script runs dry and
only prints the pulses it would submit.

Do not run this while continuum_driver_server.py is live: safe_boot_and_home()
triggers a PLC reset and would corrupt the running PVT session.
"""
from __future__ import annotations

import argparse
from contextlib import ExitStack
import math
import time

import numpy as np
from pynput import keyboard

from continuum_sdk.control.axis_mapper import ContinuumAxisMapper
from continuum_sdk.core.config_loader import load_continuum_config
from continuum_sdk.core.factory import build_tendon_mapper
from continuum_sdk.kinematics.joint_motor_model import MotorAngles
from pmac_sdk.controller.robot_api import PMACRobotController
from pmac_sdk.core.config_model import PMACConfig

# Keys are grouped by logical motor so the layout matches the printed readout.
# Logical index 0..3 are alpha1..alpha4; index 4 is insertion d.
MOTOR_KEYS = {
    'q': (0, -1.), 'w': (0, +1.),
    'a': (1, -1.), 's': (1, +1.),
    'z': (2, -1.), 'x': (2, +1.),
    'e': (3, -1.), 'r': (3, +1.),
    'c': (4, -1.), 'v': (4, +1.),
}
HELP = """
Hold a key to move, release to stop. Logical motors, not physical axis numbers.
  alpha1  q / w       alpha2  a / s
  alpha3  z / x       alpha4  e / r
  insertion d  c / v
  ctrl  fast      shift  slow      space  hold still
  p  print now    esc  quit (stops motion)
"""


class KeyboardDevice:
    """Same press/release tracking as the other keyboard entry points."""

    def __init__(self):
        self.pressed = set()
        self.stop_requested = False

    @staticmethod
    def key_name(key):
        if key == keyboard.Key.esc:
            return 'esc'
        if key == keyboard.Key.space:
            return 'space'
        if key in (keyboard.Key.shift, keyboard.Key.shift_r):
            return 'shift'
        if key in (keyboard.Key.ctrl, keyboard.Key.ctrl_l, keyboard.Key.ctrl_r):
            return 'ctrl'
        try:
            return key.char.lower() if key.char else None
        except AttributeError:
            return None

    def _on_press(self, key):
        name = self.key_name(key)
        if name == 'esc':
            self.stop_requested = True
        elif name is not None:
            self.pressed.add(name)

    def _on_release(self, key):
        name = self.key_name(key)
        if name is not None:
            self.pressed.discard(name)

    def __enter__(self):
        self.listener = keyboard.Listener(on_press=self._on_press,
                                          on_release=self._on_release)
        self.listener.start()
        return self

    def __exit__(self, *exc):
        self.listener.stop()


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='Hold-to-move motor jog for recovering a non-straight startup pose.')
    parser.add_argument('--execute', action='store_true',
                        help='Connect to PMAC and move the real robot.')
    parser.add_argument('--pmac-ip', default='192.168.0.200')
    parser.add_argument('--config', default='config/continuum.yaml')
    # 20 deg/s on the motor is 14% of the 2.5 rad/s (143 deg/s) rotary stop
    # threshold used by the driver, leaving plenty of margin for manual work.
    parser.add_argument('--deg-per-s', type=float, default=20.0,
                        help='Motor jog rate for alpha1..alpha4, deg/s.')
    parser.add_argument('--mm-per-s', type=float, default=10.0,
                        help='Insertion jog rate, mm/s.')
    parser.add_argument('--fine-scale', type=float, default=.2,
                        help='Rate multiplier while shift is held.')
    parser.add_argument('--coarse-scale', type=float, default=2.0,
                        help='Rate multiplier while ctrl is held.')
    parser.add_argument('--rate-hz', type=float, default=50.0)
    parser.add_argument('--max-deg', type=float, default=90.0,
                        help='Refuse to travel further than this from the start, per motor.')
    parser.add_argument('--max-mm', type=float, default=30.0,
                        help='Refuse to travel further than this from the start, on insertion.')
    args = parser.parse_args(argv)
    if args.rate_hz <= 0. or not 5. < 1000./args.rate_hz <= 500.:
        parser.error('--rate-hz must give a PVT segment in (5, 500] ms.')
    for name in ('deg_per_s', 'mm_per_s', 'max_deg', 'max_mm', 'coarse_scale'):
        if getattr(args, name) <= 0.:
            parser.error(f'--{name.replace("_", "-")} must be positive.')
    if not 0. < args.fine_scale <= 1.:
        parser.error('--fine-scale must be in (0, 1].')
    if args.coarse_scale < 1.:
        parser.error('--coarse-scale must be at least 1.')
    return args


def requested_step(pressed, args, interval):
    """Logical motor delta for this period: four angles in rad, insertion in m."""
    # Shift wins over ctrl so slowing down is never overridden by accident.
    scale = (args.fine_scale if 'shift' in pressed
             else args.coarse_scale if 'ctrl' in pressed else 1.)
    step = np.zeros(5)
    for key in pressed:
        entry = MOTOR_KEYS.get(key)
        if entry is None:
            continue
        index, sign = entry
        rate = (args.mm_per_s/1000. if index == 4 else math.radians(args.deg_per_s))
        step[index] += sign*rate*scale*interval
    return step


def clamp_travel(logical, args):
    """Hold each motor inside its allowed excursion from the startup pose."""
    limit = np.array([math.radians(args.max_deg)]*4 + [args.max_mm/1000.])
    return np.clip(np.asarray(logical, dtype=float), -limit, limit)


def inferred_shape(tendon_mapper, logical):
    """Shape the model would infer from the motor deltas moved so far.

    Relative to the startup pose, which is itself unknown when the robot did
    not start straight. Useful for direction, not as an absolute measurement.
    """
    recovered = tendon_mapper.model.motor_angles_to_joint(
        MotorAngles(alpha1=logical[0], alpha2=logical[1],
                    alpha3=logical[2], alpha4=logical[3]))
    return recovered.theta_a, recovered.theta_c


def describe(logical, tendon_mapper):
    theta_a, theta_c = inferred_shape(tendon_mapper, logical)
    motors = ' '.join(f'a{i+1}{math.degrees(logical[i]):+7.2f}' for i in range(4))
    return (f'{motors} d{logical[4]*1000:+6.2f}mm | '
            f'inferred bend A{math.degrees(theta_a):+6.2f} C{math.degrees(theta_c):+6.2f}')


def main(argv=None):
    args = parse_args(argv)
    tendon_mapper = build_tendon_mapper(load_continuum_config(args.config))
    pmac = PMACConfig(ip=args.pmac_ip)
    axis = ContinuumAxisMapper(pulses_per_rad=pmac.pulses_per_rad,
                               pulses_per_meter=pmac.pulses_per_meter,
                               axis_order=pmac.axis_order, axis_signs=pmac.axis_signs)
    interval = 1./args.rate_hz
    move_time_ms = interval*1000.

    with ExitStack() as cleanup:
        robot = None
        if args.execute:
            robot = PMACRobotController(pmac)
            cleanup.callback(robot.close)
            robot.safe_boot_and_home()
            base_pulses = list(robot.base_positions)
        else:
            base_pulses = [0]*5

        logical = np.zeros(5)
        previous_pulses = np.asarray(
            axis.logical_to_pulses(base_pulses, logical.tolist()), dtype=float)

        print(f'Motor jog [{"EXECUTE" if args.execute else "DRY-RUN"}]')
        print(f'base_pulses={base_pulses}')
        print('Startup captured the current encoder reading as the reference, so the '
              'model reports zero bend even if the tip is not straight. Move motors '
              'by eye; the inferred bend below is relative to this startup pose.')
        print(f'rates: motors {args.deg_per_s:g} deg/s '
              f'(ctrl {args.deg_per_s*args.coarse_scale:g}, '
              f'shift {args.deg_per_s*args.fine_scale:g}) | '
              f'insertion {args.mm_per_s:g} mm/s | '
              f'travel limit {args.max_deg:g} deg / {args.max_mm:g} mm')
        print(HELP)

        kbd = cleanup.enter_context(KeyboardDevice())
        next_call = time.perf_counter()
        printed = 0.
        frozen = False

        while not kbd.stop_requested:
            pressed = set(kbd.pressed)
            # Space freezes the target rather than aborting: stop_motion ends the
            # PVT session and resuming would need a full restart.
            if 'space' in pressed:
                step = np.zeros(5)
                if not frozen:
                    print('hold: target frozen, release space to resume jogging')
                frozen = True
            else:
                step = requested_step(pressed, args, interval)
                frozen = False

            logical = clamp_travel(logical + step, args)
            target = np.asarray(axis.logical_to_pulses(base_pulses, logical.tolist()),
                                dtype=float)
            velocities = ((target - previous_pulses)/move_time_ms).tolist()
            if robot is not None:
                robot.move_pvt_stream(target_pulses=[int(v) for v in target],
                                      velocities=velocities, move_time=move_time_ms)
            previous_pulses = target

            now = time.perf_counter()
            if 'p' in pressed or now - printed > 1.:
                print(f'{describe(logical, tendon_mapper)}  '
                      f'keys={"+".join(sorted(pressed)) or "-"}')
                printed = now
                kbd.pressed.discard('p')

            next_call += interval
            remaining = next_call - time.perf_counter()
            if remaining > 0.:
                time.sleep(remaining)
            else:
                next_call = time.perf_counter()

        if robot is not None:
            robot.stop_motion()
        print(f'final {describe(logical, tendon_mapper)}')
        print('When the tip looks straight, restart the driver so it captures this '
              'pose as the reference, then teleoperate.')
        return 0


if __name__ == '__main__':
    raise SystemExit(main())
