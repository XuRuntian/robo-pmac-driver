from __future__ import annotations

import argparse
import time

import numpy as np

from lerobot_teleoperator_omega_continuum import OmegaContinuum, OmegaContinuumConfig
from lerobot_teleoperator_omega_continuum.mapping import _matrix_to_rotvec


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Print raw Omega motion and mapped continuum action.")
    parser.add_argument("--omega-map", default="zxy")
    parser.add_argument("--scale-x", type=float, default=0.25)
    parser.add_argument("--scale-y", type=float, default=0.08)
    parser.add_argument("--scale-z", type=float, default=-0.25)
    parser.add_argument("--max-delta-x", type=float, default=0.03)
    parser.add_argument("--max-delta-y", type=float, default=0.01)
    parser.add_argument("--max-delta-z", type=float, default=0.03)
    parser.add_argument("--max-rotation-x", type=float, default=0.45)
    parser.add_argument("--max-rotation-y", type=float, default=0.45)
    parser.add_argument("--max-rotation-z", type=float, default=0.0)
    parser.add_argument("--rotation-deadband-rad", type=float, default=0.01)
    parser.add_argument("--interval", type=float, default=0.1)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    omega = OmegaContinuum(
        OmegaContinuumConfig(
            id="omega_continuum",
            omega_map=args.omega_map,
            scale_x=args.scale_x,
            scale_y=args.scale_y,
            scale_z=args.scale_z,
            max_delta_x=args.max_delta_x,
            max_delta_y=args.max_delta_y,
            max_delta_z=args.max_delta_z,
            max_rotation_x=args.max_rotation_x,
            max_rotation_y=args.max_rotation_y,
            max_rotation_z=args.max_rotation_z,
            rotation_deadband_rad=args.rotation_deadband_rad,
        )
    )
    omega.connect()
    try:
        while True:
            position, orientation = omega._read_pose()
            action = omega._mapper.map_pose(position, orientation)
            zero_orientation = omega._mapper._zero_orientation
            if zero_orientation is None:
                raw_rotvec = np.zeros(3, dtype=float)
            else:
                raw_rotvec = _matrix_to_rotvec(zero_orientation.T @ orientation)
            print(
                "omega_mm="
                f"[{position[0] * 1000:+7.2f}, {position[1] * 1000:+7.2f}, {position[2] * 1000:+7.2f}] | "
                "tip_mm="
                f"[{action['tip_delta_x'] * 1000:+7.2f}, "
                f"{action['tip_delta_y'] * 1000:+7.2f}, "
                f"{action['tip_delta_z'] * 1000:+7.2f}] | "
                "omega_rot_rad="
                f"[{raw_rotvec[0]:+6.3f}, {raw_rotvec[1]:+6.3f}, {raw_rotvec[2]:+6.3f}] | "
                "tip_rot_rad="
                f"[{action['tip_delta_rx']:+6.3f}, "
                f"{action['tip_delta_ry']:+6.3f}, "
                f"{action['tip_delta_rz']:+6.3f}]"
            )
            time.sleep(args.interval)
    except KeyboardInterrupt:
        pass
    finally:
        omega.disconnect()


if __name__ == "__main__":
    main()
