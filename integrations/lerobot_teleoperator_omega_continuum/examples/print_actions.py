from __future__ import annotations

import argparse
import time

from lerobot_teleoperator_omega_continuum import OmegaContinuum, OmegaContinuumConfig


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Print raw Omega motion and mapped continuum action.")
    parser.add_argument("--omega-map", default="zxy")
    parser.add_argument("--scale-x", type=float, default=0.25)
    parser.add_argument("--scale-y", type=float, default=0.08)
    parser.add_argument("--scale-z", type=float, default=-0.25)
    parser.add_argument("--max-delta-x", type=float, default=0.03)
    parser.add_argument("--max-delta-y", type=float, default=0.01)
    parser.add_argument("--max-delta-z", type=float, default=0.03)
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
            max_rotation_x=0.0,
            max_rotation_y=0.0,
            max_rotation_z=0.0,
        )
    )
    omega.connect()
    try:
        while True:
            position, orientation = omega._read_pose()
            action = omega._mapper.map_pose(position, orientation)
            print(
                "omega_mm="
                f"[{position[0] * 1000:+7.2f}, {position[1] * 1000:+7.2f}, {position[2] * 1000:+7.2f}] | "
                "tip_mm="
                f"[{action['tip_delta_x'] * 1000:+7.2f}, "
                f"{action['tip_delta_y'] * 1000:+7.2f}, "
                f"{action['tip_delta_z'] * 1000:+7.2f}]"
            )
            time.sleep(args.interval)
    except KeyboardInterrupt:
        pass
    finally:
        omega.disconnect()


if __name__ == "__main__":
    main()
