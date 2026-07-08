from __future__ import annotations

import argparse
import csv
import time
from pathlib import Path

import numpy as np

from lerobot_teleoperator_omega_continuum import OmegaContinuum, OmegaContinuumConfig
from lerobot_teleoperator_omega_continuum.mapping import _matrix_to_rotvec


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Print raw Omega motion and mapped continuum action.")
    parser.add_argument("--omega-map", default="zxy")
    parser.add_argument("--scale-x", type=float, default=0.5)
    parser.add_argument("--scale-y", type=float, default=0.08)
    parser.add_argument("--scale-z", type=float, default=0.25)
    parser.add_argument("--rotation-map", default="zxy")
    parser.add_argument("--rotation-scale-x", type=float, default=-0.3)
    parser.add_argument("--rotation-scale-y", type=float, default=0.3)
    parser.add_argument("--rotation-scale-z", type=float, default=0.0)
    parser.add_argument("--position-offset-x", type=float, default=0.0)
    parser.add_argument("--position-offset-y", type=float, default=0.0)
    parser.add_argument("--position-offset-z", type=float, default=0.0)
    parser.add_argument("--max-delta-x", type=float, default=0.03)
    parser.add_argument("--max-delta-y", type=float, default=0.01)
    parser.add_argument("--max-delta-z", type=float, default=0.03)
    parser.add_argument("--max-rotation-x", type=float, default=0.45)
    parser.add_argument("--max-rotation-y", type=float, default=0.45)
    parser.add_argument("--max-rotation-z", type=float, default=0.0)
    parser.add_argument("--rotation-deadband-rad", type=float, default=0.01)
    parser.add_argument("--interval", type=float, default=0.1)
    parser.add_argument("--duration", type=float, default=0.0, help="0 means run until Ctrl+C.")
    parser.add_argument("--csv", default="", help="Optional CSV path for raw Omega pose samples.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    omega = OmegaContinuum(
        OmegaContinuumConfig(
            id="omega_continuum",
            omega_map=args.omega_map,
            rotation_map=args.rotation_map,
            scale_x=args.scale_x,
            scale_y=args.scale_y,
            scale_z=args.scale_z,
            rotation_scale_x=args.rotation_scale_x,
            rotation_scale_y=args.rotation_scale_y,
            rotation_scale_z=args.rotation_scale_z,
            position_offset_x=args.position_offset_x,
            position_offset_y=args.position_offset_y,
            position_offset_z=args.position_offset_z,
            max_delta_x=args.max_delta_x,
            max_delta_y=args.max_delta_y,
            max_delta_z=args.max_delta_z,
            max_rotation_x=args.max_rotation_x,
            max_rotation_y=args.max_rotation_y,
            max_rotation_z=args.max_rotation_z,
            rotation_deadband_rad=args.rotation_deadband_rad,
            clutch_enabled=False,
        )
    )
    omega.connect()
    csv_file = None
    writer = None
    start = time.perf_counter()
    try:
        if args.csv:
            csv_path = Path(args.csv)
            csv_path.parent.mkdir(parents=True, exist_ok=True)
            csv_file = csv_path.open("w", newline="", encoding="utf-8")
            fieldnames = [
                "t_s",
                "raw_x_m",
                "raw_y_m",
                "raw_z_m",
                "ctrl_x_m",
                "ctrl_y_m",
                "ctrl_z_m",
                "ctrl_dx_m",
                "ctrl_dy_m",
                "ctrl_dz_m",
                "raw_rx_rad",
                "raw_ry_rad",
                "raw_rz_rad",
                "tip_delta_x",
                "tip_delta_y",
                "tip_delta_z",
                "tip_delta_rx",
                "tip_delta_ry",
                "tip_delta_rz",
            ] + [f"r{row}{col}" for row in range(3) for col in range(3)]
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            writer.writeheader()

        while True:
            t_s = time.perf_counter() - start
            if args.duration > 0.0 and t_s >= args.duration:
                break
            position, orientation = omega._read_pose()
            control_position = omega._mapper.control_position(position, orientation)
            action = omega._mapper.map_pose(position, orientation)
            zero_position = omega._mapper._zero
            zero_orientation = omega._mapper._zero_orientation
            if zero_orientation is None:
                raw_rotvec = np.zeros(3, dtype=float)
            else:
                raw_rotvec = _matrix_to_rotvec(zero_orientation.T @ orientation)
            if zero_position is None:
                control_delta = np.zeros(3, dtype=float)
            else:
                control_delta = control_position - zero_position
            print(
                "raw_mm="
                f"[{position[0] * 1000:+7.2f}, {position[1] * 1000:+7.2f}, {position[2] * 1000:+7.2f}] | "
                "ctrl_dmm="
                f"[{control_delta[0] * 1000:+7.2f}, {control_delta[1] * 1000:+7.2f}, "
                f"{control_delta[2] * 1000:+7.2f}] | "
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
            if writer is not None:
                row = {
                    "t_s": t_s,
                    "raw_x_m": float(position[0]),
                    "raw_y_m": float(position[1]),
                    "raw_z_m": float(position[2]),
                    "ctrl_x_m": float(control_position[0]),
                    "ctrl_y_m": float(control_position[1]),
                    "ctrl_z_m": float(control_position[2]),
                    "ctrl_dx_m": float(control_delta[0]),
                    "ctrl_dy_m": float(control_delta[1]),
                    "ctrl_dz_m": float(control_delta[2]),
                    "raw_rx_rad": float(raw_rotvec[0]),
                    "raw_ry_rad": float(raw_rotvec[1]),
                    "raw_rz_rad": float(raw_rotvec[2]),
                    "tip_delta_x": float(action["tip_delta_x"]),
                    "tip_delta_y": float(action["tip_delta_y"]),
                    "tip_delta_z": float(action["tip_delta_z"]),
                    "tip_delta_rx": float(action["tip_delta_rx"]),
                    "tip_delta_ry": float(action["tip_delta_ry"]),
                    "tip_delta_rz": float(action["tip_delta_rz"]),
                }
                for matrix_row in range(3):
                    for matrix_col in range(3):
                        row[f"r{matrix_row}{matrix_col}"] = float(
                            orientation[matrix_row, matrix_col]
                        )
                writer.writerow(row)
            time.sleep(args.interval)
    except KeyboardInterrupt:
        pass
    finally:
        if csv_file is not None:
            csv_file.close()
        omega.disconnect()


if __name__ == "__main__":
    main()
