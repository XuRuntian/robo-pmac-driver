from __future__ import annotations

import argparse
import csv
import shutil
import time
from pathlib import Path

import numpy as np

from lerobot_teleoperator_omega_continuum import OmegaContinuum, OmegaContinuumConfig


def print_rotation_status(joints, joint_delta, action) -> None:
    status = (
        f"q=[{joints[0]:+.3f},{joints[1]:+.3f},{joints[2]:+.3f}] "
        f"dq=[{joint_delta[0]:+.3f},{joint_delta[1]:+.3f},{joint_delta[2]:+.3f}] "
        f"cmd=[{action['tip_delta_rx']:+.3f},"
        f"{action['tip_delta_ry']:+.3f},{action['tip_delta_rz']:+.3f}]"
    )
    # Leave the final column unused to prevent terminal auto-wrap and scrolling.
    width = max(0, shutil.get_terminal_size(fallback=(80, 24)).columns - 1)
    print("\r\033[2K" + status[:width], end="", flush=True)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Print Omega wrist q/dq and mapped rotation (rad); no robot connection. Omega initialization may autoInit/enableForce.")
    parser.add_argument("--omega-config", default="config/omega_teleop.yaml")
    parser.add_argument("--omega-map", default=None, help="Deprecated explicit override")
    parser.add_argument("--scale-x", type=float, default=None, help="Deprecated explicit override")
    parser.add_argument("--scale-y", type=float, default=None, help="Deprecated explicit override")
    parser.add_argument("--scale-z", type=float, default=None, help="Deprecated explicit override")
    parser.add_argument("--rotation-map", default=None, help="Deprecated explicit override")
    parser.add_argument("--rotation-scale-x", type=float, default=None, help="Deprecated explicit override")
    parser.add_argument("--rotation-scale-y", type=float, default=None, help="Deprecated explicit override")
    parser.add_argument("--rotation-scale-z", type=float, default=None, help="Deprecated explicit override")
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
            omega_config=args.omega_config,
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
    print("Omega only: no robot/PMAC commands. Initialization may autoInit/enableForce. q/dq/cmd units: rad.")
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
                "q1_rad", "q2_rad", "q3_rad",
                "dq1_rad", "dq2_rad", "dq3_rad",
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
            joints = omega._read_joint_angles()
            control_position = omega._mapper.control_position(position, orientation)
            action = omega._mapper.map_pose(position, orientation, joints)
            zero_position = omega._mapper._zero
            joint_delta = omega._mapper.omega_joint_delta(joints)
            if zero_position is None:
                control_delta = np.zeros(3, dtype=float)
            else:
                control_delta = control_position - zero_position
            print_rotation_status(joints, joint_delta, action)
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
                    "q1_rad": float(joints[0]),
                    "q2_rad": float(joints[1]),
                    "q3_rad": float(joints[2]),
                    "dq1_rad": float(joint_delta[0]),
                    "dq2_rad": float(joint_delta[1]),
                    "dq3_rad": float(joint_delta[2]),
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
        print()  # Leave the shell prompt on a new line after the live status.
        if csv_file is not None:
            csv_file.close()
        omega.disconnect()


if __name__ == "__main__":
    main()
