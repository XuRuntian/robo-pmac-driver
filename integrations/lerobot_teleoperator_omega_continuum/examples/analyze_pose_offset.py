from __future__ import annotations

import argparse
import csv
from pathlib import Path

import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Estimate an Omega local-frame position_offset from a CSV recorded while "
            "rotating the master around the desired control point."
        )
    )
    parser.add_argument("csv_path", type=Path)
    parser.add_argument(
        "--min-rotation-rad",
        type=float,
        default=0.02,
        help="Ignore samples with less relative rotation than this.",
    )
    return parser.parse_args()


def _float(row: dict[str, str], key: str) -> float:
    return float(row[key])


def _position(row: dict[str, str]) -> np.ndarray:
    return np.array([_float(row, "raw_x_m"), _float(row, "raw_y_m"), _float(row, "raw_z_m")])


def _rotation(row: dict[str, str]) -> np.ndarray:
    return np.array(
        [
            [_float(row, "r00"), _float(row, "r01"), _float(row, "r02")],
            [_float(row, "r10"), _float(row, "r11"), _float(row, "r12")],
            [_float(row, "r20"), _float(row, "r21"), _float(row, "r22")],
        ],
        dtype=float,
    )


def _rotvec_norm(row: dict[str, str]) -> float:
    return float(
        np.linalg.norm(
            [
                _float(row, "raw_rx_rad"),
                _float(row, "raw_ry_rad"),
                _float(row, "raw_rz_rad"),
            ]
        )
    )


def _span_mm(values: np.ndarray) -> list[float]:
    span = (np.max(values, axis=0) - np.min(values, axis=0)) * 1000.0
    return [float(value) for value in span]


def main() -> None:
    args = parse_args()
    with args.csv_path.open("r", newline="", encoding="utf-8") as file:
        rows = list(csv.DictReader(file))
    if len(rows) < 2:
        raise SystemExit("Need at least two samples.")

    ref = rows[0]
    p0 = _position(ref)
    r0 = _rotation(ref)

    a_blocks = []
    b_blocks = []
    used_rows = []
    for row in rows[1:]:
        if _rotvec_norm(row) < args.min_rotation_rad:
            continue
        p = _position(row)
        r = _rotation(row)
        a_blocks.append(r - r0)
        b_blocks.append(-(p - p0))
        used_rows.append(row)

    if not a_blocks:
        raise SystemExit("No samples passed --min-rotation-rad.")

    a = np.vstack(a_blocks)
    b = np.concatenate(b_blocks)
    offset, *_ = np.linalg.lstsq(a, b, rcond=None)

    all_positions = np.asarray([_position(row) for row in rows])
    all_rotations = np.asarray([_rotation(row) for row in rows])
    raw_delta = all_positions - all_positions[0]
    compensated = np.asarray(
        [position + rotation @ offset for position, rotation in zip(all_positions, all_rotations)]
    )
    compensated_delta = compensated - compensated[0]

    raw_rms = float(np.sqrt(np.mean(np.sum(raw_delta**2, axis=1))) * 1000.0)
    compensated_rms = float(np.sqrt(np.mean(np.sum(compensated_delta**2, axis=1))) * 1000.0)
    print(f"samples: {len(rows)} | used for fit: {len(used_rows)}")
    print(
        "estimated position_offset_m: "
        f"x={offset[0]:+.6f}, y={offset[1]:+.6f}, z={offset[2]:+.6f}"
    )
    print(
        "estimated position_offset_mm: "
        f"x={offset[0] * 1000:+.2f}, y={offset[1] * 1000:+.2f}, z={offset[2] * 1000:+.2f}"
    )
    print(f"raw translation RMS during recording: {raw_rms:.2f} mm")
    print(f"compensated translation RMS: {compensated_rms:.2f} mm")
    print(
        "raw span mm xyz: "
        + ", ".join(f"{value:.2f}" for value in _span_mm(all_positions))
    )
    print(
        "compensated span mm xyz: "
        + ", ".join(f"{value:.2f}" for value in _span_mm(compensated))
    )
    print("\nLeRobot override:")
    print(
        "  --teleop.position_offset_x="
        f"{offset[0]:.6f} --teleop.position_offset_y={offset[1]:.6f} "
        f"--teleop.position_offset_z={offset[2]:.6f}"
    )
    print("\nprint_actions override:")
    print(
        "  --position-offset-x "
        f"{offset[0]:.6f} --position-offset-y {offset[1]:.6f} "
        f"--position-offset-z {offset[2]:.6f}"
    )


if __name__ == "__main__":
    main()
