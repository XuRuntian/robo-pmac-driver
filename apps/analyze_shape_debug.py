from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from statistics import mean


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Summarize continuum shape debug CSV logs.")
    parser.add_argument("csv_path", type=Path)
    parser.add_argument(
        "--feedback-delay",
        type=float,
        default=0.22,
        help="Compare feedback with the target this many seconds earlier.",
    )
    parser.add_argument(
        "--direction-threshold",
        type=float,
        default=0.010,
        help="Command threshold in meters for directional summaries.",
    )
    return parser.parse_args()


def _float(row: dict[str, str], key: str) -> float:
    value = row.get(key, "")
    return float(value) if value else float("nan")


def _mean_abs(rows: list[dict[str, str]], key: str) -> float:
    values = [_float(row, key) for row in rows]
    values = [abs(value) for value in values if math.isfinite(value)]
    return mean(values) if values else float("nan")


def _max_abs(rows: list[dict[str, str]], key: str) -> float:
    values = [_float(row, key) for row in rows]
    values = [abs(value) for value in values if math.isfinite(value)]
    return max(values) if values else float("nan")


def _deg(value_rad: float) -> float:
    return value_rad * 180.0 / math.pi


def _delayed_pairs(
    rows: list[dict[str, str]],
    delay_s: float,
) -> list[tuple[dict[str, str], dict[str, str]]]:
    pairs = []
    target_index = 0
    times = [_float(row, "t_s") for row in rows]
    for feedback_row in rows:
        target_time = _float(feedback_row, "t_s") - delay_s
        if target_time < times[0]:
            continue
        while target_index + 1 < len(rows) and times[target_index + 1] <= target_time:
            target_index += 1
        pairs.append((rows[target_index], feedback_row))
    return pairs


def _mean_abs_delayed(
    pairs: list[tuple[dict[str, str], dict[str, str]]],
    target_key: str,
    feedback_key: str,
) -> float:
    values = [
        abs(_float(feedback_row, feedback_key) - _float(target_row, target_key))
        for target_row, feedback_row in pairs
    ]
    values = [value for value in values if math.isfinite(value)]
    return mean(values) if values else float("nan")


def _summarize_pairs(
    label: str,
    pairs: list[tuple[dict[str, str], dict[str, str]]],
) -> None:
    if not pairs:
        print(f"{label}: no samples")
        return

    theta_a_err = _deg(_mean_abs_delayed(pairs, "target_theta_a_rad", "feedback_theta_a_rad"))
    theta_c_err = _deg(_mean_abs_delayed(pairs, "target_theta_c_rad", "feedback_theta_c_rad"))
    pulse_errs = [
        _mean_abs_delayed(pairs, f"target_p{index}", f"feedback_p{index}")
        for index in range(1, 6)
    ]
    ratios = [
        abs(_float(target_row, "target_theta_c_over_a"))
        for target_row, _ in pairs
        if math.isfinite(_float(target_row, "target_theta_c_over_a"))
    ]
    ratio_mean = mean(ratios) if ratios else float("nan")
    print(
        f"{label:10s} delayed err deg a/c={theta_a_err:.2f}/{theta_c_err:.2f} | "
        f"target |c/a|={ratio_mean:.2f} | "
        + "pulse err="
        + ",".join(f"p{index}:{value:.0f}" for index, value in enumerate(pulse_errs, start=1))
    )


def main() -> None:
    args = parse_args()
    with args.csv_path.open("r", newline="", encoding="utf-8") as file:
        rows = list(csv.DictReader(file))

    if not rows:
        raise SystemExit("No rows found in shape debug CSV.")

    duration = _float(rows[-1], "t_s") - _float(rows[0], "t_s")
    print(f"samples: {len(rows)} | duration: {duration:.2f}s")
    print(
        "command range mm: "
        f"x={_max_abs(rows, 'applied_x_m') * 1000:.2f}, "
        f"y={_max_abs(rows, 'applied_y_m') * 1000:.2f}, "
        f"z={_max_abs(rows, 'applied_z_m') * 1000:.2f}"
    )

    for label in ("target", "feedback"):
        theta_a_mean = _deg(_mean_abs(rows, f"{label}_theta_a_rad"))
        theta_c_mean = _deg(_mean_abs(rows, f"{label}_theta_c_rad"))
        theta_a_max = _deg(_max_abs(rows, f"{label}_theta_a_rad"))
        theta_c_max = _deg(_max_abs(rows, f"{label}_theta_c_rad"))
        ratio = _mean_abs(rows, f"{label}_theta_c_over_a")
        print(
            f"{label:8s} theta mean/max deg: "
            f"a={theta_a_mean:.2f}/{theta_a_max:.2f}, "
            f"c={theta_c_mean:.2f}/{theta_c_max:.2f}, "
            f"|c/a| mean={ratio:.2f}"
        )

    theta_a_err = _deg(
        mean(
            abs(_float(row, "feedback_theta_a_rad") - _float(row, "target_theta_a_rad"))
            for row in rows
        )
    )
    theta_c_err = _deg(
        mean(
            abs(_float(row, "feedback_theta_c_rad") - _float(row, "target_theta_c_rad"))
            for row in rows
        )
    )
    print(f"shape tracking mean abs err deg: theta_a={theta_a_err:.2f}, theta_c={theta_c_err:.2f}")

    alpha_errs = [
        _mean_abs(rows, f"alpha{index}_err_rad")
        for index in range(1, 5)
    ]
    pulse_errs = [
        _mean_abs(rows, f"pulse_err{index}")
        for index in range(1, 6)
    ]
    print(
        "motor alpha mean abs err rad: "
        + ", ".join(f"a{index}={value:.4f}" for index, value in enumerate(alpha_errs, start=1))
    )
    print(
        "pulse mean abs err: "
        + ", ".join(f"p{index}={value:.0f}" for index, value in enumerate(pulse_errs, start=1))
    )

    pairs = _delayed_pairs(rows, args.feedback_delay)
    print(f"\ndelayed comparison: {args.feedback_delay:.3f}s")
    _summarize_pairs("all", pairs)

    threshold = args.direction_threshold
    eps = 1e-9
    directions = (
        ("x positive", lambda row: _float(row, "applied_x_m") >= threshold - eps),
        ("x negative", lambda row: _float(row, "applied_x_m") <= -threshold + eps),
        ("z positive", lambda row: _float(row, "applied_z_m") >= threshold - eps),
        ("z negative", lambda row: _float(row, "applied_z_m") <= -threshold + eps),
        ("y positive", lambda row: _float(row, "applied_y_m") >= threshold * 0.3 - eps),
        ("y negative", lambda row: _float(row, "applied_y_m") <= -threshold * 0.3 + eps),
    )
    for label, predicate in directions:
        _summarize_pairs(
            label,
            [(target_row, feedback_row) for target_row, feedback_row in pairs if predicate(target_row)],
        )


if __name__ == "__main__":
    main()
