from __future__ import annotations

import argparse
import csv
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from lerobot_teleoperator_omega_continuum import OmegaContinuum, OmegaContinuumConfig
from lerobot_teleoperator_omega_continuum.mapping import _matrix_to_rotvec


AXES = ("x", "y", "z")


@dataclass(frozen=True)
class Phase:
    name: str
    kind: str
    desired_axis: str
    prompt: str


DESIRED_ROBOT_PHASES = (
    Phase(
        "robot_plus_x",
        "translation",
        "x",
        "Move the Omega in the direction that should command ROBOT +X (right), then hold.",
    ),
    Phase(
        "robot_plus_y",
        "translation",
        "y",
        "Move the Omega in the direction that should command ROBOT +Y (inward/insertion), then hold.",
    ),
    Phase(
        "robot_plus_z",
        "translation",
        "z",
        "Move the Omega in the direction that should command ROBOT +Z (up), then hold.",
    ),
    Phase(
        "robot_plus_rx",
        "rotation",
        "x",
        "Rotate the Omega in the direction that should command ROBOT +RX tip tilt, then hold.",
    ),
    Phase(
        "robot_plus_ry",
        "rotation",
        "y",
        "Rotate the Omega in the direction that should command ROBOT +RY tip tilt, then hold.",
    ),
    Phase(
        "robot_plus_rz",
        "rotation",
        "z",
        "Rotate the Omega in the direction that would command ROBOT +RZ roll, then hold.",
    ),
)


OMEGA_PHYSICAL_PHASES = (
    Phase(
        "omega_plus_x",
        "translation",
        "x",
        "Move the Omega along physical +X (outward), then hold.",
    ),
    Phase(
        "omega_plus_y",
        "translation",
        "y",
        "Move the Omega along physical +Y (right), then hold.",
    ),
    Phase(
        "omega_plus_z",
        "translation",
        "z",
        "Move the Omega along physical +Z (up), then hold.",
    ),
    Phase(
        "omega_plus_rx",
        "rotation",
        "x",
        "Rotate the Omega around physical +X, then hold.",
    ),
    Phase(
        "omega_plus_ry",
        "rotation",
        "y",
        "Rotate the Omega around physical +Y, then hold.",
    ),
    Phase(
        "omega_plus_rz",
        "rotation",
        "z",
        "Rotate the Omega around physical +Z, then hold.",
    ),
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Interactively identify Omega raw translation/rotation axes."
    )
    parser.add_argument(
        "--phase-set",
        choices=("desired-robot", "omega-physical"),
        default="desired-robot",
        help="desired-robot directly suggests teleop maps; omega-physical verifies SDK axes.",
    )
    parser.add_argument(
        "--skip-rotations",
        action="store_true",
        help="Only run translation phases.",
    )
    parser.add_argument(
        "--skip-translations",
        action="store_true",
        help="Only run rotation phases.",
    )
    parser.add_argument("--baseline-s", type=float, default=0.5)
    parser.add_argument("--sample-s", type=float, default=1.0)
    parser.add_argument("--interval", type=float, default=0.01)
    parser.add_argument("--position-threshold-mm", type=float, default=1.0)
    parser.add_argument("--rotation-threshold-rad", type=float, default=0.03)
    parser.add_argument("--csv", default="", help="Optional CSV path for all samples.")
    parser.add_argument("--simulate", action="store_true")
    return parser.parse_args()


def average_rotation(rotations: list[np.ndarray]) -> np.ndarray:
    mean_rotation = np.mean(rotations, axis=0)
    u, _, vt = np.linalg.svd(mean_rotation)
    rotation = u @ vt
    if np.linalg.det(rotation) < 0.0:
        u[:, -1] *= -1.0
        rotation = u @ vt
    return rotation


def collect_samples(
    omega: OmegaContinuum,
    duration_s: float,
    interval_s: float,
    *,
    phase_name: str,
    stage: str,
    writer: csv.DictWriter[str] | None,
    start_time: float,
) -> tuple[np.ndarray, np.ndarray, list[tuple[np.ndarray, np.ndarray]]]:
    samples: list[tuple[np.ndarray, np.ndarray]] = []
    deadline = time.perf_counter() + max(duration_s, interval_s)
    index = 0
    while time.perf_counter() < deadline:
        position, orientation = omega._read_pose()
        samples.append((position, orientation))
        if writer is not None:
            row = {
                "t_s": time.perf_counter() - start_time,
                "phase": phase_name,
                "stage": stage,
                "sample": index,
                "raw_x_m": float(position[0]),
                "raw_y_m": float(position[1]),
                "raw_z_m": float(position[2]),
            }
            for matrix_row in range(3):
                for matrix_col in range(3):
                    row[f"r{matrix_row}{matrix_col}"] = float(
                        orientation[matrix_row, matrix_col]
                    )
            writer.writerow(row)
        index += 1
        time.sleep(interval_s)

    positions = np.asarray([sample[0] for sample in samples], dtype=float)
    rotations = [sample[1] for sample in samples]
    return np.mean(positions, axis=0), average_rotation(rotations), samples


def dominant_axis(values: np.ndarray, threshold: float) -> tuple[str | None, float, float]:
    abs_values = np.abs(values)
    max_index = int(np.argmax(abs_values))
    max_value = float(values[max_index])
    if abs(max_value) < threshold:
        return None, max_value, 0.0

    sorted_abs = np.sort(abs_values)
    second = float(sorted_abs[-2]) if len(sorted_abs) >= 2 else 0.0
    confidence = abs(max_value) / max(second, 1e-12)
    return AXES[max_index], max_value, confidence


def format_vec(values: np.ndarray, scale: float, unit: str) -> str:
    return (
        f"[{values[0] * scale:+8.3f}, "
        f"{values[1] * scale:+8.3f}, "
        f"{values[2] * scale:+8.3f}] {unit}"
    )


def phase_list(args: argparse.Namespace) -> list[Phase]:
    phases = list(DESIRED_ROBOT_PHASES if args.phase_set == "desired-robot" else OMEGA_PHYSICAL_PHASES)
    if args.skip_rotations:
        phases = [phase for phase in phases if phase.kind != "rotation"]
    if args.skip_translations:
        phases = [phase for phase in phases if phase.kind != "translation"]
    return phases


def summarize_suggestion(
    args: argparse.Namespace,
    results: list[dict[str, object]],
) -> None:
    if args.phase_set != "desired-robot":
        return

    translation = [result for result in results if result["kind"] == "translation"]
    rotation = [result for result in results if result["kind"] == "rotation"]

    if len(translation) == 3 and all(result["dominant_position_axis"] for result in translation):
        omega_map = "".join(str(result["dominant_position_axis"]) for result in translation)
        scale_signs = [
            "+" if float(result["dominant_position_value"]) > 0.0 else "-"
            for result in translation
        ]
        print("\nSuggested translation config, if each phase was a desired positive robot axis:")
        print(f"  omega_map: {omega_map}")
        print(
            "  scale signs: "
            f"scale_x={scale_signs[0]}, scale_y={scale_signs[1]}, scale_z={scale_signs[2]}"
        )
        if len(set(omega_map)) != 3:
            print("  WARNING: duplicate raw axes found; repeat the translation phases.")

    if len(rotation) == 3 and all(result["dominant_rotation_axis"] for result in rotation):
        rotation_map = "".join(str(result["dominant_rotation_axis"]) for result in rotation)
        rotation_signs = [
            "+" if float(result["dominant_rotation_value"]) > 0.0 else "-"
            for result in rotation
        ]
        print("\nSuggested rotation config, if each phase was a desired positive robot rotation:")
        print(f"  rotation_map: {rotation_map}")
        print(
            "  rotation scale signs: "
            f"rotation_scale_x={rotation_signs[0]}, "
            f"rotation_scale_y={rotation_signs[1]}, "
            f"rotation_scale_z={rotation_signs[2]} "
            "(keep rotation_scale_z=0 for the current 5-DOF robot)"
        )
        if len(set(rotation_map)) != 3:
            print("  WARNING: duplicate raw rotation axes found; repeat the rotation phases.")


def main() -> None:
    args = parse_args()
    if args.skip_rotations and args.skip_translations:
        raise ValueError("Cannot skip both translations and rotations.")

    omega = OmegaContinuum(
        OmegaContinuumConfig(
            id="omega_axis_identifier",
            simulate=args.simulate,
            clutch_enabled=False,
        )
    )

    csv_file = None
    writer = None
    fieldnames = [
        "t_s",
        "phase",
        "stage",
        "sample",
        "raw_x_m",
        "raw_y_m",
        "raw_z_m",
    ] + [f"r{row}{col}" for row in range(3) for col in range(3)]

    if args.csv:
        csv_path = Path(args.csv)
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        csv_file = csv_path.open("w", newline="", encoding="utf-8")
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()

    results: list[dict[str, object]] = []
    start_time = time.perf_counter()

    try:
        omega.connect()
        print("Omega axis identification connected. This does not command the robot.")
        print("For each phase: first hold the start pose, then perform exactly one motion and hold.\n")

        for index, phase in enumerate(phase_list(args), start=1):
            print(f"Phase {index}: {phase.name}")
            print(f"  {phase.prompt}")
            input("  Hold the start pose and press Enter to zero this phase...")
            base_position, base_rotation, _ = collect_samples(
                omega,
                args.baseline_s,
                args.interval,
                phase_name=phase.name,
                stage="baseline",
                writer=writer,
                start_time=start_time,
            )

            input("  Perform the motion, hold it, then press Enter to sample...")
            target_position, target_rotation, target_samples = collect_samples(
                omega,
                args.sample_s,
                args.interval,
                phase_name=phase.name,
                stage="target",
                writer=writer,
                start_time=start_time,
            )

            position_delta = target_position - base_position
            rotvecs = np.asarray(
                [_matrix_to_rotvec(base_rotation.T @ sample[1]) for sample in target_samples],
                dtype=float,
            )
            rotation_delta = np.mean(rotvecs, axis=0)

            position_axis, position_value, position_confidence = dominant_axis(
                position_delta,
                args.position_threshold_mm / 1000.0,
            )
            rotation_axis, rotation_value, rotation_confidence = dominant_axis(
                rotation_delta,
                args.rotation_threshold_rad,
            )

            result = {
                "name": phase.name,
                "kind": phase.kind,
                "desired_axis": phase.desired_axis,
                "dominant_position_axis": position_axis,
                "dominant_position_value": position_value,
                "dominant_rotation_axis": rotation_axis,
                "dominant_rotation_value": rotation_value,
            }
            results.append(result)

            print("  raw position delta: " + format_vec(position_delta, 1000.0, "mm"))
            if position_axis is None:
                print("  dominant position: none above threshold")
            else:
                sign = "+" if position_value > 0.0 else "-"
                print(
                    f"  dominant position: raw {sign}{position_axis}, "
                    f"value={position_value * 1000.0:+.3f} mm, "
                    f"confidence={position_confidence:.2f}x"
                )

            print("  raw rotation delta: " + format_vec(rotation_delta, 1.0, "rad"))
            if rotation_axis is None:
                print("  dominant rotation: none above threshold\n")
            else:
                sign = "+" if rotation_value > 0.0 else "-"
                print(
                    f"  dominant rotation: raw {sign}r{rotation_axis}, "
                    f"value={rotation_value:+.4f} rad, "
                    f"confidence={rotation_confidence:.2f}x\n"
                )

        summarize_suggestion(args, results)
        print("\nDone.")
    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        if omega.is_connected:
            omega.disconnect()
        if csv_file is not None:
            csv_file.close()


if __name__ == "__main__":
    main()
