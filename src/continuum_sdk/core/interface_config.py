from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Literal

import yaml


InitialPositionMode = Literal["capture_current", "configured_reference"]


def _five_ints(values: object, field_name: str) -> tuple[int, int, int, int, int]:
    if not isinstance(values, (list, tuple)) or len(values) != 5:
        raise ValueError(f"{field_name} must contain exactly five integers.")
    return tuple(int(value) for value in values)


def _axis_map(value: object, field_name: str) -> str:
    mapping = str(value).lower()
    if sorted(mapping) != ["x", "y", "z"]:
        raise ValueError(f"{field_name} must be a permutation of xyz.")
    return mapping


def _axis_signs(value: object, field_name: str) -> tuple[int, int, int]:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"{field_name} must contain exactly three signs.")
    signs = tuple(int(sign) for sign in value)
    if any(sign not in (-1, 1) for sign in signs):
        raise ValueError(f"{field_name} values must be +1 or -1.")
    return signs


@dataclass(frozen=True)
class InitialPositionConfig:
    mode: InitialPositionMode
    reference_pulses: tuple[int, int, int, int, int] | None
    reject_all_zero_feedback: bool
    require_near_reference: bool
    tolerance_pulses: tuple[int, int, int, int, int] | None

    def resolve_reference(self, current_pulses: list[int]) -> list[int]:
        current = list(_five_ints(current_pulses, "current_pulses"))
        if self.reject_all_zero_feedback and not any(current):
            raise RuntimeError("PMAC returned an invalid all-zero startup position.")

        if self.mode == "capture_current":
            return current

        if self.reference_pulses is None:
            raise ValueError("configured_reference mode requires reference_pulses.")

        reference = list(self.reference_pulses)
        if self.require_near_reference:
            if self.tolerance_pulses is None:
                raise ValueError("require_near_reference requires tolerance_pulses.")
            errors = [abs(actual - expected) for actual, expected in zip(current, reference)]
            if any(error > tolerance for error, tolerance in zip(errors, self.tolerance_pulses)):
                raise RuntimeError(
                    "Startup position is outside the configured reference tolerance: "
                    f"errors={errors}, tolerances={list(self.tolerance_pulses)}"
                )
        return reference


@dataclass(frozen=True)
class CartesianCommandConfig:
    max_delta_m: tuple[float, float, float]
    max_speed_m_s: tuple[float, float, float]
    orientation_enabled: bool
    max_rotation_delta_rad: tuple[float, float, float]
    max_angular_speed_rad_s: tuple[float, float, float]
    deadband_m: float
    smooth_alpha: float


@dataclass(frozen=True)
class CartesianFrameConfig:
    """Command-to-robot Cartesian axis permutations and sign corrections."""

    translation_map: str
    translation_signs: tuple[int, int, int]
    rotation_map: str
    rotation_signs: tuple[int, int, int]


@dataclass(frozen=True)
class RobotInterfaceConfig:
    control_hz: int
    omega_map: str
    frame: CartesianFrameConfig
    initial_position: InitialPositionConfig
    command: CartesianCommandConfig


def load_robot_interface_config(
    path: str | Path = "config/robot_interface.yaml",
) -> RobotInterfaceConfig:
    with open(path, "r", encoding="utf-8") as file:
        raw = yaml.safe_load(file)

    initial_raw = raw["initial_position"]
    reference_raw = initial_raw.get("reference_pulses")
    tolerance_raw = initial_raw.get("tolerance_pulses")
    initial = InitialPositionConfig(
        mode=initial_raw["mode"],
        reference_pulses=(
            None if reference_raw is None else _five_ints(reference_raw, "reference_pulses")
        ),
        reject_all_zero_feedback=bool(initial_raw.get("reject_all_zero_feedback", True)),
        require_near_reference=bool(initial_raw.get("require_near_reference", False)),
        tolerance_pulses=(
            None if tolerance_raw is None else _five_ints(tolerance_raw, "tolerance_pulses")
        ),
    )
    if initial.mode not in ("capture_current", "configured_reference"):
        raise ValueError(f"Unsupported initial position mode: {initial.mode}")
    if initial.mode == "configured_reference" and initial.reference_pulses is None:
        raise ValueError("configured_reference mode requires reference_pulses.")
    if initial.mode == "configured_reference" and not initial.require_near_reference:
        raise ValueError(
            "configured_reference mode requires require_near_reference=true for safe startup."
        )
    if initial.mode == "configured_reference" and initial.tolerance_pulses is None:
        raise ValueError("configured_reference mode requires tolerance_pulses.")

    command_raw = raw["command"]
    command = CartesianCommandConfig(
        max_delta_m=tuple(float(value) for value in command_raw["max_delta_m"]),
        max_speed_m_s=tuple(float(value) for value in command_raw["max_speed_m_s"]),
        orientation_enabled=bool(command_raw.get("orientation_enabled", False)),
        max_rotation_delta_rad=tuple(
            float(value) for value in command_raw.get("max_rotation_delta_rad", (0.0, 0.0, 0.0))
        ),
        max_angular_speed_rad_s=tuple(
            float(value) for value in command_raw.get("max_angular_speed_rad_s", (0.0, 0.0, 0.0))
        ),
        deadband_m=float(command_raw["deadband_m"]),
        smooth_alpha=float(command_raw["smooth_alpha"]),
    )
    if any(
        len(values) != 3
        for values in (
            command.max_delta_m,
            command.max_speed_m_s,
            command.max_rotation_delta_rad,
            command.max_angular_speed_rad_s,
        )
    ):
        raise ValueError("Cartesian translation and rotation limits must contain three values.")
    if not 0.0 <= command.smooth_alpha <= 1.0:
        raise ValueError("smooth_alpha must be within [0, 1].")
    if command.deadband_m < 0.0:
        raise ValueError("deadband_m must be non-negative.")
    if any(value < 0.0 for value in command.max_delta_m):
        raise ValueError("max_delta_m values must be non-negative.")
    if any(value < 0.0 for value in command.max_speed_m_s):
        raise ValueError("max_speed_m_s values must be non-negative.")
    if any(value < 0.0 for value in command.max_rotation_delta_rad):
        raise ValueError("max_rotation_delta_rad values must be non-negative.")
    if any(value < 0.0 for value in command.max_angular_speed_rad_s):
        raise ValueError("max_angular_speed_rad_s values must be non-negative.")
    if command.orientation_enabled and (
        not any(command.max_rotation_delta_rad)
        or not any(command.max_angular_speed_rad_s)
    ):
        raise ValueError(
            "Enabled orientation control requires non-zero rotation and angular speed limits."
        )

    # Kept only as a backwards-compatible metadata field.  Robot-frame
    # conversion is controlled by frame.*; Omega source mapping belongs to
    # the Omega adapter and is intentionally independent.
    omega_map = _axis_map(raw.get("omega_map", "xyz"), "omega_map")
    frame_raw = raw.get("frame", {})
    frame = CartesianFrameConfig(
        translation_map=_axis_map(frame_raw.get("translation_map", "yxz"), "frame.translation_map"),
        translation_signs=_axis_signs(
            frame_raw.get("translation_signs", (-1, -1, 1)),
            "frame.translation_signs",
        ),
        rotation_map=_axis_map(frame_raw.get("rotation_map", "yxz"), "frame.rotation_map"),
        rotation_signs=_axis_signs(
            frame_raw.get("rotation_signs", (1, -1, 1)),
            "frame.rotation_signs",
        ),
    )

    return RobotInterfaceConfig(
        control_hz=int(raw["control_hz"]),
        omega_map=omega_map,
        frame=frame,
        initial_position=initial,
        command=command,
    )
