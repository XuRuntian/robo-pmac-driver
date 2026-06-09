from __future__ import annotations

import math
from typing import Mapping, TypedDict


class TipPoseCommand(TypedDict):
    """Tip pose offset from the configured neutral pose."""

    tip_delta_x: float
    tip_delta_y: float
    tip_delta_z: float
    tip_delta_rx: float
    tip_delta_ry: float
    tip_delta_rz: float


class ActuatorState(TypedDict):
    """Five actuator positions expressed in their physical SI units."""

    axis_1_pos: float
    axis_2_pos: float
    axis_3_pos: float
    axis_4_pos: float
    axis_5_pos: float


COMMAND_FIELDS = {
    "tip_delta_x": {"dtype": "float32", "shape": (1,), "unit": "m"},
    "tip_delta_y": {"dtype": "float32", "shape": (1,), "unit": "m"},
    "tip_delta_z": {"dtype": "float32", "shape": (1,), "unit": "m"},
    "tip_delta_rx": {"dtype": "float32", "shape": (1,), "unit": "rad"},
    "tip_delta_ry": {"dtype": "float32", "shape": (1,), "unit": "rad"},
    "tip_delta_rz": {"dtype": "float32", "shape": (1,), "unit": "rad"},
}

STATE_FIELDS = {
    "axis_1_pos": {"dtype": "float32", "shape": (1,), "unit": "rad"},
    "axis_2_pos": {"dtype": "float32", "shape": (1,), "unit": "rad"},
    "axis_3_pos": {"dtype": "float32", "shape": (1,), "unit": "rad"},
    "axis_4_pos": {"dtype": "float32", "shape": (1,), "unit": "rad"},
    "axis_5_pos": {"dtype": "float32", "shape": (1,), "unit": "m"},
}


def normalize_tip_pose_command(command: Mapping[str, float]) -> TipPoseCommand:
    required = tuple(COMMAND_FIELDS)
    missing = [key for key in required if key not in command]
    if missing:
        raise KeyError(f"Missing command fields: {missing}")

    normalized = {
        key: float(command[key])
        for key in required
    }
    invalid = [key for key, value in normalized.items() if not math.isfinite(value)]
    if invalid:
        raise ValueError(f"Command fields must be finite: {invalid}")
    return normalized  # type: ignore[return-value]
