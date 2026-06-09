from __future__ import annotations

import numpy as np


ACTION_FIELDS = (
    "tip_delta_x",
    "tip_delta_y",
    "tip_delta_z",
    "tip_delta_rx",
    "tip_delta_ry",
    "tip_delta_rz",
)


class OmegaContinuumMapper:
    """Map Omega translation from a sampled zero pose into robot XYZ offsets."""

    def __init__(
        self,
        *,
        scale_xyz: tuple[float, float, float],
        max_delta_xyz: tuple[float, float, float],
        deadband_m: float,
        omega_map: str,
    ) -> None:
        axis_index = {"x": 0, "y": 1, "z": 2}
        omega_map = omega_map.lower()
        if sorted(omega_map) != ["x", "y", "z"]:
            raise ValueError("omega_map must be a permutation of xyz.")

        self.scale = np.asarray(scale_xyz, dtype=float)
        self.max_delta = np.asarray(max_delta_xyz, dtype=float)
        self.deadband_m = float(deadband_m)
        self._omega_to_robot = np.asarray([axis_index[axis] for axis in omega_map], dtype=int)
        self._zero: np.ndarray | None = None

    def set_zero(self, position: np.ndarray) -> None:
        position = np.asarray(position, dtype=float)
        if position.shape != (3,) or not np.all(np.isfinite(position)):
            raise ValueError("Omega zero position must contain three finite values.")
        self._zero = position.copy()

    def map_position(self, position: np.ndarray) -> dict[str, float]:
        if self._zero is None:
            raise RuntimeError("Omega zero position has not been sampled.")

        position = np.asarray(position, dtype=float)
        if position.shape != (3,) or not np.all(np.isfinite(position)):
            raise ValueError("Omega position must contain three finite values.")

        delta = (position - self._zero)[self._omega_to_robot] * self.scale
        if self.deadband_m > 0.0:
            delta[np.abs(delta) < self.deadband_m] = 0.0
        delta = np.clip(delta, -self.max_delta, self.max_delta)
        return {
            "tip_delta_x": float(delta[0]),
            "tip_delta_y": float(delta[1]),
            "tip_delta_z": float(delta[2]),
            "tip_delta_rx": 0.0,
            "tip_delta_ry": 0.0,
            "tip_delta_rz": 0.0,
        }
