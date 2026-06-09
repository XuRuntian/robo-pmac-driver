from __future__ import annotations

import numpy as np

from continuum_sdk.control.interface_contract import TipPoseCommand, normalize_tip_pose_command
from continuum_sdk.core.interface_config import CartesianCommandConfig


class TipCommandFilter:
    """Apply the driver-side Cartesian safety limits at a fixed control rate."""

    def __init__(self, config: CartesianCommandConfig, update_interval_s: float) -> None:
        if update_interval_s <= 0.0:
            raise ValueError("update_interval_s must be positive.")

        self.config = config
        self.update_interval_s = float(update_interval_s)
        self._target_delta = np.zeros(3, dtype=float)
        self._applied_delta = np.zeros(3, dtype=float)

    @property
    def applied_delta(self) -> np.ndarray:
        return self._applied_delta.copy()

    def set_command(self, command: dict[str, float]) -> TipPoseCommand:
        normalized = normalize_tip_pose_command(command)
        rotation = np.asarray(
            [
                normalized["tip_delta_rx"],
                normalized["tip_delta_ry"],
                normalized["tip_delta_rz"],
            ],
            dtype=float,
        )
        if not self.config.orientation_enabled and np.any(np.abs(rotation) > 1e-12):
            raise ValueError("Non-zero rotation commands are disabled by the robot interface config.")

        translation = np.asarray(
            [
                normalized["tip_delta_x"],
                normalized["tip_delta_y"],
                normalized["tip_delta_z"],
            ],
            dtype=float,
        )
        if self.config.deadband_m > 0.0:
            translation[np.abs(translation) < self.config.deadband_m] = 0.0
        self._target_delta = np.clip(
            translation,
            -np.asarray(self.config.max_delta_m, dtype=float),
            np.asarray(self.config.max_delta_m, dtype=float),
        )
        return normalized

    def hold(self) -> None:
        """Stop progressing toward a stale target without jumping back to neutral."""
        self._target_delta = self._applied_delta.copy()

    def step(self) -> np.ndarray:
        filtered_target = self._target_delta
        if self.config.smooth_alpha < 1.0:
            filtered_target = self._applied_delta + self.config.smooth_alpha * (
                self._target_delta - self._applied_delta
            )

        max_step = np.asarray(self.config.max_speed_m_s, dtype=float) * self.update_interval_s
        step = np.clip(filtered_target - self._applied_delta, -max_step, max_step)
        self._applied_delta = np.clip(
            self._applied_delta + step,
            -np.asarray(self.config.max_delta_m, dtype=float),
            np.asarray(self.config.max_delta_m, dtype=float),
        )
        return self._applied_delta.copy()

    def applied_command(self) -> TipPoseCommand:
        return {
            "tip_delta_x": float(self._applied_delta[0]),
            "tip_delta_y": float(self._applied_delta[1]),
            "tip_delta_z": float(self._applied_delta[2]),
            "tip_delta_rx": 0.0,
            "tip_delta_ry": 0.0,
            "tip_delta_rz": 0.0,
        }
