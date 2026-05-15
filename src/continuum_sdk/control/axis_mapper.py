import numpy as np

from continuum_sdk.core.config import ContinuumAxisConfig


class ContinuumAxisMapper:
    def __init__(self, config: ContinuumAxisConfig):
        self.config = config

    def logical_to_pulses(
        self,
        base_pulses: list[int],
        logical_targets: list[float],
    ) -> list[int]:
        self._check(logical_targets)

        scale = [
            self.config.pulses_per_rad[0],
            self.config.pulses_per_rad[1],
            self.config.pulses_per_rad[2],
            self.config.pulses_per_rad[3],
            self.config.pulses_per_meter,
        ]

        out = [0] * 5

        for logical_idx, value in enumerate(logical_targets):
            physical_idx = self.config.axis_order[logical_idx]
            sign = self.config.axis_signs[logical_idx]
            out[physical_idx] = int(base_pulses[physical_idx] + sign * value * scale[logical_idx])

        return out

    def logical_velocity_to_pulses_per_ms(
        self,
        logical_velocity: list[float],
    ) -> list[float]:
        scale = [
            self.config.pulses_per_rad[0],
            self.config.pulses_per_rad[1],
            self.config.pulses_per_rad[2],
            self.config.pulses_per_rad[3],
            self.config.pulses_per_meter,
        ]

        out = [0.0] * 5

        for logical_idx, value_per_s in enumerate(logical_velocity):
            physical_idx = self.config.axis_order[logical_idx]
            sign = self.config.axis_signs[logical_idx]
            out[physical_idx] = sign * value_per_s * scale[logical_idx] / 1000.0

        return out

    def diff_velocity(
        self,
        prev_targets: list[float],
        next_targets: list[float],
        dt_s: float,
    ) -> list[float]:
        return ((np.asarray(next_targets) - np.asarray(prev_targets)) / dt_s).tolist()

    def _check(self, logical_targets: list[float]) -> None:
        for i, value in enumerate(logical_targets):
            lo, hi = self.config.soft_limits[i]
            if value < lo or value > hi:
                raise ValueError(f"logical axis {i} out of soft limit: {value}, limit=({lo}, {hi})")
