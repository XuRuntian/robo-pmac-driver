from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from continuum_sdk.control.axis_mapper import ContinuumAxisMapper
from continuum_sdk.control.tendon_mapper import ContinuumTendonMapper
from continuum_sdk.kinematics.dls_ik import DLSIK, IKResult


@dataclass(frozen=True)
class ContinuumPVTCommand:
    p_goal: np.ndarray
    ik_result: IKResult
    axis_targets: list[float]
    target_pulses: list[int]
    velocities: list[float]


class ContinuumPVTMapper:
    """Map Cartesian tip goals into PMAC PVT position and velocity commands."""

    def __init__(
        self,
        ik: DLSIK,
        tendon_mapper: ContinuumTendonMapper,
        axis_mapper: ContinuumAxisMapper,
        base_pulses: list[int],
        update_interval_s: float,
        max_inner_steps: int,
    ) -> None:
        self.ik = ik
        self.tendon_mapper = tendon_mapper
        self.axis_mapper = axis_mapper
        self.base_pulses = list(base_pulses)
        self.update_interval_s = float(update_interval_s)
        self.max_inner_steps = int(max_inner_steps)
        self._prev_axis_targets: list[float] | None = None

    def build_command(self, p_goal: np.ndarray) -> ContinuumPVTCommand:
        result = self.ik.solve(
            p_goal=np.asarray(p_goal, dtype=float),
            max_steps=self.max_inner_steps,
        )
        axis_targets = self.tendon_mapper.to_axis_targets(result.u)
        target_pulses = self.axis_mapper.logical_to_pulses(
            base_pulses=self.base_pulses,
            logical_targets=axis_targets,
        )
        velocities = self._build_velocities(axis_targets)
        self._prev_axis_targets = axis_targets

        return ContinuumPVTCommand(
            p_goal=np.asarray(p_goal, dtype=float),
            ik_result=result,
            axis_targets=axis_targets,
            target_pulses=target_pulses,
            velocities=velocities,
        )

    def _build_velocities(self, axis_targets: list[float]) -> list[float]:
        if self._prev_axis_targets is None:
            return [0.0] * len(axis_targets)

        logical_vel = self.axis_mapper.diff_velocity(
            prev_targets=self._prev_axis_targets,
            next_targets=axis_targets,
            dt_s=self.update_interval_s,
        )
        return self.axis_mapper.logical_velocity_to_pulses_per_ms(logical_vel)
