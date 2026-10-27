from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from continuum_sdk.control.axis_mapper import ContinuumAxisMapper
from continuum_sdk.control.tendon_mapper import ContinuumTendonMapper
from continuum_sdk.control.rotary_target_guard import RotaryTargetGuard, RotaryTargetJump
from continuum_sdk.kinematics.dls_ik import DLSIK, IKResult
from continuum_sdk.kinematics.joint_motor_model import MotorAngles


@dataclass(frozen=True)
class ContinuumPVTCommand:
    p_goal: np.ndarray
    r_goal: np.ndarray | None
    z_goal: np.ndarray | None
    ik_result: IKResult
    axis_targets: list[float]
    target_pulses: list[int]
    velocities: list[float]
    feasibility_checks: int = 0
    feasibility_rejections: int = 0


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

    def commit_pulses(self, target_pulses: list[int]) -> None:
        """Warm-start from the accepted command after physical-axis clipping."""
        logical = self.axis_mapper.pulses_to_logical(self.base_pulses, target_pulses)
        joint = self.tendon_mapper.model.motor_angles_to_joint(MotorAngles(*logical[:4]))
        self.ik.reset(np.array([
            logical[4], joint.theta_a, joint.phi_a, joint.theta_c, joint.phi_c,
        ]))
        self._prev_axis_targets = logical

    def build_command(
        self,
        p_goal: np.ndarray,
        r_goal: np.ndarray | None = None,
        z_goal: np.ndarray | None = None,
        *,
        rotary_guard: RotaryTargetGuard | None = None,
        previous_pulses: list[int] | None = None,
        previous_velocities: list[float] | None = None,
    ) -> ContinuumPVTCommand:
        """Optionally constrain IK trials against the whole next rotary segment.

        Call commit_pulses after accepting a command. The validator uses the
        same mapped positions and endpoint velocities as the final PVT guard;
        it is not a replacement for that independent submission-time check.
        """
        normalized_r_goal = None if r_goal is None else np.asarray(r_goal, dtype=float)
        normalized_z_goal = None if z_goal is None else np.asarray(z_goal, dtype=float)
        checks = rejections = 0
        validate = None
        if rotary_guard is not None:
            if previous_pulses is None or previous_velocities is None or self._prev_axis_targets is None:
                raise ValueError('Guarded IK requires the previous committed position and endpoint velocity.')
            anchor_pulses = list(previous_pulses)
            anchor_velocities = list(previous_velocities)
            def validate(u: np.ndarray) -> bool:
                nonlocal checks, rejections
                logical = self.tendon_mapper.to_axis_targets(u)
                candidate = self.axis_mapper.logical_to_pulses(self.base_pulses, logical)
                velocities = self._build_velocities(logical)
                checks += 1
                try:
                    rotary_guard.check(anchor_pulses, candidate, anchor_velocities,
                                       velocities, self.update_interval_s)
                except RotaryTargetJump:
                    rejections += 1
                    return False
                return True
        solve_options = {} if validate is None else {'candidate_validator': validate}
        result = self.ik.solve(
            p_goal=np.asarray(p_goal, dtype=float),
            r_goal=normalized_r_goal,
            z_goal=normalized_z_goal,
            max_steps=self.max_inner_steps,
            **solve_options,
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
            r_goal=normalized_r_goal,
            z_goal=normalized_z_goal,
            ik_result=result,
            axis_targets=axis_targets,
            target_pulses=target_pulses,
            velocities=velocities,
            feasibility_checks=checks,
            feasibility_rejections=rejections,
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
