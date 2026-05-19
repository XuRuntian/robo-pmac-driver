from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional


def wrap_to_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def angle_diff(a: float, b: float) -> float:
    return wrap_to_pi(a - b)


def is_singular_theta(theta: float, eps: float = 1e-10) -> bool:
    return abs(theta) < eps


@dataclass(frozen=True)
class JointSpace:
    phi_a: float
    theta_a: float
    phi_c: float
    theta_c: float


@dataclass(frozen=True)
class CCComponents:
    u_ax: float
    u_ay: float
    u_cx: float
    u_cy: float


@dataclass(frozen=True)
class TendonLengths:
    dl1: float
    dl2: float
    dl3: float
    dl4: float
    dl5: float
    dl6: float
    dl7: float
    dl8: float

    def as_tuple(self) -> tuple[float, ...]:
        return (self.dl1, self.dl2, self.dl3, self.dl4, self.dl5, self.dl6, self.dl7, self.dl8)


@dataclass(frozen=True)
class MotorAngles:
    alpha1: float
    alpha2: float
    alpha3: float
    alpha4: float

    def as_tuple(self) -> tuple[float, ...]:
        return (self.alpha1, self.alpha2, self.alpha3, self.alpha4)


@dataclass(frozen=True)
class RecoveredJointSpace:
    phi_a: float
    theta_a: float
    phi_c: float
    theta_c: float
    singular_a: bool
    singular_c: bool


class TDRCJointMotorModel:
    """
    Analytical mapping for the TDRC joint, tendon, and motor coordinates.

    The PMAC axis layer can still apply wiring order and sign corrections. Keep
    motor_index_map and motor_direction_map unset when that layer owns the
    physical mapping.
    """

    def __init__(
        self,
        hole_radius: float,
        spool_diameter: float,
        cc_sign: float = -1.0,
        zero_eps: float = 1e-10,
        motor_index_map: Optional[dict[int, int]] = None,
        motor_direction_map: Optional[dict[int, int]] = None,
    ) -> None:
        if hole_radius <= 0.0:
            raise ValueError("hole_radius must be positive.")
        if spool_diameter <= 0.0:
            raise ValueError("spool_diameter must be positive.")
        if cc_sign == 0.0:
            raise ValueError("cc_sign must be non-zero.")

        self.r_hole = float(hole_radius)
        self.d_spool = float(spool_diameter)
        self.cc_sign = float(cc_sign)
        self.zero_eps = float(zero_eps)
        self._ideal_to_real = self._validate_motor_index_map(motor_index_map)
        self._real_direction = self._validate_motor_direction_map(motor_direction_map)
        self._real_to_ideal = {real_idx: ideal_idx for ideal_idx, real_idx in self._ideal_to_real.items()}

    @staticmethod
    def _validate_motor_index_map(motor_index_map: Optional[dict[int, int]]) -> dict[int, int]:
        identity = {1: 1, 2: 2, 3: 3, 4: 4}
        if motor_index_map is None:
            return identity
        if set(motor_index_map.keys()) != {1, 2, 3, 4}:
            raise ValueError("motor_index_map keys must be exactly {1,2,3,4}.")
        if set(motor_index_map.values()) != {1, 2, 3, 4}:
            raise ValueError("motor_index_map values must be a permutation of {1,2,3,4}.")
        return {idx: int(motor_index_map[idx]) for idx in (1, 2, 3, 4)}

    @staticmethod
    def _validate_motor_direction_map(motor_direction_map: Optional[dict[int, int]]) -> dict[int, int]:
        identity = {1: 1, 2: 1, 3: 1, 4: 1}
        if motor_direction_map is None:
            return identity
        if set(motor_direction_map.keys()) != {1, 2, 3, 4}:
            raise ValueError("motor_direction_map keys must be exactly {1,2,3,4}.")
        out = {}
        for idx in (1, 2, 3, 4):
            sign = int(motor_direction_map[idx])
            if sign not in (-1, 1):
                raise ValueError("motor_direction_map values must be +1 or -1.")
            out[idx] = sign
        return out

    @property
    def K(self) -> float:
        return 2.0 * self.r_hole / self.d_spool

    @staticmethod
    def gamma_a(i: int) -> float:
        if i in (1, 2, 3, 4):
            return (i - 1) * math.pi / 2.0
        if i in (5, 6, 7, 8):
            return (i - 5) * math.pi / 2.0 + math.pi / 4.0
        raise ValueError("gamma_a index must be in {1,...,8}.")

    @staticmethod
    def gamma_c(j: int) -> float:
        if j in (5, 6, 7, 8):
            return (j - 5) * math.pi / 2.0 + math.pi / 4.0
        raise ValueError("gamma_c index must be in {5,6,7,8}.")

    def joint_to_cc_components(self, joint: JointSpace) -> CCComponents:
        s = self.cc_sign
        return CCComponents(
            u_ax=s * joint.theta_a * math.cos(joint.phi_a),
            u_ay=s * joint.theta_a * math.sin(joint.phi_a),
            u_cx=s * joint.theta_c * math.cos(joint.phi_c),
            u_cy=s * joint.theta_c * math.sin(joint.phi_c),
        )

    def joint_to_tendon_lengths(self, joint: JointSpace) -> TendonLengths:
        r = self.r_hole
        phi_a, theta_a = joint.phi_a, joint.theta_a
        phi_c, theta_c = joint.phi_c, joint.theta_c

        dl1 = r * theta_a * math.cos(self.gamma_a(1) - phi_a)
        dl2 = r * theta_a * math.cos(self.gamma_a(2) - phi_a)
        dl3 = r * theta_a * math.cos(self.gamma_a(3) - phi_a)
        dl4 = r * theta_a * math.cos(self.gamma_a(4) - phi_a)
        dl5 = r * theta_a * math.cos(self.gamma_a(5) - phi_a) + r * theta_c * math.cos(self.gamma_c(5) - phi_c)
        dl6 = r * theta_a * math.cos(self.gamma_a(6) - phi_a) + r * theta_c * math.cos(self.gamma_c(6) - phi_c)
        dl7 = r * theta_a * math.cos(self.gamma_a(7) - phi_a) + r * theta_c * math.cos(self.gamma_c(7) - phi_c)
        dl8 = r * theta_a * math.cos(self.gamma_a(8) - phi_a) + r * theta_c * math.cos(self.gamma_c(8) - phi_c)

        return TendonLengths(dl1=dl1, dl2=dl2, dl3=dl3, dl4=dl4, dl5=dl5, dl6=dl6, dl7=dl7, dl8=dl8)

    def tendon_lengths_to_motor_angles(self, tendon: TendonLengths) -> MotorAngles:
        scale = -2.0 / self.d_spool
        motor_ideal = MotorAngles(
            alpha1=scale * tendon.dl1,
            alpha2=scale * tendon.dl2,
            alpha3=scale * tendon.dl5,
            alpha4=scale * tendon.dl6,
        )
        motor_real = self._permute_motor_ideal_to_real(motor_ideal)
        return self._apply_motor_direction_real(motor_real)

    def joint_to_motor_angles(self, joint: JointSpace) -> MotorAngles:
        K = self.K
        phi_a, theta_a = joint.phi_a, joint.theta_a
        phi_c, theta_c = joint.phi_c, joint.theta_c

        alpha1 = -K * theta_a * math.cos(phi_a)
        alpha2 = -K * theta_a * math.sin(phi_a)
        alpha3 = -K * (
            theta_a * math.cos(math.pi / 4.0 - phi_a)
            + theta_c * math.cos(math.pi / 4.0 - phi_c)
        )
        alpha4 = -K * (
            theta_a * math.cos(3.0 * math.pi / 4.0 - phi_a)
            + theta_c * math.cos(3.0 * math.pi / 4.0 - phi_c)
        )

        motor_ideal = MotorAngles(alpha1=alpha1, alpha2=alpha2, alpha3=alpha3, alpha4=alpha4)
        motor_real = self._permute_motor_ideal_to_real(motor_ideal)
        return self._apply_motor_direction_real(motor_real)

    def motor_angles_to_joint(self, motor: MotorAngles) -> RecoveredJointSpace:
        K = self.K
        if abs(K) < self.zero_eps:
            raise ZeroDivisionError("Invalid K: too close to zero.")

        motor_real = self._remove_motor_direction_real(motor)
        motor_ideal = self._permute_motor_real_to_ideal(motor_real)
        a1, a2, a3, a4 = motor_ideal.as_tuple()

        theta_a = math.sqrt(a1 * a1 + a2 * a2) / K
        singular_a = is_singular_theta(theta_a, self.zero_eps)
        phi_a = 0.0 if singular_a else wrap_to_pi(math.atan2(-a2, -a1))

        u = -a3 / K - theta_a * math.cos(math.pi / 4.0 - phi_a)
        v = -a4 / K - theta_a * math.cos(3.0 * math.pi / 4.0 - phi_a)

        theta_c = math.sqrt(u * u + v * v)
        singular_c = is_singular_theta(theta_c, self.zero_eps)
        phi_c = 0.0 if singular_c else wrap_to_pi(math.atan2(u + v, u - v))

        return RecoveredJointSpace(
            phi_a=phi_a,
            theta_a=theta_a,
            phi_c=phi_c,
            theta_c=theta_c,
            singular_a=singular_a,
            singular_c=singular_c,
        )

    def _permute_motor_ideal_to_real(self, motor_ideal: MotorAngles) -> MotorAngles:
        ideal = {1: motor_ideal.alpha1, 2: motor_ideal.alpha2, 3: motor_ideal.alpha3, 4: motor_ideal.alpha4}
        real = {self._ideal_to_real[idx]: ideal[idx] for idx in (1, 2, 3, 4)}
        return MotorAngles(alpha1=real[1], alpha2=real[2], alpha3=real[3], alpha4=real[4])

    def _permute_motor_real_to_ideal(self, motor_real: MotorAngles) -> MotorAngles:
        real = {1: motor_real.alpha1, 2: motor_real.alpha2, 3: motor_real.alpha3, 4: motor_real.alpha4}
        ideal = {self._real_to_ideal[idx]: real[idx] for idx in (1, 2, 3, 4)}
        return MotorAngles(alpha1=ideal[1], alpha2=ideal[2], alpha3=ideal[3], alpha4=ideal[4])

    def _apply_motor_direction_real(self, motor_real: MotorAngles) -> MotorAngles:
        return MotorAngles(
            alpha1=self._real_direction[1] * motor_real.alpha1,
            alpha2=self._real_direction[2] * motor_real.alpha2,
            alpha3=self._real_direction[3] * motor_real.alpha3,
            alpha4=self._real_direction[4] * motor_real.alpha4,
        )

    def _remove_motor_direction_real(self, motor_real: MotorAngles) -> MotorAngles:
        return self._apply_motor_direction_real(motor_real)
