import numpy as np

from continuum_sdk.kinematics.joint_motor_model import JointSpace, TDRCJointMotorModel

class ContinuumTendonMapper:
    """
    IK u = [d, theta_a, phi_a, theta_c, phi_c] (单位: m, rad)
    输出 逻辑轴目标：[alpha1, alpha2, alpha3, alpha4, d] (单位: rad, m)
    """

    def __init__(
        self,
        compensate_world: bool = False,
        gamma_world: float = 0.0,
        hole_radius: float = 0.003,
        spool_diameter: float = 0.012,
        motor_model: TDRCJointMotorModel | None = None,
    ):
        self.compensate_world = compensate_world
        self.gamma_world = gamma_world
        self.model = motor_model or TDRCJointMotorModel(
            hole_radius=hole_radius,
            spool_diameter=spool_diameter,
            cc_sign=-1.0,
        )

    def to_axis_targets(self, u: np.ndarray) -> list[float]:
        d, theta_a, phi_a, theta_c, phi_c = np.asarray(u, dtype=float)

        if self.compensate_world:
            phi_a -= self.gamma_world
            phi_c -= self.gamma_world

        motor = self.model.joint_to_motor_angles(
            JointSpace(
                phi_a=phi_a,
                theta_a=theta_a,
                phi_c=phi_c,
                theta_c=theta_c,
            )
        )

        return [motor.alpha1, motor.alpha2, motor.alpha3, motor.alpha4, float(d)]
