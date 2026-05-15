import numpy as np


class ContinuumTendonMapper:
    """
    IK u = [d, theta_a, phi_a, theta_c, phi_c]
    输出 PMAC 5 轴逻辑目标：
    [a_x, a_y, c_x, c_y, d]
    """

    def __init__(self, compensate_world: bool = False, gamma_world: float = 0.0):
        self.compensate_world = compensate_world
        self.gamma_world = gamma_world

    def to_axis_targets(self, u: np.ndarray) -> list[float]:
        d, theta_a, phi_a, theta_c, phi_c = np.asarray(u, dtype=float)

        a_x, a_y = self._constcurv_to_tendon(theta_a, phi_a)
        c_x, c_y = self._constcurv_to_tendon(theta_c, phi_c)

        return [a_x, a_y, c_x, c_y, d]

    def _constcurv_to_tendon(self, theta: float, phi: float) -> tuple[float, float]:
        v = np.array([theta * np.cos(phi), theta * np.sin(phi)], dtype=float)

        if self.compensate_world:
            cg, sg = np.cos(-self.gamma_world), np.sin(-self.gamma_world)
            r2 = np.array([[cg, -sg], [sg, cg]], dtype=float)
            v = r2 @ v

        return float(-v[0]), float(-v[1])
