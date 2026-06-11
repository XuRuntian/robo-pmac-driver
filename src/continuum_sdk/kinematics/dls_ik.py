import math
from dataclasses import dataclass
from typing import Literal

import numpy as np

from .geometry import ContinuumGeometry

TaskMode = Literal["position", "pos_z", "pose"]


@dataclass
class IKResult:
    u: np.ndarray
    error: np.ndarray
    du: np.ndarray
    converged: bool


def rotx(x: float) -> np.ndarray:
    c, s = np.cos(x), np.sin(x)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], dtype=float)


def roty(x: float) -> np.ndarray:
    c, s = np.cos(x), np.sin(x)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=float)


def rotz(x: float) -> np.ndarray:
    c, s = np.cos(x), np.sin(x)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=float)


def so3_log_vec(r: np.ndarray) -> np.ndarray:
    tr = np.clip((np.trace(r) - 1.0) / 2.0, -1.0, 1.0)
    theta = np.arccos(tr)
    if theta < 1e-8:
        return np.zeros(3, dtype=float)
    s = (r - r.T) / (2.0 * np.sin(theta))
    return theta * np.array([s[2, 1], s[0, 2], s[1, 0]], dtype=float)


def rotvec_to_matrix(rotvec: np.ndarray) -> np.ndarray:
    """Convert a rotation vector into a 3x3 rotation matrix."""
    vector = np.asarray(rotvec, dtype=float)
    if vector.shape != (3,):
        raise ValueError("rotvec must contain exactly three values")

    theta = float(np.linalg.norm(vector))
    if theta < 1e-12:
        return np.eye(3, dtype=float)

    axis = vector / theta
    skew = np.array(
        [
            [0.0, -axis[2], axis[1]],
            [axis[2], 0.0, -axis[0]],
            [-axis[1], axis[0], 0.0],
        ],
        dtype=float,
    )
    return (
        np.eye(3, dtype=float)
        + np.sin(theta) * skew
        + (1.0 - np.cos(theta)) * (skew @ skew)
    )


def safe_sinc(x: float) -> float:
    if abs(x) < 1e-6:
        x2 = x * x
        return 1.0 - x2 / 6.0 + x2 * x2 / 120.0
    return float(np.sin(x) / x)


def safe_one_minus_cos_over_th(x: float) -> float:
    if abs(x) < 1e-6:
        x2 = x * x
        x4 = x2 * x2
        return float(x / 2.0 - x * x2 / 24.0 + x * x4 / 720.0)
    return float((1.0 - np.cos(x)) / x)


def const_curv_segment_rp(length: float, theta: float, phi: float) -> tuple[np.ndarray, np.ndarray]:
    if abs(theta) < 1e-6:
        px = length * safe_one_minus_cos_over_th(theta)
        pz = length * safe_sinc(theta)
    else:
        radius = length / theta
        px = radius * (1.0 - np.cos(theta))
        pz = radius * np.sin(theta)

    r_seg = rotz(phi) @ roty(theta) @ rotz(-phi)
    p_seg = rotz(phi) @ np.array([px, 0.0, pz], dtype=float)
    return r_seg, p_seg


def u_to_qu(u: np.ndarray) -> np.ndarray:
    d, th_a, ph_a, th_c, ph_c = u
    return np.array(
        [
            d,
            th_a * np.cos(ph_a),
            th_a * np.sin(ph_a),
            th_c * np.cos(ph_c),
            th_c * np.sin(ph_c),
        ],
        dtype=float,
    )


def qu_to_u(qu: np.ndarray) -> np.ndarray:
    d, ux_a, uy_a, ux_c, uy_c = qu
    th_a = float(np.hypot(ux_a, uy_a))
    th_c = float(np.hypot(ux_c, uy_c))
    ph_a = float(np.arctan2(uy_a, ux_a)) if th_a > 1e-12 else 0.0
    ph_c = float(np.arctan2(uy_c, ux_c)) if th_c > 1e-12 else 0.0
    return np.array([d, th_a, ph_a, th_c, ph_c], dtype=float)


def clamp_qu(qu: np.ndarray, d_range: tuple[float, float], cap_a: float, cap_c: float) -> np.ndarray:
    q = qu.copy()
    q[0] = np.clip(q[0], d_range[0], d_range[1])

    ra = float(np.hypot(q[1], q[2]))
    if ra > cap_a:
        q[1:3] *= cap_a / (ra + 1e-12)

    rc = float(np.hypot(q[3], q[4]))
    if rc > cap_c:
        q[3:5] *= cap_c / (rc + 1e-12)

    return q


class DLSIK:
    def __init__(self, geometry: ContinuumGeometry | None = None, task_mode: TaskMode = "position"):
        self.geometry = geometry or ContinuumGeometry()
        self.task_mode = task_mode
        self.u = np.zeros(5, dtype=float)

        self.lmbda = 8e-3
        self.alpha = 1.0

        self.pos_tol = 2e-4
        self.dir_tol = 1e-3
        self.ori_tol = 5e-4

        self.eps_j = np.array([1e-3, 5e-3, 5e-3, 5e-3, 5e-3], dtype=float)
        self.max_du = 0.05 * np.array([0.006, 0.05, 0.05, 0.05, 0.05], dtype=float)

        self.w_pos = 1.0
        self.w_dir = 0.1
        self.w_ori = 0.1

    def reset(self, u: np.ndarray | None = None) -> None:
        self.u = np.zeros(5, dtype=float) if u is None else self._clip_u(np.asarray(u, dtype=float))

    def fk_tip(self, u: np.ndarray | None = None) -> tuple[np.ndarray, np.ndarray]:
        g = self.geometry
        d, th_a, ph_a, th_c, ph_c = self._clip_u(np.asarray(self.u if u is None else u, dtype=float))

        r = rotx(-math.pi / 2.0)
        p = np.array(g.base_offset, dtype=float)

        p = p + r @ np.array([0.0, 0.0, d], dtype=float)

        r_a, p_a = const_curv_segment_rp(g.s_a, th_a, ph_a)
        p = p + r @ p_a
        r = r @ r_a

        p = p + r @ np.array([0.0, 0.0, g.h_bc - g.s_s], dtype=float)

        r_c, p_c = const_curv_segment_rp(g.s_c, th_c, ph_c)
        p = p + r @ p_c
        r = r @ r_c

        p = p + r @ np.array([0.0, 0.0, g.h_de - g.s_s], dtype=float)
        return p, r

    def solve(
        self,
        p_goal: np.ndarray,
        r_goal: np.ndarray | None = None,
        z_goal: np.ndarray | None = None,
        max_steps: int = 5,
    ) -> IKResult:
        err = None
        du = np.zeros(5, dtype=float)

        for _ in range(max_steps):
            err, du = self.step(p_goal=p_goal, r_goal=r_goal, z_goal=z_goal)
            if self.has_converged(err):
                break

        assert err is not None
        return IKResult(
            u=self.u.copy(),
            error=err.copy(),
            du=du.copy(),
            converged=self.has_converged(err),
        )

    def step(
        self,
        p_goal: np.ndarray,
        r_goal: np.ndarray | None = None,
        z_goal: np.ndarray | None = None,
    ) -> tuple[np.ndarray, np.ndarray]:
        g = self.geometry
        cap_a = g.theta_a_max
        cap_c = g.theta_c_max
        d_range = (g.d_min, g.d_max)

        qu = clamp_qu(u_to_qu(self.u), d_range, cap_a, cap_c)

        j, e = self._jac_fd_qu(qu, p_goal, r_goal, z_goal)
        a = j @ j.T + (self.lmbda**2) * np.eye(j.shape[0])
        dqu = -self.alpha * (j.T @ np.linalg.solve(a, e))

        if self.task_mode != "position":
            # Preserve the coupled DLS direction. Per-axis clipping can cancel
            # the relative segment motion needed for orientation control.
            step_ratio = float(np.max(np.abs(dqu) / self.max_du))
            if step_ratio > 1.0:
                dqu /= step_ratio
        else:
            dqu = np.clip(dqu, -self.max_du, self.max_du)

        err0 = float(np.linalg.norm(e))
        best_qu = qu.copy()
        best_err = err0

        scale = 1.0
        for _ in range(3):
            qu_try = clamp_qu(qu + scale * dqu, d_range, cap_a, cap_c)
            e_try = self._task_qu(qu_try, p_goal, r_goal, z_goal)
            err_try = float(np.linalg.norm(e_try))

            if err_try < best_err:
                best_qu = qu_try
                best_err = err_try
                break

            scale *= 0.5

        u_new = self._clip_u(qu_to_u(best_qu))
        du_report = u_new - self.u
        self.u = u_new

        e_out = self._task_qu(best_qu, p_goal, r_goal, z_goal)
        return e_out, du_report

    def has_converged(self, e: np.ndarray) -> bool:
        if self.task_mode == "position":
            return float(np.linalg.norm(e)) <= self.pos_tol

        if self.task_mode == "pos_z":
            return (
                float(np.linalg.norm(e[:3])) <= self.pos_tol
                and float(np.linalg.norm(e[3:])) <= self.dir_tol
            )

        if self.task_mode == "pose":
            return (
                float(np.linalg.norm(e[:3])) <= self.pos_tol
                and float(np.linalg.norm(e[3:])) <= self.ori_tol
            )

        raise ValueError(f"invalid task_mode: {self.task_mode}")

    def _task_qu(
        self,
        qu: np.ndarray,
        p_goal: np.ndarray,
        r_goal: np.ndarray | None,
        z_goal: np.ndarray | None,
    ) -> np.ndarray:
        return self._task_u(qu_to_u(qu), p_goal, r_goal, z_goal)

    def _task_u(
        self,
        u: np.ndarray,
        p_goal: np.ndarray,
        r_goal: np.ndarray | None,
        z_goal: np.ndarray | None,
    ) -> np.ndarray:
        p, r = self.fk_tip(u)
        e_pos = self.w_pos * (np.asarray(p_goal, dtype=float) - p)

        if self.task_mode == "position":
            return e_pos

        if self.task_mode == "pos_z":
            if z_goal is None:
                raise ValueError("pos_z mode requires z_goal")
            z = r[:, 2] / (np.linalg.norm(r[:, 2]) + 1e-9)
            zg = np.asarray(z_goal, dtype=float)
            zg = zg / (np.linalg.norm(zg) + 1e-9)
            return np.hstack([e_pos, self.w_dir * (zg - z)])

        if self.task_mode == "pose":
            if r_goal is None:
                raise ValueError("pose mode requires r_goal")
            e_rot = self.w_ori * so3_log_vec(r.T @ np.asarray(r_goal, dtype=float))
            return np.hstack([e_pos, e_rot])

        raise ValueError(f"invalid task_mode: {self.task_mode}")

    def _jac_fd_qu(
        self,
        qu: np.ndarray,
        p_goal: np.ndarray,
        r_goal: np.ndarray | None,
        z_goal: np.ndarray | None,
    ) -> tuple[np.ndarray, np.ndarray]:
        g = self.geometry
        f0 = self._task_qu(qu, p_goal, r_goal, z_goal)
        j = np.zeros((f0.size, qu.size), dtype=float)

        for k in range(qu.size):
            q1 = qu.copy()
            q1[k] += self.eps_j[k]
            q1 = clamp_qu(q1, (g.d_min, g.d_max), g.theta_a_max, g.theta_c_max)

            denom = q1[k] - qu[k]
            if abs(denom) < 1e-12:
                continue

            f1 = self._task_qu(q1, p_goal, r_goal, z_goal)
            j[:, k] = (f1 - f0) / denom

        return j, f0

    def _clip_u(self, u: np.ndarray) -> np.ndarray:
        g = self.geometry
        out = u.copy()
        out[0] = np.clip(out[0], g.d_min, g.d_max)
        out[1] = np.clip(out[1], 0.0, g.theta_a_max)
        out[2] = (out[2] + np.pi) % (2.0 * np.pi) - np.pi
        out[3] = np.clip(out[3], 0.0, g.theta_c_max)
        out[4] = (out[4] + np.pi) % (2.0 * np.pi) - np.pi
        return out
