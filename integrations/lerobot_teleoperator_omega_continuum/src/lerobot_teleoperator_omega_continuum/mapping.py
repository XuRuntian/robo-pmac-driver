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


def _matrix_to_rotvec(rotation: np.ndarray) -> np.ndarray:
    cosine = float(np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0))
    theta = float(np.arccos(cosine))
    if theta < 1e-8:
        return np.zeros(3, dtype=float)
    if np.pi - theta < 1e-5:
        eigenvalues, eigenvectors = np.linalg.eig(rotation)
        axis = np.real(eigenvectors[:, np.argmin(np.abs(eigenvalues - 1.0))])
        axis /= np.linalg.norm(axis)
        return theta * axis
    skew = (rotation - rotation.T) / (2.0 * np.sin(theta))
    return theta * np.array([skew[2, 1], skew[0, 2], skew[1, 0]], dtype=float)


class OmegaContinuumMapper:
    """Map an Omega pose from a sampled zero pose into robot tip offsets."""

    def __init__(
        self,
        *,
        scale_xyz: tuple[float, float, float],
        max_delta_xyz: tuple[float, float, float],
        deadband_m: float,
        omega_map: str,
        rotation_map: str | None = None,
        position_offset_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0),
        rotation_scale_xyz: tuple[float, float, float] = (1.0, 1.0, 1.0),
        max_rotation_xyz: tuple[float, float, float] = (0.15, 0.15, 0.0),
        rotation_deadband_rad: float = 0.005,
    ) -> None:
        axis_index = {"x": 0, "y": 1, "z": 2}
        omega_map = omega_map.lower()
        if sorted(omega_map) != ["x", "y", "z"]:
            raise ValueError("omega_map must be a permutation of xyz.")
        rotation_map = omega_map if rotation_map is None else rotation_map.lower()
        if sorted(rotation_map) != ["x", "y", "z"]:
            raise ValueError("rotation_map must be a permutation of xyz.")

        self.scale = np.asarray(scale_xyz, dtype=float)
        self.max_delta = np.asarray(max_delta_xyz, dtype=float)
        self.position_offset = np.asarray(position_offset_xyz, dtype=float)
        if self.position_offset.shape != (3,) or not np.all(np.isfinite(self.position_offset)):
            raise ValueError("position_offset_xyz must contain three finite values.")
        self.deadband_m = float(deadband_m)
        self.rotation_scale = np.asarray(rotation_scale_xyz, dtype=float)
        self.max_rotation = np.asarray(max_rotation_xyz, dtype=float)
        self.rotation_deadband_rad = float(rotation_deadband_rad)
        self._omega_to_robot = np.asarray([axis_index[axis] for axis in omega_map], dtype=int)
        self._omega_rotation_to_robot = np.asarray(
            [axis_index[axis] for axis in rotation_map],
            dtype=int,
        )
        self._zero: np.ndarray | None = None
        self._zero_orientation: np.ndarray | None = None

    @staticmethod
    def _validate_position(position: np.ndarray) -> np.ndarray:
        position = np.asarray(position, dtype=float)
        if position.shape != (3,) or not np.all(np.isfinite(position)):
            raise ValueError("Omega position must contain three finite values.")
        return position

    @staticmethod
    def _validate_orientation(orientation: np.ndarray) -> np.ndarray:
        orientation = np.asarray(orientation, dtype=float)
        if orientation.shape != (3, 3) or not np.all(np.isfinite(orientation)):
            raise ValueError("Omega orientation must be a finite 3x3 matrix.")
        return orientation

    def control_position(self, position: np.ndarray, orientation: np.ndarray | None = None) -> np.ndarray:
        position = self._validate_position(position)
        if orientation is None:
            orientation = np.eye(3, dtype=float)
        orientation = self._validate_orientation(orientation)
        return position + orientation @ self.position_offset

    def omega_position_delta(
        self,
        position: np.ndarray,
        orientation: np.ndarray | None = None,
    ) -> np.ndarray:
        if self._zero is None:
            raise RuntimeError("Omega zero position has not been sampled.")
        return self.control_position(position, orientation) - self._zero

    def omega_rotation_delta(self, orientation: np.ndarray | None = None) -> np.ndarray:
        if self._zero_orientation is None:
            raise RuntimeError("Omega zero orientation has not been sampled.")
        if orientation is None:
            orientation = self._zero_orientation
        orientation = self._validate_orientation(orientation)
        return _matrix_to_rotvec(self._zero_orientation.T @ orientation)

    def set_zero(self, position: np.ndarray, orientation: np.ndarray | None = None) -> None:
        if orientation is None:
            orientation = np.eye(3, dtype=float)
        position = self._validate_position(position)
        orientation = self._validate_orientation(orientation)
        self._zero = self.control_position(position, orientation)
        self._zero_orientation = orientation.copy()

    def map_position(self, position: np.ndarray) -> dict[str, float]:
        return self.map_pose(position, self._zero_orientation)

    def map_pose(
        self,
        position: np.ndarray,
        orientation: np.ndarray | None,
    ) -> dict[str, float]:
        if self._zero is None:
            raise RuntimeError("Omega zero position has not been sampled.")
        if self._zero_orientation is None:
            raise RuntimeError("Omega zero orientation has not been sampled.")

        if orientation is None:
            orientation = self._zero_orientation
        position = self._validate_position(position)
        orientation = self._validate_orientation(orientation)

        control_position = self.control_position(position, orientation)
        delta = (control_position - self._zero)[self._omega_to_robot] * self.scale
        if self.deadband_m > 0.0:
            delta[np.abs(delta) < self.deadband_m] = 0.0
        delta = np.clip(delta, -self.max_delta, self.max_delta)

        omega_rotation = self.omega_rotation_delta(orientation)
        robot_rotation = omega_rotation[self._omega_rotation_to_robot] * self.rotation_scale
        if np.linalg.norm(robot_rotation) < self.rotation_deadband_rad:
            robot_rotation.fill(0.0)
        robot_rotation = np.clip(robot_rotation, -self.max_rotation, self.max_rotation)
        return {
            "tip_delta_x": float(delta[0]),
            "tip_delta_y": float(delta[1]),
            "tip_delta_z": float(delta[2]),
            "tip_delta_rx": float(robot_rotation[0]),
            "tip_delta_ry": float(robot_rotation[1]),
            "tip_delta_rz": float(robot_rotation[2]),
        }
