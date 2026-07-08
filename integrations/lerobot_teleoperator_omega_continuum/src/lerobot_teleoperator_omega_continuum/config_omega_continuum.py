from dataclasses import dataclass

import numpy as np

from lerobot.teleoperators.config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("omega_continuum")
@dataclass
class OmegaContinuumConfig(TeleoperatorConfig):
    simulate: bool = False
    scale_x: float = 0.5
    scale_y: float = 0.08
    scale_z: float = 0.25
    omega_map: str = "zxy"
    rotation_map: str = "zxy"
    position_offset_x: float = 0.0
    position_offset_y: float = 0.0
    position_offset_z: float = 0.0
    max_delta_x: float = 0.03
    max_delta_y: float = 0.01
    max_delta_z: float = 0.03
    deadband_m: float = 0.0003
    rotation_scale_x: float = -0.3
    rotation_scale_y: float = 0.3
    rotation_scale_z: float = 0.0
    max_rotation_x: float = 0.45
    max_rotation_y: float = 0.45
    max_rotation_z: float = 0.0
    rotation_deadband_rad: float = 0.01
    zero_samples: int = 20
    zero_sample_period_s: float = 0.005
    clutch_enabled: bool = True
    clutch_key: str = "space"
    axis_debug_enabled: bool = False
    axis_debug_hz: float = 4.0
    axis_debug_min_omega_mm: float = 1.0
    axis_debug_min_tip_mm: float = 0.2
    axis_debug_change_omega_mm: float = 2.0
    axis_debug_change_tip_mm: float = 0.5
    axis_debug_show_xyz: bool = False
    axis_debug_min_omega_rot_rad: float = 0.03
    axis_debug_min_tip_rot_rad: float = 0.01
    axis_debug_change_omega_rot_rad: float = 0.03
    axis_debug_change_tip_rot_rad: float = 0.01

    def __post_init__(self) -> None:
        self.omega_map = self.omega_map.lower()
        self.rotation_map = self.rotation_map.lower()
        self.clutch_key = self.clutch_key.lower()
        if sorted(self.omega_map) != ["x", "y", "z"]:
            raise ValueError("omega_map must be a permutation of xyz.")
        if sorted(self.rotation_map) != ["x", "y", "z"]:
            raise ValueError("rotation_map must be a permutation of xyz.")
        if not all(
            np.isfinite(value)
            for value in (self.position_offset_x, self.position_offset_y, self.position_offset_z)
        ):
            raise ValueError("position offsets must be finite.")
        if any(value < 0.0 for value in (self.max_delta_x, self.max_delta_y, self.max_delta_z)):
            raise ValueError("Omega maximum deltas must be non-negative.")
        if self.deadband_m < 0.0:
            raise ValueError("deadband_m must be non-negative.")
        if any(
            value < 0.0
            for value in (self.max_rotation_x, self.max_rotation_y, self.max_rotation_z)
        ):
            raise ValueError("Omega maximum rotations must be non-negative.")
        if self.rotation_deadband_rad < 0.0:
            raise ValueError("rotation_deadband_rad must be non-negative.")
        if self.zero_samples <= 0:
            raise ValueError("zero_samples must be positive.")
        if self.zero_sample_period_s < 0.0:
            raise ValueError("zero_sample_period_s must be non-negative.")
        if not self.clutch_key:
            raise ValueError("clutch_key must be non-empty.")
        if self.axis_debug_hz <= 0.0:
            raise ValueError("axis_debug_hz must be positive.")
        if self.axis_debug_min_omega_mm < 0.0:
            raise ValueError("axis_debug_min_omega_mm must be non-negative.")
        if self.axis_debug_min_tip_mm < 0.0:
            raise ValueError("axis_debug_min_tip_mm must be non-negative.")
        if self.axis_debug_change_omega_mm < 0.0:
            raise ValueError("axis_debug_change_omega_mm must be non-negative.")
        if self.axis_debug_change_tip_mm < 0.0:
            raise ValueError("axis_debug_change_tip_mm must be non-negative.")
        if self.axis_debug_min_omega_rot_rad < 0.0:
            raise ValueError("axis_debug_min_omega_rot_rad must be non-negative.")
        if self.axis_debug_min_tip_rot_rad < 0.0:
            raise ValueError("axis_debug_min_tip_rot_rad must be non-negative.")
        if self.axis_debug_change_omega_rot_rad < 0.0:
            raise ValueError("axis_debug_change_omega_rot_rad must be non-negative.")
        if self.axis_debug_change_tip_rot_rad < 0.0:
            raise ValueError("axis_debug_change_tip_rot_rad must be non-negative.")
