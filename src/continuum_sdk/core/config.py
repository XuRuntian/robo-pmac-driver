from dataclasses import dataclass
from typing import Literal


@dataclass(frozen=True)
class ContinuumGeometryConfig:
    s_s: float
    s_a: float
    s_c: float
    h_bc: float
    h_de: float

    theta_a_max_rad: float
    theta_c_max_rad: float

    d_min_m: float
    d_max_m: float

    base_offset_m: tuple[float, float, float]
    use_sheath: bool = False


@dataclass(frozen=True)
class ContinuumIKConfig:
    task_mode: Literal["position", "pos_z", "pose"]

    lambda_damping: float
    alpha: float

    pos_tol_m: float
    dir_tol: float
    ori_tol_rad: float

    max_inner_steps: int
    jacobian_eps: tuple[float, float, float, float, float] = (
        1e-3,
        5e-3,
        5e-3,
        5e-3,
        5e-3,
    )
    position_weight: float = 1.0
    direction_weight: float = 0.1
    orientation_weight: float = 0.1
    line_search_steps: int = 3
    gradient_fallback_step: float = 0.01
    step_limit_enabled: bool = False
    max_delta_u: tuple[float, float, float, float, float] = (
        0.0003,
        0.0025,
        0.0025,
        0.0025,
        0.0025,
    )


@dataclass(frozen=True)
class ContinuumActuationConfig:
    hole_radius_m: float
    spool_diameter_m: float


@dataclass(frozen=True)
class ContinuumControlConfig:
    update_hz: int
    pvt_velocity_from_diff: bool


@dataclass(frozen=True)
class ContinuumConfig:
    geometry: ContinuumGeometryConfig
    ik: ContinuumIKConfig
    actuation: ContinuumActuationConfig
    control: ContinuumControlConfig
