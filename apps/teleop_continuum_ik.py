from continuum_sdk.core.config_loader import load_continuum_config
from continuum_sdk.kinematics.geometry import ContinuumGeometry
from continuum_sdk.kinematics.dls_ik import DLSIK
from continuum_sdk.control.axis_mapper import ContinuumAxisMapper


cfg = load_continuum_config("config/continuum.yaml")

geometry = ContinuumGeometry(
    s_s=cfg.geometry.s_s,
    s_a=cfg.geometry.s_a,
    s_c=cfg.geometry.s_c,
    h_bc=cfg.geometry.h_bc,
    h_de=cfg.geometry.h_de,
    theta_a_max=cfg.geometry.theta_a_max_rad,
    theta_c_max=cfg.geometry.theta_c_max_rad,
    d_min=cfg.geometry.d_min_m,
    d_max=cfg.geometry.d_max_m,
    base_offset=cfg.geometry.base_offset_m,
)

ik = DLSIK(
    geometry=geometry,
    task_mode=cfg.ik.task_mode,
)

ik.lmbda = cfg.ik.lambda_damping
ik.alpha = cfg.ik.alpha
ik.pos_tol = cfg.ik.pos_tol_m
ik.dir_tol = cfg.ik.dir_tol
ik.ori_tol = cfg.ik.ori_tol_rad

axis_mapper = ContinuumAxisMapper(cfg.axis)

update_hz = cfg.control.update_hz
update_interval = 1.0 / update_hz