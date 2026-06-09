from __future__ import annotations

from continuum_sdk.control.tendon_mapper import ContinuumTendonMapper
from continuum_sdk.core.config import ContinuumConfig
from continuum_sdk.kinematics.dls_ik import DLSIK
from continuum_sdk.kinematics.geometry import ContinuumGeometry


def build_continuum_ik(cfg: ContinuumConfig) -> DLSIK:
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

    ik = DLSIK(geometry=geometry, task_mode=cfg.ik.task_mode)
    ik.lmbda = cfg.ik.lambda_damping
    ik.alpha = cfg.ik.alpha
    ik.pos_tol = cfg.ik.pos_tol_m
    ik.dir_tol = cfg.ik.dir_tol
    ik.ori_tol = cfg.ik.ori_tol_rad
    return ik


def build_tendon_mapper(cfg: ContinuumConfig) -> ContinuumTendonMapper:
    return ContinuumTendonMapper(
        hole_radius=cfg.actuation.hole_radius_m,
        spool_diameter=cfg.actuation.spool_diameter_m,
    )
