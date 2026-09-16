"""Full-chain continuum debug: Cartesian goal -> IK -> motors -> PMAC pulses -> feedback.

Dry-run is the default. Add --execute only after checking the printed mapping.
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from continuum_sdk.control.axis_mapper import ContinuumAxisMapper
from continuum_sdk.control.cartesian_frame import apply_axis_transform
from continuum_sdk.core.config_loader import load_continuum_config
from continuum_sdk.core.factory import build_continuum_ik, build_tendon_mapper
from continuum_sdk.kinematics.joint_motor_model import MotorAngles
from pmac_sdk.controller.robot_api import PMACRobotController
from pmac_sdk.core.config_model import PMACConfig


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--execute", action="store_true")
    p.add_argument("--pmac-ip", default="192.168.0.200")
    p.add_argument("--config", default="config/continuum.yaml")
    p.add_argument("--dx", type=float, default=0.0, help="external X offset, mm")
    p.add_argument("--dy", type=float, default=0.0, help="external Y offset, mm")
    p.add_argument("--dz", type=float, default=0.0, help="external Z offset, mm")
    p.add_argument("--rx", type=float, default=0.0, help="external Rx, degrees")
    p.add_argument("--rz", type=float, default=0.0, help="external/world Rz, degrees")
    args = p.parse_args()

    cfg = load_continuum_config(args.config)
    pmac_cfg = PMACConfig(ip=args.pmac_ip)
    ik = build_continuum_ik(cfg)
    tendon = build_tendon_mapper(cfg)
    mapper = ContinuumAxisMapper(pmac_cfg.pulses_per_rad, pmac_cfg.pulses_per_meter,
                                 pmac_cfg.axis_order, pmac_cfg.axis_signs)
    robot = PMACRobotController(pmac_cfg) if args.execute else None
    try:
        if robot:
            robot.safe_boot_and_home()
            base = robot.base_positions.copy()
        else:
            base = [0, 0, 0, 0, 0]

        center, center_r = ik.fk_tip()
        # External frame is X/right, Y/insertion, Z/up; convert to IK frame.
        delta_ext = np.array([args.dx, args.dy, args.dz], dtype=float) / 1000.0
        goal = center + apply_axis_transform(delta_ext, "xyz", (1, -1, 1))
        # World Ry is intentionally absent: it maps to continuum-frame Rz,
        # the disabled tool-roll DOF. World Rz maps to continuum-frame -Ry
        # and is a valid tip-axis tilt command.
        rot_ext = np.deg2rad([args.rx, 0.0, args.rz])
        r_goal = center_r @ np.eye(3)
        z_goal = center_r[:, 2]
        if np.any(rot_ext):
            from continuum_sdk.kinematics.dls_ik import rotvec_to_matrix
            r_goal = center_r @ rotvec_to_matrix(apply_axis_transform(rot_ext, "xzy", (1, -1, 1)))
            # The current five-DOF mechanism controls the tip-axis direction
            # (pos_z), not an independent tool-roll angle.  Passing the full
            # r_goal to pos_z is invalid; use its z axis as the task target.
            z_goal = r_goal[:, 2]

        ik.task_mode = "pos_z"
        result = ik.solve(goal, z_goal=z_goal,
                          max_steps=cfg.ik.max_inner_steps)
        logical = tendon.to_axis_targets(result.u)
        pulses = mapper.logical_to_pulses(base, logical)
        recovered_logical = mapper.pulses_to_logical(base, pulses)
        recovered_joint = tendon.model.motor_angles_to_joint(MotorAngles(*recovered_logical[:4]))
        recovered_u = np.array([recovered_logical[4], recovered_joint.theta_a,
                                recovered_joint.phi_a, recovered_joint.theta_c,
                                recovered_joint.phi_c])
        recovered_p, recovered_r = ik.fk_tip(recovered_u)

        print("=== FULL CHAIN ===")
        print("logical -> PMAC:", [(i + 1, pmac_cfg.axis_order[i] + 1, pmac_cfg.axis_signs[i]) for i in range(5)])
        print("center:", np.round(center, 6), "goal:", np.round(goal, 6))
        print("IK u:", np.round(result.u, 8), "converged:", result.converged,
              "error:", np.round(result.error, 8))
        print("logical [a1,a2,a3,a4,d]:", np.round(logical, 8))
        print("target pulses:", pulses, "delta:", np.asarray(pulses) - np.asarray(base))
        print("pulse roundtrip logical:", np.round(recovered_logical, 8))
        print("FK roundtrip tip:", np.round(recovered_p, 6),
              "position error m:", float(np.linalg.norm(recovered_p - goal)))
        if robot:
            robot.move_pvt_stream(target_pulses=pulses, velocities=[0.0] * 5, move_time=1000.0)
            print("Command sent. Read feedback after motion:", robot.read_positions())
    finally:
        if robot:
            robot.close()


if __name__ == "__main__":
    main()
