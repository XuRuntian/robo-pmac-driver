import math

import numpy as np

from continuum_sdk.control.tendon_mapper import ContinuumTendonMapper
from continuum_sdk.control.axis_mapper import ContinuumAxisMapper
from continuum_sdk.control.pvt_mapper import ContinuumPVTMapper
from continuum_sdk.control.tip_command_filter import TipCommandFilter
from continuum_sdk.core.factory import build_continuum_ik, build_tendon_mapper
from continuum_sdk.core.config_loader import load_continuum_config
from continuum_sdk.core.interface_config import CartesianCommandConfig
from continuum_sdk.kinematics.dls_ik import rotvec_to_matrix
from continuum_sdk.kinematics.joint_motor_model import JointSpace, TDRCJointMotorModel, angle_diff
from continuum_sdk.transport.zmq_protocol import (
    build_command_message,
    build_state_message,
    parse_control_message,
)


R_HOLE = 0.003
D_SPOOL = 0.012


def assert_close(a: float, b: float, tol: float = 1e-10) -> None:
    assert math.isclose(a, b, abs_tol=tol, rel_tol=0.0)


def assert_angle_close(a: float, b: float, tol: float = 1e-9) -> None:
    assert abs(angle_diff(a, b)) <= tol


def test_joint_to_motor_matches_tendon_formula() -> None:
    model = TDRCJointMotorModel(hole_radius=R_HOLE, spool_diameter=D_SPOOL)
    joint = JointSpace(phi_a=0.3, theta_a=0.8, phi_c=-0.4, theta_c=0.5)

    tendon = model.joint_to_tendon_lengths(joint)
    motor_from_tendon = model.tendon_lengths_to_motor_angles(tendon)
    motor_direct = model.joint_to_motor_angles(joint)

    for a, b in zip(motor_from_tendon.as_tuple(), motor_direct.as_tuple()):
        assert_close(a, b)


def test_motor_to_joint_roundtrip() -> None:
    model = TDRCJointMotorModel(hole_radius=R_HOLE, spool_diameter=D_SPOOL)
    joint = JointSpace(phi_a=-0.7, theta_a=1.2, phi_c=0.9, theta_c=0.6)

    motor = model.joint_to_motor_angles(joint)
    recovered = model.motor_angles_to_joint(motor)

    assert_close(recovered.theta_a, joint.theta_a, tol=1e-9)
    assert_close(recovered.theta_c, joint.theta_c, tol=1e-9)
    assert_angle_close(recovered.phi_a, joint.phi_a)
    assert_angle_close(recovered.phi_c, joint.phi_c)


def test_tendon_mapper_outputs_motor_angles_with_distal_coupling() -> None:
    mapper = ContinuumTendonMapper(hole_radius=R_HOLE, spool_diameter=D_SPOOL)

    d = 0.01
    theta_a = 0.8
    phi_a = 0.3
    theta_c = 0.5
    phi_c = -0.4
    out = mapper.to_axis_targets(np.array([d, theta_a, phi_a, theta_c, phi_c]))

    K = 2.0 * R_HOLE / D_SPOOL
    expected_alpha3 = -K * (
        theta_a * math.cos(math.pi / 4.0 - phi_a)
        + theta_c * math.cos(math.pi / 4.0 - phi_c)
    )
    expected_alpha4 = -K * (
        theta_a * math.cos(3.0 * math.pi / 4.0 - phi_a)
        + theta_c * math.cos(3.0 * math.pi / 4.0 - phi_c)
    )

    assert_close(out[0], -K * theta_a * math.cos(phi_a))
    assert_close(out[1], -K * theta_a * math.sin(phi_a))
    assert_close(out[2], expected_alpha3)
    assert_close(out[3], expected_alpha4)
    assert_close(out[4], d)


def test_factory_uses_configured_actuation_parameters() -> None:
    cfg = load_continuum_config("config/continuum.yaml")

    ik = build_continuum_ik(cfg)
    mapper = build_tendon_mapper(cfg)

    assert ik.geometry.s_a == cfg.geometry.s_a
    assert_close(mapper.model.r_hole, cfg.actuation.hole_radius_m)
    assert_close(mapper.model.d_spool, cfg.actuation.spool_diameter_m)


def test_pvt_mapper_builds_five_axis_command() -> None:
    cfg = load_continuum_config("config/continuum.yaml")
    ik = build_continuum_ik(cfg)
    tendon_mapper = build_tendon_mapper(cfg)
    axis_mapper = ContinuumAxisMapper(
        pulses_per_rad=1000.0,
        pulses_per_meter=100000.0,
        axis_order=[0, 1, 2, 3, 4],
        axis_signs=[1, 1, 1, 1, 1],
    )
    pvt_mapper = ContinuumPVTMapper(
        ik=ik,
        tendon_mapper=tendon_mapper,
        axis_mapper=axis_mapper,
        base_pulses=[0, 0, 0, 0, 0],
        update_interval_s=0.02,
        max_inner_steps=cfg.ik.max_inner_steps,
    )

    center_p, _ = ik.fk_tip()
    command = pvt_mapper.build_command(center_p + np.array([0.001, 0.0, 0.0]))

    assert len(command.axis_targets) == 5
    assert len(command.target_pulses) == 5
    assert len(command.velocities) == 5
    assert all(isinstance(value, int) for value in command.target_pulses)


def test_pos_z_ik_responds_to_tip_direction_command() -> None:
    cfg = load_continuum_config("config/continuum.yaml")
    ik = build_continuum_ik(cfg)
    ik.task_mode = "pos_z"
    center_p, center_r = ik.fk_tip()
    goal_r = center_r @ rotvec_to_matrix(np.array([0.05, 0.0, 0.0]))

    result = ik.solve(
        p_goal=center_p,
        z_goal=goal_r[:, 2],
        max_steps=20,
    )

    assert np.linalg.norm(result.u) > 0.0
    assert result.error.shape == (6,)


def test_axis_mapper_feedback_roundtrip() -> None:
    mapper = ContinuumAxisMapper(
        pulses_per_rad=1000.0,
        pulses_per_meter=100000.0,
        axis_order=[1, 0, 2, 3, 4],
        axis_signs=[1, -1, -1, -1, 1],
    )
    base = [100, 200, 300, 400, 500]
    logical = [0.2, -0.3, 0.4, -0.5, 0.006]

    pulses = mapper.logical_to_pulses(base, logical)
    recovered = mapper.pulses_to_logical(base, pulses)

    assert np.allclose(recovered, logical, atol=1.0 / 100000.0)


def test_tip_command_filter_limits_speed_and_holds() -> None:
    command_config = CartesianCommandConfig(
        max_delta_m=(0.03, 0.01, 0.03),
        max_speed_m_s=(0.08, 0.003, 0.08),
        orientation_enabled=False,
        max_rotation_delta_rad=(0.0, 0.0, 0.0),
        max_angular_speed_rad_s=(0.0, 0.0, 0.0),
        deadband_m=0.0003,
        smooth_alpha=1.0,
    )
    command_filter = TipCommandFilter(command_config, update_interval_s=0.02)
    command_filter.set_command(
        {
            "tip_delta_x": 0.03,
            "tip_delta_y": 0.01,
            "tip_delta_z": -0.03,
            "tip_delta_rx": 0.0,
            "tip_delta_ry": 0.0,
            "tip_delta_rz": 0.0,
        }
    )

    first = command_filter.step()
    assert np.allclose(first, [0.0016, 0.00006, -0.0016])

    command_filter.hold()
    assert np.allclose(command_filter.step(), first)


def test_tip_command_filter_rejects_disabled_rotation() -> None:
    cfg = load_continuum_config("config/continuum.yaml")
    command_config = CartesianCommandConfig(
        max_delta_m=(0.03, 0.01, 0.03),
        max_speed_m_s=(0.08, 0.003, 0.08),
        orientation_enabled=False,
        max_rotation_delta_rad=(0.0, 0.0, 0.0),
        max_angular_speed_rad_s=(0.0, 0.0, 0.0),
        deadband_m=0.0003,
        smooth_alpha=0.8,
    )
    command_filter = TipCommandFilter(
        command_config,
        update_interval_s=1.0 / cfg.control.update_hz,
    )

    try:
        command_filter.set_command(
            {
                "tip_delta_x": 0.0,
                "tip_delta_y": 0.0,
                "tip_delta_z": 0.0,
                "tip_delta_rx": 0.1,
                "tip_delta_ry": 0.0,
                "tip_delta_rz": 0.0,
            }
        )
    except ValueError as exc:
        assert "rotation" in str(exc)
    else:
        raise AssertionError("A disabled rotation command must be rejected.")


def test_tip_command_filter_limits_rotation_speed_and_holds() -> None:
    command_config = CartesianCommandConfig(
        max_delta_m=(0.03, 0.01, 0.03),
        max_speed_m_s=(0.08, 0.003, 0.08),
        orientation_enabled=True,
        max_rotation_delta_rad=(0.15, 0.15, 0.0),
        max_angular_speed_rad_s=(0.3, 0.3, 0.0),
        deadband_m=0.0003,
        smooth_alpha=1.0,
    )
    command_filter = TipCommandFilter(command_config, update_interval_s=0.02)
    command_filter.set_command(
        {
            "tip_delta_x": 0.0,
            "tip_delta_y": 0.0,
            "tip_delta_z": 0.0,
            "tip_delta_rx": 0.3,
            "tip_delta_ry": -0.1,
            "tip_delta_rz": 0.05,
        }
    )

    command_filter.step()
    first_rotation = command_filter.applied_rotation
    assert np.allclose(first_rotation, [0.006, -0.006, 0.0])
    command_filter.hold()
    command_filter.step()
    assert np.allclose(command_filter.applied_rotation, first_rotation)


def test_zmq_protocol_roundtrip() -> None:
    action = {
        "tip_delta_x": 0.001,
        "tip_delta_y": 0.002,
        "tip_delta_z": 0.003,
        "tip_delta_rx": 0.0,
        "tip_delta_ry": 0.0,
        "tip_delta_rz": 0.0,
    }
    kind, parsed = parse_control_message(build_command_message(3, action))
    assert kind == "command"
    assert parsed == action

    state_message = build_state_message(
        4,
        {
            "axis_1_pos": 0.1,
            "axis_2_pos": 0.2,
            "axis_3_pos": 0.3,
            "axis_4_pos": 0.4,
            "axis_5_pos": 0.005,
        },
        status={"watchdog_holding": False},
        applied_action=action,
    )
    assert state_message["state"]["axis_5_pos"] == 0.005
