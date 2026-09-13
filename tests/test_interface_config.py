import pytest

from continuum_sdk.control.tip_command_filter import TipCommandFilter
from continuum_sdk.core.interface_config import (
    CartesianCommandConfig,
    InitialPositionConfig,
    load_robot_interface_config,
)
from continuum_sdk.transport.zmq_protocol import (
    build_command_message,
    build_state_message,
    parse_control_message,
)


def test_load_robot_interface_config() -> None:
    config = load_robot_interface_config("config/robot_interface.yaml")

    assert config.control_hz == 50
    assert config.omega_map == "xzy"
    assert config.command.max_speed_m_s[0] == 0.003
    assert config.command.max_speed_m_s[1] == 0.05
    assert config.command.orientation_enabled
    assert config.command.max_rotation_delta_rad == (0.45, 0.45, 0.0)
    assert config.command.smooth_alpha == 0.5
    assert config.command.ik_delta_order == ("y", "x", "z")


def test_capture_current_rejects_all_zero_feedback() -> None:
    config = InitialPositionConfig(
        mode="capture_current",
        reference_pulses=None,
        reject_all_zero_feedback=True,
        require_near_reference=False,
        tolerance_pulses=None,
    )

    with pytest.raises(RuntimeError, match="all-zero"):
        config.resolve_reference([0, 0, 0, 0, 0])


def test_configured_reference_validates_tolerance() -> None:
    config = InitialPositionConfig(
        mode="configured_reference",
        reference_pulses=(100, 200, 300, 400, 500),
        reject_all_zero_feedback=True,
        require_near_reference=True,
        tolerance_pulses=(10, 10, 10, 10, 10),
    )

    assert config.resolve_reference([105, 195, 300, 400, 500]) == [100, 200, 300, 400, 500]
    with pytest.raises(RuntimeError, match="outside"):
        config.resolve_reference([111, 200, 300, 400, 500])


def test_loader_rejects_unchecked_configured_reference(tmp_path) -> None:
    path = tmp_path / "interface.yaml"
    path.write_text(
        """
control_hz: 50
omega_map: zxy
initial_position:
  mode: configured_reference
  reference_pulses: [1, 2, 3, 4, 5]
  reject_all_zero_feedback: true
  require_near_reference: false
  tolerance_pulses: null
command:
  max_delta_m: [0.03, 0.01, 0.03]
  max_speed_m_s: [0.08, 0.003, 0.08]
  deadband_m: 0.0003
  smooth_alpha: 0.8
""",
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="require_near_reference"):
        load_robot_interface_config(path)


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
    assert first.tolist() == pytest.approx([0.0016, 0.00006, -0.0016])

    command_filter.hold()
    assert command_filter.step().tolist() == pytest.approx(first.tolist())


def test_tip_command_filter_limits_enabled_rotation() -> None:
    config = load_robot_interface_config("config/robot_interface_rotation.yaml")
    command_filter = TipCommandFilter(config.command, update_interval_s=0.02)

    command_filter.set_command(
        {
            "tip_delta_x": 0.0,
            "tip_delta_y": 0.0,
            "tip_delta_z": 0.0,
            "tip_delta_rx": 0.2,
            "tip_delta_ry": -0.1,
            "tip_delta_rz": 0.05,
        }
    )
    command_filter.step()

    assert command_filter.applied_rotation.tolist() == pytest.approx([0.006, -0.006, 0.0])


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
