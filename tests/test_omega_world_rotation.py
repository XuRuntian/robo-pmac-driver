"""Offline regression tests: load math modules without the device/plugin entry point."""
import ast
import importlib
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[1]
PLUGIN = ROOT / "integrations/lerobot_teleoperator_omega_continuum/src/lerobot_teleoperator_omega_continuum"


@pytest.fixture
def math_modules():
    # A private package path bypasses the plugin __init__ and all hardware imports.
    name = "_omega_world_math_test"
    package = ModuleType(name)
    package.__path__ = [str(PLUGIN)]
    sys.modules[name] = package
    try:
        mapping = importlib.import_module(f"{name}.mapping")
        config = importlib.import_module(f"{name}.omega_config")
        yield mapping, config.load_omega_teleop_config(ROOT / "config/omega_teleop.yaml")
    finally:
        for key in list(sys.modules):
            if key == name or key.startswith(name + "."):
                del sys.modules[key]


def rotation(axis, degrees):
    angle = np.deg2rad(degrees)
    c, s = np.cos(angle), np.sin(angle)
    result = np.eye(3)
    i, j = (axis + 1) % 3, (axis + 2) % 3
    result[i, i] = result[j, j] = c
    result[i, j], result[j, i] = -s, s
    return result


def make_mapper(mapping, config):
    return mapping.OmegaContinuumMapper(
        translation_config=config.translation,
        rotation_config=config.rotation,
        max_delta_xyz=(0.03, 0.01, 0.03),
        deadband_m=0.0003,
        max_rotation_xyz=(0.45, 0.0, 0.45),
        rotation_deadband_rad=0.01,
    )


@pytest.mark.parametrize("axis", [0, 1, 2])
def test_world_axis_rotation(math_modules, axis):
    # Retained SO(3) utility test, no longer the active teleoperation strategy.
    mapping, _ = math_modules
    r0 = rotation(2, 30) @ rotation(0, 20)
    r = rotation(axis, 10) @ r0  # Fixed WORLD axis: left multiplication.
    delta = r @ r0.T
    np.testing.assert_allclose(delta.T @ delta, np.eye(3), atol=1e-8, rtol=0)
    assert np.linalg.det(delta) == pytest.approx(1.0, abs=1e-8)
    phi = mapping._matrix_to_rotvec(delta)
    np.testing.assert_allclose(phi, np.eye(3)[axis] * np.deg2rad(10), atol=1e-8, rtol=0)


def test_world_vs_body_regression(math_modules):
    mapping, _ = math_modules
    r0 = rotation(2, 30) @ rotation(0, 20)
    r = rotation(1, 10) @ r0
    old = mapping._matrix_to_rotvec(r0.T @ r)
    new = mapping._matrix_to_rotvec(r @ r0.T)
    expected = np.deg2rad([0.0, 10.0, 0.0])
    np.testing.assert_allclose(new, expected, atol=1e-8, rtol=0)
    np.testing.assert_allclose(old, r0.T @ expected, atol=1e-8, rtol=0)
    assert not np.allclose(old, expected, atol=1e-8, rtol=0)
    print(f"WORLD +Y regression: old_deg={np.rad2deg(old).round(8)}, "
          f"new_deg={np.rad2deg(new).round(8)}")


@pytest.mark.parametrize("q, expected", [
    ([0.4, -0.4, 0.7], [0, 0, 0]),
    ([0.3, -0.3, 0.7], [0.1, 0, 0]),
    ([0.3, -0.4, 0.8], [0, 0, 0.1]),
])
def test_wrist_joint_mapping(math_modules, monkeypatch, q, expected):
    mapping, config = math_modules
    mapper = make_mapper(mapping, config)
    q0 = np.array([0.3, -0.4, 0.7])
    mapper.set_zero(np.zeros(3), rotation(0, 20), q0)
    # Any accidental return to matrix-based rotation must fail this test.
    def forbidden(*args):
        raise AssertionError("Joint control must not use SO(3) log")
    monkeypatch.setattr(mapping, "_matrix_to_rotvec", forbidden)
    for frame in (np.eye(3), rotation(1, 70)):
        action = mapper.map_pose(np.zeros(3), frame, q)
        cmd = [action[f"tip_delta_r{axis}"] for axis in "xyz"]
        np.testing.assert_allclose(cmd, expected, atol=1e-8, rtol=0)
    dq = mapper.omega_joint_delta(q)
    np.testing.assert_allclose(dq, np.asarray(q) - q0, atol=1e-8, rtol=0)
    print(f"q={q} dq={dq.round(8)} cmd={np.round(cmd, 8)} rad")


def test_joint_rezero_and_translation_offset(math_modules):
    mapping, config = math_modules
    mapper = make_mapper(mapping, config)
    mapper.position_offset = np.array([0.001, 0.002, 0.003])
    q = np.array([0.3, -0.4, 0.7])
    mapper.set_zero(np.zeros(3), np.eye(3), q)
    q[1] += 0.1
    assert mapper.omega_joint_delta(q)[1] == pytest.approx(0.1)
    frame = rotation(2, 30)
    mapper.set_zero(np.ones(3) * .001, frame, q)
    np.testing.assert_allclose(mapper.omega_joint_delta(q), 0, atol=1e-8)
    action = mapper.map_pose(np.ones(3) * .002, frame, q)
    np.testing.assert_allclose([action[f"tip_delta_{a}"] for a in "xyz"], [.0005, -.0005, .0005])
    np.testing.assert_allclose(mapper.control_position(np.zeros(3), frame), frame @ mapper.position_offset)
    q[2] += 0.1
    assert mapper.map_pose(np.ones(3) * .001, frame, q)["tip_delta_rz"] == pytest.approx(.1)


def test_joint_read_startup_and_clutch_with_fake_sdk(math_modules):
    mapping, config = math_modules
    # Execute only the class definition, never plugin/SDK imports or __init__.
    source = ast.parse((PLUGIN / "omega_continuum.py").read_text(encoding="utf-8"))
    cls = next(node for node in source.body if isinstance(node, ast.ClassDef))
    env = dict(np=np, Teleoperator=object, OmegaContinuumConfig=object,
               RobotAction=dict, Any=object, DeviceNotConnectedError=RuntimeError,
               ACTION_FIELDS=mapping.ACTION_FIELDS)
    exec(compile(ast.Module(body=[cls], type_ignores=[]), "omega_class_only", "exec"), env)
    omega = env["OmegaContinuum"].__new__(env["OmegaContinuum"])
    omega._mapper = make_mapper(mapping, config)
    omega.config = SimpleNamespace(
        simulate=False, zero_samples=2, zero_sample_period_s=0, clutch_enabled=True,
        max_delta_x=.03, max_delta_y=.01, max_delta_z=.03,
        max_rotation_x=.45, max_rotation_y=0, max_rotation_z=.45,
    )
    samples = iter([[.2, -.4, .7], [.4, -.4, .7]])
    current = np.array([.3, -.4, .7])
    def get_angles(out):
        out[:] = next(samples, current)
        return 0
    omega._joints = np.zeros(3)
    omega._dhd = SimpleNamespace(close=lambda: None, enableForce=lambda value: None,
                                 getOrientationRad=get_angles)
    omega._drd = SimpleNamespace(open=lambda: 0, isInitialized=lambda: True,
                                 stop=lambda value: None)
    omega._is_connected = False
    omega._reset_clutch_state = lambda: None
    omega._start_clutch_listener = lambda: None
    omega._read_pose = lambda: (np.zeros(3), np.eye(3))
    omega.connect()  # Fake SDK objects only.
    np.testing.assert_allclose(omega._mapper._zero_joints, [.3, -.4, .7])
    read = omega._read_joint_angles()
    read[:] = 99
    np.testing.assert_allclose(omega._joints, current)  # Returned snapshot is a copy.
    pressed = False
    omega._is_clutch_pressed = lambda: pressed
    omega._clutch_active = False
    omega._action_anchor = np.zeros(6)
    omega._last_action = np.zeros(6)
    omega._maybe_print_axis_debug = lambda *args: None
    current[1] += .1
    assert omega.get_action()["tip_delta_rx"] == pytest.approx(.1)
    pressed = True
    current[1] += .2
    assert omega.get_action()["tip_delta_rx"] == pytest.approx(.1)
    np.testing.assert_allclose(omega._mapper.omega_joint_delta(current), 0)
    pressed = False
    assert omega.get_action()["tip_delta_rx"] == pytest.approx(.1)
    current[1] += .05
    assert omega.get_action()["tip_delta_rx"] == pytest.approx(.15)
    omega._dhd.getOrientationRad = lambda out: -1
    with pytest.raises(RuntimeError, match="wrist joints"):
        omega._read_joint_angles()


def test_joint_diagnostic_single_line():
    source = ast.parse((PLUGIN.parents[1] / "examples/print_actions.py").read_text(encoding="utf-8"))
    fn = next(node for node in source.body if isinstance(node, ast.FunctionDef)
              and node.name == "print_rotation_status")
    for width in (40, 80, 120):
        output = []
        env = dict(shutil=SimpleNamespace(get_terminal_size=lambda fallback: SimpleNamespace(columns=width)),
                   print=lambda text, **kwargs: output.append((text, kwargs)))
        exec(compile(ast.Module(body=[fn], type_ignores=[]), "display_only", "exec"), env)
        env["print_rotation_status"]([.3, -.4, .7], [0, .1, 0],
                                      dict(tip_delta_rx=.1, tip_delta_ry=0, tip_delta_rz=0))
        text, kwargs = output[0]
        assert text.startswith("\r\033[2K") and "\n" not in text
        assert len(text[5:]) < width and kwargs == dict(end="", flush=True)
        if width >= 80:
            assert "cmd=[+0.100,+0.000,+0.000]" in text
