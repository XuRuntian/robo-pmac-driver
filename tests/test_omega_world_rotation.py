"""Offline regression tests: load math modules without the device/plugin entry point."""
import importlib
import sys
from pathlib import Path
from types import ModuleType

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


@pytest.mark.parametrize("axis, expected_robot_deg", [
    (0, [0.0, 0.0, 0.0]),
    (1, [10.0, 0.0, 0.0]),
    (2, [0.0, 0.0, 10.0]),
])
def test_world_axis_rotation(math_modules, axis, expected_robot_deg):
    mapping, config = math_modules
    mapper = make_mapper(mapping, config)
    r0 = rotation(2, 30) @ rotation(0, 20)
    r = rotation(axis, 10) @ r0  # Fixed WORLD axis: left multiplication.
    mapper.set_zero(np.zeros(3), r0)
    delta = r @ r0.T
    np.testing.assert_allclose(delta.T @ delta, np.eye(3), atol=1e-8, rtol=0)
    assert np.linalg.det(delta) == pytest.approx(1.0, abs=1e-8)
    phi = mapper.omega_rotation_delta(r)
    np.testing.assert_allclose(phi, np.eye(3)[axis] * np.deg2rad(10), atol=1e-8, rtol=0)
    action = mapper.map_pose(np.zeros(3), r)
    robot = np.array([action[f"tip_delta_r{a}"] for a in "xyz"])
    np.testing.assert_allclose(robot, np.deg2rad(expected_robot_deg), atol=1e-8, rtol=0)
    print(f"WORLD +{'XYZ'[axis]}: phi_deg={np.rad2deg(phi).round(8)}, "
          f"robot_deg={np.rad2deg(robot).round(8)}")


def test_world_vs_body_regression(math_modules):
    mapping, config = math_modules
    mapper = make_mapper(mapping, config)
    r0 = rotation(2, 30) @ rotation(0, 20)
    r = rotation(1, 10) @ r0
    mapper.set_zero(np.zeros(3), r0)
    old = mapping._matrix_to_rotvec(r0.T @ r)
    new = mapper.omega_rotation_delta(r)
    expected = np.deg2rad([0.0, 10.0, 0.0])
    np.testing.assert_allclose(new, expected, atol=1e-8, rtol=0)
    np.testing.assert_allclose(old, r0.T @ expected, atol=1e-8, rtol=0)
    assert not np.allclose(old, expected, atol=1e-8, rtol=0)
    print(f"WORLD +Y regression: old_deg={np.rad2deg(old).round(8)}, "
          f"new_deg={np.rad2deg(new).round(8)}")
