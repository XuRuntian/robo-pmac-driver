"""Verify the operator-calibrated simulation against an independent robot IK.

The two branches solve independently, so this compares the actual tip
displacement each one produces, expressed in one common frame. No hardware,
no Omega device and no LeRobot package import.
"""
import importlib.util
from pathlib import Path
import sys
import uuid

import numpy as np
import pytest

from continuum_sdk.control.cartesian_frame import apply_axis_transform
from continuum_sdk.control.reference_teleop import (
    load_reference,
    real_branch_translation_map,
    ReferenceTeleop,
)
from continuum_sdk.control.tip_command_filter import TipCommandFilter
from continuum_sdk.core.config_loader import load_continuum_config
from continuum_sdk.core.factory import build_continuum_ik
from continuum_sdk.core.interface_config import load_robot_interface_config
from continuum_sdk.core.omega_config import load_omega_teleop_config

ROOT = Path(__file__).resolve().parents[1]
REFERENCE = ROOT.parent/'surgical_continuum_robot/remote_control_ex'
PLUGIN = ROOT/'integrations/lerobot_teleoperator_omega_continuum/src/lerobot_teleoperator_omega_continuum'
INTERFACE = 'config/robot_interface_stop_priority_trial.yaml'
# Enough cycles for the driver's speed limit to reach the commanded offset.
STEPS = 400
RAMP = 50


@pytest.fixture(scope='module')
def plugin_mapper_type():
    """Load the installed plugin's mapper without importing lerobot."""
    name = '_branch_match_mapping'
    package = type(sys)('_branch_match_pkg')
    package.__path__ = [str(PLUGIN)]
    sys.modules['_branch_match_pkg'] = package
    spec = importlib.util.spec_from_file_location('_branch_match_pkg.mapping', PLUGIN/'mapping.py')
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = sys.modules['_branch_match_pkg.mapping'] = module
    try:
        spec.loader.exec_module(module)
        yield module.OmegaContinuumMapper
    finally:
        for key in ('_branch_match_pkg', '_branch_match_pkg.mapping', name):
            sys.modules.pop(key, None)


@pytest.fixture(scope='module')
def reference():
    if not (REFERENCE/'teleop_core').exists():
        pytest.skip('Reference project is not installed')
    return load_reference(REFERENCE)


def reference_to_internal(vector):
    """Reference base frame [x,y,z] -> internal IK frame [x,-z,y]."""
    return np.array([vector[0], -vector[2], vector[1]])


def robot_branch_tip(mapper_type, hand):
    """LeRobot mapper -> driver filter -> frame transform -> driver IK."""
    omega = load_omega_teleop_config(ROOT/'config/omega_teleop.yaml')
    interface = load_robot_interface_config(ROOT/INTERFACE)
    mapper = mapper_type(
        max_delta_xyz=(None, None, None), deadband_m=interface.command.deadband_m,
        translation_config=omega.translation, rotation_config=omega.rotation,
        max_rotation_xyz=(None, 0., None), rotation_deadband_rad=0.)
    mapper.set_zero(np.zeros(3), np.eye(3), np.zeros(3))
    ik = build_continuum_ik(load_continuum_config(ROOT/'config/continuum.yaml'))
    ik.task_mode = 'pos_z'
    center_p, center_r = ik.fk_tip()
    command = TipCommandFilter(interface.command, 1./interface.control_hz)
    for step in range(1, STEPS+1):
        command.set_command(mapper.map_pose(hand*min(1., step/RAMP), np.eye(3), np.zeros(3)))
        goal = center_p + apply_axis_transform(command.step(), interface.frame.translation_map,
                                               interface.frame.translation_signs)
        ik.solve(goal, z_goal=center_r[:, 2])
    return ik.fk_tip()[0] - center_p


def simulation_tip(reference, hand, **kwargs):
    omega = load_omega_teleop_config(ROOT/'config/omega_teleop.yaml')
    interface = load_robot_interface_config(ROOT/INTERFACE)
    options = dict(translation_map=real_branch_translation_map(omega, interface.frame))
    options.update(kwargs)
    shared = ReferenceTeleop(reference, np.zeros(5), swap_rotation_xz=True, **options)
    session = uuid.uuid4().hex

    def feed(sequence, position):
        shared.receive(dict(protocol_version=1, kind='omega_pose', session=session,
                            sequence=sequence, position=list(map(float, position)),
                            rotation=np.eye(3).tolist(), connected=True, clutch=False))
        shared.step()

    feed(0, np.zeros(3))
    start = shared.kinematics.fk_tip_from_u(shared.ik.u)[0].copy()
    for step in range(1, STEPS+1):
        feed(step, hand*min(1., step/RAMP))
    moved = shared.kinematics.fk_tip_from_u(shared.ik.u)[0] - start
    return reference_to_internal(moved)


DIRECTIONS = [('+Y', (0, 1, 0)), ('-Y', (0, -1, 0)), ('+Z', (0, 0, 1)), ('-Z', (0, 0, -1)),
              ('+Y+Z', (0, .7071, .7071)), ('+Y-Z', (0, .7071, -.7071))]


@pytest.mark.parametrize('label,direction', DIRECTIONS, ids=[d[0] for d in DIRECTIONS])
def test_simulation_matches_robot_ik_after_calibrated_input_basis(plugin_mapper_type, reference, label, direction):
    hand = np.asarray(direction, dtype=float)*.015
    robot = robot_branch_tip(plugin_mapper_type, hand[[0,2,1]] * [1,1,-1])
    simulation = simulation_tip(reference, hand)
    # Use a movement above IK tolerance after the configured gain.
    assert np.linalg.norm(robot) > .003, 'Expected a measurable robot-branch displacement.'
    # Both are now in the internal IK frame, so compare them directly.
    np.testing.assert_allclose(simulation, robot, atol=2e-4)


def test_calibrated_bend_plane_has_no_additional_rotation(plugin_mapper_type, reference):
    """A frame error shows up as a consistent angle offset, not as noise."""
    for _, direction in DIRECTIONS:
        hand = np.asarray(direction, dtype=float)*.015
        robot = robot_branch_tip(plugin_mapper_type, hand[[0,2,1]] * [1,1,-1])[:2]
        simulation = simulation_tip(reference, hand)[:2]
        angle = np.degrees(np.arctan2(robot[0]*simulation[1]-robot[1]*simulation[0],
                                      robot @ simulation))
        assert abs(angle) < 1., f'Bend plane differs by {angle:.2f} deg for hand {direction}'


def test_reference_default_translation_is_still_available_and_differs(reference):
    """The reference mapping stays reachable, and both known differences are real."""
    omega = load_omega_teleop_config(ROOT/'config/omega_teleop.yaml')
    hand = np.array([0., .015, 0.])
    aligned = simulation_tip(reference, hand)
    legacy = simulation_tip(reference, hand, translation_map=None)
    # The reference default keeps position_scale 1.0 instead of the configured gain.
    ratio = np.linalg.norm(legacy)/np.linalg.norm(aligned)
    assert ratio == pytest.approx(1./omega.translation.gain[1], rel=.05)
    angle = np.degrees(np.arctan2(aligned[0]*legacy[1]-aligned[1]*legacy[0],
                                  aligned[:2] @ legacy[:2]))
    assert abs(abs(angle)-90.) < 1., f'Expected the known 90 deg offset, measured {angle:.2f}'


def test_translation_map_folds_in_the_configured_gain(reference):
    omega = load_omega_teleop_config(ROOT/'config/omega_teleop.yaml')
    interface = load_robot_interface_config(ROOT/INTERFACE)
    matrix = real_branch_translation_map(omega, interface.frame)
    norms = [float(np.linalg.norm(matrix[:, i])) for i in range(3)]
    np.testing.assert_allclose(norms, omega.translation.gain, rtol=1e-12)
    rotation = matrix/norms[0]
    np.testing.assert_allclose(rotation.T @ rotation, np.eye(3), atol=1e-12)
    assert np.linalg.det(rotation) == pytest.approx(1.)


def test_translation_map_rejects_malformed_matrices(reference):
    for bad in (np.zeros((2, 3)), np.full((3, 3), np.nan)):
        with pytest.raises(ValueError, match='finite 3x3'):
            ReferenceTeleop(reference, np.zeros(5), translation_map=bad)
