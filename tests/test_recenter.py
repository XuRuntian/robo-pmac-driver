"""Recenter clears teleoperation command state without commanding motion."""
import numpy as np
import pytest

from continuum_sdk.control.stop_priority_filter import StopPriorityCommandFilter
from continuum_sdk.control.tip_command_filter import TipCommandFilter
from continuum_sdk.core.interface_config import load_robot_interface_config
from continuum_sdk.transport.zmq_protocol import (
    build_recenter_message,
    parse_control_message,
)

INTERFACE = load_robot_interface_config('config/robot_interface.yaml')
CONFIG = INTERFACE.command
# The shipped config has stop_priority off; the trial config exercises that path,
# which is the one the driver re-anchors while held.
STOP_PRIORITY = load_robot_interface_config(
    'config/robot_interface_stop_priority_trial.yaml').command
INTERVAL = .01


def action(**kwargs):
    base = dict(tip_delta_x=0., tip_delta_y=0., tip_delta_z=0.,
                tip_delta_rx=0., tip_delta_ry=0., tip_delta_rz=0.)
    base.update(kwargs)
    return base


def drive_away(filter_, steps=40, **kwargs):
    """Accumulate real applied offset, as a teleoperation session would."""
    for _ in range(steps):
        filter_.set_command(action(**kwargs))
        filter_.step()
    return filter_


# --- protocol ---

def test_recenter_message_round_trips():
    kind, parsed = parse_control_message(build_recenter_message(7))
    assert kind == 'recenter'
    assert parsed is None


def test_recenter_message_carries_no_action():
    """A reset that could smuggle an action would be a motion command."""
    assert 'action' not in build_recenter_message(1)


def test_recenter_message_keeps_the_protocol_version():
    assert build_recenter_message(3)['protocol_version'] == 1


def test_unknown_kind_is_still_rejected():
    message = build_recenter_message(1)
    message['kind'] = 'reset_everything'
    with pytest.raises(ValueError):
        parse_control_message(message)


# --- TipCommandFilter ---

def test_recenter_clears_accumulated_offset():
    f = drive_away(TipCommandFilter(CONFIG, INTERVAL), tip_delta_x=.02)
    assert np.linalg.norm(f.applied_delta) > 1e-6
    f.recenter()
    np.testing.assert_allclose(f.applied_delta, np.zeros(3))
    np.testing.assert_allclose(f.applied_rotation, np.zeros(3))


def test_recenter_leaves_nothing_for_the_next_step_to_drive():
    """The reset must not merely zero the output, it must zero the target too.

    If the target survived, the very next step() would move the robot, which is
    the opposite of a data-only reset.
    """
    f = drive_away(TipCommandFilter(CONFIG, INTERVAL), tip_delta_x=.02)
    f.recenter()
    before = f.applied_delta
    moved = f.step()
    np.testing.assert_allclose(moved, before, atol=1e-12)
    np.testing.assert_allclose(moved, np.zeros(3), atol=1e-12)


def test_many_steps_after_recenter_stay_still_without_new_commands():
    f = drive_away(TipCommandFilter(CONFIG, INTERVAL), tip_delta_x=.02)
    f.recenter()
    for _ in range(200):
        np.testing.assert_allclose(f.step(), np.zeros(3), atol=1e-12)


def test_recenter_is_idempotent():
    f = drive_away(TipCommandFilter(CONFIG, INTERVAL), tip_delta_x=.02)
    f.recenter()
    first = f.applied_delta
    f.recenter()
    np.testing.assert_allclose(f.applied_delta, first)


def test_recenter_on_a_fresh_filter_is_a_noop():
    f = TipCommandFilter(CONFIG, INTERVAL)
    f.recenter()
    np.testing.assert_allclose(f.applied_delta, np.zeros(3))


def test_filter_still_accepts_commands_after_recenter():
    """A reset that bricked the session would be useless."""
    f = drive_away(TipCommandFilter(CONFIG, INTERVAL), tip_delta_x=.02)
    f.recenter()
    drive_away(f, tip_delta_x=.01)
    assert np.linalg.norm(f.applied_delta) > 1e-6


def test_recenter_clears_the_rotation_clamp_flag():
    f = TipCommandFilter(CONFIG, INTERVAL)
    f.rotation_command_limited = True
    f.recenter()
    assert not f.rotation_command_limited


# --- StopPriorityCommandFilter ---

def stop_priority_moved():
    """A stop-priority filter with real accumulated offset.

    It ignores an initial absolute offset by design, so motion has to be built
    up by moving the input gradually.
    """
    f = StopPriorityCommandFilter(STOP_PRIORITY, .02)
    f.set_command(action())
    f.step()
    for i in range(1, 60):
        f.set_command(action(tip_delta_x=i*.0002))
        f.step()
    assert np.linalg.norm(f.applied_delta) > 1e-6, 'fixture built no motion'
    return f


def test_stop_priority_recenter_clears_offset_and_holds():
    f = stop_priority_moved()
    f.recenter()
    np.testing.assert_allclose(f.applied_delta, np.zeros(3))
    # Held, so stale operator input becomes a fresh anchor instead of a jump.
    assert f.input_holding


def test_stop_priority_does_not_jump_on_the_next_stale_input():
    """The failure this guards: resuming from a far-away input snaps the robot."""
    f = stop_priority_moved()
    f.recenter()
    f.set_command(action(tip_delta_x=.03))   # stale, far from the new origin
    np.testing.assert_allclose(f.step(), np.zeros(3), atol=1e-12)


def test_stop_priority_stays_still_while_stale_input_repeats():
    f = stop_priority_moved()
    f.recenter()
    for _ in range(50):
        f.set_command(action(tip_delta_x=.03))
        np.testing.assert_allclose(f.step(), np.zeros(3), atol=1e-12)


def test_stop_priority_resumes_from_the_new_origin():
    """After the reset, fresh incremental input must still drive motion."""
    f = stop_priority_moved()
    f.recenter()
    for i in range(1, 60):
        f.set_command(action(tip_delta_x=.03 + i*.0002))
        f.step()
    assert np.linalg.norm(f.applied_delta) > 1e-6


def test_stop_priority_recenter_resets_input_discarded():
    f = StopPriorityCommandFilter(STOP_PRIORITY, .02)
    f.input_discarded = True
    f.recenter()
    assert not f.input_discarded


def test_both_filters_expose_recenter():
    assert callable(TipCommandFilter(CONFIG, INTERVAL).recenter)
    assert callable(StopPriorityCommandFilter(STOP_PRIORITY, .02).recenter)
