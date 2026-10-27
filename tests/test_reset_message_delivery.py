"""One-shot resets must survive the command stream, and not snap back after.

Covers the two bugs found by end-to-end testing: socket conflation silently
dropped reset requests sent while pose commands were streaming, and a reset was
immediately undone by the stale offset still arriving afterwards.
"""
import importlib.util
from pathlib import Path
import sys

import pytest

from continuum_sdk.core.interface_config import load_robot_interface_config
from continuum_sdk.transport import (build_command_message, build_hold_message,
                                     build_recenter_message, build_straighten_message)

ROOT = Path(__file__).resolve().parents[1]
COMMAND = load_robot_interface_config('config/robot_interface.yaml').command


@pytest.fixture(scope='module')
def driver():
    """Load the driver module without running main() or touching hardware."""
    spec = importlib.util.spec_from_file_location(
        '_driver_reset_delivery', ROOT/'apps/continuum_driver_server.py')
    module = importlib.util.module_from_spec(spec)
    sys.modules['_driver_reset_delivery'] = module
    try:
        spec.loader.exec_module(module)
        yield module
    finally:
        sys.modules.pop('_driver_reset_delivery', None)


class FakeSocket:
    """Minimal stand-in for a PULL socket drained with NOBLOCK."""

    def __init__(self, messages):
        self.messages = list(messages)

    def recv_json(self, flags=0):
        import zmq
        if not self.messages:
            raise zmq.Again()
        return self.messages.pop(0)


def action(x=0., rz=0.):
    return dict(tip_delta_x=x, tip_delta_y=0., tip_delta_z=0.,
                tip_delta_rx=0., tip_delta_ry=0., tip_delta_rz=rz)


def kinds(drained):
    return [m['kind'] for m in drained]


# --- delivery: the conflation bug ---

def test_straighten_survives_a_flood_of_pose_commands(driver):
    """The bug: CONFLATE kept only the newest message, dropping the reset."""
    messages = [build_command_message(i, action(.02)) for i in range(50)]
    messages.insert(10, build_straighten_message(99))
    messages += [build_command_message(i, action(.02)) for i in range(50, 100)]
    assert 'straighten' in kinds(driver._drain_messages(FakeSocket(messages)))


def test_recenter_survives_a_flood_of_pose_commands(driver):
    messages = [build_command_message(i, action(.02)) for i in range(40)]
    messages.insert(20, build_recenter_message(99))
    assert 'recenter' in kinds(driver._drain_messages(FakeSocket(messages)))


def test_hold_survives_a_flood_of_pose_commands(driver):
    messages = [build_command_message(i, action(.02)) for i in range(40)]
    messages.insert(5, build_hold_message(99))
    assert 'hold' in kinds(driver._drain_messages(FakeSocket(messages)))


def test_only_the_newest_pose_command_is_kept(driver):
    messages = [build_command_message(i, action(i*.001)) for i in range(30)]
    drained = driver._drain_messages(FakeSocket(messages))
    assert kinds(drained) == ['command']
    assert drained[0]['action']['tip_delta_x'] == pytest.approx(29*.001)


def test_every_one_shot_request_is_kept(driver):
    """Two presses must not collapse into one."""
    messages = [build_straighten_message(1), build_command_message(2, action(.01)),
                build_recenter_message(3)]
    assert kinds(driver._drain_messages(FakeSocket(messages))) == [
        'straighten', 'command', 'recenter']


def test_arrival_order_is_preserved(driver):
    """A reset pressed after a pose command must be applied after it."""
    messages = [build_command_message(1, action(.01)), build_straighten_message(2)]
    assert kinds(driver._drain_messages(FakeSocket(messages))) == ['command', 'straighten']
    messages = [build_straighten_message(1), build_command_message(2, action(.01))]
    assert kinds(driver._drain_messages(FakeSocket(messages))) == ['straighten', 'command']


def test_empty_socket_drains_to_nothing(driver):
    assert driver._drain_messages(FakeSocket([])) == []


def test_the_driver_does_not_conflate_the_command_socket(driver):
    """Conflation at the socket layer would drop resets before we see them."""
    source = (ROOT/'apps/continuum_driver_server.py').read_text(encoding='utf-8')
    command_section = source[source.index('command_socket = context.socket'):
                             source.index('state_socket = context.socket')]
    # Match the actual setsockopt call, not the comment explaining its absence.
    assert 'command_socket.setsockopt(zmq.CONFLATE' not in command_section


# --- the snap-back bug ---

def test_unmoved_input_is_gated_after_a_reset(driver):
    """The bug: the stale offset drove the tip straight back after the reset."""
    gate = action(.02, .3)
    assert driver._within_reset_gate(action(.02, .3), gate, COMMAND)


def test_gate_releases_once_translation_moves(driver):
    gate = action(.02, .3)
    moved = action(.02 + 10*COMMAND.input_translation_epsilon_m, .3)
    assert not driver._within_reset_gate(moved, gate, COMMAND)


def test_gate_releases_once_rotation_moves(driver):
    gate = action(.02, .3)
    moved = action(.02, .3 + 10*COMMAND.input_rotation_epsilon_rad)
    assert not driver._within_reset_gate(moved, gate, COMMAND)


def test_gate_tolerates_noise_below_the_configured_threshold(driver):
    """Sensor noise must not count as operator motion and release the gate."""
    gate = action(.02, .3)
    noisy = action(.02 + COMMAND.input_translation_epsilon_m*.5, .3)
    assert driver._within_reset_gate(noisy, gate, COMMAND)


def test_gate_is_inactive_without_an_action(driver):
    assert not driver._within_reset_gate(None, action(.02), COMMAND)


def test_gate_checks_every_axis(driver):
    gate = action()
    for key in ('tip_delta_x', 'tip_delta_y', 'tip_delta_z'):
        candidate = action()
        candidate[key] = 10*COMMAND.input_translation_epsilon_m
        assert not driver._within_reset_gate(candidate, gate, COMMAND), key
    for key in ('tip_delta_rx', 'tip_delta_ry', 'tip_delta_rz'):
        candidate = action()
        candidate[key] = 10*COMMAND.input_rotation_epsilon_rad
        assert not driver._within_reset_gate(candidate, gate, COMMAND), key
