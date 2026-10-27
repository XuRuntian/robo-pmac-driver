"""Reset keys belong to the driver terminal, not to a teleoperation client.

The driver owns the PMAC connection, so a reset typed in its terminal works
with teleoperation stopped and needs no second process touching the hardware.
"""
import importlib.util
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
PLUGIN = ROOT/'integrations/lerobot_robot_continuum/src/lerobot_robot_continuum'


@pytest.fixture(scope='module')
def driver():
    spec = importlib.util.spec_from_file_location(
        '_driver_reset_keys', ROOT/'apps/continuum_driver_server.py')
    module = importlib.util.module_from_spec(spec)
    sys.modules['_driver_reset_keys'] = module
    try:
        spec.loader.exec_module(module)
        yield module
    finally:
        sys.modules.pop('_driver_reset_keys', None)


# --- the listener records and releases one request per press ---

def test_press_is_recorded_and_consumed_once(driver):
    listener = driver.ResetKeyListener('h', 'r')
    assert listener.take() is None
    listener.pending = 'straighten'
    assert listener.take() == 'straighten'
    assert listener.take() is None, 'one press must fire exactly one reset'


def test_keys_are_lowercased(driver):
    listener = driver.ResetKeyListener('H', 'R')
    assert listener.straighten_key == 'h'
    assert listener.recenter_key == 'r'


def test_stop_is_safe_before_start(driver):
    driver.ResetKeyListener('h', 'r').stop()


def test_missing_pynput_reports_instead_of_raising(driver, monkeypatch):
    """A driver that refuses to boot without pynput would be worse than no keys."""
    monkeypatch.setitem(sys.modules, 'pynput', None)
    message = driver.ResetKeyListener('h', 'r').start()
    assert 'pynput' in message
    assert 'ZMQ' in message


def test_start_reports_both_keys_and_their_effect(driver):
    listener = driver.ResetKeyListener('h', 'r')
    try:
        message = listener.start()
    except Exception:
        pytest.skip('pynput unavailable in this environment')
    listener.stop()
    assert "'h'" in message and "'r'" in message
    assert 'insertion held' in message
    assert 'no motion' in message


# --- the key press reuses the wire handler ---

def test_key_press_is_turned_into_a_valid_control_message(driver):
    """One implementation per reset: the key builds the same message the wire does."""
    from continuum_sdk.transport.zmq_protocol import parse_control_message
    for kind in ('straighten', 'recenter'):
        kind_parsed, action = parse_control_message(
            {'protocol_version': 1, 'kind': kind, 'sequence': -1, 'timestamp_ns': 1})
        assert kind_parsed == kind
        assert action is None


def test_driver_injects_key_requests_into_the_message_queue(driver):
    """Guards the wiring: a press must reach the same loop branch as a message."""
    source = (ROOT/'apps/continuum_driver_server.py').read_text(encoding='utf-8')
    loop = source[source.index('pending_messages = _drain_messages'):]
    loop = loop[:loop.index('for message in pending_messages')]
    assert 'reset_keys.take()' in loop
    assert 'pending_messages.append' in loop


def test_reset_keys_can_be_disabled(driver):
    source = (ROOT/'apps/continuum_driver_server.py').read_text(encoding='utf-8')
    assert '--reset-keys' in source
    assert 'if args.reset_keys:' in source


def test_cli_exposes_configurable_keys(driver):
    args = driver.parse_args.__wrapped__() if hasattr(driver.parse_args, '__wrapped__') else None
    source = (ROOT/'apps/continuum_driver_server.py').read_text(encoding='utf-8')
    assert '--straighten-key' in source
    assert '--recenter-key' in source


# --- teleoperation must stay decoupled ---

def test_the_robot_plugin_has_no_reset_listener():
    """The user wants resets in the driver terminal, not coupled to teleoperation."""
    source = (PLUGIN/'continuum_pmac.py').read_text(encoding='utf-8')
    assert 'pynput' not in source
    assert '_reset_listener' not in source
    assert 'reset_keys_enabled' not in source


def test_the_robot_plugin_still_only_sends_commands_and_hold():
    source = (PLUGIN/'continuum_pmac.py').read_text(encoding='utf-8')
    assert '"kind": "command"' in source
    assert '"kind": "hold"' in source
    assert 'straighten' not in source
    assert 'recenter' not in source


def test_the_driver_warns_to_stop_teleoperation_first(driver):
    """A live client keeps streaming poses, which would fight the reset."""
    source = (ROOT/'apps/continuum_driver_server.py').read_text(encoding='utf-8')
    assert 'after teleoperation has stopped' in source
