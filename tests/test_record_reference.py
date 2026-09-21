import time

import pytest

from apps.record_reference import held_feedback


def message():
    return dict(protocol_version=1, kind="state", timestamp_ns=time.time_ns(),
                status=dict(execute=True, feedback_valid=True, watchdog_holding=True,
                            feedback_pulses=[1, 2, 3, 4, 5], target_pulses=[9]*5))


def test_records_actual_not_target_positions():
    assert held_feedback(message()) == [1, 2, 3, 4, 5]


@pytest.mark.parametrize("flag", ["execute", "feedback_valid", "watchdog_holding"])
def test_rejects_simulation_invalid_feedback_and_moving_state(flag):
    msg = message()
    msg["status"][flag] = False
    assert held_feedback(msg) is None


def test_rejects_stale_and_zero_feedback():
    msg = message()
    msg["timestamp_ns"] -= 10_000_000_000
    assert held_feedback(msg) is None
    msg = message()
    msg["status"]["feedback_pulses"] = [0]*5
    assert held_feedback(msg) is None
