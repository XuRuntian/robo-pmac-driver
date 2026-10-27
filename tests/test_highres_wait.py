from types import SimpleNamespace

import pytest

from pmac_sdk.core import waiting
from pmac_sdk.controller.protocol import PMACProtocol


@pytest.fixture
def kernel(monkeypatch):
    state = SimpleNamespace(closed=0, cancelled=0, due=None, result=0, fail_set=False)

    def set_timer(handle, due, *args):
        state.due = due._obj.value
        return not state.fail_set

    def wait(*args):
        if isinstance(state.result, BaseException):
            raise state.result
        return state.result

    def close(_):
        state.closed += 1
        return True

    def cancel(_):
        state.cancelled += 1
        return True

    fake = SimpleNamespace(
        CreateWaitableTimerExW=lambda *args: 123,
        SetWaitableTimer=set_timer,
        WaitForSingleObjectEx=wait,
        CloseHandle=close,
        CancelWaitableTimer=cancel,
    )
    monkeypatch.setattr(waiting, "_load_kernel", lambda: fake)
    return state


def test_relative_timer_and_idempotent_cleanup(kernel):
    timer = waiting.WindowsHighResolutionWait()
    timer.sleep(0.001)
    assert kernel.due == -10000
    timer.close()
    timer.close()
    assert kernel.closed == 1
    with pytest.raises(RuntimeError, match="closed"):
        timer.sleep(0.001)


@pytest.mark.parametrize("value", [-1, float("nan"), float("inf"), 3601])
def test_invalid_duration(kernel, value):
    timer = waiting.WindowsHighResolutionWait()
    try:
        with pytest.raises(ValueError):
            timer.sleep(value)
        assert kernel.due is None
    finally:
        timer.close()


def test_interrupt_cancels_pending_timer_and_can_close(kernel):
    timer = waiting.WindowsHighResolutionWait()
    kernel.result = KeyboardInterrupt()
    try:
        with pytest.raises(KeyboardInterrupt):
            timer.sleep(0.02)
        assert kernel.cancelled == 1
    finally:
        timer.close()
    assert kernel.closed == 1


def test_non_signaling_timer_is_bounded(kernel, monkeypatch):
    timer = waiting.WindowsHighResolutionWait()
    kernel.result = 258
    times = iter([0, 2])
    monkeypatch.setattr(waiting.time, "monotonic", lambda: next(times))
    try:
        with pytest.raises(TimeoutError):
            timer.sleep(0.001)
        assert kernel.cancelled == 1
    finally:
        timer.close()


def test_protocol_uses_injected_wait_without_changing_poll_interval():
    protocol = PMACProtocol(None)
    waits = []
    protocol.sleep_fn = waits.append
    replies = iter([False, "ack"])
    assert protocol.wait(lambda: next(replies), "test") == "ack"
    assert waits == [0.001]
