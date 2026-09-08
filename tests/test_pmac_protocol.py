"""Offline failure-injection tests. No PMAC or haptic connection is made."""
from threading import RLock
from types import SimpleNamespace

import pytest

from pmac_sdk.controller import robot_api
from pmac_sdk.controller.protocol import PMACProtocol
from pmac_sdk.controller.robot_api import PMACRobotController, VisualHomingManager
from pmac_sdk.core.config_model import PMACConfig


class Mailbox:
    """Independent mailbox peer: requests consume on a subsequent status poll."""
    def __init__(self):
        self.transaction_lock = RLock()
        self.ram = {200: 0, 202: 0, 204: 0, 206: 0, 208: 0, 210: 0,
                    212: 2, 214: 1, 216: 1, 218: 3000, 220: 0}
        self.positions = [100, 200, -300, 400, 500]
        self.ram.update({10 + i * 2: -999 for i in range(5)})
        self.home = [0, 0, -123, 123456, 0]
        self.events = []
        self.frames = []
        self.fail_address = None
        self.reject = False
        self.drop_ack = False
        self.clear_without_ack = False
        self.freeze_feedback = False
        self.zero_fails = False
        self.reset_polls = 0
        self.close_count = 0

    def connect(self):
        self.events.append('connect')
        return True

    def disconnect(self):
        self.close_count += 1
        self.events.append('disconnect')

    def write_int32_array(self, address, values):
        self.events.append(('write', address, list(values)))
        if address == self.fail_address:
            return False
        self.ram.update({address + i * 2: value for i, value in enumerate(values)})
        return True

    def read_int32_array(self, address, count):
        self.events.append(('read', address, count))
        if address == 216 and self.reset_polls:
            self.reset_polls -= 1
            if not self.reset_polls:
                self.ram[216] = 1
        if address == 200:
            if self.ram[200] and not self.drop_ack:
                token = self.ram[200]
                if self.reject:
                    self.ram[204] = 1
                elif not self.clear_without_ack:
                    self.frames.append((
                        [self.ram[i * 2] for i in range(5)],
                        [self.ram[50 + i * 2] for i in range(5)], self.ram[40],
                    ))
                    self.ram[202] = token
                self.ram[200] = 0
            if self.ram[208] and not self.freeze_feedback:
                self.ram.update({10 + i * 2: value for i, value in enumerate(self.positions)})
                self.ram.update({222 + i * 2: value for i, value in enumerate(self.home)})
                self.ram[210] = self.ram[208]
                self.ram[208] = 0
        if address == 220 and self.ram[220]:
            command = self.ram[220]
            if command == 1 and self.home[0] == 0:
                self.home[0], self.home[4] = 1, 0
            elif command == 2:
                self.home[0], self.home[1] = 3, 9
            elif command == 3 and self.home[0] == 3:
                if self.zero_fails:
                    self.home[1] = 10
                else:
                    self.positions[4] = 0
                    self.home[0], self.home[4] = 0, 1
            elif command == 4:
                self.home[0], self.home[4] = 0, 0
            self.ram[220] = 0
        return [self.ram.get(address + i * 2, 0) for i in range(count)]


@pytest.fixture
def rig(monkeypatch):
    mailbox = Mailbox()
    hw = SimpleNamespace(aborts=0, starts=0)

    def reset():
        mailbox.events.append('reset')
        mailbox.reset_polls = 3
        mailbox.ram[214] = 0

    def start():
        mailbox.events.append('start')
        # Startup must initialize targets from acknowledged fresh feedback first.
        assert [mailbox.ram[i * 2] for i in range(5)] == mailbox.positions
        mailbox.ram[214] = 1
        hw.starts += 1

    def abort():
        mailbox.events.append('abort')
        hw.aborts += 1
        mailbox.ram[214] = 0

    hw.reset_with_plc4 = reset
    hw.start_prog = start
    hw.stop_pvt = abort
    monkeypatch.setattr(robot_api, 'ModbusClient32Bit', lambda *a: mailbox)
    monkeypatch.setattr(robot_api, 'PMACHardwareManager', lambda *a: hw)
    robot = PMACRobotController(PMACConfig(handshake_timeout_s=.005, poll_interval_s=.0001))
    return robot, mailbox, hw


def test_startup_waits_for_reset_then_captures_fresh_position(rig):
    robot, peer, hw = rig
    robot.safe_boot_and_home()
    assert robot.base_positions == peer.positions
    assert robot.base_positions != [-999] * 5
    assert peer.events.index('start') > peer.events.index(('write', 208, [1]))
    assert sum(event == ('read', 216, 1) for event in peer.events) == 3
    assert robot.pvt_axis5_max_step == 3000
    robot.close()
    robot.close()
    assert hw.aborts == 1


@pytest.mark.parametrize('cause', ['zero', 'version', 'stale', 'read_failure'])
def test_failed_startup_never_starts_program(rig, cause):
    robot, peer, hw = rig
    if cause == 'zero':
        peer.positions = [0] * 5
    elif cause == 'version':
        peer.ram[212] = 0
    elif cause == 'stale':
        peer.freeze_feedback = True
    else:
        original = peer.read_int32_array
        def failed_read(address, count):
            if address == 10:
                raise ConnectionError('feedback read failed')
            return original(address, count)
        peer.read_int32_array = failed_read
    with pytest.raises((RuntimeError, TimeoutError, ConnectionError)):
        robot.safe_boot_and_home()
    assert hw.starts == 0
    assert hw.aborts == 1
    assert peer.close_count == 1


@pytest.mark.parametrize('address', [0, 40, 50])
def test_partial_write_does_not_trigger_execution(rig, address):
    robot, peer, hw = rig
    peer.fail_address = address
    with pytest.raises(ConnectionError):
        robot.move_pvt_stream([10] * 5, [2] * 5, 20)
    assert not any(e[:2] == ('write', 200) for e in peer.events if isinstance(e, tuple))
    assert not peer.frames
    assert hw.aborts == 1
    with pytest.raises(RuntimeError, match='faulted'):
        robot.move_pvt_stream([10] * 5, [2] * 5, 20)


@pytest.mark.parametrize('failure', ['timeout', 'full', 'missing_ack', 'trigger_failure'])
def test_pvt_failure_is_latched_without_retry(rig, failure):
    robot, peer, hw = rig
    robot._session_active = True
    if failure == 'timeout':
        peer.drop_ack = True
    elif failure == 'full':
        peer.reject = True
    elif failure == 'missing_ack':
        peer.clear_without_ack = True
    else:
        peer.fail_address = 200
    with pytest.raises((RuntimeError, TimeoutError, ConnectionError)):
        robot.move_pvt_stream([10] * 5, [0] * 5, 20)
    triggers = [e for e in peer.events if isinstance(e, tuple) and e[:2] == ('write', 200)]
    assert len(triggers) == 1
    assert hw.aborts == 1
    assert robot._stream_faulted
    robot.close()
    assert hw.aborts == 1


def test_busy_mailbox_is_not_overwritten(rig):
    robot, peer, hw = rig
    peer.ram[200] = 7
    peer.drop_ack = True
    with pytest.raises(TimeoutError):
        robot.move_pvt_stream([10] * 5, [0] * 5, 20)
    assert not [e for e in peer.events if isinstance(e, tuple) and e[0] == 'write']


def test_ack_wrap_and_scaled_signed_payload(rig):
    robot, peer, hw = rig
    peer.ram[202] = 255
    robot.move_pvt_stream([1, -2, 3, -4, 5], [.25, -.5, 0, 1, -2], 20)
    assert peer.ram[202] == 1
    assert peer.frames == [([1, -2, 3, -4, 5], [2500, -5000, 0, 10000, -20000], 200000)]
    assert hw.aborts == 0


@pytest.mark.parametrize('positions,velocities,duration', [
    ([1] * 4, [0] * 5, 20),
    ([1] * 5, [0] * 4, 20),
    ([float('nan')] * 5, [0] * 5, 20),
    ([1] * 5, [float('inf')] * 5, 20),
    ([2**31] * 5, [0] * 5, 20),
    ([1] * 5, [2**31 / 10000] * 5, 20),
    ([1] * 5, [0] * 5, 5),
    ([1] * 5, [0] * 5, 501),
])
def test_invalid_pvt_rejected_before_any_io(rig, positions, velocities, duration):
    robot, peer, hw = rig
    with pytest.raises(ValueError):
        robot.move_pvt_stream(positions, velocities, duration)
    assert peer.events == []


def test_homing_stop_confirm_uses_fresh_32bit_status(rig):
    robot, peer, _ = rig
    homer = VisualHomingManager(peer)
    homer.start_homing()
    assert homer.read_status() == (1, 0, -123, 123456)
    homer.stop_movement()
    assert homer.read_status()[0] == 3
    homer.confirm_and_set_zero()
    assert peer.positions[4] == 0
    assert peer.home[4] == 1
    reads = [e[1:] for e in peer.events if isinstance(e, tuple) and e[0] == 'read']
    assert (222, 5) in reads
    assert (444, 2) not in reads


def test_homing_cancel_never_sets_zero(rig):
    _, peer, _ = rig
    homer = VisualHomingManager(peer)
    homer.start_homing()
    homer.cancel_homing()
    assert homer.read_status()[0] == 0
    assert peer.positions[4] == 500
    assert peer.home[4] == 0


def test_failed_homing_zero_is_not_reported_as_success(rig):
    _, peer, _ = rig
    peer.zero_fails = True
    homer = VisualHomingManager(peer)
    homer.start_homing()
    homer.stop_movement()
    with pytest.raises(RuntimeError, match='confirm encoder zero'):
        homer.confirm_and_set_zero()


def test_feedback_snapshot_does_not_reuse_a_stale_ack():
    peer = Mailbox()
    peer.ram[210] = 255
    protocol = PMACProtocol(peer, .005, .0001)
    positions, _ = protocol.snapshot()
    assert positions == peer.positions
    assert peer.ram[210] == 1
    peer.freeze_feedback = True
    with pytest.raises(TimeoutError):
        protocol.snapshot()
