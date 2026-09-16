import importlib.util
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
from types import SimpleNamespace

import numpy as np
import pytest
import zmq

from continuum_sdk.control.axis_mapper import ContinuumAxisMapper
from continuum_sdk.control.pvt_mapper import ContinuumPVTMapper
from continuum_sdk.core.config_loader import load_continuum_config
from continuum_sdk.core.factory import build_continuum_ik, build_tendon_mapper
from continuum_sdk.kinematics.joint_motor_model import JointSpace
from pmac_sdk.core.config_model import PMACConfig

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('driver_server', ROOT / 'apps/continuum_driver_server.py')
server = importlib.util.module_from_spec(spec)
spec.loader.exec_module(server)


def test_mapper_commits_clipped_command_as_next_ik_seed():
    cfg = load_continuum_config(ROOT / 'config/continuum.yaml')
    pmac = PMACConfig()
    ik = build_continuum_ik(cfg)
    tendon = build_tendon_mapper(cfg)
    axes = ContinuumAxisMapper(pmac.pulses_per_rad, pmac.pulses_per_meter, pmac.axis_order, pmac.axis_signs)
    mapper = ContinuumPVTMapper(ik, tendon, axes, [0] * 5, .02, 8)
    requested = np.array([.02, .8, .3, .4, -.2])
    ik.reset(requested)
    pulses = axes.logical_to_pulses([0] * 5, tendon.to_axis_targets(requested))
    pulses[pmac.axis_order[4]] = 3000
    mapper.commit_pulses(pulses)
    assert abs(ik.u[0] - 3000 / pmac.pulses_per_meter) < 1e-12
    assert abs(ik.u[1] - requested[1]) < 2e-6
    assert np.allclose(mapper._prev_axis_targets, axes.pulses_to_logical([0] * 5, pulses))


@pytest.fixture
def fake_clock(monkeypatch):
    clock = SimpleNamespace(now=0.)
    monkeypatch.setattr(server.time, 'monotonic', lambda: clock.now)
    monkeypatch.setattr(server.time, 'perf_counter', lambda: clock.now)
    monkeypatch.setattr(server.time, 'sleep', lambda delay: setattr(clock, 'now', clock.now + delay))
    return clock


def test_startup_return_does_not_replace_zero_feedback_with_reference(fake_clock):
    robot = SimpleNamespace(
        pvt_axis5_max_step=3000,
        move_pvt_stream=lambda **kwargs: None,
        read_positions=lambda: [0] * 5,
    )
    with pytest.raises(RuntimeError, match='all-zero'):
        server._return_to_reference_pvt(robot, start_pulses=[10]*5, reference_pulses=[20]*5,
                                        update_hz=50, duration_s=.2, check_tolerance_pulses=1)


def test_startup_return_waits_for_feedback_not_just_enqueue(fake_clock):
    sent = []
    robot = SimpleNamespace(
        pvt_axis5_max_step=3000,
        move_pvt_stream=lambda **kwargs: sent.append(kwargs),
        read_positions=lambda: [20]*5 if fake_clock.now >= 1 else [10]*5,
    )
    result = server._return_to_reference_pvt(robot, start_pulses=[10]*5, reference_pulses=[20]*5,
                                           update_hz=50, duration_s=.2, check_tolerance_pulses=1)
    assert result == [20]*5
    assert fake_clock.now >= 1
    assert len(sent) > 13
    assert sent[-1]['velocities'] == [0]*5


def test_return_ramp_rejects_excessive_axis5_step_before_motion():
    sent = []
    robot = SimpleNamespace(pvt_axis5_max_step=3000, move_pvt_stream=lambda **kw: sent.append(kw))
    with pytest.raises(ValueError, match='increase --return-duration'):
        server._return_to_reference_pvt(robot, start_pulses=[0]*5,
                                        reference_pulses=[0,0,0,0,1000000],
                                        update_hz=50, duration_s=.1, check_tolerance_pulses=1)
    assert not sent


def test_reference_rejection_during_startup_closes_robot(monkeypatch):
    args = SimpleNamespace(
        watchdog_timeout=.2, feedback_hz=10, return_duration=8, return_check_tolerance_pulses=0,
        shape_debug_hz=0, motor_debug_hz=0, motor_debug_min_tip_mm=.5,
        motor_debug_min_rotation_rad=.01, motor_debug_change_pulses=1000,
        config=str(ROOT/'config/continuum.yaml'), interface_config=str(ROOT/'config/robot_interface.yaml'),
        pmac_ip='unused', execute=True, return_to_reference_on_start=False,
    )
    robot = SimpleNamespace(safe_boot_and_home=lambda: None, base_positions=[0]*5)
    closed = []
    robot.close = lambda: closed.append(True)
    monkeypatch.setattr(server, 'parse_args', lambda: args)
    monkeypatch.setattr(server, 'PMACRobotController', lambda _: robot)
    with pytest.raises(RuntimeError, match='all-zero'):
        server.main()
    assert closed == [True]


def _free_port():
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]


@pytest.mark.parametrize('explicit_hold', [False, True])
def test_dry_run_holds_motor_targets_and_exits_on_sigterm(explicit_hold):
    cmd_port, state_port = _free_port(), _free_port()
    env = os.environ.copy()
    env['PYTHONPATH'] = str(ROOT / 'src')
    proc = subprocess.Popen([
        sys.executable, '-u', str(ROOT/'apps/continuum_driver_server.py'),
        '--command-port', str(cmd_port), '--state-port', str(state_port),
        '--watchdog-timeout', '.1',
    ], cwd=ROOT, env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    context = zmq.Context()
    command = context.socket(zmq.PUSH)
    command.setsockopt(zmq.LINGER, 0)
    command.connect(f'tcp://127.0.0.1:{cmd_port}')
    state = context.socket(zmq.PULL)
    state.setsockopt(zmq.LINGER, 0)
    state.connect(f'tcp://127.0.0.1:{state_port}')
    try:
        assert state.poll(5000), 'driver startup timed out'
        state.recv_json()
        action = dict.fromkeys(['tip_delta_x','tip_delta_y','tip_delta_z','tip_delta_rx','tip_delta_ry','tip_delta_rz'], 0.)
        action.update(tip_delta_x=.03, tip_delta_z=.03, tip_delta_rx=.4)
        for _ in range(5):
            command.send_json(dict(protocol_version=1, kind='command', action=action))
            time.sleep(.03)
        if explicit_hold:
            command.send_json(dict(protocol_version=1, kind='hold'))
        held = []
        moved = False
        deadline = time.monotonic() + 3
        while len(held) < 12 and time.monotonic() < deadline:
            if not state.poll(100):
                continue
            msg = state.recv_json()
            status = msg['status']
            if not status['watchdog_holding']:
                moved = True
                held.clear()
            elif moved:
                held.append(status['target_pulses'])
        assert moved and len(held) == 12
        assert all(values == held[0] for values in held)
        proc.send_signal(signal.SIGTERM)
        out, err = proc.communicate(timeout=5)
        assert proc.returncode == 0, (out, err)
        assert 'server stopped' in out
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.communicate()
        command.close()
        state.close()
        context.term()
