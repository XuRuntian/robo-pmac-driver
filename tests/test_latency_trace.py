import csv
import importlib.util
import json
import sys
from types import SimpleNamespace
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("trace_server", ROOT / "apps/continuum_driver_server.py")
server = importlib.util.module_from_spec(spec)
spec.loader.exec_module(server)


@pytest.mark.parametrize("mock_execute", [False, True])
@pytest.mark.parametrize("wait_mode", ["system", "windows-highres"])
@pytest.mark.parametrize("profile_trial", [False, True, 'stop_priority'])
@pytest.mark.parametrize("live_debug", [False, True])
def test_trace_dry_run_keeps_feedback_unknown_and_records_hold(tmp_path, monkeypatch, mock_execute, wait_mode, profile_trial, live_debug):
    if wait_mode == "windows-highres" and sys.platform != "win32":
        pytest.skip("Native high resolution backend requires Windows")
    path = tmp_path / "trace.csv"
    # Exercise the optional independent publisher with no consumers attached.
    import socket
    with socket.socket() as probe:
        probe.bind(('127.0.0.1',0))
        visual_port=probe.getsockname()[1]
    monkeypatch.chdir(ROOT)
    monkeypatch.setattr(sys, "argv", ["driver", "--latency-csv", str(path),
                                      "--command-port", "0", "--state-port", "0", "--wait-mode", wait_mode]
                        + (["--execute"] if mock_execute else [])
                        + (["--live-debug"] if live_debug else [])
                        + (["--visual-state-port", str(visual_port)] if live_debug else [])
                        + (["--interface-config", "config/robot_interface_stop_priority_trial.yaml" if profile_trial == 'stop_priority'
                            else "config/robot_interface_axis5_profile_trial.yaml"] if profile_trial else []))
    def forbidden(*args, **kwargs):
        raise AssertionError("Dry-run must not construct a hardware controller")
    positions = [1000, 2000, 3000, 4000, 1000000]
    fake = SimpleNamespace(base_positions=positions, pvt_axis5_max_step=5023 if profile_trial else 3000, last_axis5_error_abs_counts=0,
        safe_boot_and_home=lambda: None, close=lambda: None,
        read_positions=lambda: positions.copy(), move_pvt_stream=lambda **kwargs: None,
        protocol=SimpleNamespace(last_pvt_status=SimpleNamespace(queued=7, accepted=1),
            last_submit_timing=server.TransactionTiming(read_ms=3, sleep_ms=15, success=True),
            last_snapshot_timing=server.TransactionTiming(read_ms=4, sleep_ms=15, success=True)))
    monkeypatch.setattr(server, "PMACRobotController", (lambda _: fake) if mock_execute else forbidden)
    action = {"tip_delta_" + key: 0.0 for key in ("x", "y", "z", "rx", "ry", "rz")}
    action["tip_delta_x"] = .001
    messages = iter([dict(protocol_version=1, kind="command", action=action), None,
                     dict(protocol_version=1, kind="hold"), None])
    def receive(_):
        try:
            return next(messages)
        except StopIteration:
            raise KeyboardInterrupt
    monkeypatch.setattr(server, "_latest_message", receive)
    server.main()
    with path.open(encoding="utf-8") as f:
        metadata = json.loads(f.readline()[2:])
        rows = list(csv.DictReader(f))
    assert metadata["execute"] is mock_execute
    assert metadata["latency_schema"] == 2
    assert metadata['wait_mode'] == wait_mode
    assert len(rows) == 4
    expected_limit = 5022.53952 if profile_trial else (3000 if mock_execute else 12556.3488)
    assert all(float(r['insertion_step_limit_counts']) == pytest.approx(expected_limit) for r in rows)
    for previous, row in zip(rows, rows[1:]):
        if profile_trial:
            assert float(row['insertion_peak_speed_m_s']) <= .004 + 1e-10
            assert float(row['insertion_peak_accel_m_s2']) <= .005 + 1e-10
        assert int(row['insertion_applied_step_counts']) == int(row['target_p5']) - int(previous['target_p5'])
        assert (row['insertion_step_clipped'] == 'True') == (
            abs(float(row['insertion_requested_step_counts'])) > expected_limit)
    if mock_execute:
        if wait_mode == 'windows-highres':
            assert callable(fake.protocol.sleep_fn)
        assert fake.protocol.trace_enabled
        assert rows[0]["feedback_new"] == "True"
        assert all(r["feedback_p1"] == "1000" and r["queued_at_ack"] == "7" for r in rows)
        for row in rows:
            assert float(row["pvt_sleep_ms"]) == 15
            assert row["snapshot_read_ms"] == ('4' if row["feedback_new"] == 'True' else '')
            assert float(row["submit_ms"]) >= 0
            assert float(row["feedback_request_end_s"]) >= float(row["feedback_request_start_s"])
    else:
        assert all(r["pvt_sleep_ms"] == '' and r["snapshot_read_ms"] == '' for r in rows)
        assert all(r["feedback_p1"] == "" and r["queued_at_ack"] == "" for r in rows)
        assert all(r["feedback_new"] == "False" for r in rows)
    assert float(rows[0]["raw_tip_delta_x"]) == .001
    if profile_trial == 'stop_priority':
        assert rows[0]['input_holding'] == 'True'
        assert float(rows[0]['applied_tip_delta_x']) == pytest.approx(0., abs=1e-8)
    else:
        assert 0 < float(rows[0]["applied_tip_delta_x"]) < .001
    assert rows[2]["holding"] == "True"
    assert rows[2]["target_p1"] == rows[3]["target_p1"]
    # Existing data is rejected before any controller creation or motion.
    with pytest.raises(FileExistsError):
        server.main()


def test_stop_priority_driver_cancels_static_input_and_reanchors_on_resume(tmp_path, monkeypatch):
    """Run the actual loop with hardware construction forbidden, in dry-run."""
    monkeypatch.chdir(ROOT)
    path=tmp_path/'stop_trace.csv'
    monkeypatch.setattr(sys,'argv',['driver','--interface-config','config/robot_interface_stop_priority_trial.yaml',
        '--command-port','0','--state-port','0','--latency-csv',str(path)])
    def forbidden(*args,**kwargs):
        raise AssertionError('No hardware in stop-priority integration test')
    monkeypatch.setattr(server,'PMACRobotController',forbidden)
    # Avoid wall-clock waiting; watchdog sees fresh commands on every cycle.
    monkeypatch.setattr(server.time,'sleep',lambda _:None)
    commands=[]
    for i in range(110):
        action={'tip_delta_'+k:0. for k in ['x','y','z','rx','ry','rz']}
        action['tip_delta_y']=min(i,30)*.0001
        if i>=100:action['tip_delta_y']+=(i-99)*.0001
        commands.append(dict(protocol_version=1,kind='command',action=action))
    messages=iter(commands)
    def receive(_):
        try:return next(messages)
        except StopIteration:raise KeyboardInterrupt
    monkeypatch.setattr(server,'_latest_message',receive)
    server.main()
    with path.open(encoding='utf-8') as f:
        next(f);rows=list(csv.DictReader(f))
    assert all(r['input_holding']=='True' for r in rows[36:100])
    assert all(r['watchdog_holding']=='False' for r in rows)
    assert all(r['input_holding']=='False' for r in rows[100:])
    for axis in range(1,6):
        assert len({r[f'target_p{axis}'] for r in rows[90:100]})==1
    # Restart continues from committed pose, not the unexecuted original offset.
    for row in rows[100:]:
        assert float(row['rotary_peak_rad_s'])<=2.5
        assert float(row['insertion_peak_accel_m_s2'])<=.005+1e-10


def test_rotary_jump_is_never_submitted_and_cleanup_precedes_evidence(tmp_path, monkeypatch):
    monkeypatch.chdir(ROOT)
    monkeypatch.setattr(sys,'argv',['driver','--execute','--latency-csv',str(tmp_path/'trace.csv'),
        '--command-port','0','--state-port','0'])
    calls=[]
    positions=[1000,2000,3000,4000,1000000]
    fake=SimpleNamespace(base_positions=positions,pvt_axis5_max_step=3000,
        safe_boot_and_home=lambda:None,close=lambda:calls.append('close'),
        move_pvt_stream=lambda **kw:calls.append('submit'),
        protocol=SimpleNamespace())
    monkeypatch.setattr(server,'PMACRobotController',lambda _:fake)
    monkeypatch.setattr(server,'_latest_message',lambda _:None)
    original=server.ContinuumPVTMapper.build_command
    def corrupt(self,*args,**kwargs):
        result=original(self,*args,**kwargs)
        result.target_pulses[1]+=1000000
        return result
    monkeypatch.setattr(server.ContinuumPVTMapper,'build_command',corrupt)
    original_write=Path.write_text
    def observe_write(path,*args,**kwargs):
        if path.name.startswith('rotary_stop_'):
            assert calls==['close']
        return original_write(path,*args,**kwargs)
    monkeypatch.setattr(Path,'write_text',observe_write)
    with pytest.raises(server.RotaryTargetJump,match='Candidate not submitted'):
        server.main()
    assert calls==['close']
    evidence=json.loads(next(tmp_path.glob('rotary_stop_*.json')).read_text())
    assert evidence['physical_axis']==2
    assert evidence['sequence']==0
    assert evidence['execute'] is True
