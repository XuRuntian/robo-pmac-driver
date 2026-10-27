"""Offline broadcast and independent simulation tests; no device imports."""
import importlib.util
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
import pytest
import zmq
import yaml

ROOT = Path(__file__).resolve().parents[1]
PLUGIN = ROOT/'integrations/lerobot_teleoperator_omega_continuum/src/lerobot_teleoperator_omega_continuum'


def publisher_type():
    spec = importlib.util.spec_from_file_location('raw_pose_test', PLUGIN/'raw_pose.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.RawPosePublisher


def test_optional_broadcast_failure_does_not_raise():
    publisher = publisher_type()('invalid://endpoint')
    with pytest.warns(RuntimeWarning, match='broadcast disabled'):
        publisher.publish(np.zeros(3), np.eye(3), False)
    assert publisher.disabled and publisher.socket is None and publisher.context is None
    publisher.publish(np.zeros(3), np.eye(3), False)
    publisher.close()


@pytest.mark.parametrize('zero,encoder', [(0,0), (-1255635,321)])
def test_parallel_simulation_process_and_disconnect(tmp_path, zero, encoder):
    reference = ROOT.parent/'surgical_continuum_robot/remote_control_ex'
    if not reference.exists():
        pytest.skip('Reference project unavailable')
    config = yaml.safe_load((ROOT/'config/robot_interface_stop_priority_trial.yaml').read_text(encoding='utf-8'))
    config['initial_position']['insertion_zero_pulses'] = zero
    config_path = tmp_path/'interface.yaml'
    config_path.write_text(yaml.safe_dump(config), encoding='utf-8')
    initial = np.array([(encoder-zero)/62781744.,0,0,0,0])
    publisher = publisher_type()('tcp://127.0.0.1:*')
    publisher.publish(np.zeros(3), np.eye(3), False)
    raw_endpoint = publisher.socket.getsockopt_string(zmq.LAST_ENDPOINT)
    with zmq.Context() as context:
        subscriber = context.socket(zmq.SUB)
        subscriber.setsockopt(zmq.LINGER, 0)
        subscriber.setsockopt(zmq.SUBSCRIBE, b'')
        # Reserve a local port, then hand it to the independent process.
        probe = context.socket(zmq.PUB)
        port = probe.bind_to_random_port('tcp://127.0.0.1')
        probe.close(0)
        endpoint = f'tcp://127.0.0.1:{port}'
        subscriber.connect(endpoint)
        child = subprocess.Popen([sys.executable, str(ROOT/'apps/omega_reference_sim.py'),
            '--endpoint', raw_endpoint, '--visual-endpoint', endpoint, '--duration', '4',
            '--interface-config', str(config_path), '--initial-axis5-pulses', str(encoder)],
            cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
        frames = []
        try:
            deadline = time.monotonic()+3
            while time.monotonic() < deadline:
                publisher.publish(np.zeros(3), np.eye(3), False)
                if subscriber.poll(20):
                    frames.append(subscriber.recv_json())
                    if len(frames) >= 4:
                        break
            assert len(frames) >= 4
            np.testing.assert_allclose(frames[0]['ideal_joint'], initial, atol=1e-12)
            # The hand motion must exceed the reference IK position tolerance
            # after the configured translation gain, or no joint motion is due.
            for n in range(30):
                publisher.publish(np.array([n*4e-5,n*4e-5,0]), np.eye(3), False)
                if subscriber.poll(20):
                    frames.append(subscriber.recv_json())
            assert any(np.linalg.norm(np.array(f['ideal_joint'])[:2]-initial[:2]) > 1e-6 for f in frames)
            assert all(f['ideal_source'] == 'independent_reference_ik' for f in frames)
            assert all('target_joint' not in f for f in frames)
            publisher.close()
            time.sleep(.35)
            while subscriber.poll(0):
                subscriber.recv_json()
            assert not subscriber.poll(150), 'Stale simulation should stop publishing'
            output, _ = child.communicate(timeout=6)
            assert child.returncode == 0, output.decode(errors='replace')
        finally:
            publisher.close()
            subscriber.close(0)
            if child.poll() is None:
                child.kill()
            child.communicate()


def test_invalid_startup_insertion_is_rejected_before_sockets():
    from types import SimpleNamespace
    from continuum_sdk.control.reference_teleop import load_reference
    reference_root = ROOT.parent/'surgical_continuum_robot/remote_control_ex'
    if not reference_root.exists():
        pytest.skip('Reference project unavailable')
    spec = importlib.util.spec_from_file_location('independent_sim_test', ROOT/'apps/omega_reference_sim.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    reference = load_reference(reference_root)
    interface = SimpleNamespace(initial_position=SimpleNamespace(insertion_zero_pulses=100))
    with pytest.raises(ValueError, match='startup insertion'):
        module.initial_simulation_joint(reference, interface, 0)
    interface.initial_position.insertion_zero_pulses = -100000000
    with pytest.raises(ValueError, match='startup insertion'):
        module.initial_simulation_joint(reference, interface, 0)
    interface.initial_position.insertion_zero_pulses = None
    np.testing.assert_array_equal(module.initial_simulation_joint(reference, interface, 12345), np.zeros(5))
