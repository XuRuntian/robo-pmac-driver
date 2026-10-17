"""Offline broadcast and independent simulation tests; no device imports."""
import importlib.util
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
import pytest
import zmq

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


def test_parallel_simulation_process_and_disconnect():
    reference = ROOT.parent/'surgical_continuum_robot/remote_control_ex'
    if not reference.exists():
        pytest.skip('Reference project unavailable')
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
            '--endpoint', raw_endpoint, '--visual-endpoint', endpoint, '--duration', '4'],
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
            np.testing.assert_array_equal(frames[0]['ideal_joint'], np.zeros(5))
            for n in range(30):
                publisher.publish(np.array([n*1e-5,n*1e-5,0]), np.eye(3), False)
                if subscriber.poll(20):
                    frames.append(subscriber.recv_json())
            assert any(np.linalg.norm(f['ideal_joint'][:2]) > 1e-6 for f in frames)
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
