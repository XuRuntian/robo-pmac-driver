"""Best-effort raw pose fan-out. Simulation failures never block robot actions."""
import time
import uuid
import warnings


class RawPosePublisher:
    def __init__(self, endpoint):
        self.endpoint = endpoint
        self.socket = None
        self.context = None
        self.disabled = False
        self.session = uuid.uuid4().hex
        self.sequence = 0

    def publish(self, position, rotation, clutch):
        if not self.endpoint or self.disabled:
            return
        try:
            import zmq
            if self.socket is None:
                self.context = zmq.Context()
                self.socket = self.context.socket(zmq.PUB)
                self.socket.setsockopt(zmq.LINGER, 0)
                self.socket.setsockopt(zmq.SNDHWM, 1)
                self.socket.bind(self.endpoint)
            message = dict(protocol_version=1, kind='omega_pose', session=self.session,
                           sequence=self.sequence, timestamp_ns=time.time_ns(),
                           position=position.tolist(), rotation=rotation.tolist(),
                           connected=True, clutch=bool(clutch))
            self.sequence += 1
            self.socket.send_json(message, flags=zmq.NOBLOCK)
        except Exception as exc:
            # Optional observer must not interrupt an otherwise valid real action.
            self.close()
            self.disabled = True
            warnings.warn(f'Omega simulation broadcast disabled: {exc}', RuntimeWarning)

    def close(self):
        if self.socket is not None:
            self.socket.close(linger=0)
            self.socket = None
        if self.context is not None:
            self.context.term()
            self.context = None
        self.session = uuid.uuid4().hex
        self.sequence = 0
