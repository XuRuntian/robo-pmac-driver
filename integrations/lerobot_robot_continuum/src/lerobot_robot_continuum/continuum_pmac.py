from __future__ import annotations

import math
import time
from collections.abc import Mapping
from typing import Any

import zmq

from lerobot.cameras import make_cameras_from_configs
from lerobot.robots.robot import Robot
from lerobot.types import RobotAction, RobotObservation
from lerobot.utils.errors import DeviceNotConnectedError

from .config_continuum import ContinuumPMACConfig


PROTOCOL_VERSION = 1
ACTION_FIELDS = (
    "tip_delta_x",
    "tip_delta_y",
    "tip_delta_z",
    "tip_delta_rx",
    "tip_delta_ry",
    "tip_delta_rz",
)
STATE_FIELDS = (
    "axis_1_pos",
    "axis_2_pos",
    "axis_3_pos",
    "axis_4_pos",
    "axis_5_pos",
)


def _normalize_action(action: Mapping[str, Any]) -> dict[str, float]:
    missing = [key for key in ACTION_FIELDS if key not in action]
    if missing:
        raise KeyError(f"Missing continuum action fields: {missing}")

    normalized = {key: float(action[key]) for key in ACTION_FIELDS}
    invalid = [key for key, value in normalized.items() if not math.isfinite(value)]
    if invalid:
        raise ValueError(f"Continuum action fields must be finite: {invalid}")
    return normalized


class ContinuumPMAC(Robot):
    """LeRobot client for the standalone PMAC continuum driver service."""

    config_class = ContinuumPMACConfig
    name = "continuum_pmac"

    def __init__(self, config: ContinuumPMACConfig):
        super().__init__(config)
        self.config = config
        self.cameras = make_cameras_from_configs(config.cameras)
        self._context: zmq.Context | None = None
        self._command_socket: zmq.Socket | None = None
        self._state_socket: zmq.Socket | None = None
        self._is_connected = False
        self._command_sequence = 0
        self._last_state: dict[str, float] | None = None
        self._last_status: dict[str, Any] = {}
        self._last_applied_action: dict[str, float] = dict.fromkeys(ACTION_FIELDS, 0.0)
        self._action_offset: dict[str, float] = dict.fromkeys(ACTION_FIELDS, 0.0)
        self._last_state_received_at: float | None = None

    @property
    def observation_features(self) -> dict[str, type | tuple[int, int, int]]:
        state_features = dict.fromkeys(STATE_FIELDS, float)
        camera_features = {
            name: (camera_config.height, camera_config.width, 3)
            for name, camera_config in self.config.cameras.items()
        }
        return {**state_features, **camera_features}

    @property
    def action_features(self) -> dict[str, type]:
        return dict.fromkeys(ACTION_FIELDS, float)

    @property
    def is_connected(self) -> bool:
        return self._is_connected and all(camera.is_connected for camera in self.cameras.values())

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        return None

    def configure(self) -> None:
        return None

    def connect(self, calibrate: bool = True) -> None:
        if self._is_connected:
            raise RuntimeError(f"{self} is already connected.")

        self._context = zmq.Context()
        self._command_socket = self._context.socket(zmq.PUSH)
        self._command_socket.setsockopt(zmq.CONFLATE, 1)
        self._command_socket.setsockopt(zmq.SNDHWM, 1)
        self._command_socket.setsockopt(zmq.SNDTIMEO, self.config.polling_timeout_ms)
        self._command_socket.setsockopt(zmq.LINGER, 0)
        self._command_socket.connect(
            f"tcp://{self.config.remote_ip}:{self.config.command_port}"
        )

        self._state_socket = self._context.socket(zmq.PULL)
        self._state_socket.setsockopt(zmq.CONFLATE, 1)
        self._state_socket.setsockopt(zmq.RCVHWM, 1)
        self._state_socket.setsockopt(zmq.LINGER, 0)
        self._state_socket.connect(
            f"tcp://{self.config.remote_ip}:{self.config.state_port}"
        )

        try:
            received = self._receive_state(self.config.connect_timeout_s * 1000.0)
        except Exception:
            self._close_sockets()
            raise
        if not received:
            self._close_sockets()
            raise DeviceNotConnectedError(
                "Timed out waiting for the continuum driver service. "
                "Start apps/continuum_driver_server.py first."
            )
        try:
            for camera in self.cameras.values():
                camera.connect()
        except Exception:
            for camera in self.cameras.values():
                if camera.is_connected:
                    camera.disconnect()
            self._close_sockets()
            raise
        if self.config.preserve_applied_action_on_connect:
            self._action_offset = dict(self._last_applied_action)
        else:
            self._action_offset = dict.fromkeys(ACTION_FIELDS, 0.0)
        self._is_connected = True

    def _receive_state(self, timeout_ms: float) -> bool:
        assert self._state_socket is not None
        poller = zmq.Poller()
        poller.register(self._state_socket, zmq.POLLIN)
        events = dict(poller.poll(max(0, int(timeout_ms))))
        if self._state_socket not in events:
            return False

        latest = None
        while True:
            try:
                latest = self._state_socket.recv_json(flags=zmq.NOBLOCK)
            except zmq.Again:
                break
        if latest is None:
            return False
        self._update_state(latest)
        return True

    def _update_state(self, message: Mapping[str, Any]) -> None:
        if int(message.get("protocol_version", -1)) != PROTOCOL_VERSION:
            raise ValueError(
                f"Unsupported continuum driver protocol: {message.get('protocol_version')}"
            )
        if message.get("kind") != "state":
            raise ValueError(f"Unexpected continuum driver message: {message.get('kind')!r}")

        raw_state = message.get("state")
        if not isinstance(raw_state, Mapping):
            raise ValueError("Continuum state message has no state mapping.")
        missing = [key for key in STATE_FIELDS if key not in raw_state]
        if missing:
            raise KeyError(f"Missing continuum state fields: {missing}")

        state = {key: float(raw_state[key]) for key in STATE_FIELDS}
        if not all(math.isfinite(value) for value in state.values()):
            raise ValueError("Continuum state contains non-finite values.")

        status = message.get("status", {})
        raw_applied_action = message.get("applied_action", {})
        self._last_state = state
        self._last_status = dict(status) if isinstance(status, Mapping) else {}
        if isinstance(raw_applied_action, Mapping):
            self._last_applied_action = _normalize_action(raw_applied_action)
        self._last_state_received_at = time.perf_counter()

    def get_observation(self) -> RobotObservation:
        if not self._is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        self._receive_state(self.config.polling_timeout_ms)
        if self._last_state is None or self._last_state_received_at is None:
            raise DeviceNotConnectedError("No continuum state has been received.")

        state_age = time.perf_counter() - self._last_state_received_at
        if state_age > self.config.state_timeout_s:
            raise DeviceNotConnectedError(
                f"Continuum driver state is stale ({state_age:.3f}s)."
            )
        observation: RobotObservation = dict(self._last_state)
        for name, camera in self.cameras.items():
            observation[name] = camera.read_latest()
        return observation

    def send_action(self, action: RobotAction) -> RobotAction:
        if not self._is_connected or self._command_socket is None:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        teleop_action = _normalize_action(action)
        normalized = {
            key: teleop_action[key] + self._action_offset.get(key, 0.0)
            for key in ACTION_FIELDS
        }
        message = {
            "protocol_version": PROTOCOL_VERSION,
            "kind": "command",
            "sequence": self._command_sequence,
            "timestamp_ns": time.time_ns(),
            "action": normalized,
        }
        try:
            self._command_socket.send_json(message, flags=zmq.NOBLOCK)
        except zmq.Again as exc:
            raise DeviceNotConnectedError(
                "The continuum driver service is not accepting commands."
            ) from exc
        self._command_sequence += 1
        return normalized

    @property
    def driver_status(self) -> dict[str, Any]:
        return dict(self._last_status)

    def disconnect(self) -> None:
        if not self._is_connected:
            return

        for camera in self.cameras.values():
            if camera.is_connected:
                camera.disconnect()
        if self._command_socket is not None:
            hold_message = {
                "protocol_version": PROTOCOL_VERSION,
                "kind": "hold",
                "sequence": self._command_sequence,
                "timestamp_ns": time.time_ns(),
            }
            try:
                self._command_socket.send_json(hold_message, flags=zmq.NOBLOCK)
            except zmq.Again:
                pass
        self._close_sockets()
        self._is_connected = False

    def _close_sockets(self) -> None:
        if self._state_socket is not None:
            self._state_socket.close()
        if self._command_socket is not None:
            self._command_socket.close()
        if self._context is not None:
            self._context.term()
        self._state_socket = None
        self._command_socket = None
        self._context = None
