from __future__ import annotations

import time
from collections.abc import Mapping
from typing import Any, Literal

from continuum_sdk.control.interface_contract import (
    STATE_FIELDS,
    ActuatorState,
    TipPoseCommand,
    normalize_tip_pose_command,
)


PROTOCOL_VERSION = 1
ControlMessageKind = Literal["command", "hold"]


def _message_header(kind: str, sequence: int) -> dict[str, int | str]:
    return {
        "protocol_version": PROTOCOL_VERSION,
        "kind": kind,
        "sequence": int(sequence),
        "timestamp_ns": time.time_ns(),
    }


def build_command_message(sequence: int, action: Mapping[str, float]) -> dict[str, Any]:
    message = _message_header("command", sequence)
    message["action"] = normalize_tip_pose_command(action)
    return message


def build_hold_message(sequence: int) -> dict[str, Any]:
    return _message_header("hold", sequence)


def parse_control_message(message: Mapping[str, Any]) -> tuple[ControlMessageKind, TipPoseCommand | None]:
    version = int(message.get("protocol_version", -1))
    if version != PROTOCOL_VERSION:
        raise ValueError(f"Unsupported protocol version: {version}")

    kind = message.get("kind")
    if kind == "hold":
        return "hold", None
    if kind != "command":
        raise ValueError(f"Unsupported control message kind: {kind!r}")

    action = message.get("action")
    if not isinstance(action, Mapping):
        raise ValueError("Command message must contain an action mapping.")
    return "command", normalize_tip_pose_command(action)


def build_state_message(
    sequence: int,
    state: Mapping[str, float],
    *,
    status: Mapping[str, Any],
    applied_action: Mapping[str, float],
) -> dict[str, Any]:
    missing = [key for key in STATE_FIELDS if key not in state]
    if missing:
        raise KeyError(f"Missing state fields: {missing}")

    normalized_state: ActuatorState = {
        key: float(state[key])
        for key in STATE_FIELDS
    }  # type: ignore[assignment]
    message = _message_header("state", sequence)
    message["state"] = normalized_state
    message["status"] = dict(status)
    message["applied_action"] = normalize_tip_pose_command(applied_action)
    return message
