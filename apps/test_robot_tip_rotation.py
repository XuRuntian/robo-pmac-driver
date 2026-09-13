from __future__ import annotations

import argparse
import time
from collections.abc import Mapping
from typing import Any

import zmq

from continuum_sdk.transport.zmq_protocol import build_command_message, build_hold_message


ACTION_KEYS = (
    "tip_delta_x",
    "tip_delta_y",
    "tip_delta_z",
    "tip_delta_rx",
    "tip_delta_ry",
    "tip_delta_rz",
)
ROTATION_AXES = ("rx", "ry")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Send small tip-axis rotation tests to a running continuum driver."
    )
    parser.add_argument("--remote-ip", default="127.0.0.1")
    parser.add_argument("--command-port", type=int, default=5555)
    parser.add_argument("--state-port", type=int, default=5556)
    parser.add_argument("--rate-hz", type=float, default=30.0)
    parser.add_argument("--ramp-s", type=float, default=5.0)
    parser.add_argument("--hold-s", type=float, default=2.0)
    parser.add_argument("--zero-hold-s", type=float, default=1.0)
    parser.add_argument("--amplitude-rx-rad", type=float, default=0.02)
    parser.add_argument("--amplitude-ry-rad", type=float, default=0.02)
    parser.add_argument("--axes", default="rxry", help="Rotation axes to test: rx, ry, or rxry.")
    return parser.parse_args()


def zero_action() -> dict[str, float]:
    return dict.fromkeys(ACTION_KEYS, 0.0)


def action_for(axis: str, value: float) -> dict[str, float]:
    action = zero_action()
    action[f"tip_delta_{axis}"] = float(value)
    return action


def blend_action(
    start: Mapping[str, float], end: Mapping[str, float], alpha: float
) -> dict[str, float]:
    alpha = max(0.0, min(1.0, alpha))
    return {
        key: float(start[key]) + alpha * (float(end[key]) - float(start[key]))
        for key in ACTION_KEYS
    }


def latest_message(socket: zmq.Socket) -> dict[str, Any] | None:
    latest = None
    while True:
        try:
            latest = socket.recv_json(flags=zmq.NOBLOCK)
        except zmq.Again:
            return latest


def rotation_text(state_message: Mapping[str, Any] | None) -> str:
    if not state_message:
        return "applied=n/a"
    applied = state_message.get("applied_action")
    if not isinstance(applied, Mapping):
        return "applied=n/a"
    return (
        "applied_rad="
        f"[{float(applied.get('tip_delta_rx', 0.0)):+.4f}, "
        f"{float(applied.get('tip_delta_ry', 0.0)):+.4f}, "
        f"{float(applied.get('tip_delta_rz', 0.0)):+.4f}]"
    )


def send_action(socket: zmq.Socket, sequence: int, action: Mapping[str, float]) -> None:
    socket.send_json(build_command_message(sequence, action))


def run_phase(
    *,
    name: str,
    command_socket: zmq.Socket,
    state_socket: zmq.Socket,
    sequence: int,
    start_action: Mapping[str, float],
    end_action: Mapping[str, float],
    duration_s: float,
    rate_hz: float,
) -> tuple[int, dict[str, float]]:
    steps = max(1, int(round(duration_s * rate_hz)))
    interval = 1.0 / rate_hz
    next_call = time.perf_counter()

    for index in range(steps + 1):
        action = blend_action(start_action, end_action, index / steps)
        send_action(command_socket, sequence, action)
        sequence += 1
        if index == 0 or index == steps or index % max(1, int(rate_hz)) == 0:
            state_message = latest_message(state_socket)
            print(
                f"{name:12s} target_rad="
                f"[{action['tip_delta_rx']:+.4f}, {action['tip_delta_ry']:+.4f}, "
                f"{action['tip_delta_rz']:+.4f}] | {rotation_text(state_message)}"
            )

        next_call += interval
        sleep_time = next_call - time.perf_counter()
        if sleep_time > 0.0:
            time.sleep(sleep_time)
        else:
            next_call = time.perf_counter()

    return sequence, dict(end_action)


def validate_axes(raw_axes: str) -> list[str]:
    normalized = raw_axes.lower().replace(",", "").replace(" ", "")
    if not normalized or len(normalized) % 2 != 0:
        raise ValueError("--axes must contain rx and/or ry, for example rx, ry, or rxry.")
    axes = []
    for index in range(0, len(normalized), 2):
        axis = normalized[index : index + 2]
        if axis not in ROTATION_AXES or axis in axes:
            raise ValueError("--axes must contain rx and/or ry, for example rx, ry, or rxry.")
        axes.append(axis)
    return axes


def main() -> None:
    args = parse_args()
    if args.rate_hz <= 0.0 or args.ramp_s < 0.0 or args.hold_s < 0.0 or args.zero_hold_s < 0.0:
        raise SystemExit("rate and durations must be non-negative, with rate > 0")
    try:
        axes = validate_axes(args.axes)
    except ValueError as exc:
        raise SystemExit(f"error: {exc}") from None

    amplitudes = {"rx": abs(float(args.amplitude_rx_rad)), "ry": abs(float(args.amplitude_ry_rad))}
    if any(value <= 0.0 or value > 2.0 for value in amplitudes.values()):
        raise SystemExit("rotation amplitudes must be in (0, 1.8] rad for the rotation config")

    context = zmq.Context()
    command_socket = context.socket(zmq.PUSH)
    command_socket.setsockopt(zmq.SNDHWM, 1)
    command_socket.setsockopt(zmq.LINGER, 0)
    command_socket.connect(f"tcp://{args.remote_ip}:{args.command_port}")
    state_socket = context.socket(zmq.PULL)
    state_socket.setsockopt(zmq.CONFLATE, 1)
    state_socket.setsockopt(zmq.RCVHWM, 1)
    state_socket.setsockopt(zmq.LINGER, 0)
    state_socket.connect(f"tcp://{args.remote_ip}:{args.state_port}")

    sequence = 0
    current = zero_action()
    print("Tip rotation test connected. rx/ry tilt the tip axis; rz is not tested.")
    print(f"Requested amplitudes: rx={amplitudes['rx']:.4f} rad, ry={amplitudes['ry']:.4f} rad.")
    print("Press Ctrl+C to stop and hold the current driver target.")

    try:
        for axis in axes:
            for sign, label in ((1.0, "+"), (-1.0, "-")):
                target = action_for(axis, sign * amplitudes[axis])
                sequence, current = run_phase(
                    name=f"ramp {label}{axis.upper()}",
                    command_socket=command_socket,
                    state_socket=state_socket,
                    sequence=sequence,
                    start_action=current,
                    end_action=target,
                    duration_s=args.ramp_s,
                    rate_hz=args.rate_hz,
                )
                sequence, current = run_phase(
                    name=f"hold {label}{axis.upper()}",
                    command_socket=command_socket,
                    state_socket=state_socket,
                    sequence=sequence,
                    start_action=current,
                    end_action=target,
                    duration_s=args.hold_s,
                    rate_hz=args.rate_hz,
                )
                sequence, current = run_phase(
                    name="return 0",
                    command_socket=command_socket,
                    state_socket=state_socket,
                    sequence=sequence,
                    start_action=current,
                    end_action=zero_action(),
                    duration_s=args.ramp_s,
                    rate_hz=args.rate_hz,
                )
                sequence, current = run_phase(
                    name="zero hold",
                    command_socket=command_socket,
                    state_socket=state_socket,
                    sequence=sequence,
                    start_action=current,
                    end_action=zero_action(),
                    duration_s=args.zero_hold_s,
                    rate_hz=args.rate_hz,
                )
        command_socket.send_json(build_hold_message(sequence))
        print("Tip rotation test complete. Sent hold command.")
    except KeyboardInterrupt:
        command_socket.send_json(build_hold_message(sequence))
        print("\nTip rotation test interrupted. Sent hold command.")
    finally:
        command_socket.close()
        state_socket.close()
        context.term()


if __name__ == "__main__":
    main()
