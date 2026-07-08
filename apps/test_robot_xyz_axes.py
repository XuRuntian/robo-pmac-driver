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

MAX_REASONABLE_TARGET_M = 0.5


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Send robot-frame +X/-X/+Y/-Y/+Z/-Z tip offsets to a running "
            "continuum_driver_server.py instance."
        )
    )
    parser.add_argument("--remote-ip", default="127.0.0.1")
    parser.add_argument("--command-port", type=int, default=5555)
    parser.add_argument("--state-port", type=int, default=5556)
    parser.add_argument("--rate-hz", type=float, default=30.0)
    parser.add_argument("--ramp-s", type=float, default=2.0)
    parser.add_argument("--hold-s", type=float, default=1.5)
    parser.add_argument("--zero-hold-s", type=float, default=1.0)
    parser.add_argument("--amplitude-x", type=float, default=0.005, help="Robot X test offset in meters.")
    parser.add_argument("--amplitude-y", type=float, default=0.001, help="Robot Y insertion test offset in meters.")
    parser.add_argument("--amplitude-z", type=float, default=0.005, help="Robot Z test offset in meters.")
    parser.add_argument("--amplitude-x-mm", type=float, default=None, help="Robot X test offset in millimeters.")
    parser.add_argument("--amplitude-y-mm", type=float, default=None, help="Robot Y insertion test offset in millimeters.")
    parser.add_argument("--amplitude-z-mm", type=float, default=None, help="Robot Z test offset in millimeters.")
    parser.add_argument(
        "--allow-large-target",
        action="store_true",
        help="Allow meter-unit amplitudes larger than 0.5 m. Usually this indicates a mm/m mixup.",
    )
    parser.add_argument(
        "--axes",
        default="xyz",
        help="Axes to test, any combination/order of x, y, z. Default: xyz.",
    )
    return parser.parse_args()


def zero_action() -> dict[str, float]:
    return dict.fromkeys(ACTION_KEYS, 0.0)


def action_for(axis: str, signed_amplitude_m: float) -> dict[str, float]:
    action = zero_action()
    action[f"tip_delta_{axis}"] = float(signed_amplitude_m)
    return action


def smoothstep(value: float) -> float:
    value = max(0.0, min(1.0, value))
    return value * value * (3.0 - 2.0 * value)


def blend_action(start: Mapping[str, float], end: Mapping[str, float], alpha: float) -> dict[str, float]:
    alpha = smoothstep(alpha)
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


def applied_xyz_mm(state_message: Mapping[str, Any] | None) -> str:
    if not state_message:
        return "applied=n/a"
    applied = state_message.get("applied_action")
    if not isinstance(applied, Mapping):
        return "applied=n/a"
    return (
        "applied_mm="
        f"[{float(applied.get('tip_delta_x', 0.0)) * 1000:+6.2f}, "
        f"{float(applied.get('tip_delta_y', 0.0)) * 1000:+6.2f}, "
        f"{float(applied.get('tip_delta_z', 0.0)) * 1000:+6.2f}]"
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
    if duration_s <= 0.0:
        send_action(command_socket, sequence, end_action)
        return sequence + 1, dict(end_action)

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
                f"{name:12s} target_mm="
                f"[{action['tip_delta_x'] * 1000:+6.2f}, "
                f"{action['tip_delta_y'] * 1000:+6.2f}, "
                f"{action['tip_delta_z'] * 1000:+6.2f}] | "
                f"{applied_xyz_mm(state_message)}"
            )

        next_call += interval
        sleep_time = next_call - time.perf_counter()
        if sleep_time > 0.0:
            time.sleep(sleep_time)
        else:
            next_call = time.perf_counter()

    return sequence, dict(end_action)


def validate_axes(axes: str) -> list[str]:
    out = []
    for axis in axes.lower():
        if axis not in ("x", "y", "z"):
            raise ValueError("--axes may only contain x, y, and z.")
        if axis not in out:
            out.append(axis)
    if not out:
        raise ValueError("--axes must include at least one of x, y, z.")
    return out


def resolve_amplitude_m(args: argparse.Namespace, axis: str) -> float:
    value_m = float(getattr(args, f"amplitude_{axis}"))
    value_mm = getattr(args, f"amplitude_{axis}_mm")
    if value_mm is not None:
        default_m = {"x": 0.005, "y": 0.001, "z": 0.005}[axis]
        if abs(value_m - default_m) > 1e-12:
            raise ValueError(
                f"Use either --amplitude-{axis} or --amplitude-{axis}-mm, not both."
            )
        value_m = float(value_mm) / 1000.0

    value_m = abs(value_m)
    if value_m > MAX_REASONABLE_TARGET_M and not args.allow_large_target:
        raise ValueError(
            f"--amplitude-{axis}={value_m:g} m is unusually large. "
            f"If you meant {value_m:g} mm, use --amplitude-{axis}-mm {value_m:g}; "
            "or pass --allow-large-target if this was intentional."
        )
    return value_m


def main() -> None:
    args = parse_args()
    try:
        if args.rate_hz <= 0.0:
            raise ValueError("--rate-hz must be positive.")
        if args.ramp_s < 0.0 or args.hold_s < 0.0 or args.zero_hold_s < 0.0:
            raise ValueError("Durations must be non-negative.")

        axes = validate_axes(args.axes)
        amplitudes = {
            "x": resolve_amplitude_m(args, "x"),
            "y": resolve_amplitude_m(args, "y"),
            "z": resolve_amplitude_m(args, "z"),
        }
    except ValueError as exc:
        raise SystemExit(f"error: {exc}") from None

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
    print(
        "Robot-frame axis test connected. Expected physical directions: "
        "+X right, +Y inward/insertion, +Z upward."
    )
    print(
        "Requested amplitudes: "
        f"X={amplitudes['x'] * 1000:.2f} mm, "
        f"Y={amplitudes['y'] * 1000:.2f} mm, "
        f"Z={amplitudes['z'] * 1000:.2f} mm."
    )
    print("Press Ctrl+C to stop and hold the current driver target.")

    try:
        for axis in axes:
            for sign, label in ((1.0, "+"), (-1.0, "-")):
                target = action_for(axis, sign * amplitudes[axis])
                phase = f"{label}{axis.upper()}"
                sequence, current = run_phase(
                    name=f"ramp {phase}",
                    command_socket=command_socket,
                    state_socket=state_socket,
                    sequence=sequence,
                    start_action=current,
                    end_action=target,
                    duration_s=args.ramp_s,
                    rate_hz=args.rate_hz,
                )
                sequence, current = run_phase(
                    name=f"hold {phase}",
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
        print("Axis test complete. Sent hold command.")
    except KeyboardInterrupt:
        command_socket.send_json(build_hold_message(sequence))
        print("\nAxis test interrupted. Sent hold command.")
    finally:
        command_socket.close()
        state_socket.close()
        context.term()


if __name__ == "__main__":
    main()
