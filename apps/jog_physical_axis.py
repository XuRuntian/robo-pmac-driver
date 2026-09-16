from __future__ import annotations

import argparse
import time

import numpy as np

from pmac_sdk.controller.robot_api import PMACRobotController
from pmac_sdk.core.config_model import PMACConfig


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Move one physical PMAC axis by a small relative offset."
    )
    parser.add_argument("--execute", action="store_true", help="Connect to PMAC and execute motion.")
    parser.add_argument("--pmac-ip", default="192.168.0.200")
    parser.add_argument("--axis", type=int, required=True, choices=range(1, 6), help="Physical PMAC axis number, 1..5.")
    parser.add_argument("--deg", type=float, default=0.0, help="Relative move for axes 1..4, in motor output degrees.")
    parser.add_argument("--mm", type=float, default=0.0, help="Relative move for axis 5, in millimeters.")
    parser.add_argument("--duration-s", type=float, default=2.0, help="Ramp duration.")
    parser.add_argument("--hold-s", type=float, default=2.0, help="Hold at target before exiting.")
    parser.add_argument("--rate-hz", type=float, default=50.0)
    parser.add_argument("--return-to-start", action="store_true", help="Return to the startup position before exiting.")
    return parser.parse_args()


def smoothstep(value: float) -> float:
    value = max(0.0, min(1.0, value))
    return value * value * (3.0 - 2.0 * value)


def validate_args(args: argparse.Namespace) -> None:
    if args.rate_hz <= 0.0:
        raise ValueError("--rate-hz must be positive.")
    if args.duration_s <= 0.0 or args.hold_s < 0.0:
        raise ValueError("--duration-s must be positive and --hold-s must be non-negative.")
    if args.axis < 5:
        if abs(args.mm) > 1e-12:
            raise ValueError("--mm is only for axis 5.")
        if not 0.0 < abs(args.deg) <= 10.0:
            raise ValueError("--deg must be within (0, 10] for axes 1..4.")
    else:
        if abs(args.deg) > 1e-12:
            raise ValueError("--deg is only for axes 1..4.")
        if not 0.0 < abs(args.mm) <= 5.0:
            raise ValueError("--mm must be within (0, 5] for axis 5.")


def target_from_delta(config: PMACConfig, start_pulses: list[int], axis: int, deg: float, mm: float) -> list[int]:
    target = list(start_pulses)
    index = axis - 1
    if axis < 5:
        target[index] += int(round(deg * config.pulses_per_degree))
    else:
        target[index] += int(round((mm / 1000.0) * config.pulses_per_meter))
    return target


def run_ramp(
    robot: PMACRobotController | None,
    *,
    start_pulses: list[int],
    target_pulses: list[int],
    duration_s: float,
    rate_hz: float,
) -> None:
    steps = max(1, int(round(duration_s * rate_hz)))
    interval_s = 1.0 / rate_hz
    move_time_ms = interval_s * 1000.0
    start = np.asarray(start_pulses, dtype=float)
    target = np.asarray(target_pulses, dtype=float)
    previous = start.copy()
    next_call = time.perf_counter()

    for index in range(1, steps + 1):
        alpha = smoothstep(index / steps)
        command = np.rint(start + alpha * (target - start)).astype(int)
        velocities = ((command.astype(float) - previous) / move_time_ms).tolist()
        if robot is not None:
            robot.move_pvt_stream(
                target_pulses=command.tolist(),
                velocities=velocities,
                move_time=move_time_ms,
            )
        previous = command.astype(float)

        if index == 1 or index == steps or index % max(1, int(rate_hz)) == 0:
            delta = [int(value - base) for value, base in zip(command.tolist(), start_pulses)]
            print(f"ramp {index:4d}/{steps}: target={command.tolist()} dpulses={delta}")

        next_call += interval_s
        sleep_time = next_call - time.perf_counter()
        if sleep_time > 0.0:
            time.sleep(sleep_time)
        else:
            next_call = time.perf_counter()


def hold_target(robot: PMACRobotController | None, target_pulses: list[int], hold_s: float, rate_hz: float) -> None:
    if hold_s <= 0.0:
        return
    steps = max(1, int(round(hold_s * rate_hz)))
    move_time_ms = 1000.0 / rate_hz
    next_call = time.perf_counter()
    for _ in range(steps):
        if robot is not None:
            robot.move_pvt_stream(
                target_pulses=target_pulses,
                velocities=[0.0, 0.0, 0.0, 0.0, 0.0],
                move_time=move_time_ms,
            )
        next_call += 1.0 / rate_hz
        sleep_time = next_call - time.perf_counter()
        if sleep_time > 0.0:
            time.sleep(sleep_time)
        else:
            next_call = time.perf_counter()


def main() -> None:
    args = parse_args()
    try:
        validate_args(args)
    except ValueError as exc:
        raise SystemExit(f"error: {exc}") from None

    config = PMACConfig(ip=args.pmac_ip)
    robot = PMACRobotController(config) if args.execute else None

    try:
        if robot is not None:
            robot.safe_boot_and_home()
            start_pulses = robot.base_positions.copy()
        else:
            start_pulses = [0, 0, 0, 0, 0]

        target_pulses = target_from_delta(
            config,
            start_pulses,
            axis=args.axis,
            deg=args.deg,
            mm=args.mm,
        )
        delta = [target - start for target, start in zip(target_pulses, start_pulses)]

        mode = "EXECUTE" if args.execute else "DRY-RUN"
        print(f"Physical axis jog [{mode}]")
        print(f"start={start_pulses}")
        print(f"target={target_pulses}")
        print(f"dpulses={delta}")
        if args.axis < 5:
            print(
                f"axis #{args.axis}: {args.deg:+.4f} deg "
                f"({args.deg * config.pulses_per_degree:+.0f} pulses)"
            )
        else:
            print(
                f"axis #5: {args.mm:+.4f} mm "
                f"({args.mm / 1000.0 * config.pulses_per_meter:+.0f} pulses)"
            )

        run_ramp(
            robot,
            start_pulses=start_pulses,
            target_pulses=target_pulses,
            duration_s=args.duration_s,
            rate_hz=args.rate_hz,
        )
        hold_target(robot, target_pulses, args.hold_s, args.rate_hz)

        if robot is not None:
            feedback = robot.read_positions()
            print(f"feedback={feedback}")
            print(f"feedback_delta={[actual - start for actual, start in zip(feedback, start_pulses)]}")

        if args.return_to_start:
            print("Returning to start position.")
            run_ramp(
                robot,
                start_pulses=target_pulses,
                target_pulses=start_pulses,
                duration_s=args.duration_s,
                rate_hz=args.rate_hz,
            )
            hold_target(robot, start_pulses, min(args.hold_s, 1.0), args.rate_hz)
    finally:
        if robot is not None:
            robot.close()


if __name__ == "__main__":
    main()
