from __future__ import annotations

import argparse
import time
from collections.abc import Mapping

import numpy as np

from continuum_sdk.control.axis_mapper import ContinuumAxisMapper
from pmac_sdk.controller.robot_api import PMACRobotController
from pmac_sdk.core.config_model import PMACConfig


LOGICAL_AXIS_NAMES = ("alpha1", "alpha2", "alpha3", "alpha4", "d")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Move logical PMAC continuum axes directly, bypassing Cartesian IK. "
            "Stop continuum_driver_server.py before running with --execute."
        )
    )
    parser.add_argument("--execute", action="store_true", help="Connect to PMAC and execute motion.")
    parser.add_argument("--pmac-ip", default="192.168.0.200")
    parser.add_argument("--axes", default="1234", help="Logical axes to test: any of 1,2,3,4,5,d.")
    parser.add_argument("--alpha-rad", type=float, default=0.3, help="Amplitude for alpha1..4 in rad.")
    parser.add_argument("--d-mm", type=float, default=1.0, help="Amplitude for logical d/axis5 in mm.")
    parser.add_argument("--rate-hz", type=float, default=50.0)
    parser.add_argument("--ramp-s", type=float, default=2.0)
    parser.add_argument("--hold-s", type=float, default=2.0)
    parser.add_argument("--zero-hold-s", type=float, default=1.0)
    parser.add_argument("--feedback-hz", type=float, default=5.0)
    return parser.parse_args()


def smoothstep(value: float) -> float:
    value = max(0.0, min(1.0, value))
    return value * value * (3.0 - 2.0 * value)


def parse_axes(raw_axes: str) -> list[int]:
    axes = []
    for char in raw_axes.lower():
        if char in "1234":
            axis = int(char) - 1
        elif char in ("5", "d"):
            axis = 4
        else:
            raise ValueError("--axes may only contain 1,2,3,4,5,d.")
        if axis not in axes:
            axes.append(axis)
    if not axes:
        raise ValueError("--axes must include at least one axis.")
    return axes


def logical_target(axis: int, signed_alpha_rad: float, signed_d_m: float) -> list[float]:
    target = [0.0, 0.0, 0.0, 0.0, 0.0]
    target[axis] = float(signed_d_m if axis == 4 else signed_alpha_rad)
    return target


def blend(start: list[float], end: list[float], alpha: float) -> list[float]:
    alpha = smoothstep(alpha)
    return [float(a + alpha * (b - a)) for a, b in zip(start, end)]


def logical_label(values: list[float]) -> str:
    return (
        f"alpha_rad=[{values[0]:+.3f}, {values[1]:+.3f}, {values[2]:+.3f}, {values[3]:+.3f}], "
        f"d_mm={values[4] * 1000:+.2f}"
    )


def pulse_delta_label(base_pulses: list[int], pulses: list[int]) -> str:
    return "dpulses=[" + ", ".join(f"{actual - base:+d}" for actual, base in zip(pulses, base_pulses)) + "]"


def maybe_read_feedback(
    robot: PMACRobotController | None,
    axis_mapper: ContinuumAxisMapper,
    base_pulses: list[int],
) -> tuple[list[int] | None, list[float] | None]:
    if robot is None:
        return None, None
    feedback_pulses = robot.read_positions()
    if not any(feedback_pulses):
        return feedback_pulses, None
    return feedback_pulses, axis_mapper.pulses_to_logical(base_pulses, feedback_pulses)


def run_phase(
    *,
    name: str,
    robot: PMACRobotController | None,
    axis_mapper: ContinuumAxisMapper,
    base_pulses: list[int],
    previous: list[float],
    target: list[float],
    duration_s: float,
    rate_hz: float,
    feedback_hz: float,
) -> list[float]:
    if duration_s <= 0.0:
        target_pulses = axis_mapper.logical_to_pulses(base_pulses, target)
        velocities = [0.0] * len(target_pulses)
        if robot is not None:
            robot.move_pvt_stream(
                target_pulses=target_pulses,
                velocities=velocities,
                move_time=1000.0 / rate_hz,
            )
        print(
            f"{name:13s} target {logical_label(target)} | "
            f"{pulse_delta_label(base_pulses, target_pulses)}"
        )
        feedback_pulses, feedback_logical = maybe_read_feedback(robot, axis_mapper, base_pulses)
        if feedback_pulses is not None:
            feedback_text = (
                "feedback logical unavailable"
                if feedback_logical is None
                else logical_label(feedback_logical)
            )
            print(
                f"{'':13s} feedback {feedback_text} | "
                f"{pulse_delta_label(base_pulses, feedback_pulses)}"
            )
        return list(target)

    steps = max(1, int(round(duration_s * rate_hz)))
    interval = 1.0 / rate_hz
    move_time_ms = interval * 1000.0
    next_call = time.perf_counter()
    next_feedback = 0.0
    prev_command = list(previous)

    for index in range(steps + 1):
        command = blend(previous, target, index / steps)
        target_pulses = axis_mapper.logical_to_pulses(base_pulses, command)
        logical_velocity = axis_mapper.diff_velocity(prev_command, command, interval)
        velocities = axis_mapper.logical_velocity_to_pulses_per_ms(logical_velocity)
        prev_command = command

        if robot is not None:
            robot.move_pvt_stream(
                target_pulses=target_pulses,
                velocities=velocities,
                move_time=move_time_ms,
            )

        now = time.perf_counter()
        if index == 0 or index == steps or now >= next_feedback:
            feedback_pulses, feedback_logical = maybe_read_feedback(robot, axis_mapper, base_pulses)
            print(
                f"{name:13s} target {logical_label(command)} | "
                f"{pulse_delta_label(base_pulses, target_pulses)}"
            )
            if feedback_pulses is not None:
                feedback_text = (
                    "feedback logical unavailable"
                    if feedback_logical is None
                    else logical_label(feedback_logical)
                )
                print(
                    f"{'':13s} feedback {feedback_text} | "
                    f"{pulse_delta_label(base_pulses, feedback_pulses)}"
                )
            next_feedback = now + 1.0 / feedback_hz

        next_call += interval
        sleep_time = next_call - time.perf_counter()
        if sleep_time > 0.0:
            time.sleep(sleep_time)
        else:
            next_call = time.perf_counter()

    return list(target)


def validate_args(args: argparse.Namespace) -> list[int]:
    if args.rate_hz <= 0.0 or args.feedback_hz <= 0.0:
        raise ValueError("--rate-hz and --feedback-hz must be positive.")
    if args.ramp_s < 0.0 or args.hold_s < 0.0 or args.zero_hold_s < 0.0:
        raise ValueError("Durations must be non-negative.")
    if not 0.0 < abs(args.alpha_rad) <= 1.0:
        raise ValueError("--alpha-rad must be within (0, 1] rad for this diagnostic.")
    if not 0.0 <= abs(args.d_mm) <= 10.0:
        raise ValueError("--d-mm must be within [0, 10] mm for this diagnostic.")
    return parse_axes(args.axes)


def main() -> None:
    args = parse_args()
    try:
        axes = validate_args(args)
    except ValueError as exc:
        raise SystemExit(f"error: {exc}") from None

    pmac_cfg = PMACConfig(ip=args.pmac_ip)
    axis_mapper = ContinuumAxisMapper(
        pulses_per_rad=pmac_cfg.pulses_per_rad,
        pulses_per_meter=pmac_cfg.pulses_per_meter,
        axis_order=pmac_cfg.axis_order,
        axis_signs=pmac_cfg.axis_signs,
    )

    robot = PMACRobotController(pmac_cfg) if args.execute else None
    if robot is not None:
        robot.safe_boot_and_home()
        base_pulses = robot.base_positions.copy()
    else:
        base_pulses = [0, 0, 0, 0, 0]

    mode = "EXECUTE" if args.execute else "DRY-RUN"
    print(f"Logical axis diagnostic [{mode}]")
    print(f"Base pulses: {base_pulses}")
    print(
        "Logical to physical mapping: "
        + ", ".join(
            f"{LOGICAL_AXIS_NAMES[i]} -> PMAC #{axis_mapper.axis_order[i] + 1} sign {axis_mapper.axis_signs[i]:+d}"
            for i in range(5)
        )
    )
    print(
        f"Amplitudes: alpha={abs(args.alpha_rad):.3f} rad "
        f"({np.degrees(abs(args.alpha_rad)):.1f} deg output), d={abs(args.d_mm):.2f} mm"
    )

    current = [0.0, 0.0, 0.0, 0.0, 0.0]
    try:
        for axis in axes:
            for sign, label in ((1.0, "+"), (-1.0, "-")):
                target = logical_target(
                    axis=axis,
                    signed_alpha_rad=sign * abs(args.alpha_rad),
                    signed_d_m=sign * abs(args.d_mm) / 1000.0,
                )
                phase = f"{label}{LOGICAL_AXIS_NAMES[axis]}"
                current = run_phase(
                    name=f"ramp {phase}",
                    robot=robot,
                    axis_mapper=axis_mapper,
                    base_pulses=base_pulses,
                    previous=current,
                    target=target,
                    duration_s=args.ramp_s,
                    rate_hz=args.rate_hz,
                    feedback_hz=args.feedback_hz,
                )
                current = run_phase(
                    name=f"hold {phase}",
                    robot=robot,
                    axis_mapper=axis_mapper,
                    base_pulses=base_pulses,
                    previous=current,
                    target=target,
                    duration_s=args.hold_s,
                    rate_hz=args.rate_hz,
                    feedback_hz=args.feedback_hz,
                )
                current = run_phase(
                    name="return 0",
                    robot=robot,
                    axis_mapper=axis_mapper,
                    base_pulses=base_pulses,
                    previous=current,
                    target=[0.0, 0.0, 0.0, 0.0, 0.0],
                    duration_s=args.ramp_s,
                    rate_hz=args.rate_hz,
                    feedback_hz=args.feedback_hz,
                )
                current = run_phase(
                    name="zero hold",
                    robot=robot,
                    axis_mapper=axis_mapper,
                    base_pulses=base_pulses,
                    previous=current,
                    target=[0.0, 0.0, 0.0, 0.0, 0.0],
                    duration_s=args.zero_hold_s,
                    rate_hz=args.rate_hz,
                    feedback_hz=args.feedback_hz,
                )
    except KeyboardInterrupt:
        print("\nInterrupted; returning to logical zero.")
        run_phase(
            name="return 0",
            robot=robot,
            axis_mapper=axis_mapper,
            base_pulses=base_pulses,
            previous=current,
            target=[0.0, 0.0, 0.0, 0.0, 0.0],
            duration_s=args.ramp_s,
            rate_hz=args.rate_hz,
            feedback_hz=args.feedback_hz,
        )
    finally:
        if robot is not None:
            robot.close()


if __name__ == "__main__":
    main()
