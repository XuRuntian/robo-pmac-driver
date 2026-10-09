"""Small axis-5 PVT response test. Default: offline preview, no controller imports.

Execution performs the existing PLC reset/start lifecycle. Stop the regular
driver first. Ctrl+C aborts through controller.close(), never returns automatically.
This tests a specified trajectory, not an automatic search for a hardware limit.
"""

from __future__ import annotations

import argparse
from contextlib import ExitStack
import csv
import json
import math
from pathlib import Path
import signal
import time

from continuum_sdk.control.axis_mapper import ContinuumAxisMapper
from pmac_sdk.core.config_model import PMACConfig

DT = 0.02
STEP_CAP = 3000
VELOCITY_CAP = 400  # ct/ms; observed controller value, not a hardware rating


def segment_extrema(p0, p1, v0, v1, duration_ms):
    """Exact extrema of the cubic PVT interpolant in counts and milliseconds."""
    a = 2 * (p0 - p1) + duration_ms * (v0 + v1)
    b = 3 * (p1 - p0) - duration_ms * (2 * v0 + v1)
    c = duration_ms * v0
    points = [0.0, 1.0]
    if a and 0 < -b / (3 * a) < 1:
        points.append(-b / (3 * a))
    velocity = max(abs((3 * a * s * s + 2 * b * s + c) / duration_ms) for s in points)
    acceleration = max(abs(2 * b), abs(6 * a + 2 * b)) / duration_ms**2
    return velocity, acceleration


def plan(config, base, distance_mm=1.0, peak_mm_s=0.5):
    if not math.isfinite(distance_mm) or not 0 < abs(distance_mm) <= 5:
        raise ValueError("This test requires 0 < abs(distance-mm) <= 5.")
    if not math.isfinite(peak_mm_s) or not 0.1 <= peak_mm_s <= 2.3:
        raise ValueError("This test requires 0.1 <= peak-mm-s <= 2.3.")
    mapper = ContinuumAxisMapper(
        config.pulses_per_rad,
        config.pulses_per_meter,
        config.axis_order,
        config.axis_signs,
    )
    if sorted(mapper.axis_order) != list(range(5)) or mapper.axis_order[4] != 4:
        raise ValueError("Expected logical d mapped to physical motor 5.")
    if any(s not in (-1, 1) for s in mapper.axis_signs):
        raise ValueError("Invalid axis signs.")
    steps = math.ceil(1.875 * abs(distance_mm) / peak_mm_s / DT)
    duration = steps * DT
    rows = []
    # The initial hold primes the queue without moving; tail holds give the
    # unchanged 5-point queue time to consume the return trajectory.
    for phase, start, end, count in [
        ("prime", 0.0, 0.0, 50),
        ("out", 0.0, distance_mm, steps),
        ("hold", distance_mm, distance_mm, 100),
        ("return", distance_mm, 0.0, steps),
        ("settle", 0.0, 0.0, 100),
    ]:
        for j in range(1, count + 1):
            u = j / count
            h = 10 * u**3 - 15 * u**4 + 6 * u**5
            dh = 30 * u * u * (1 - u) ** 2
            d = (start + (end - start) * h) / 1000
            velocity = (end - start) * dh / (count * DT) / 1000
            p = mapper.logical_to_pulses(base, [0.0, 0.0, 0.0, 0.0, d])
            v = mapper.logical_velocity_to_pulses_per_ms([0.0, 0.0, 0.0, 0.0, velocity])
            # Match controller API truncation of velocity in wire units.
            v = [int(x * 10000) / 10000 for x in v]
            rows.append(
                dict(
                    phase=phase,
                    planned_s=(len(rows) + 1) * DT,
                    positions=p,
                    velocities=v,
                )
            )
    previous = list(base)
    previous_v = [0.0] * 5
    max_step = max_v = max_a = 0.0
    for row in rows:
        p, v = row["positions"], row["velocities"]
        if p[:4] != list(base[:4]):
            raise ValueError("Non-test motor target changed.")
        if any(not -(2**31) <= x < 2**31 for x in p):
            raise ValueError("Position exceeds wire int32 range.")
        max_step = max(max_step, abs(p[4] - previous[4]))
        sv, sa = segment_extrema(previous[4], p[4], previous_v[4], v[4], DT * 1000)
        max_v = max(max_v, sv)
        max_a = max(max_a, sa)
        previous, previous_v = p, v
    if max_step > STEP_CAP or max_v > VELOCITY_CAP:
        raise ValueError("Trajectory exceeds the recorded axis-5 PVT limits.")
    return rows, dict(
        distance_mm=distance_mm,
        requested_peak_mm_s=peak_mm_s,
        ramp_s=duration,
        total_s=len(rows) * DT,
        max_step_counts=max_step,
        interpolant_peak_counts_ms=max_v,
        interpolant_peak_mm_s=max_v * 1e6 / config.pulses_per_meter,
        interpolant_peak_accel_mm_s2=max_a * 1e9 / config.pulses_per_meter,
        axis_order=mapper.axis_order,
        axis_signs=mapper.axis_signs,
    )


def arguments(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--execute", action="store_true")
    p.add_argument("--distance-mm", type=float, default=1.0)
    p.add_argument("--peak-mm-s", type=float, default=0.5)
    p.add_argument(
        "--csv", type=Path, required=True, help="New output file; never overwritten."
    )
    p.add_argument("--pmac-ip", default="192.168.0.200")
    p.add_argument(
        "--confirmed-clearance-mm",
        type=float,
        help="User-verified available travel from current position in the chosen logical d direction.",
    )
    return p.parse_args(argv)


def run(args, controller_factory=None):
    cfg = PMACConfig(ip=args.pmac_ip)
    frames, summary = plan(cfg, [0] * 5, args.distance_mm, args.peak_mm_s)
    if args.execute and (
        args.confirmed_clearance_mm is None
        or not math.isfinite(args.confirmed_clearance_mm)
        or args.confirmed_clearance_mm < abs(args.distance_mm)
    ):
        raise ValueError(
            "Execution requires user-verified clearance covering the planned excursion."
        )
    args.csv.parent.mkdir(parents=True, exist_ok=True)
    with ExitStack() as stack:
        log = stack.enter_context(args.csv.open("x", newline="", encoding="utf-8"))
        robot = None
        if args.execute:
            from pmac_sdk.core.waiting import WindowsHighResolutionWait

            timer = WindowsHighResolutionWait()
            stack.callback(timer.close)
            if controller_factory is None:
                from pmac_sdk.controller.robot_api import PMACRobotController

                controller_factory = PMACRobotController
            robot = controller_factory(cfg)
            stack.callback(robot.close)
            robot.protocol.sleep_fn = timer.sleep
            robot.protocol.trace_enabled = True
            robot.safe_boot_and_home()
            base = list(robot.base_positions)
            frames, summary = plan(cfg, base, args.distance_mm, args.peak_mm_s)
            if summary["max_step_counts"] > robot.pvt_axis5_max_step:
                raise ValueError("Plan exceeds the live controller step limit.")
            sleeper = timer.sleep
        print(("EXECUTE" if robot else "DRY-RUN") + " | logical d -> physical 5")
        print(json.dumps(summary, indent=2))
        log.write(
            "# "
            + json.dumps(
                dict(
                    execute=args.execute,
                    control_hz=50,
                    **summary,
                    note="Target minus feedback includes buffering; it is not servo following error. "
                    "Preview feedback is empty. No limits are raised by this tool.",
                )
            )
            + "\n"
        )
        fields = [
            "phase",
            "planned_s",
            "t_s",
            "period_ms",
            "submit_ms",
            "queued_at_ack",
            "feedback_start_s",
            "feedback_end_s",
            "feedback_ms",
            "read_axis5_error_abs_counts",
        ]
        fields += [
            f"{prefix}_p{i}"
            for prefix in ("target", "velocity", "feedback")
            for i in range(1, 6)
        ]
        writer = csv.DictWriter(log, fieldnames=fields)
        writer.writeheader()
        start = deadline = time.perf_counter()
        previous = None
        next_feedback = start
        final_feedback = None
        for index, frame in enumerate(frames):
            now = time.perf_counter()
            row = dict(phase=frame["phase"], planned_s=frame["planned_s"])
            row.update(
                {f"target_p{i + 1}": v for i, v in enumerate(frame["positions"])}
            )
            row.update(
                {f"velocity_p{i + 1}": v for i, v in enumerate(frame["velocities"])}
            )
            if robot:
                row.update(
                    t_s=now - start,
                    period_ms=None if previous is None else (now - previous) * 1000,
                )
                robot.move_pvt_stream(
                    frame["positions"], frame["velocities"], DT * 1000
                )
                row["submit_ms"] = (time.perf_counter() - now) * 1000
                row["queued_at_ack"] = robot.protocol.last_pvt_status.queued
                if now >= next_feedback or index == len(frames) - 1:
                    lo = time.perf_counter()
                    feedback, homing = robot.protocol.snapshot()
                    hi = time.perf_counter()
                    if not any(feedback):
                        raise RuntimeError("Invalid all-zero feedback.")
                    row.update(
                        feedback_start_s=lo - start,
                        feedback_end_s=hi - start,
                        feedback_ms=(hi - lo) * 1000,
                        read_axis5_error_abs_counts=homing[3],
                    )
                    row.update(
                        {f"feedback_p{i + 1}": v for i, v in enumerate(feedback)}
                    )
                    next_feedback = now + 0.1
                    final_feedback = feedback
                previous = now
            writer.writerow(row)
            if index % 50 == 0:
                log.flush()
            if robot:
                deadline += DT
                remaining = deadline - time.perf_counter()
                if remaining > 0:
                    sleeper(remaining)
                else:
                    deadline = time.perf_counter()
        if robot:
            print(
                "Final feedback minus startup (counts):",
                [a - b for a, b in zip(final_feedback, base)],
            )
            print(
                "Trajectory submitted; inspect recorded feedback to assess completion."
            )
        else:
            print("Preview saved; no connection, motion, or real-time waiting.")


def shutdown(signum, frame):
    raise KeyboardInterrupt


if __name__ == "__main__":
    signal.signal(signal.SIGTERM, shutdown)
    try:
        run(arguments())
    except KeyboardInterrupt:
        print("Interrupted; controller cleanup requested. No automatic return motion.")
