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


def plan(
    config,
    base,
    distance_mm=1.0,
    peak_mm_s=0.5,
    *,
    preview_unlimited=False,
    move_duration_s=None,
    step_cap_counts=STEP_CAP,
    accel_mm_s2=None,
    profile_events=False,
    repeat_reversals=False,
):
    if repeat_reversals and (not profile_events or accel_mm_s2 != 5):
        raise ValueError("Repeated reversals require --profile-events and acceleration 5.")
    if profile_events:
        if (abs(distance_mm) != 5 or peak_mm_s != 4 or accel_mm_s2 not in (5, 6, 7.5)
                or step_cap_counts != 6500 or preview_unlimited or move_duration_s is not None):
            raise ValueError("Profile events require 5 mm, 4 mm/s, accel 5, 6 or 7.5 mm/s^2, cap 6500, no duration/unlimited override.")
        return plan_profile_events(config, base, distance_mm, accel_mm_s2, repeat_reversals)
    if step_cap_counts not in (3000, 4000, 6500):
        raise ValueError(
            "step-cap-counts must be 3000, 4000 or 6500 for this experiment."
        )
    distance_ceiling = 15 if step_cap_counts == 6500 else 5
    if not math.isfinite(distance_mm) or not 0 < abs(distance_mm) <= distance_ceiling:
        raise ValueError(
            f"This test requires 0 < abs(distance-mm) <= {distance_ceiling}."
        )
    speed_ceiling = {3000: 2.3, 4000: 3.0, 6500: 5.0}[step_cap_counts]
    if preview_unlimited:
        speed_ceiling = max(speed_ceiling, 3.0)
    if not math.isfinite(peak_mm_s) or not 0.1 <= peak_mm_s <= speed_ceiling:
        raise ValueError(f"This test requires 0.1 <= peak-mm-s <= {speed_ceiling}.")
    if accel_mm_s2 is not None and (
        accel_mm_s2 not in (1.95, 5.0)
        or step_cap_counts != 6500
        or peak_mm_s != 4.0
        or abs(distance_mm) != 15.0
        or move_duration_s is not None
    ):
        raise ValueError(
            "Acceleration comparison requires 15 mm, 4 mm/s, step cap 6500, "
            "accel-mm-s2 1.95 or 5, and no move-duration-s."
        )
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
    if move_duration_s is not None:
        if not math.isfinite(move_duration_s) or not 0 < move_duration_s <= 120:
            raise ValueError("move-duration-s must be finite, positive and <= 120.")
        if move_duration_s < 1.875 * abs(distance_mm) / peak_mm_s:
            raise ValueError("move-duration-s would exceed the requested peak speed.")
        steps = math.ceil(move_duration_s / DT)
    duration = steps * DT
    ramp_time = cruise_time = actual_speed = None
    if accel_mm_s2 is not None:
        # Same distance and actual peak speed in both trials. Only ramp/cruise
        # allocation changes. Cosine velocity ramps have zero endpoint acceleration.
        ramp_steps = math.ceil(math.pi * peak_mm_s / (2 * accel_mm_s2) / DT)
        cruise_steps = math.ceil(
            max(0.0, abs(distance_mm) / peak_mm_s / DT - ramp_steps)
        )
        ramp_time, cruise_time = ramp_steps * DT, cruise_steps * DT
        actual_speed = abs(distance_mm) / (ramp_time + cruise_time)
        steps = 2 * ramp_steps + cruise_steps
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
            if accel_mm_s2 is not None and phase in ("out", "return"):
                t = j * DT
                if t <= ramp_time:
                    travel = (
                        actual_speed
                        / 2
                        * (t - ramp_time / math.pi * math.sin(math.pi * t / ramp_time))
                    )
                    speed = actual_speed / 2 * (1 - math.cos(math.pi * t / ramp_time))
                elif t <= ramp_time + cruise_time:
                    travel = actual_speed * (ramp_time / 2 + t - ramp_time)
                    speed = actual_speed
                else:
                    td = min(ramp_time, t - ramp_time - cruise_time)
                    travel = actual_speed * (
                        ramp_time / 2 + cruise_time
                    ) + actual_speed / 2 * (
                        td + ramp_time / math.pi * math.sin(math.pi * td / ramp_time)
                    )
                    speed = actual_speed / 2 * (1 + math.cos(math.pi * td / ramp_time))
                direction = 1 if end > start else -1
                d = (start + direction * travel) / 1000
                velocity = direction * speed / 1000
                if j == count:
                    d, velocity = end / 1000, 0.0
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
    within_limits = max_step <= STEP_CAP and max_v <= VELOCITY_CAP
    # The 5 mm/s experiment must not introduce a large acceleration increase.
    # Check the quantized cubic interpolant, not just the analytic quintic.
    accel_ceiling = 6.0 if accel_mm_s2 == 5.0 else 3.1
    if (
        step_cap_counts == 6500
        and max_a * 1e9 / config.pulses_per_meter > accel_ceiling
    ):
        raise ValueError(
            f"Experimental interpolant acceleration exceeds {accel_ceiling} mm/s^2; "
            "extend move-duration-s or use the planned 15 mm excursion."
        )
    if (max_step > step_cap_counts or max_v > VELOCITY_CAP) and not preview_unlimited:
        raise ValueError("Trajectory exceeds the selected axis-5 PVT limits.")
    return rows, dict(
        distance_mm=distance_mm,
        preview_only=preview_unlimited,
        within_recorded_limits=within_limits,
        recorded_step_cap_counts=STEP_CAP,
        selected_step_cap_counts=step_cap_counts,
        recorded_velocity_cap_counts_ms=VELOCITY_CAP,
        minimum_step_cap_counts=math.ceil(max_step),
        requested_peak_mm_s=peak_mm_s,
        trajectory_profile="cosine_velocity"
        if accel_mm_s2 is not None
        else "quintic_position",
        requested_accel_mm_s2=accel_mm_s2,
        acceleration_ramp_s=ramp_time,
        cruise_s=cruise_time,
        ramp_s=duration,
        total_s=len(rows) * DT,
        max_step_counts=max_step,
        interpolant_peak_counts_ms=max_v,
        interpolant_peak_mm_s=max_v * 1e6 / config.pulses_per_meter,
        interpolant_peak_accel_mm_s2=max_a * 1e9 / config.pulses_per_meter,
        axis_order=mapper.axis_order,
        axis_signs=mapper.axis_signs,
    )


def plan_profile_events(config, base, distance_mm, accel_mm_s2, repeat_reversals=False):
    """Bounded target changes using the same candidate as the driver, offline first."""
    from continuum_sdk.control.insertion_profile import InsertionProfile

    mapper = ContinuumAxisMapper(config.pulses_per_rad, config.pulses_per_meter,
                                 config.axis_order, config.axis_signs)
    if sorted(mapper.axis_order) != list(range(5)) or mapper.axis_order[4] != 4:
        raise ValueError("Expected logical d mapped to physical motor 5.")
    if any(s not in (-1, 1) for s in mapper.axis_signs):
        raise ValueError("Invalid axis signs.")
    profile = InsertionProfile(base[4], DT, .004*config.pulses_per_meter,
                               accel_mm_s2/1000*config.pulses_per_meter, .004*config.pulses_per_meter*DT)
    rows = []
    previous, previous_v = base[4], 0.
    max_step = max_v = max_a = 0.
    # Reverse while moving, settle; accelerate again, hold while moving,
    # then explicitly return. Aborts skip all remaining phases.
    events = [('prime', 0., 1., True), ('out', distance_mm, 1., False),
              ('reverse', 0., 6., False), ('rest', 0., 1., False),
              ('out_hold', distance_mm, 1., False), ('brake_hold', distance_mm, 2., True),
              ('return', 0., 6., False), ('settle', 0., 2., False)]
    if repeat_reversals:
        events = [('prime', 0., 1., True)]
        for cycle in range(1, 7):
            events += [(f'out_{cycle}', distance_mm, 1., False),
                       (f'reverse_{cycle}', 0., 1., False)]
        events += [('brake_hold', 0., 2., True), ('return', 0., 6., False),
                   ('settle', 0., 2., False)]
    for phase, requested_mm, duration, holding in events:
        requested = mapper.logical_to_pulses(base, [0., 0., 0., 0., requested_mm/1000])
        for _ in range(round(duration/DT)):
            seg = profile.step(requested[4], holding=holding)
            positions = list(base)
            positions[4] = seg.position
            velocities = [0.] * 5
            velocities[4] = seg.velocity_counts_ms
            sv, sa = segment_extrema(previous, seg.position, previous_v, seg.velocity_counts_ms, DT*1000)
            max_step = max(max_step, abs(seg.position-previous))
            max_v, max_a = max(max_v, sv), max(max_a, sa)
            logical = mapper.pulses_to_logical(base, positions)
            if not -.0000001 <= logical[4]*(1 if distance_mm > 0 else -1) <= abs(distance_mm)/1000+.0000001:
                raise ValueError("Profile event trajectory exceeds confirmed one-sided excursion.")
            if any(not -(2**31) <= p < 2**31 for p in positions):
                raise ValueError("Position exceeds wire int32 range.")
            rows.append(dict(phase=phase, planned_s=(len(rows)+1)*DT,
                             positions=positions, velocities=velocities,
                             requested_d_mm=requested_mm, profile_holding=holding))
            previous, previous_v = seg.position, seg.velocity_counts_ms
    if (max_step > 6500 or max_v*1e6/config.pulses_per_meter > 4+1e-9
            or max_a*1e9/config.pulses_per_meter > accel_mm_s2+1e-9):
        raise ValueError("Candidate exceeds quantized PVT budgets.")
    if rows[-1]['positions'] != list(base) or rows[-1]['velocities'] != [0.]*5:
        raise ValueError("Candidate did not return and settle within the planned time.")
    return rows, dict(distance_mm=distance_mm, preview_only=False,
                     trajectory_profile='insertion_profile_events', selected_step_cap_counts=6500,
                     requested_peak_mm_s=4., requested_accel_mm_s2=accel_mm_s2, total_s=len(rows)*DT,
                     max_step_counts=max_step, interpolant_peak_counts_ms=max_v,
                     interpolant_peak_mm_s=max_v*1e6/config.pulses_per_meter,
                     interpolant_peak_accel_mm_s2=max_a*1e9/config.pulses_per_meter,
                     axis_order=mapper.axis_order, axis_signs=mapper.axis_signs,
                     repeat_reversals=repeat_reversals,
                     event_note=('Six pairs of 1 s target changes; then hold and return. Abort never returns.'
                                 if repeat_reversals else
                                 'Reverse after 1 s; later hold after 1 s. Normal completion returns; abort never returns.'))


def arguments(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--execute", action="store_true")
    p.add_argument("--repeat-reversals", action="store_true",
                   help="Six pairs of 1 s target changes, then hold and return; requires profile-events, accel 5.")
    p.add_argument("--profile-events", action="store_true",
                   help="Candidate generator: moving reversal and hold; requires 5 mm, 4 mm/s, accel 5, 6 or 7.5, cap 6500.")
    p.add_argument("--distance-mm", type=float, default=1.0)
    p.add_argument("--peak-mm-s", type=float, default=0.5)
    p.add_argument(
        "--accel-mm-s2",
        type=float,
        choices=(1.95, 5.0, 6.0, 7.5),
        help="Matched 15 mm / 4 mm/s cosine-ramp comparison, or fixed 5 mm "
        "candidate test with --profile-events; requires cap 6500. Error threshold unchanged.",
    )
    p.add_argument(
        "--step-cap-counts",
        type=int,
        choices=(3000, 4000, 6500),
        default=3000,
        help="Expected controller budget; does NOT write it. Must match live readback.",
    )
    p.add_argument(
        "--stop-error-mm",
        type=float,
        default=0.05,
        help="Abort on sampled absolute following error >= this value (0 < value <= 0.05). "
        "Host sampling is ~10 Hz and does not replace controller protection.",
    )
    p.add_argument(
        "--preview-unlimited",
        action="store_true",
        help="Offline only: preview beyond the selected step/velocity limits; cannot execute. "
        "The selected experiment's speed, distance and acceleration ceilings still apply.",
    )
    p.add_argument(
        "--move-duration-s",
        type=float,
        help="Optional duration of EACH outbound/return move, rounded up to 20 ms. "
        "Must not exceed the requested peak speed; reports resulting acceleration.",
    )
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
    if args.execute and args.preview_unlimited:
        raise ValueError("preview-unlimited cannot be combined with execute.")
    if not math.isfinite(args.stop_error_mm) or not 0 < args.stop_error_mm <= 0.05:
        raise ValueError("stop-error-mm must be finite and in (0, 0.05].")
    cfg = PMACConfig(ip=args.pmac_ip)
    plan_options = dict(
        preview_unlimited=args.preview_unlimited,
        move_duration_s=args.move_duration_s,
        step_cap_counts=args.step_cap_counts,
        accel_mm_s2=args.accel_mm_s2,
        profile_events=args.profile_events,
        repeat_reversals=args.repeat_reversals,
    )
    frames, summary = plan(
        cfg, [0] * 5, args.distance_mm, args.peak_mm_s, **plan_options
    )
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
            frames, summary = plan(
                cfg, base, args.distance_mm, args.peak_mm_s, **plan_options
            )
            if robot.pvt_axis5_max_step != args.step_cap_counts:
                raise ValueError(
                    "Live controller step limit does not match --step-cap-counts; "
                    "no test trajectory submitted, no limit changed."
                )
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
                    stop_error_mm=args.stop_error_mm,
                    feedback_period_s=0.1,
                    live_step_cap_counts=robot.pvt_axis5_max_step if robot else None,
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
            "stop_reason",
            "requested_d_mm",
            "profile_holding",
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
            if "requested_d_mm" in frame:
                row.update(requested_d_mm=frame["requested_d_mm"], profile_holding=frame["profile_holding"])
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
            if robot and "read_axis5_error_abs_counts" in row:
                error = row["read_axis5_error_abs_counts"]
                if (
                    error < 0
                    or error * 1000 / cfg.pulses_per_meter >= args.stop_error_mm
                ):
                    # Write the triggering sample before unwinding to robot.close().
                    # No return trajectory is sent after this condition.
                    row["stop_reason"] = (
                        "invalid_error_feedback"
                        if error < 0
                        else "following_error_threshold"
                    )
                    writer.writerow(row)
                    log.flush()
                    raise RuntimeError(
                        f"Axis 5 stopped: {row['stop_reason']}; "
                        f"sampled error={error} counts. No automatic return."
                    )
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
