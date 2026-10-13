from __future__ import annotations

import argparse
from contextlib import ExitStack
from dataclasses import replace
import csv
import json
import math
import signal
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np
import zmq

from continuum_sdk.control.axis_mapper import ContinuumAxisMapper
from continuum_sdk.control.live_diagnostic import LiveDiagnostic, diagnostic_lines
from continuum_sdk.control.cartesian_frame import apply_axis_transform
from continuum_sdk.control.pvt_mapper import ContinuumPVTMapper
from continuum_sdk.control.tip_command_filter import TipCommandFilter
from continuum_sdk.control.ik_limit_monitor import IKLimitMonitor
from continuum_sdk.control.insertion_profile import InsertionProfile
from continuum_sdk.core.config_loader import load_continuum_config
from continuum_sdk.core.factory import build_continuum_ik, build_tendon_mapper
from continuum_sdk.core.interface_config import load_robot_interface_config
from continuum_sdk.kinematics.dls_ik import rotvec_to_matrix
from continuum_sdk.kinematics.joint_motor_model import MotorAngles
from continuum_sdk.transport.zmq_protocol import (
    build_state_message,
    parse_control_message,
)
from pmac_sdk.controller.robot_api import PMACRobotController
from pmac_sdk.core.config_model import PMACConfig
from pmac_sdk.controller.protocol import TransactionTiming
from pmac_sdk.core.waiting import WindowsHighResolutionWait


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run the fixed-rate PMAC continuum driver and expose a ZMQ control boundary."
    )
    parser.add_argument("--config", default="config/continuum.yaml")
    parser.add_argument("--live-debug", action="store_true", help="Refresh a fixed four-line IK/axis diagnostic at 4 Hz; interactive terminal only.")
    parser.add_argument("--latency-csv", help="Optional per-cycle latency trace; exclusive-create, no extra PMAC reads.")
    parser.add_argument("--wait-mode", choices=("system", "windows-highres"), default="system",
                        help="Wait backend for protocol polling and main loop; default keeps existing behavior.")
    parser.add_argument("--interface-config", default="config/robot_interface.yaml")
    parser.add_argument("--pmac-ip", default="192.168.0.200")
    parser.add_argument("--bind-host", default="127.0.0.1")
    parser.add_argument("--command-port", type=int, default=5555)
    parser.add_argument("--state-port", type=int, default=5556)
    parser.add_argument(
        "--watchdog-timeout",
        type=float,
        default=0.2,
        help="Freeze the current commanded pose after this many seconds without a command.",
    )
    parser.add_argument(
        "--feedback-hz",
        type=float,
        default=10.0,
        help="PMAC encoder feedback read rate. State messages are still published at control_hz.",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Connect to PMAC and execute motion. Without this flag the service is a dry-run simulator.",
    )
    parser.add_argument(
        "--return-to-reference-on-start",
        action="store_true",
        help=(
            "Before accepting teleoperation commands, move from the startup feedback "
            "position to initial_position.reference_pulses using a fixed-rate PVT ramp."
        ),
    )
    parser.add_argument(
        "--return-duration",
        type=float,
        default=8.0,
        help="Seconds used by --return-to-reference-on-start.",
    )
    parser.add_argument(
        "--return-check-tolerance-pulses",
        type=int,
        default=0,
        help=(
            "If positive, read feedback after startup return and fail when any axis "
            "is farther than this many pulses from the configured reference."
        ),
    )
    parser.add_argument(
        "--shape-debug-hz",
        type=float,
        default=0.0,
        help=(
            "Print proximal/distal shape diagnostics at this rate. "
            "0 disables terminal diagnostic output."
        ),
    )
    parser.add_argument(
        "--shape-debug-csv",
        default="",
        help="Optional CSV path for proximal/distal shape diagnostics.",
    )
    parser.add_argument(
        "--motor-debug-hz",
        type=float,
        default=0.0,
        help=(
            "Print compact motor target/feedback pulse diagnostics at this rate. "
            "0 disables terminal diagnostic output."
        ),
    )
    parser.add_argument(
        "--motor-debug-min-tip-mm",
        type=float,
        default=0.5,
        help="Suppress motor diagnostics while the dominant applied tip delta is below this value.",
    )
    parser.add_argument(
        "--motor-debug-min-rotation-rad",
        type=float,
        default=0.01,
        help="Suppress motor diagnostics while the dominant applied tip rotation is below this value.",
    )
    parser.add_argument(
        "--motor-debug-change-pulses",
        type=int,
        default=1000,
        help="Suppress repeated motor diagnostics until target pulses change by at least this much.",
    )
    parser.add_argument(
        "--lock-linear-axis",
        action="store_true",
        help=(
            "Keep the physical insertion/linear axis at the startup base pulse. "
            "Useful for checking whether lateral commands are coupled through d/M5."
        ),
    )
    return parser.parse_args()


def _latest_message(socket: zmq.Socket) -> dict[str, Any] | None:
    latest = None
    while True:
        try:
            latest = socket.recv_json(flags=zmq.NOBLOCK)
        except zmq.Again:
            return latest


def _state_from_feedback(
    axis_mapper: ContinuumAxisMapper,
    base_pulses: list[int],
    feedback_pulses: list[int],
) -> dict[str, float]:
    logical = axis_mapper.pulses_to_logical(base_pulses, feedback_pulses)
    return {
        "axis_1_pos": logical[0],
        "axis_2_pos": logical[1],
        "axis_3_pos": logical[2],
        "axis_4_pos": logical[3],
        "axis_5_pos": logical[4],
    }


def _smoothstep(value: float) -> float:
    return value * value * (3.0 - 2.0 * value)


def _return_to_reference_pvt(
    robot: PMACRobotController,
    *,
    start_pulses: list[int],
    reference_pulses: list[int],
    update_hz: int,
    duration_s: float,
    check_tolerance_pulses: int,
) -> list[int]:
    if duration_s <= 0.0:
        raise ValueError("--return-duration must be positive.")
    if update_hz <= 0:
        raise ValueError("update_hz must be positive.")

    steps = max(1, int(round(duration_s * update_hz)))
    move_time_ms = 1000.0 / update_hz
    start = np.asarray(start_pulses, dtype=float)
    reference = np.asarray(reference_pulses, dtype=float)
    if 1.5 * abs(reference[4] - start[4]) / steps > robot.pvt_axis5_max_step - 1:
        raise ValueError(
            "Startup return would exceed the PLC axis-5 step limit; increase --return-duration."
        )
    previous = start.copy()
    next_call = time.perf_counter()

    print(
        "Startup return-to-reference enabled | "
        f"duration={duration_s:.2f}s | steps={steps} | reference={reference_pulses}"
    )
    for index in range(1, steps + 1):
        alpha = _smoothstep(index / steps)
        target = np.rint(start + alpha * (reference - start)).astype(int)
        velocities = ((target - previous) / move_time_ms).astype(float).tolist()
        robot.move_pvt_stream(
            target_pulses=target.tolist(),
            velocities=velocities,
            move_time=move_time_ms,
        )
        previous = target.astype(float)

        next_call += 1.0 / update_hz
        sleep_time = next_call - time.perf_counter()
        if sleep_time > 0.0:
            time.sleep(sleep_time)
        else:
            next_call = time.perf_counter()

    # Keep supplying a stationary tail while the priming queue catches up.
    # Acceptance only confirms queue insertion, not physical completion.
    tail_start = time.monotonic()
    tail_next_call = time.perf_counter()
    while True:
        robot.move_pvt_stream(
            target_pulses=reference_pulses,
            velocities=[0.0] * 5,
            move_time=move_time_ms,
        )
        feedback = robot.read_positions()
        if not any(feedback):
            raise RuntimeError("Invalid all-zero feedback during startup return.")
        errors = [actual - expected for actual, expected in zip(feedback, reference_pulses)]
        elapsed = time.monotonic() - tail_start
        if elapsed >= 0.5 and (
            check_tolerance_pulses == 0
            or all(abs(error) <= check_tolerance_pulses for error in errors)
        ):
            print(f"Startup return feedback: {feedback} | errors={errors}")
            return feedback
        if elapsed >= 3.0:
            raise RuntimeError(f"Startup return did not reach tolerance: errors={errors}")
        tail_next_call += 1.0 / update_hz
        remaining = tail_next_call - time.perf_counter()
        if remaining > 0:
            time.sleep(remaining)
        else:
            tail_next_call = time.perf_counter()


def _shape_from_logical_axes(tendon_mapper: Any, logical_axes: list[float]) -> dict[str, float]:
    recovered = tendon_mapper.model.motor_angles_to_joint(
        MotorAngles(
            alpha1=logical_axes[0],
            alpha2=logical_axes[1],
            alpha3=logical_axes[2],
            alpha4=logical_axes[3],
        )
    )
    return {
        "d_m": float(logical_axes[4]),
        "theta_a_rad": float(recovered.theta_a),
        "phi_a_rad": float(recovered.phi_a),
        "theta_c_rad": float(recovered.theta_c),
        "phi_c_rad": float(recovered.phi_c),
    }


def _shape_ratio(theta_a: float, theta_c: float) -> float:
    if abs(theta_a) < 1e-9:
        return float("nan")
    return float(theta_c / theta_a)


def _build_shape_diagnostic(
    *,
    t_s: float,
    state_sequence: int,
    axis_mapper: ContinuumAxisMapper,
    tendon_mapper: Any,
    base_pulses: list[int],
    pvt_command: Any,
    feedback_pulses: list[int],
    applied_action: dict[str, float],
    watchdog_holding: bool,
) -> dict[str, float | int | bool]:
    target_logical = axis_mapper.pulses_to_logical(base_pulses, pvt_command.target_pulses)
    feedback_logical = axis_mapper.pulses_to_logical(base_pulses, feedback_pulses)
    target_shape = _shape_from_logical_axes(tendon_mapper, target_logical)
    feedback_shape = _shape_from_logical_axes(tendon_mapper, feedback_logical)
    pulse_error = [
        int(actual - target)
        for actual, target in zip(feedback_pulses, pvt_command.target_pulses)
    ]
    alpha_error = [
        float(actual - target)
        for actual, target in zip(feedback_logical[:4], target_logical[:4])
    ]
    ik_u = pvt_command.ik_result.u

    row: dict[str, float | int | bool] = {
        "t_s": float(t_s),
        "sequence": int(state_sequence),
        "watchdog_holding": bool(watchdog_holding),
        "applied_x_m": float(applied_action["tip_delta_x"]),
        "applied_y_m": float(applied_action["tip_delta_y"]),
        "applied_z_m": float(applied_action["tip_delta_z"]),
        "ik_d_m": float(ik_u[0]),
        "ik_theta_a_rad": float(ik_u[1]),
        "ik_phi_a_rad": float(ik_u[2]),
        "ik_theta_c_rad": float(ik_u[3]),
        "ik_phi_c_rad": float(ik_u[4]),
        "target_d_m": target_shape["d_m"],
        "target_theta_a_rad": target_shape["theta_a_rad"],
        "target_phi_a_rad": target_shape["phi_a_rad"],
        "target_theta_c_rad": target_shape["theta_c_rad"],
        "target_phi_c_rad": target_shape["phi_c_rad"],
        "feedback_d_m": feedback_shape["d_m"],
        "feedback_theta_a_rad": feedback_shape["theta_a_rad"],
        "feedback_phi_a_rad": feedback_shape["phi_a_rad"],
        "feedback_theta_c_rad": feedback_shape["theta_c_rad"],
        "feedback_phi_c_rad": feedback_shape["phi_c_rad"],
        "target_theta_c_over_a": _shape_ratio(
            target_shape["theta_a_rad"],
            target_shape["theta_c_rad"],
        ),
        "feedback_theta_c_over_a": _shape_ratio(
            feedback_shape["theta_a_rad"],
            feedback_shape["theta_c_rad"],
        ),
        "ik_error_m": float(np.linalg.norm(pvt_command.ik_result.error[:3])),
    }
    for index, value in enumerate(target_logical[:4], start=1):
        row[f"target_alpha{index}_rad"] = float(value)
    for index, value in enumerate(feedback_logical[:4], start=1):
        row[f"feedback_alpha{index}_rad"] = float(value)
    for index, value in enumerate(alpha_error, start=1):
        row[f"alpha{index}_err_rad"] = float(value)
    for index, value in enumerate(pvt_command.target_pulses, start=1):
        row[f"target_p{index}"] = int(value)
    for index, value in enumerate(feedback_pulses, start=1):
        row[f"feedback_p{index}"] = int(value)
    for index, value in enumerate(pulse_error, start=1):
        row[f"pulse_err{index}"] = int(value)
    return row


def _print_shape_diagnostic(row: dict[str, float | int | bool]) -> None:
    rad_to_deg = 180.0 / np.pi
    target_a = float(row["target_theta_a_rad"]) * rad_to_deg
    target_c = float(row["target_theta_c_rad"]) * rad_to_deg
    feedback_a = float(row["feedback_theta_a_rad"]) * rad_to_deg
    feedback_c = float(row["feedback_theta_c_rad"]) * rad_to_deg
    print(
        "shape "
        f"t={float(row['t_s']):.2f}s | "
        f"cmd_xyz_mm=[{float(row['applied_x_m']) * 1000:+.1f}, "
        f"{float(row['applied_y_m']) * 1000:+.1f}, "
        f"{float(row['applied_z_m']) * 1000:+.1f}] | "
        f"theta tgt(a,c)=[{target_a:+.2f}, {target_c:+.2f}]deg "
        f"fb=[{feedback_a:+.2f}, {feedback_c:+.2f}]deg | "
        f"ratio tgt/fb=[{float(row['target_theta_c_over_a']):+.2f}, "
        f"{float(row['feedback_theta_c_over_a']):+.2f}] | "
        f"alpha_err=[{float(row['alpha1_err_rad']):+.4f}, "
        f"{float(row['alpha2_err_rad']):+.4f}, "
        f"{float(row['alpha3_err_rad']):+.4f}, "
        f"{float(row['alpha4_err_rad']):+.4f}]rad"
    )


def _format_pulse_list(values: np.ndarray) -> str:
    return "[" + ",".join(f"{int(value):+d}" for value in values) + "]"


def _dominant_motor_label(values: np.ndarray) -> str:
    index = int(np.argmax(np.abs(values)))
    value = int(values[index])
    return f"M{index + 1} {value:+d}"


def _dominant_tip_label(applied_action: dict[str, float]) -> tuple[str, float]:
    tip = np.asarray(
        [
            applied_action["tip_delta_x"],
            applied_action["tip_delta_y"],
            applied_action["tip_delta_z"],
        ],
        dtype=float,
    )
    index = int(np.argmax(np.abs(tip)))
    value_mm = float(tip[index] * 1000.0)
    sign = "+" if value_mm >= 0.0 else "-"
    return f"{sign}{'XYZ'[index]}", abs(value_mm)


def _dominant_rotation_label(applied_action: dict[str, float]) -> tuple[str, float]:
    rotation = np.asarray(
        [
            applied_action["tip_delta_rx"],
            applied_action["tip_delta_ry"],
            applied_action["tip_delta_rz"],
        ],
        dtype=float,
    )
    index = int(np.argmax(np.abs(rotation)))
    value = float(rotation[index])
    sign = "+" if value >= 0.0 else "-"
    return f"{sign}R{'XYZ'[index]}", abs(value)


def _print_motor_diagnostic(
    *,
    applied_action: dict[str, float],
    base_pulses: list[int],
    last_target_pulses: list[int],
    target_pulses: list[int],
    feedback_pulses: list[int],
) -> None:
    base = np.asarray(base_pulses, dtype=int)
    target = np.asarray(target_pulses, dtype=int)
    feedback = np.asarray(feedback_pulses, dtype=int)
    last_target = np.asarray(last_target_pulses, dtype=int)
    target_delta = target - base
    feedback_delta = feedback - base
    target_step = target - last_target
    tip_label, tip_mm = _dominant_tip_label(applied_action)
    cmd_mm = [
        float(applied_action["tip_delta_x"] * 1000.0),
        float(applied_action["tip_delta_y"] * 1000.0),
        float(applied_action["tip_delta_z"] * 1000.0),
    ]
    cmd_rot = [
        float(applied_action["tip_delta_rx"]),
        float(applied_action["tip_delta_ry"]),
        float(applied_action["tip_delta_rz"]),
    ]
    rotation_label, rotation_rad = _dominant_rotation_label(applied_action)

    print(
        "motor | "
        f"tip {tip_label} {tip_mm:5.1f}mm | "
        f"rot {rotation_label} {rotation_rad:5.3f}rad | "
        f"cmdXYZ=[{cmd_mm[0]:+5.1f},{cmd_mm[1]:+5.1f},{cmd_mm[2]:+5.1f}]mm | "
        f"cmdR=[{cmd_rot[0]:+5.3f},{cmd_rot[1]:+5.3f},{cmd_rot[2]:+5.3f}] | "
        f"dom tgt {_dominant_motor_label(target_delta)} "
        f"fb {_dominant_motor_label(feedback_delta)} "
        f"step {_dominant_motor_label(target_step)} | "
        f"tgtΔ={_format_pulse_list(target_delta)} "
        f"fbΔ={_format_pulse_list(feedback_delta)} "
        f"step={_format_pulse_list(target_step)}"
    )


def _request_shutdown(signum, frame):
    raise KeyboardInterrupt


def main() -> None:
    args = parse_args()
    if args.live_debug and (args.shape_debug_hz > 0 or args.motor_debug_hz > 0):
        raise ValueError("Use --live-debug without --shape-debug-hz or --motor-debug-hz to keep the display fixed.")
    if args.watchdog_timeout <= 0.0:
        raise ValueError("--watchdog-timeout must be positive.")
    if args.feedback_hz <= 0.0:
        raise ValueError("--feedback-hz must be positive.")
    if args.return_duration <= 0.0:
        raise ValueError("--return-duration must be positive.")
    if args.return_check_tolerance_pulses < 0:
        raise ValueError("--return-check-tolerance-pulses must be non-negative.")
    if args.shape_debug_hz < 0.0:
        raise ValueError("--shape-debug-hz must be non-negative.")
    if args.motor_debug_hz < 0.0:
        raise ValueError("--motor-debug-hz must be non-negative.")
    if args.motor_debug_min_tip_mm < 0.0:
        raise ValueError("--motor-debug-min-tip-mm must be non-negative.")
    if args.motor_debug_min_rotation_rad < 0.0:
        raise ValueError("--motor-debug-min-rotation-rad must be non-negative.")
    if args.motor_debug_change_pulses < 0:
        raise ValueError("--motor-debug-change-pulses must be non-negative.")

    continuum_cfg = load_continuum_config(args.config)
    interface_cfg = load_robot_interface_config(args.interface_config)
    print("WORLD → IK:")
    print(f"  translation map={interface_cfg.frame.translation_map} signs={list(interface_cfg.frame.translation_signs)}")
    print(f"  rotation map={interface_cfg.frame.rotation_map} signs={list(interface_cfg.frame.rotation_signs)}")
    if continuum_cfg.control.update_hz != interface_cfg.control_hz:
        raise ValueError(
            "continuum control update_hz and robot interface control_hz must match: "
            f"{continuum_cfg.control.update_hz} != {interface_cfg.control_hz}"
        )

    pmac_cfg = PMACConfig(ip=args.pmac_ip)
    update_hz = interface_cfg.control_hz
    update_interval = 1.0 / update_hz
    # Validate explicit actuator policy before opening the hardware connection.
    max_linear_step_pulses = interface_cfg.insertion_step_limit(
        pmac_cfg.pulses_per_meter, update_interval,
    )
    use_insertion_profile = interface_cfg.insertion_max_accel_m_s2 is not None
    if use_insertion_profile and interface_cfg.insertion_max_speed_m_s is None:
        raise ValueError("Set execution.insertion_max_speed_m_s before enabling the insertion profile.")
    if use_insertion_profile and interface_cfg.insertion_error_stop_m is None:
        raise ValueError("Set execution.insertion_error_stop_m before running the driver.")
    if use_insertion_profile and args.return_to_reference_on_start:
        raise ValueError("Startup return has not been validated with the insertion profile; "
                         "use capture_current without --return-to-reference-on-start.")
    # Validate quantization margins before opening a hardware connection.
    if use_insertion_profile:
        InsertionProfile(0, update_interval,
                         interface_cfg.insertion_max_speed_m_s * pmac_cfg.pulses_per_meter,
                         interface_cfg.insertion_max_accel_m_s2 * pmac_cfg.pulses_per_meter,
                         max_linear_step_pulses)
    feedback_interval = 1.0 / args.feedback_hz

    ik = build_continuum_ik(continuum_cfg)
    if interface_cfg.command.orientation_enabled:
        ik.task_mode = "pos_z"
    else:
        ik.task_mode = "position"
    tendon_mapper = build_tendon_mapper(continuum_cfg)
    axis_mapper = ContinuumAxisMapper(
        pulses_per_rad=pmac_cfg.pulses_per_rad,
        pulses_per_meter=pmac_cfg.pulses_per_meter,
        axis_order=pmac_cfg.axis_order,
        axis_signs=pmac_cfg.axis_signs,
    )

    with ExitStack() as cleanup:
        wait_mode = getattr(args, "wait_mode", "system")
        cycle_sleep = time.sleep
        if wait_mode == "windows-highres":
            wait_backend = WindowsHighResolutionWait()
            cleanup.callback(wait_backend.close)
            cycle_sleep = wait_backend.sleep
        print(f"Wait mode: {wait_mode}")
        latency_file = None
        latency_writer = None
        if getattr(args, "latency_csv", None):
            latency_path = Path(args.latency_csv)
            latency_path.parent.mkdir(parents=True, exist_ok=True)
            latency_file = latency_path.open("x", newline="", encoding="utf-8")
            # Register before the controller so abort/close precedes final log flushing.
            cleanup.callback(latency_file.close)
            latency_file.write("# " + json.dumps({
                "latency_schema": 2,
                "wait_mode": wait_mode,
                "python_version": sys.version,
                "protocol_poll_interval_s": pmac_cfg.poll_interval_s,
                "execute": args.execute, "control_hz": update_hz,
                "feedback_hz": args.feedback_hz,
                "interface_yaml": Path(args.interface_config).read_text(encoding="utf-8"),
                "continuum_yaml": Path(args.config).read_text(encoding="utf-8"),
                "axis_order": pmac_cfg.axis_order, "axis_signs": pmac_cfg.axis_signs,
                "note": "Host monotonic times; feedback request interval bounds acquisition; queued is software queue only.",
            }, ensure_ascii=True) + "\n")
        old_sigterm = signal.signal(signal.SIGTERM, _request_shutdown)
        cleanup.callback(signal.signal, signal.SIGTERM, old_sigterm)
        robot = PMACRobotController(pmac_cfg) if args.execute else None
        if robot is not None:
            cleanup.callback(robot.close)
            if wait_mode == "windows-highres":
                robot.protocol.sleep_fn = cycle_sleep
            if latency_file is not None:
                robot.protocol.trace_enabled = True
        if robot is not None:
            robot.safe_boot_and_home()
            current_pulses = robot.base_positions.copy()
            if interface_cfg.initial_position.reject_all_zero_feedback and not any(current_pulses):
                raise RuntimeError("PMAC returned an invalid all-zero startup position.")

            if args.return_to_reference_on_start:
                if interface_cfg.initial_position.reference_pulses is None:
                    raise ValueError(
                        "--return-to-reference-on-start requires initial_position.reference_pulses."
                    )
                base_pulses = list(interface_cfg.initial_position.reference_pulses)
                current_pulses = _return_to_reference_pvt(
                    robot,
                    start_pulses=current_pulses,
                    reference_pulses=base_pulses,
                    update_hz=update_hz,
                    duration_s=args.return_duration,
                    check_tolerance_pulses=args.return_check_tolerance_pulses,
                )
            else:
                base_pulses = interface_cfg.initial_position.resolve_reference(current_pulses)
            robot.base_positions = base_pulses.copy()
        else:
            base_pulses = list(
                interface_cfg.initial_position.reference_pulses
                or (0, 0, 0, 0, 0)
            )
            current_pulses = base_pulses.copy()
            if args.return_to_reference_on_start:
                print("Dry-run return-to-reference: using configured reference as base pulses.")

        linear_physical_idx = axis_mapper.axis_order[4]
        if interface_cfg.initial_position.insertion_zero_pulses is not None:
            base_pulses[linear_physical_idx] = interface_cfg.initial_position.insertion_zero_pulses
        initial_logical = axis_mapper.pulses_to_logical(base_pulses, current_pulses)
        d_min, d_max = ik.geometry.insertion_bounds()
        if not d_min <= initial_logical[4] <= d_max:
            raise ValueError(
                f"Startup insertion d={initial_logical[4]:.6f} m is outside [{d_min}, {d_max}]; "
                "check axis-5 homing and insertion_zero_pulses."
            )
        if robot is not None:
            robot.base_positions = base_pulses.copy()
        pvt_mapper = ContinuumPVTMapper(
            ik=ik,
            tendon_mapper=tendon_mapper,
            axis_mapper=axis_mapper,
            base_pulses=base_pulses,
            update_interval_s=update_interval,
            max_inner_steps=continuum_cfg.ik.max_inner_steps,
        )
        command_filter = TipCommandFilter(interface_cfg.command, update_interval)
        # Seed from the actual encoder pose before forming relative WORLD goals.
        pvt_mapper.commit_pulses(current_pulses)
        center_p, center_r = ik.fk_tip()
        print(f"Startup IK insertion d={ik.u[0]*1000:.3f} mm | "
              f"axis-5 zero={base_pulses[linear_physical_idx]} pulses | "
              f"current={current_pulses[linear_physical_idx]} pulses")
        ik_limit_monitor = IKLimitMonitor()
        linear_physical_idx = axis_mapper.axis_order[4]
        if robot is not None:
            if use_insertion_profile and robot.pvt_axis5_max_step != math.ceil(max_linear_step_pulses):
                raise ValueError("PMAC Axis5_MaxStep must match the configured insertion budget "
                                 f"({math.ceil(max_linear_step_pulses)} counts). "
                                 "Update and verify the paired PMAC parameters before teleoperation.")
            max_linear_step_pulses = interface_cfg.insertion_step_limit(
                pmac_cfg.pulses_per_meter, update_interval, robot.pvt_axis5_max_step,
            )
        print(f"Insertion step budget: {max_linear_step_pulses:.3f} counts/segment "
              f"({max_linear_step_pulses / pmac_cfg.pulses_per_meter / update_interval * 1000:.3f} mm/s); "
              "independent of WORLD Z. Insertion profile only; not coordinated limiting.")
        insertion_profile = InsertionProfile(
            current_pulses[linear_physical_idx], update_interval,
            interface_cfg.insertion_max_speed_m_s * pmac_cfg.pulses_per_meter,
            interface_cfg.insertion_max_accel_m_s2 * pmac_cfg.pulses_per_meter,
            max_linear_step_pulses,
        ) if use_insertion_profile else None

        context = zmq.Context()
        cleanup.callback(context.term)
        command_socket = context.socket(zmq.PULL)
        cleanup.callback(command_socket.close)
        command_socket.setsockopt(zmq.CONFLATE, 1)
        command_socket.setsockopt(zmq.RCVHWM, 1)
        command_socket.setsockopt(zmq.LINGER, 0)
        command_socket.bind(f"tcp://{args.bind_host}:{args.command_port}")

        state_socket = context.socket(zmq.PUSH)
        cleanup.callback(state_socket.close)
        state_socket.setsockopt(zmq.CONFLATE, 1)
        state_socket.setsockopt(zmq.SNDHWM, 1)
        state_socket.setsockopt(zmq.LINGER, 0)
        state_socket.bind(f"tcp://{args.bind_host}:{args.state_port}")

        last_command_time: float | None = None
        watchdog_holding = True
        rejected_commands = 0
        last_command_error = ""
        feedback_valid = True
        feedback_pulses = current_pulses.copy()
        last_target_pulses = current_pulses.copy()
        live_debug = LiveDiagnostic(sys.stdout)
        next_live_debug = 0.0
        previous_command = None
        trace_raw_action = {}
        trace_previous_start = None
        trace_feedback_start = None
        trace_feedback_end = None
        trace_previous_log_ms = 0.0
        trace_previous_sleep_requested_ms = 0.0
        trace_previous_sleep_actual_ms = 0.0
        next_feedback = time.perf_counter()
        next_call = time.perf_counter()
        state_sequence = 0
        next_shape_debug = time.perf_counter()
        shape_debug_interval = (
            float("inf") if args.shape_debug_hz <= 0.0 else 1.0 / args.shape_debug_hz
        )
        next_motor_debug = time.perf_counter()
        motor_debug_interval = (
            float("inf") if args.motor_debug_hz <= 0.0 else 1.0 / args.motor_debug_hz
        )
        last_motor_debug_target: np.ndarray | None = None
        shape_log_file = None
        shape_log_writer = None
        shape_log_fields = None
        if args.shape_debug_csv:
            shape_log_path = Path(args.shape_debug_csv)
            shape_log_path.parent.mkdir(parents=True, exist_ok=True)
            shape_log_file = shape_log_path.open("w", newline="", encoding="utf-8")
            cleanup.callback(shape_log_file.close)

        mode = "EXECUTE" if args.execute else "DRY-RUN"
        print(
            f"Continuum driver server [{mode}] | control={update_hz} Hz | "
            f"commands=tcp://{args.bind_host}:{args.command_port} | "
            f"state=tcp://{args.bind_host}:{args.state_port}"
        )
        print(
            "Command frame: +X right, +Y forward/insertion, +Z up; "
            "internal IK frame: +X right, +Y down, +Z insertion."
        )
        print(f"Base pulses: {base_pulses}")
        if args.lock_linear_axis:
            print("Linear insertion axis locked at captured startup position for diagnostics.")
        if args.shape_debug_hz > 0.0:
            print(f"Shape diagnostics printing at {args.shape_debug_hz:.2f} Hz")
        if args.motor_debug_hz > 0.0:
            print(f"Motor diagnostics printing at {args.motor_debug_hz:.2f} Hz")
        if shape_log_file is not None:
            print(f"Shape diagnostics CSV: {args.shape_debug_csv}")

        try:
            start_time = time.perf_counter()
            while True:
                now = time.perf_counter()
                trace_fresh = False
                trace_submit_start = trace_submit_end = None
                trace_receive_kind = ""
                message = _latest_message(command_socket)
                if message is not None:
                    try:
                        kind, action = parse_control_message(message)
                        if kind == "hold":
                            command_filter.hold()
                            watchdog_holding = True
                        else:
                            assert action is not None
                            previously_limited = command_filter.rotation_command_limited
                            command_filter.set_command(action)
                            if command_filter.rotation_command_limited and not previously_limited and not args.live_debug:
                                print("WORLD ROTATION CLAMPED | requested rotation exceeds "
                                      f"driver range {interface_cfg.command.max_rotation_delta_rad} rad; "
                                      "this is a command cap, not a motor limit.", flush=True)
                            trace_raw_action = dict(action)
                            watchdog_holding = False
                        last_command_time = now
                        trace_receive_kind = kind
                        last_command_error = ""
                    except (KeyError, TypeError, ValueError) as exc:
                        rejected_commands += 1
                        last_command_error = str(exc)

                command_age_s = None if last_command_time is None else now - last_command_time
                if (
                    not watchdog_holding
                    and command_age_s is not None
                    and command_age_s > args.watchdog_timeout
                ):
                    command_filter.hold()
                    watchdog_holding = True

                trace_prepare_start = time.perf_counter() if latency_file else 0.0
                applied_delta = command_filter.step()
                applied_rotation = apply_axis_transform(
                    command_filter.applied_rotation,
                    interface_cfg.frame.rotation_map,
                    interface_cfg.frame.rotation_signs,
                )
                r_goal = (
                    center_r @ rotvec_to_matrix(applied_rotation)
                    if interface_cfg.command.orientation_enabled
                    else None
                )
                trace_map_start = time.perf_counter() if latency_file else 0.0
                diagnostic_previous_u = ik.u.copy() if args.live_debug else None
                if watchdog_holding and previous_command is not None:
                    pvt_command = replace(
                        previous_command,
                        target_pulses=list(last_target_pulses),
                        velocities=[0.0] * 5,
                    )
                else:
                    pvt_command = pvt_mapper.build_command(
                        center_p
                        + apply_axis_transform(
                            applied_delta,
                            interface_cfg.frame.translation_map,
                            interface_cfg.frame.translation_signs,
                        ),
                        z_goal=None if r_goal is None else r_goal[:, 2],
                    )

                trace_map_end = time.perf_counter() if latency_file else 0.0
                # Inspect the IK solution before PMAC pulse clipping / feedback
                # warm-start modifies it. These are model joint limits only.
                ik_limit_message = ik_limit_monitor.update(
                    None if watchdog_holding else pvt_command.ik_result, now,
                )
                if ik_limit_message is not None and not args.live_debug:
                    print(ik_limit_message, flush=True)

                diagnostic_requested = (axis_mapper.pulses_to_logical(base_pulses, pvt_command.target_pulses)
                                        if args.live_debug else None)
                if args.lock_linear_axis:
                    pvt_command.target_pulses[linear_physical_idx] = int(
                        current_pulses[linear_physical_idx]
                    )
                    pvt_command.velocities[linear_physical_idx] = 0.0

                linear_delta = (
                    pvt_command.target_pulses[linear_physical_idx]
                    - last_target_pulses[linear_physical_idx]
                )
                requested_linear_delta = linear_delta
                insertion_segment = None
                if insertion_profile is not None:
                    insertion_segment = insertion_profile.step(
                        pvt_command.target_pulses[linear_physical_idx],
                        holding=watchdog_holding or args.lock_linear_axis,
                    )
                    pvt_command.target_pulses[linear_physical_idx] = insertion_segment.position
                    pvt_command.velocities[linear_physical_idx] = insertion_segment.velocity_counts_ms
                else:
                    linear_delta = float(np.clip(linear_delta, -max_linear_step_pulses, max_linear_step_pulses))
                    pvt_command.target_pulses[linear_physical_idx] = int(round(last_target_pulses[linear_physical_idx] + linear_delta))
                    pvt_command.velocities[linear_physical_idx] = linear_delta / (update_interval * 1000.0)
                applied_linear_delta = (pvt_command.target_pulses[linear_physical_idx]
                                        - last_target_pulses[linear_physical_idx])

                if robot is not None:
                    trace_submit_start = time.perf_counter() if latency_file else None
                    robot.move_pvt_stream(
                        target_pulses=pvt_command.target_pulses,
                        velocities=pvt_command.velocities,
                        move_time=update_interval * 1000.0,
                    )
                    trace_submit_end = time.perf_counter() if latency_file else None
                    if now >= next_feedback:
                        trace_feedback_start = time.perf_counter() if latency_file else None
                        candidate_feedback = robot.read_positions()
                        error_counts = getattr(robot, "last_axis5_error_abs_counts", None)
                        if interface_cfg.insertion_error_stop_m is not None and (
                                error_counts is None or error_counts < 0 or
                                error_counts / pmac_cfg.pulses_per_meter >= interface_cfg.insertion_error_stop_m):
                            raise RuntimeError(f"Insertion following error stop: {error_counts} counts; "
                                               "abort requested, no automatic return.")
                        trace_feedback_end = time.perf_counter() if latency_file else None
                        trace_fresh = True
                        if any(candidate_feedback):
                            feedback_pulses = candidate_feedback
                            feedback_valid = True
                        else:
                            raise RuntimeError("PMAC returned invalid all-zero feedback during motion.")
                        next_feedback = now + feedback_interval
                else:
                    feedback_pulses = list(pvt_command.target_pulses)

                pvt_mapper.commit_pulses(pvt_command.target_pulses)
                commanded_tip, _ = ik.fk_tip()
                previous_command = pvt_command
                state = _state_from_feedback(axis_mapper, base_pulses, feedback_pulses)
                ik_error = pvt_command.ik_result.error
                applied_action = command_filter.applied_command()
                status = {
                    "execute": args.execute,
                    "control_hz": update_hz,
                    "watchdog_holding": watchdog_holding,
                    "command_age_ms": (
                        None if command_age_s is None else command_age_s * 1000.0
                    ),
                    "feedback_valid": feedback_valid,
                    "linear_axis_locked": args.lock_linear_axis,
                    "rotation_command_limited": command_filter.rotation_command_limited,
                    "insertion_profile_remaining_m": None if insertion_segment is None else insertion_segment.remaining_counts / pmac_cfg.pulses_per_meter,
                    "insertion_velocity_m_s": pvt_command.velocities[linear_physical_idx] * 1000 / pmac_cfg.pulses_per_meter,
                    "feedback_pulses": feedback_pulses,
                    "target_pulses": pvt_command.target_pulses,
                    "ik_error_m": float(np.linalg.norm(ik_error[:3])),
                    "commanded_tip_error_m": float(np.linalg.norm(center_p + applied_delta - commanded_tip)),
                    "ik_error_norm": float(np.linalg.norm(ik_error)),
                    "ik_joint_limits": list(ik_limit_monitor.active),
                    "ik_converged": pvt_command.ik_result.converged,
                    "ik_limit_attempts": dict(ik_limit_monitor.active),
                    "ik_orientation_error_weighted": (
                        float(np.linalg.norm(ik_error[3:])) if ik_error.size > 3 else 0.0
                    ),
                    "rejected_commands": rejected_commands,
                    "last_command_error": last_command_error,
                }
                state_message = build_state_message(
                    state_sequence,
                    state,
                    status=status,
                    applied_action=applied_action,
                )
                try:
                    state_socket.send_json(state_message, flags=zmq.NOBLOCK)
                except zmq.Again:
                    pass

                if args.motor_debug_hz > 0.0 and now >= next_motor_debug:
                    _, tip_mm = _dominant_tip_label(applied_action)
                    _, rotation_rad = _dominant_rotation_label(applied_action)
                    target_for_debug = np.asarray(pvt_command.target_pulses, dtype=int)
                    if (
                        tip_mm >= args.motor_debug_min_tip_mm
                        or rotation_rad >= args.motor_debug_min_rotation_rad
                    ):
                        target_changed = (
                            last_motor_debug_target is None
                            or np.max(np.abs(target_for_debug - last_motor_debug_target))
                            >= args.motor_debug_change_pulses
                        )
                        if target_changed:
                            _print_motor_diagnostic(
                                applied_action=applied_action,
                                base_pulses=base_pulses,
                                last_target_pulses=last_target_pulses,
                                target_pulses=pvt_command.target_pulses,
                                feedback_pulses=feedback_pulses,
                            )
                            last_motor_debug_target = target_for_debug.copy()
                    next_motor_debug = now + motor_debug_interval

                if (
                    args.shape_debug_hz > 0.0
                    and now >= next_shape_debug
                ) or shape_log_file is not None:
                    shape_row = _build_shape_diagnostic(
                        t_s=now - start_time,
                        state_sequence=state_sequence,
                        axis_mapper=axis_mapper,
                        tendon_mapper=tendon_mapper,
                        base_pulses=base_pulses,
                        pvt_command=pvt_command,
                        feedback_pulses=feedback_pulses,
                        applied_action=applied_action,
                        watchdog_holding=watchdog_holding,
                    )
                    if args.shape_debug_hz > 0.0 and now >= next_shape_debug:
                        _print_shape_diagnostic(shape_row)
                        next_shape_debug = now + shape_debug_interval
                    if shape_log_file is not None:
                        if shape_log_writer is None:
                            shape_log_fields = list(shape_row)
                            shape_log_writer = csv.DictWriter(shape_log_file, fieldnames=shape_log_fields)
                            shape_log_writer.writeheader()
                        shape_log_writer.writerow(shape_row)

                if args.live_debug and now >= next_live_debug:
                    live_debug.draw(diagnostic_lines(
                        pvt_command.ik_result, diagnostic_previous_u, diagnostic_requested,
                        axis_mapper.pulses_to_logical(base_pulses, pvt_command.target_pulses),
                        np.asarray(axis_mapper.pulses_to_logical(base_pulses, last_target_pulses)),
                        ik.geometry, holding=watchdog_holding,
                        rotation_limited=command_filter.rotation_command_limited,
                    ))
                    next_live_debug = now + .25
                state_sequence += 1
                last_target_pulses = list(pvt_command.target_pulses)
                if latency_file is not None:
                    trace_log_start = time.perf_counter()
                    cached = getattr(getattr(robot, "protocol", None), "last_pvt_status", None)
                    def relative(value):
                        return None if value is None else value - start_time
                    trace_row = {
                        "sequence": state_sequence - 1, "t_s": now - start_time,
                        "execute": args.execute, "received_kind": trace_receive_kind,
                        "period_ms": None if trace_previous_start is None else (now - trace_previous_start) * 1000,
                        "work_before_log_ms": (trace_log_start - now) * 1000,
                        "previous_log_ms": trace_previous_log_ms,
                        "previous_cycle_sleep_requested_ms": trace_previous_sleep_requested_ms,
                        "previous_cycle_sleep_actual_ms": trace_previous_sleep_actual_ms,
                        "prepare_ms": (trace_map_start - trace_prepare_start) * 1000,
                        "ik_mapping_ms": (trace_map_end - trace_map_start) * 1000,
                        "submit_start_s": relative(trace_submit_start),
                        "submit_end_s": relative(trace_submit_end),
                        "submit_ms": None if trace_submit_start is None else (trace_submit_end - trace_submit_start) * 1000,
                        "feedback_new": trace_fresh,
                        "feedback_request_start_s": relative(trace_feedback_start),
                        "feedback_request_end_s": relative(trace_feedback_end),
                        "feedback_ms": (trace_feedback_end - trace_feedback_start) * 1000 if trace_fresh else None,
                        "feedback_age_since_read_ms": None if trace_feedback_end is None else (trace_log_start - trace_feedback_end) * 1000,
                        "queued_at_ack": None if cached is None else cached.queued,
                        "insertion_sampled_error_counts": (None if robot is None else getattr(robot, "last_axis5_error_abs_counts", None)),
                        "insertion_step_limit_counts": max_linear_step_pulses,
                        "insertion_requested_step_counts": requested_linear_delta,
                        "insertion_applied_step_counts": applied_linear_delta,
                        "insertion_step_clipped": abs(requested_linear_delta) > max_linear_step_pulses,
                        "insertion_profile_limited": requested_linear_delta != applied_linear_delta,
                        "insertion_remaining_counts": None if insertion_segment is None else insertion_segment.remaining_counts,
                        "insertion_velocity_counts_ms": pvt_command.velocities[linear_physical_idx],
                        "insertion_peak_speed_m_s": None if insertion_segment is None else insertion_segment.peak_speed_counts_s / pmac_cfg.pulses_per_meter,
                        "insertion_peak_accel_m_s2": None if insertion_segment is None else insertion_segment.peak_accel_counts_s2 / pmac_cfg.pulses_per_meter,
                        "accepted_token": None if cached is None else cached.accepted,
                        "holding": watchdog_holding,
                        "rotation_command_limited": command_filter.rotation_command_limited,
                        "command_age_ms": status["command_age_ms"],
                        "ik_converged": pvt_command.ik_result.converged,
                    }
                    protocol = getattr(robot, "protocol", None)
                    for prefix, metric in (
                        ("pvt", getattr(protocol, "last_submit_timing", None)),
                        ("snapshot", getattr(protocol, "last_snapshot_timing", None) if trace_fresh else None),
                    ):
                        for field in TransactionTiming.__dataclass_fields__:
                            trace_row[f"{prefix}_{field}"] = None if metric is None else getattr(metric, field)
                    for key in applied_action:
                        trace_row["raw_" + key] = trace_raw_action.get(key)
                        trace_row["applied_" + key] = applied_action[key]
                    for idx in range(5):
                        trace_row[f"target_p{idx + 1}"] = pvt_command.target_pulses[idx]
                        trace_row[f"feedback_p{idx + 1}"] = feedback_pulses[idx] if robot is not None else None
                    if latency_writer is None:
                        latency_writer = csv.DictWriter(latency_file, fieldnames=list(trace_row))
                        latency_writer.writeheader()
                    latency_writer.writerow(trace_row)
                    if state_sequence % 50 == 0:
                        latency_file.flush()
                    trace_previous_log_ms = (time.perf_counter() - trace_log_start) * 1000
                    trace_previous_start = now
                next_call += update_interval
                sleep_time = next_call - time.perf_counter()
                if latency_file is not None:
                    trace_previous_sleep_requested_ms = max(0.0, sleep_time) * 1000
                    trace_previous_sleep_actual_ms = 0.0
                if sleep_time > 0.0:
                    trace_sleep_start = time.perf_counter() if latency_file else 0.0
                    cycle_sleep(sleep_time)
                    if latency_file is not None:
                        trace_previous_sleep_actual_ms = (time.perf_counter() - trace_sleep_start) * 1000
                else:
                    next_call = time.perf_counter()
        except KeyboardInterrupt:
            print("\nContinuum driver server stopped.")


if __name__ == "__main__":
    main()
