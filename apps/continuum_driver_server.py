from __future__ import annotations

import argparse
import csv
import time
from pathlib import Path
from typing import Any

import numpy as np
import zmq

from continuum_sdk.control.axis_mapper import ContinuumAxisMapper
from continuum_sdk.control.pvt_mapper import ContinuumPVTMapper
from continuum_sdk.control.tip_command_filter import TipCommandFilter
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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run the fixed-rate PMAC continuum driver and expose a ZMQ control boundary."
    )
    parser.add_argument("--config", default="config/continuum.yaml")
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

    for _ in range(3):
        robot.move_pvt_stream(
            target_pulses=reference_pulses,
            velocities=[0.0, 0.0, 0.0, 0.0, 0.0],
            move_time=move_time_ms,
        )
        time.sleep(1.0 / update_hz)

    feedback = robot.read_positions()
    if any(feedback):
        errors = [actual - expected for actual, expected in zip(feedback, reference_pulses)]
        print(f"Startup return feedback: {feedback} | errors={errors}")
        if check_tolerance_pulses > 0 and any(
            abs(error) > check_tolerance_pulses for error in errors
        ):
            raise RuntimeError(
                "Startup return finished outside tolerance: "
                f"errors={errors}, tolerance={check_tolerance_pulses}"
            )
        return feedback

    print("Startup return feedback read as all zero; keeping reference as startup feedback.")
    return reference_pulses.copy()


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


def main() -> None:
    args = parse_args()
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

    continuum_cfg = load_continuum_config(args.config)
    interface_cfg = load_robot_interface_config(args.interface_config)
    if continuum_cfg.control.update_hz != interface_cfg.control_hz:
        raise ValueError(
            "continuum control update_hz and robot interface control_hz must match: "
            f"{continuum_cfg.control.update_hz} != {interface_cfg.control_hz}"
        )

    pmac_cfg = PMACConfig(ip=args.pmac_ip)
    update_hz = interface_cfg.control_hz
    update_interval = 1.0 / update_hz
    feedback_interval = 1.0 / args.feedback_hz

    ik = build_continuum_ik(continuum_cfg)
    if interface_cfg.command.orientation_enabled:
        ik.task_mode = "pos_z"
    tendon_mapper = build_tendon_mapper(continuum_cfg)
    axis_mapper = ContinuumAxisMapper(
        pulses_per_rad=pmac_cfg.pulses_per_rad,
        pulses_per_meter=pmac_cfg.pulses_per_meter,
        axis_order=pmac_cfg.axis_order,
        axis_signs=pmac_cfg.axis_signs,
    )
    center_p, center_r = ik.fk_tip()

    robot = PMACRobotController(pmac_cfg) if args.execute else None
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

    pvt_mapper = ContinuumPVTMapper(
        ik=ik,
        tendon_mapper=tendon_mapper,
        axis_mapper=axis_mapper,
        base_pulses=base_pulses,
        update_interval_s=update_interval,
        max_inner_steps=continuum_cfg.ik.max_inner_steps,
    )
    command_filter = TipCommandFilter(interface_cfg.command, update_interval)
    linear_physical_idx = axis_mapper.axis_order[4]
    max_linear_step_pulses = (
        abs(pmac_cfg.pulses_per_meter * interface_cfg.command.max_speed_m_s[1] * update_interval)
    )

    context = zmq.Context()
    command_socket = context.socket(zmq.PULL)
    command_socket.setsockopt(zmq.CONFLATE, 1)
    command_socket.setsockopt(zmq.RCVHWM, 1)
    command_socket.setsockopt(zmq.LINGER, 0)
    command_socket.bind(f"tcp://{args.bind_host}:{args.command_port}")

    state_socket = context.socket(zmq.PUSH)
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
    last_target_pulses = base_pulses.copy()
    next_feedback = time.perf_counter()
    next_call = time.perf_counter()
    state_sequence = 0
    next_shape_debug = time.perf_counter()
    shape_debug_interval = (
        float("inf") if args.shape_debug_hz <= 0.0 else 1.0 / args.shape_debug_hz
    )
    shape_log_file = None
    shape_log_writer = None
    shape_log_fields = None
    if args.shape_debug_csv:
        shape_log_path = Path(args.shape_debug_csv)
        shape_log_path.parent.mkdir(parents=True, exist_ok=True)
        shape_log_file = shape_log_path.open("w", newline="", encoding="utf-8")

    mode = "EXECUTE" if args.execute else "DRY-RUN"
    print(
        f"Continuum driver server [{mode}] | control={update_hz} Hz | "
        f"commands=tcp://{args.bind_host}:{args.command_port} | "
        f"state=tcp://{args.bind_host}:{args.state_port}"
    )
    print(f"Base pulses: {base_pulses}")
    if args.shape_debug_hz > 0.0:
        print(f"Shape diagnostics printing at {args.shape_debug_hz:.2f} Hz")
    if shape_log_file is not None:
        print(f"Shape diagnostics CSV: {args.shape_debug_csv}")

    try:
        start_time = time.perf_counter()
        while True:
            now = time.perf_counter()
            message = _latest_message(command_socket)
            if message is not None:
                try:
                    kind, action = parse_control_message(message)
                    if kind == "hold":
                        command_filter.hold()
                        watchdog_holding = True
                    else:
                        assert action is not None
                        command_filter.set_command(action)
                        watchdog_holding = False
                    last_command_time = now
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

            applied_delta = command_filter.step()
            applied_rotation = command_filter.applied_rotation
            r_goal = (
                center_r @ rotvec_to_matrix(applied_rotation)
                if interface_cfg.command.orientation_enabled
                else None
            )
            pvt_command = pvt_mapper.build_command(
                center_p + applied_delta,
                z_goal=None if r_goal is None else r_goal[:, 2],
            )

            linear_delta = (
                pvt_command.target_pulses[linear_physical_idx]
                - last_target_pulses[linear_physical_idx]
            )
            linear_delta = float(
                np.clip(linear_delta, -max_linear_step_pulses, max_linear_step_pulses)
            )
            pvt_command.target_pulses[linear_physical_idx] = int(
                round(last_target_pulses[linear_physical_idx] + linear_delta)
            )
            pvt_command.velocities[linear_physical_idx] = linear_delta / (
                update_interval * 1000.0
            )

            if robot is not None:
                robot.move_pvt_stream(
                    target_pulses=pvt_command.target_pulses,
                    velocities=pvt_command.velocities,
                    move_time=update_interval * 1000.0,
                )
                if now >= next_feedback:
                    candidate_feedback = robot.read_positions()
                    if any(candidate_feedback):
                        feedback_pulses = candidate_feedback
                        feedback_valid = True
                    else:
                        feedback_valid = False
                    next_feedback = now + feedback_interval
            else:
                feedback_pulses = list(pvt_command.target_pulses)

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
                "feedback_pulses": feedback_pulses,
                "target_pulses": pvt_command.target_pulses,
                "ik_error_m": float(np.linalg.norm(ik_error[:3])),
                "ik_error_norm": float(np.linalg.norm(ik_error)),
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

            state_sequence += 1
            last_target_pulses = list(pvt_command.target_pulses)
            next_call += update_interval
            sleep_time = next_call - time.perf_counter()
            if sleep_time > 0.0:
                time.sleep(sleep_time)
            else:
                next_call = time.perf_counter()
    except KeyboardInterrupt:
        print("\nContinuum driver server stopped.")
    finally:
        if robot is not None:
            robot.close()
        if shape_log_file is not None:
            shape_log_file.close()
        command_socket.close()
        state_socket.close()
        context.term()


if __name__ == "__main__":
    main()
