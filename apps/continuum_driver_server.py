from __future__ import annotations

import argparse
import time
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


def main() -> None:
    args = parse_args()
    if args.watchdog_timeout <= 0.0:
        raise ValueError("--watchdog-timeout must be positive.")
    if args.feedback_hz <= 0.0:
        raise ValueError("--feedback-hz must be positive.")

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
        base_pulses = interface_cfg.initial_position.resolve_reference(current_pulses)
        robot.base_positions = base_pulses.copy()
    else:
        base_pulses = list(
            interface_cfg.initial_position.reference_pulses
            or (0, 0, 0, 0, 0)
        )
        current_pulses = base_pulses.copy()

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

    mode = "EXECUTE" if args.execute else "DRY-RUN"
    print(
        f"Continuum driver server [{mode}] | control={update_hz} Hz | "
        f"commands=tcp://{args.bind_host}:{args.command_port} | "
        f"state=tcp://{args.bind_host}:{args.state_port}"
    )
    print(f"Base pulses: {base_pulses}")

    try:
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
                applied_action=command_filter.applied_command(),
            )
            try:
                state_socket.send_json(state_message, flags=zmq.NOBLOCK)
            except zmq.Again:
                pass

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
        command_socket.close()
        state_socket.close()
        context.term()


if __name__ == "__main__":
    main()
