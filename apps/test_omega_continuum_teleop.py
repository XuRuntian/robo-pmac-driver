import argparse
import csv
import time
from collections import deque
from pathlib import Path

import numpy as np

from continuum_sdk.control.axis_mapper import ContinuumAxisMapper
from continuum_sdk.control.pvt_mapper import ContinuumPVTMapper
from continuum_sdk.core.config_loader import load_continuum_config
from continuum_sdk.core.factory import build_continuum_ik, build_tendon_mapper
from continuum_sdk.core.interface_config import load_robot_interface_config
from omega_sdk.haptic_device import HapticState, OmegaDevice
from pmac_sdk.controller.robot_api import PMACRobotController
from pmac_sdk.core.config_model import PMACConfig


class OmegaCartesianMapper:
    """Convert Omega pose samples into continuum tip position goals."""

    def __init__(
        self,
        center_p: np.ndarray,
        scale_xyz: tuple[float, float, float],
        max_delta_xyz: tuple[float, float, float],
        max_speed_xyz: tuple[float, float, float],
        deadband_m: float,
        smooth_alpha: float,
        omega_map: str,
    ) -> None:
        self.center_p = np.asarray(center_p, dtype=float)
        self.scale = np.asarray(scale_xyz, dtype=float)
        self.max_delta = np.asarray(max_delta_xyz, dtype=float)
        self.max_speed = np.asarray(max_speed_xyz, dtype=float)
        self.deadband_m = float(deadband_m)
        self.smooth_alpha = float(np.clip(smooth_alpha, 0.0, 1.0))
        self.omega_map = omega_map.lower()
        axis_index = {"x": 0, "y": 1, "z": 2}
        self._omega_to_robot = np.asarray([axis_index[axis] for axis in self.omega_map], dtype=int)
        self._omega_zero: np.ndarray | None = None
        self._delta = np.zeros(3, dtype=float)

    def calibrate_zero(self, haptic_state: HapticState) -> None:
        self._omega_zero = np.asarray(haptic_state.pos, dtype=float)
        self._delta[:] = 0.0

    def solve(self, haptic_state: HapticState, dt_s: float) -> np.ndarray:
        if self._omega_zero is None:
            self.calibrate_zero(haptic_state)

        omega_delta = np.asarray(haptic_state.pos, dtype=float) - self._omega_zero
        target_delta = omega_delta[self._omega_to_robot] * self.scale
        if self.deadband_m > 0.0:
            target_delta[np.abs(target_delta) < self.deadband_m] = 0.0
        target_delta = np.clip(target_delta, -self.max_delta, self.max_delta)

        if self.smooth_alpha < 1.0:
            target_delta = self._delta + self.smooth_alpha * (target_delta - self._delta)

        max_step = np.maximum(self.max_speed * dt_s, 0.0)
        step = np.clip(target_delta - self._delta, -max_step, max_step)
        self._delta = np.clip(self._delta + step, -self.max_delta, self.max_delta)
        return self.center_p + self._delta


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Omega master teleoperation test for continuum robot.")
    parser.add_argument("--config", default="config/continuum.yaml")
    parser.add_argument("--interface-config", default="config/robot_interface.yaml")
    parser.add_argument("--pmac-ip", default="192.168.0.200")
    parser.add_argument("--scale-x", type=float, default=0.6, help="Robot meters per Omega meter on X.")
    parser.add_argument("--scale-y", type=float, default=0.6, help="Robot meters per Omega meter on Y.")
    parser.add_argument("--scale-z", type=float, default=0.6, help="Robot meters per Omega meter on Z.")
    parser.add_argument(
        "--omega-map",
        default="zxy",
        choices=("xyz", "xzy", "yxz", "yzx", "zxy", "zyx"),
        help=(
            "Source Omega axes for robot XYZ. Default zxy means "
            "robot X<-Omega Z, robot Y/linear<-Omega X, robot Z<-Omega Y."
        ),
    )
    parser.add_argument("--max-delta-x", type=float, default=0.01, help="Clamp robot X offset from neutral, in meters.")
    parser.add_argument("--max-delta-y", type=float, default=0.01, help="Clamp robot Y offset from neutral, in meters.")
    parser.add_argument("--max-delta-z", type=float, default=0.01, help="Clamp robot Z offset from neutral, in meters.")
    parser.add_argument("--max-speed-x", type=float, default=0.02, help="Limit robot X target slew rate, in m/s.")
    parser.add_argument("--max-speed-y", type=float, default=0.002, help="Limit robot Y/linear-axis target slew rate, in m/s.")
    parser.add_argument("--max-speed-z", type=float, default=0.02, help="Limit robot Z target slew rate, in m/s.")
    parser.add_argument("--deadband", type=float, default=0.0003, help="Ignore small robot-space Omega deltas, in meters.")
    parser.add_argument(
        "--smooth-alpha",
        type=float,
        default=0.25,
        help="Low-pass factor for Omega target delta. 1 disables smoothing.",
    )
    parser.add_argument(
        "--lock-linear-axis",
        action="store_true",
        help="Keep the logical d/physical linear axis at its startup position for bend-only tests.",
    )
    parser.add_argument("--duration", type=float, default=0.0, help="0 means run until Ctrl+C.")
    parser.add_argument("--execute", action="store_true", help="Send commands to PMAC. Without this flag, only dry-run.")
    parser.add_argument("--feedback-hz", type=float, default=10.0, help="PMAC position feedback sampling rate in Hz.")
    parser.add_argument(
        "--feedback-delay",
        type=float,
        default=0.22,
        help="Compare feedback with a target this many seconds old to account for PMAC PVT buffering.",
    )
    parser.add_argument("--log-csv", default="", help="Optional CSV path for target/feedback/error logging.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    continuum_cfg = load_continuum_config(args.config)
    interface_cfg = load_robot_interface_config(args.interface_config)
    pmac_config = PMACConfig(ip=args.pmac_ip)

    ik = build_continuum_ik(continuum_cfg)
    tendon_mapper = build_tendon_mapper(continuum_cfg)
    axis_mapper = ContinuumAxisMapper(
        pulses_per_rad=pmac_config.pulses_per_rad,
        pulses_per_meter=pmac_config.pulses_per_meter,
        axis_order=pmac_config.axis_order,
        axis_signs=pmac_config.axis_signs,
    )

    update_hz = continuum_cfg.control.update_hz
    update_interval = 1.0 / update_hz
    move_time_ms = update_interval * 1000.0
    center_p, _ = ik.fk_tip()

    omega = OmegaDevice()
    if not omega.connect():
        return

    robot = PMACRobotController(pmac_config) if args.execute else None
    log_file = None

    try:
        if robot is not None:
            robot.safe_boot_and_home()
            current_pulses = robot.base_positions.copy()
            base_pulses = interface_cfg.initial_position.resolve_reference(current_pulses)
            robot.base_positions = base_pulses.copy()
            print(
                f"Initial position mode={interface_cfg.initial_position.mode} | "
                f"current={current_pulses} | reference={base_pulses}"
            )
        else:
            if interface_cfg.initial_position.mode == "configured_reference":
                base_pulses = list(interface_cfg.initial_position.reference_pulses or (0, 0, 0, 0, 0))
            else:
                base_pulses = [0, 0, 0, 0, 0]
            print("Dry-run mode: Omega is read and commands are computed, but PMAC is not commanded.")

        pvt_mapper = ContinuumPVTMapper(
            ik=ik,
            tendon_mapper=tendon_mapper,
            axis_mapper=axis_mapper,
            base_pulses=base_pulses,
            update_interval_s=update_interval,
            max_inner_steps=continuum_cfg.ik.max_inner_steps,
        )
        omega_mapper = OmegaCartesianMapper(
            center_p=center_p,
            scale_xyz=(args.scale_x, args.scale_y, args.scale_z),
            max_delta_xyz=(args.max_delta_x, args.max_delta_y, args.max_delta_z),
            max_speed_xyz=(args.max_speed_x, args.max_speed_y, args.max_speed_z),
            deadband_m=args.deadband,
            smooth_alpha=args.smooth_alpha,
            omega_map=args.omega_map,
        )
        omega_mapper.calibrate_zero(omega.get_state())
        linear_physical_idx = axis_mapper.axis_order[4]
        last_target_pulses = list(base_pulses)
        max_linear_step_pulses = abs(pmac_config.pulses_per_meter * args.max_speed_y * update_interval)
        feedback_positions = None
        feedback_error = None
        delayed_feedback_error = None
        delayed_target_pulses = None
        feedback_interval = 1.0 / args.feedback_hz if args.feedback_hz > 0.0 else float("inf")
        target_history = deque()

        log_writer = None
        if args.log_csv:
            log_path = Path(args.log_csv)
            log_path.parent.mkdir(parents=True, exist_ok=True)
            log_file = log_path.open("w", newline="")
            log_writer = csv.writer(log_file)
            log_writer.writerow(
                [
                    "t_s",
                    "omega_x_m",
                    "omega_y_m",
                    "omega_z_m",
                    "delta_x_m",
                    "delta_y_m",
                    "delta_z_m",
                    "ik_error_m",
                    "target_p1",
                    "target_p2",
                    "target_p3",
                    "target_p4",
                    "target_p5",
                    "actual_p1",
                    "actual_p2",
                    "actual_p3",
                    "actual_p4",
                    "actual_p5",
                    "err_p1",
                    "err_p2",
                    "err_p3",
                    "err_p4",
                    "err_p5",
                    "delayed_target_p1",
                    "delayed_target_p2",
                    "delayed_target_p3",
                    "delayed_target_p4",
                    "delayed_target_p5",
                    "delayed_err_p1",
                    "delayed_err_p2",
                    "delayed_err_p3",
                    "delayed_err_p4",
                    "delayed_err_p5",
                    "vel_p1_per_ms",
                    "vel_p2_per_ms",
                    "vel_p3_per_ms",
                    "vel_p4_per_ms",
                    "vel_p5_per_ms",
                ]
            )
            print(f"Logging target/feedback/error to {log_path}")

        start_time = time.perf_counter()
        next_call = start_time
        next_feedback_call = start_time
        print("Omega teleop started. Hold the master at the neutral pose during startup.")
        print(
            f"Omega map robot XYZ <- Omega {args.omega_map.upper()} | "
            f"Scale XYZ: {[args.scale_x, args.scale_y, args.scale_z]} | "
                    f"Max delta XYZ: {[args.max_delta_x, args.max_delta_y, args.max_delta_z]} m | "
                    f"Max speed XYZ: {[args.max_speed_x, args.max_speed_y, args.max_speed_z]} m/s | "
            f"deadband={args.deadband} m | smooth-alpha={args.smooth_alpha} | "
            f"feedback-delay={args.feedback_delay}s"
        )

        while True:
            now = time.perf_counter()
            t = now - start_time
            if args.duration > 0.0 and t >= args.duration:
                break

            haptic_state = omega.get_state()
            p_goal = omega_mapper.solve(haptic_state, update_interval)
            command = pvt_mapper.build_command(p_goal)
            if args.lock_linear_axis:
                command.axis_targets[4] = 0.0
                command.target_pulses[linear_physical_idx] = base_pulses[linear_physical_idx]
                command.velocities[linear_physical_idx] = 0.0
                ik.u[0] = 0.0
            else:
                linear_delta = command.target_pulses[linear_physical_idx] - last_target_pulses[linear_physical_idx]
                linear_delta = float(np.clip(linear_delta, -max_linear_step_pulses, max_linear_step_pulses))
                command.target_pulses[linear_physical_idx] = int(
                    round(last_target_pulses[linear_physical_idx] + linear_delta)
                )
                command.velocities[linear_physical_idx] = linear_delta / move_time_ms

            target_history.append((t, list(command.target_pulses)))
            while target_history and t - target_history[0][0] > max(args.feedback_delay + 2.0, 2.0):
                target_history.popleft()

            if robot is not None:
                robot.move_pvt_stream(
                    target_pulses=command.target_pulses,
                    velocities=command.velocities,
                    move_time=move_time_ms,
                )

                if now >= next_feedback_call:
                    feedback_positions = robot.read_positions()
                    feedback_error = (
                        np.asarray(command.target_pulses, dtype=int)
                        - np.asarray(feedback_positions, dtype=int)
                    ).tolist()
                    delayed_target_pulses = list(command.target_pulses)
                    target_time = t - args.feedback_delay
                    for hist_t, hist_target in target_history:
                        if hist_t <= target_time:
                            delayed_target_pulses = hist_target
                        else:
                            break
                    delayed_feedback_error = (
                        np.asarray(delayed_target_pulses, dtype=int)
                        - np.asarray(feedback_positions, dtype=int)
                    ).tolist()
                    next_feedback_call += feedback_interval
                    if next_feedback_call < now:
                        next_feedback_call = now + feedback_interval

                    if log_writer is not None:
                        delta = command.p_goal - center_p
                        log_writer.writerow(
                            [
                                f"{t:.6f}",
                                *[f"{v:.9f}" for v in haptic_state.pos],
                                *[f"{v:.9f}" for v in delta],
                                f"{np.linalg.norm(command.ik_result.error):.9f}",
                                *command.target_pulses,
                                *feedback_positions,
                                *feedback_error,
                                *delayed_target_pulses,
                                *delayed_feedback_error,
                                *[f"{v:.6f}" for v in command.velocities],
                            ]
                        )
            last_target_pulses = list(command.target_pulses)

            if int(t * update_hz) % max(1, update_hz // 2) == 0:
                feedback_text = ""
                if feedback_error is not None:
                    feedback_text = f" | fb_err={feedback_error}"
                if delayed_feedback_error is not None:
                    feedback_text += f" | delayed_fb_err={delayed_feedback_error}"
                print(
                    f"t={t:.2f}s | omega={np.round(haptic_state.pos, 4)} | "
                    f"delta={np.round(command.p_goal - center_p, 4)} | "
                    f"err={np.linalg.norm(command.ik_result.error):.5f} | "
                    f"dpulses={(np.asarray(command.target_pulses) - np.asarray(base_pulses)).tolist()}"
                    f"{feedback_text}"
                )

            next_call += update_interval
            sleep_time = next_call - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_call = time.perf_counter()
    except KeyboardInterrupt:
        print("\nOmega teleop stopped.")
    finally:
        if log_file is not None:
            log_file.close()
        if robot is not None:
            robot.close()
        omega.close()


if __name__ == "__main__":
    main()
