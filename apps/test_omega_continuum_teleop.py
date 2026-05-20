import argparse
import time

import numpy as np

from continuum_sdk.control.axis_mapper import ContinuumAxisMapper
from continuum_sdk.control.pvt_mapper import ContinuumPVTMapper
from continuum_sdk.core.config_loader import load_continuum_config
from continuum_sdk.core.factory import build_continuum_ik, build_tendon_mapper
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
    ) -> None:
        self.center_p = np.asarray(center_p, dtype=float)
        self.scale = np.asarray(scale_xyz, dtype=float)
        self.max_delta = np.asarray(max_delta_xyz, dtype=float)
        self._omega_zero: np.ndarray | None = None

    def calibrate_zero(self, haptic_state: HapticState) -> None:
        self._omega_zero = np.asarray(haptic_state.pos, dtype=float)

    def solve(self, haptic_state: HapticState) -> np.ndarray:
        if self._omega_zero is None:
            self.calibrate_zero(haptic_state)

        delta = (np.asarray(haptic_state.pos, dtype=float) - self._omega_zero) * self.scale
        delta = np.clip(delta, -self.max_delta, self.max_delta)
        return self.center_p + delta


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Omega master teleoperation test for continuum robot.")
    parser.add_argument("--config", default="config/continuum.yaml")
    parser.add_argument("--pmac-ip", default="192.168.0.200")
    parser.add_argument("--scale-x", type=float, default=0.6, help="Robot meters per Omega meter on X.")
    parser.add_argument("--scale-y", type=float, default=0.6, help="Robot meters per Omega meter on Y.")
    parser.add_argument("--scale-z", type=float, default=0.6, help="Robot meters per Omega meter on Z.")
    parser.add_argument("--max-delta-x", type=float, default=0.01, help="Clamp robot X offset from neutral, in meters.")
    parser.add_argument("--max-delta-y", type=float, default=0.01, help="Clamp robot Y offset from neutral, in meters.")
    parser.add_argument("--max-delta-z", type=float, default=0.01, help="Clamp robot Z offset from neutral, in meters.")
    parser.add_argument(
        "--lock-linear-axis",
        action="store_true",
        help="Keep the logical d/physical linear axis at its startup position for bend-only tests.",
    )
    parser.add_argument("--duration", type=float, default=0.0, help="0 means run until Ctrl+C.")
    parser.add_argument("--execute", action="store_true", help="Send commands to PMAC. Without this flag, only dry-run.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    continuum_cfg = load_continuum_config(args.config)
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

    try:
        if robot is not None:
            robot.safe_boot_and_home()
            base_pulses = robot.base_positions.copy()
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
        )
        omega_mapper.calibrate_zero(omega.get_state())
        linear_physical_idx = axis_mapper.axis_order[4]

        start_time = time.perf_counter()
        next_call = start_time
        print("Omega teleop started. Hold the master at the neutral pose during startup.")

        while True:
            now = time.perf_counter()
            t = now - start_time
            if args.duration > 0.0 and t >= args.duration:
                break

            haptic_state = omega.get_state()
            p_goal = omega_mapper.solve(haptic_state)
            command = pvt_mapper.build_command(p_goal)
            if args.lock_linear_axis:
                command.axis_targets[4] = 0.0
                command.target_pulses[linear_physical_idx] = base_pulses[linear_physical_idx]
                command.velocities[linear_physical_idx] = 0.0
                ik.u[0] = 0.0

            if robot is not None:
                robot.move_pvt_stream(
                    target_pulses=command.target_pulses,
                    velocities=command.velocities,
                    move_time=move_time_ms,
                )

            if int(t * update_hz) % max(1, update_hz // 2) == 0:
                print(
                    f"t={t:.2f}s | omega={np.round(haptic_state.pos, 4)} | "
                    f"p_goal={np.round(command.p_goal, 4)} | "
                    f"err={np.linalg.norm(command.ik_result.error):.5f} | "
                    f"dpulses={(np.asarray(command.target_pulses) - np.asarray(base_pulses)).tolist()}"
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
        if robot is not None:
            robot.close()
        omega.close()


if __name__ == "__main__":
    main()
