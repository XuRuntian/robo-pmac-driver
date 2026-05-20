# python apps/teleop_pvt_keyboard.py --execute --speed 0.03 --max-delta-x 0.02 --max-delta-y 0.003 --max-delta-z 0.02
import argparse
import sys
import time
from pathlib import Path

import numpy as np
from pynput import keyboard

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from continuum_sdk.control.axis_mapper import ContinuumAxisMapper
from continuum_sdk.control.pvt_mapper import ContinuumPVTMapper
from continuum_sdk.core.config_loader import load_continuum_config
from continuum_sdk.core.factory import build_continuum_ik, build_tendon_mapper
from pmac_sdk.controller.robot_api import PMACRobotController
from pmac_sdk.core.config_model import PMACConfig


class KeyboardDevice:
    def __init__(self) -> None:
        self.pressed_keys: set[str] = set()
        self.stop_requested = False
        self.on_speed_delta = None
        self.on_recenter = None
        self.listener = keyboard.Listener(on_press=self._on_press, on_release=self._on_release)

    def _key_name(self, key) -> str | None:
        if key == keyboard.Key.esc:
            return "esc"
        try:
            return key.char.lower() if key.char else None
        except AttributeError:
            return None

    def _on_press(self, key) -> None:
        name = self._key_name(key)
        if name is None:
            return

        if name == "esc":
            self.stop_requested = True
            return

        if name == "z" and self.on_speed_delta is not None:
            self.on_speed_delta(-1.0)
        elif name == "x" and self.on_speed_delta is not None:
            self.on_speed_delta(1.0)
        elif name == "c" and self.on_recenter is not None:
            self.on_recenter()
        else:
            self.pressed_keys.add(name)

    def _on_release(self, key) -> None:
        name = self._key_name(key)
        if name is not None:
            self.pressed_keys.discard(name)

    def start(self) -> None:
        self.listener.start()

    def stop(self) -> None:
        self.listener.stop()

    def get_state(self) -> set[str]:
        return set(self.pressed_keys)


class CartesianKeyboardPlanner:
    def __init__(
        self,
        center_p: np.ndarray,
        speed_m_s: float,
        max_delta_xyz: tuple[float, float, float],
    ) -> None:
        self.center_p = np.asarray(center_p, dtype=float)
        self.speed_m_s = float(speed_m_s)
        self.min_speed_m_s = 0.0005
        self.max_speed_m_s = 0.05
        self.speed_step_m_s = 0.0005
        self.max_delta = np.asarray(max_delta_xyz, dtype=float)
        self.delta = np.zeros(3, dtype=float)

    def adjust_speed(self, direction: float) -> None:
        self.speed_m_s = float(
            np.clip(
                self.speed_m_s + direction * self.speed_step_m_s,
                self.min_speed_m_s,
                self.max_speed_m_s,
            )
        )
        print(f"Keyboard Cartesian speed: {self.speed_m_s * 1000.0:.1f} mm/s")

    def recenter(self) -> None:
        self.delta[:] = 0.0
        print("Keyboard target recentered.")

    def solve(self, active_keys: set[str], dt_s: float) -> np.ndarray:
        direction = np.zeros(3, dtype=float)

        if "a" in active_keys:
            direction[0] -= 1.0
        if "d" in active_keys:
            direction[0] += 1.0
        if "q" in active_keys:
            direction[1] += 1.0
        if "e" in active_keys:
            direction[1] -= 1.0
        if "w" in active_keys:
            direction[2] += 1.0
        if "s" in active_keys:
            direction[2] -= 1.0

        self.delta += direction * self.speed_m_s * dt_s
        self.delta = np.clip(self.delta, -self.max_delta, self.max_delta)
        return self.center_p + self.delta


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Keyboard Cartesian teleoperation through continuum IK and PVT.")
    parser.add_argument("--config", default="config/continuum.yaml")
    parser.add_argument("--pmac-ip", default="192.168.0.200")
    parser.add_argument("--duration", type=float, default=0.0, help="0 means run until Esc or Ctrl+C.")
    parser.add_argument("--speed", type=float, default=0.003, help="Cartesian keyboard speed in m/s.")
    parser.add_argument("--max-delta-x", type=float, default=0.01, help="X travel limit from neutral, in meters.")
    parser.add_argument("--max-delta-y", type=float, default=0.0, help="Y travel limit from neutral, in meters.")
    parser.add_argument("--max-delta-z", type=float, default=0.01, help="Z travel limit from neutral, in meters.")
    parser.add_argument(
        "--lock-linear-axis",
        action="store_true",
        help="Keep the physical linear axis at its startup position.",
    )
    parser.add_argument("--execute", action="store_true", help="Send commands to PMAC. Without this flag, dry-run only.")
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

    robot = PMACRobotController(pmac_config) if args.execute else None
    kbd = KeyboardDevice()

    try:
        if robot is not None:
            robot.safe_boot_and_home()
            base_pulses = robot.base_positions.copy()
        else:
            base_pulses = [0, 0, 0, 0, 0]
            print("Dry-run mode: keyboard target is computed, but PMAC is not commanded.")

        pvt_mapper = ContinuumPVTMapper(
            ik=ik,
            tendon_mapper=tendon_mapper,
            axis_mapper=axis_mapper,
            base_pulses=base_pulses,
            update_interval_s=update_interval,
            max_inner_steps=continuum_cfg.ik.max_inner_steps,
        )
        planner = CartesianKeyboardPlanner(
            center_p=center_p,
            speed_m_s=args.speed,
            max_delta_xyz=(args.max_delta_x, args.max_delta_y, args.max_delta_z),
        )
        linear_physical_idx = axis_mapper.axis_order[4]
        kbd.on_speed_delta = planner.adjust_speed
        kbd.on_recenter = planner.recenter
        kbd.start()

        print("Keyboard Cartesian teleop started.")
        print("Keys: A/D=X, Q/E=Y, W/S=Z, Z/X=speed, C=recenter, Esc=stop.")
        print(f"Speed: {planner.speed_m_s * 1000.0:.1f} mm/s")
        print(f"Max delta XYZ: {planner.max_delta.tolist()} m")

        start_time = time.perf_counter()
        next_call = start_time

        while not kbd.stop_requested:
            now = time.perf_counter()
            t = now - start_time
            if args.duration > 0.0 and t >= args.duration:
                break

            keys = kbd.get_state()
            p_goal = planner.solve(keys, update_interval)
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
                dpulses = (np.asarray(command.target_pulses) - np.asarray(base_pulses)).tolist()
                print(
                    f"t={t:.2f}s | keys={sorted(keys)} | "
                    f"delta={np.round(planner.delta, 4)} | "
                    f"err={np.linalg.norm(command.ik_result.error):.5f} | "
                    f"dpulses={dpulses}"
                )

            next_call += update_interval
            sleep_time = next_call - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_call = time.perf_counter()
    except KeyboardInterrupt:
        print("\nKeyboard teleop stopped.")
    finally:
        kbd.stop()
        if robot is not None:
            robot.close()


if __name__ == "__main__":
    main()
