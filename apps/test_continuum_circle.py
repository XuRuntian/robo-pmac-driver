import argparse
import math
import time

import numpy as np

from continuum_sdk.control.axis_mapper import ContinuumAxisMapper
from continuum_sdk.control.pvt_mapper import ContinuumPVTMapper
from continuum_sdk.core.config_loader import load_continuum_config
from continuum_sdk.core.factory import build_continuum_ik, build_tendon_mapper
from pmac_sdk.controller.robot_api import PMACRobotController
from pmac_sdk.core.config_model import PMACConfig


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Continuum robot circular tip trajectory test.")
    parser.add_argument("--config", default="config/continuum.yaml")
    parser.add_argument("--pmac-ip", default="192.168.0.200")
    parser.add_argument("--radius", type=float, default=0.01, help="Circle radius in meters.")
    parser.add_argument("--freq", type=float, default=0.03, help="Circle frequency in Hz.")
    parser.add_argument("--duration", type=float, default=60.0, help="Run time in seconds.")
    parser.add_argument(
        "--ramp-time",
        type=float,
        default=3.0,
        help="Seconds used to ramp the circle radius from zero to --radius.",
    )
    parser.add_argument("--plane", choices=("xy", "xz", "yz"), default="xz")
    parser.add_argument("--execute", action="store_true", help="Send commands to PMAC. Without this flag, only dry-run.")
    return parser.parse_args()


def circle_goal(center: np.ndarray, radius: float, phase: float, plane: str) -> np.ndarray:
    p = center.copy()
    c = radius * math.cos(phase)
    s = radius * math.sin(phase)
    if plane == "xy":
        p[0] += c
        p[1] += s
    elif plane == "xz":
        p[0] += c
        p[2] += s
    else:
        p[1] += c
        p[2] += s
    return p


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

    try:
        if robot is not None:
            robot.safe_boot_and_home()
            base_pulses = robot.base_positions.copy()
        else:
            base_pulses = [0, 0, 0, 0, 0]
            print("Dry-run mode: IK and pulse targets are computed, but PMAC is not commanded.")

        mapper = ContinuumPVTMapper(
            ik=ik,
            tendon_mapper=tendon_mapper,
            axis_mapper=axis_mapper,
            base_pulses=base_pulses,
            update_interval_s=update_interval,
            max_inner_steps=continuum_cfg.ik.max_inner_steps,
        )

        virtual_time = 0.0
        next_call = time.perf_counter()

        while True:
            if virtual_time >= args.duration:
                break

            ramp_ratio = 1.0
            if args.ramp_time > 0.0:
                ramp_ratio = min(1.0, virtual_time / args.ramp_time)

            effective_radius = args.radius * ramp_ratio
            phase = 2.0 * math.pi * args.freq * virtual_time
            p_goal = circle_goal(center_p, effective_radius, phase, args.plane)
            command = mapper.build_command(p_goal)

            if robot is not None:
                robot.move_pvt_stream(
                    target_pulses=command.target_pulses,
                    velocities=command.velocities,
                    move_time=move_time_ms,
                )

            if int(virtual_time * update_hz) % max(1, update_hz // 2) == 0:
                print(
                    f"t={virtual_time:.2f}s | plane={args.plane} | "
                    f"r={effective_radius:.4f} | "
                    f"p_goal={np.round(command.p_goal, 4)} | "
                    f"err={np.linalg.norm(command.ik_result.error):.5f} | "
                    f"pulses={command.target_pulses}"
                )

            virtual_time += update_interval
            next_call += update_interval
            sleep_time = next_call - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_call = time.perf_counter()
    except KeyboardInterrupt:
        print("\nCircle test stopped.")
    finally:
        if robot is not None:
            robot.close()


if __name__ == "__main__":
    main()
