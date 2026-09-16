from __future__ import annotations

import argparse
import time

from lerobot_robot_continuum import ContinuumPMAC, ContinuumPMACConfig


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Smoke-test the continuum_pmac LeRobot plugin.")
    parser.add_argument("--remote-ip", default="127.0.0.1")
    parser.add_argument("--command-port", type=int, default=5555)
    parser.add_argument("--state-port", type=int, default=5556)
    parser.add_argument("--duration", type=float, default=2.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config = ContinuumPMACConfig(
        id="continuum_smoke",
        remote_ip=args.remote_ip,
        command_port=args.command_port,
        state_port=args.state_port,
    )
    robot = ContinuumPMAC(config)
    zero_action = dict.fromkeys(robot.action_features, 0.0)

    robot.connect()
    try:
        start = time.perf_counter()
        while time.perf_counter() - start < args.duration:
            observation = robot.get_observation()
            robot.send_action(zero_action)
            print(f"observation={observation} | status={robot.driver_status}")
            time.sleep(0.05)
    finally:
        robot.disconnect()


if __name__ == "__main__":
    main()
