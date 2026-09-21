from __future__ import annotations

import argparse
import csv
import math
import time
from collections.abc import Mapping
from pathlib import Path
from typing import Any

import zmq
from pynput import keyboard

from continuum_sdk.transport.zmq_protocol import build_command_message, build_hold_message
from continuum_sdk.kinematics.joint_motor_model import MotorAngles


ACTION_KEYS = (
    "tip_delta_x",
    "tip_delta_y",
    "tip_delta_z",
    "tip_delta_rx",
    "tip_delta_ry",
    "tip_delta_rz",
)


class MujocoMirror:
    """Display the real driver's logical feedback in the MuJoCo model."""

    def __init__(self, xml_path: Path, physics_steps: int = 100,
                 config_path: str = "config/continuum.yaml") -> None:
        try:
            import mujoco as mj
            import mujoco.viewer as mjviewer
        except ImportError as exc:
            raise RuntimeError(
                "MuJoCo mirror requires the mujoco package; run with "
                '`uv run --with mujoco ...`.'
            ) from exc

        from continuum_sdk.core.config_loader import load_continuum_config
        from continuum_sdk.core.factory import build_tendon_mapper

        self.motor_model = build_tendon_mapper(load_continuum_config(config_path)).model

        if not xml_path.exists():
            raise FileNotFoundError(f"MuJoCo XML not found: {xml_path}")
        self.mj = mj
        self.model = mj.MjModel.from_xml_path(str(xml_path))
        self.data = mj.MjData(self.model)
        self.viewer = mjviewer.launch_passive(self.model, self.data, show_left_ui=True)
        self.physics_steps = max(1, int(physics_steps))
        self.actuators = {
            name: int(mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_ACTUATOR, name))
            for name in ("a_x", "a_y", "c_x", "c_y", "lin_pos_control")
        }
        missing = [name for name, index in self.actuators.items() if index < 0]
        if missing:
            self.viewer.close()
            raise RuntimeError(f"MuJoCo model is missing actuators: {missing}")

    def _set_ctrl(self, name: str, value: float) -> None:
        index = self.actuators[name]
        lower, upper = self.model.actuator_ctrlrange[index]
        self.data.ctrl[index] = max(float(lower), min(float(upper), float(value)))

    def update(self, state_message: Mapping[str, Any] | None) -> None:
        if state_message is not None:
            state = state_message.get("state")
            if isinstance(state, Mapping):
                motor = MotorAngles(
                    alpha1=float(state.get("axis_1_pos", 0.0)),
                    alpha2=float(state.get("axis_2_pos", 0.0)),
                    alpha3=float(state.get("axis_3_pos", 0.0)),
                    alpha4=float(state.get("axis_4_pos", 0.0)),
                )
                joint = self.motor_model.motor_angles_to_joint(motor)
                self._set_ctrl("a_x", -joint.theta_a * math.cos(joint.phi_a))
                self._set_ctrl("a_y", -joint.theta_a * math.sin(joint.phi_a))
                self._set_ctrl("c_y", -joint.theta_c * math.cos(joint.phi_c))
                self._set_ctrl("c_x", -joint.theta_c * math.sin(joint.phi_c))
                self._set_ctrl("lin_pos_control", float(state.get("axis_5_pos", 0.0)))

        for _ in range(self.physics_steps):
            self.mj.mj_step(self.model, self.data)
        if self.viewer.is_running():
            self.viewer.sync()

    def is_running(self) -> bool:
        return bool(self.viewer.is_running())

    def close(self) -> None:
        try:
            self.viewer.close()
        except Exception:
            pass


class KeyboardDevice:
    def __init__(self) -> None:
        self.pressed: set[str] = set()
        self.stop_requested = False
        self.listener = keyboard.Listener(on_press=self._on_press, on_release=self._on_release)

    @staticmethod
    def key_name(key) -> str | None:
        if key == keyboard.Key.esc:
            return "esc"
        try:
            return key.char.lower() if key.char else None
        except AttributeError:
            return None

    def _on_press(self, key) -> None:
        name = self.key_name(key)
        if name == "esc":
            self.stop_requested = True
        elif name is not None:
            self.pressed.add(name)

    def _on_release(self, key) -> None:
        name = self.key_name(key)
        if name is not None:
            self.pressed.discard(name)

    def start(self) -> None:
        self.listener.start()

    def stop(self) -> None:
        self.listener.stop()


class PosePlanner:
    def __init__(
        self,
        linear_speed_m_s: float,
        angular_speed_rad_s: float,
        max_delta_xyz_m: tuple[float, float, float],
        max_delta_rxz_rad: tuple[float, float],
    ) -> None:
        self.linear_speed = float(linear_speed_m_s)
        self.angular_speed = float(angular_speed_rad_s)
        self.max_delta = [*map(float, max_delta_xyz_m), *map(float, max_delta_rxz_rad)]
        self.delta = [0.0] * 5

    def update(self, keys: set[str], dt_s: float) -> list[float]:
        direction = [0.0] * 5
        # World-frame commissioning layout:
        # a/d = X, q/e = Y (insertion), w/s = Z,
        # u/j = RX, i/k = RZ. World Ry is unavailable because it maps to
        # continuum-frame Rz, the disabled tool-roll DOF.
        if "a" in keys:
            direction[0] -= 1.0
        if "d" in keys:
            direction[0] += 1.0
        if "q" in keys:
            direction[1] += 1.0
        if "e" in keys:
            direction[1] -= 1.0
        if "w" in keys:
            direction[2] += 1.0
        if "s" in keys:
            direction[2] -= 1.0
        if "u" in keys:
            direction[3] += 1.0
        if "j" in keys:
            direction[3] -= 1.0
        if "i" in keys:
            direction[4] += 1.0
        if "k" in keys:
            direction[4] -= 1.0

        for index in range(3):
            self.delta[index] += direction[index] * self.linear_speed * dt_s
        for index in (3, 4):
            self.delta[index] += direction[index] * self.angular_speed * dt_s
        self.delta = [
            max(-limit, min(limit, value))
            for value, limit in zip(self.delta, self.max_delta)
        ]
        return self.delta.copy()


def zero_action() -> dict[str, float]:
    return dict.fromkeys(ACTION_KEYS, 0.0)


def action_from_delta(delta: list[float]) -> dict[str, float]:
    action = zero_action()
    for index, axis in enumerate(("x", "y", "z", "rx", "rz")):
        action[f"tip_delta_{axis}"] = float(delta[index])
    return action


def blend_action(start: Mapping[str, float], end: Mapping[str, float], alpha: float) -> dict[str, float]:
    alpha = max(0.0, min(1.0, alpha))
    return {
        key: float(start[key]) + alpha * (float(end[key]) - float(start[key]))
        for key in ACTION_KEYS
    }


def latest_message(socket: zmq.Socket) -> dict[str, Any] | None:
    latest = None
    while True:
        try:
            latest = socket.recv_json(flags=zmq.NOBLOCK)
        except zmq.Again:
            return latest


def state_text(message: Mapping[str, Any] | None) -> str:
    if not message or not isinstance(message.get("applied_action"), Mapping):
        return "applied=n/a"
    applied = message["applied_action"]
    feedback = message.get("feedback_pulses") or message.get("feedback")
    target = message.get("target_pulses") or message.get("target")
    motor_text = ""
    if isinstance(feedback, (list, tuple)) and len(feedback) >= 5:
        motor_text = " motors=" + str([int(v) for v in feedback[:5]])
        if isinstance(target, (list, tuple)) and len(target) >= 5:
            motor_text += " d=" + str([int(a)-int(b) for a,b in zip(feedback[:5], target[:5])])
    return (
        "applied="
        f"xyz=[{float(applied.get('tip_delta_x', 0.0)) * 1000:+.1f}, "
        f"{float(applied.get('tip_delta_y', 0.0)) * 1000:+.1f}, "
        f"{float(applied.get('tip_delta_z', 0.0)) * 1000:+.1f}]mm "
        f"rxz=[{float(applied.get('tip_delta_rx', 0.0)):+.3f}, "
        f"{float(applied.get('tip_delta_rz', 0.0)):+.3f}]rad" + motor_text
    )


def send_action(socket: zmq.Socket, sequence: int, action: Mapping[str, float]) -> None:
    socket.send_json(build_command_message(sequence, action))


def ramp_to_zero(
    command_socket: zmq.Socket,
    state_socket: zmq.Socket,
    sequence: int,
    current: Mapping[str, float],
    duration_s: float,
    rate_hz: float,
    mujoco_mirror: MujocoMirror | None = None,
) -> int:
    steps = max(1, int(round(duration_s * rate_hz)))
    interval = 1.0 / rate_hz
    target = zero_action()
    next_call = time.perf_counter()
    for index in range(steps + 1):
        action = blend_action(current, target, index / steps)
        send_action(command_socket, sequence, action)
        sequence += 1
        state_message = latest_message(state_socket)
        if mujoco_mirror is not None:
            mujoco_mirror.update(state_message)
        if index == 0 or index == steps or index % max(1, int(rate_hz)) == 0:
            print(f"return {index:3d}/{steps} | {state_text(state_message)}")
        next_call += interval
        remaining = next_call - time.perf_counter()
        if remaining > 0.0:
            time.sleep(remaining)
        else:
            next_call = time.perf_counter()
    return sequence


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Keyboard control of XYZ and world RX/RZ through the running continuum driver."
    )
    parser.add_argument("--remote-ip", default="127.0.0.1")
    parser.add_argument("--command-port", type=int, default=5555)
    parser.add_argument("--state-port", type=int, default=5556)
    parser.add_argument("--rate-hz", type=float, default=50.0)
    parser.add_argument("--linear-speed-m-s", type=float, default=0.01)
    parser.add_argument("--angular-speed-rad-s", type=float, default=0.1)
    parser.add_argument("--max-delta-x-mm", type=float, default=70.0)
    parser.add_argument("--max-delta-y-mm", type=float, default=70.0)
    parser.add_argument("--max-delta-z-mm", type=float, default=70.0)
    parser.add_argument("--max-delta-rx-rad", type=float, default=4.0)
    parser.add_argument(
        "--max-delta-rz-rad",
        dest="max_delta_rz_rad", type=float, default=4.0,
        help="World Rz limit in radians.",
    )
    parser.add_argument("--return-s", type=float, default=5.0)
    parser.add_argument(
        "--mujoco",
        action="store_true",
        help="Open a MuJoCo mirror driven by the real driver's feedback state.",
    )
    parser.add_argument(
        "--mujoco-xml",
        type=Path,
        default=None,
        help="MuJoCo XML path (defaults to the sibling surgical_continuum_robot scene).",
    )
    parser.add_argument("--mujoco-physics-steps", type=int, default=100)
    parser.add_argument("--config", default="config/continuum.yaml",
                        help="Continuum calibration for feedback decoding; match the driver's config.")
    parser.add_argument("--log-csv", default=None, help="CSV path for timestamped command/feedback logging.")
    return parser.parse_args()


def default_mujoco_xml() -> Path:
    return (
        Path(__file__).resolve().parents[2]
        / "surgical_continuum_robot"
        / "model"
        / "continuum_robot"
        / "mjcf"
        / "scene_single_arm.xml"
    )


def main() -> None:
    args = parse_args()
    if args.rate_hz <= 0.0 or args.linear_speed_m_s <= 0.0 or args.angular_speed_rad_s <= 0.0:
        raise SystemExit("rate and speeds must be positive")
    if args.return_s <= 0.0:
        raise SystemExit("--return-s must be positive")

    context = zmq.Context()
    command_socket = context.socket(zmq.PUSH)
    command_socket.setsockopt(zmq.SNDHWM, 1)
    command_socket.setsockopt(zmq.LINGER, 0)
    command_socket.connect(f"tcp://{args.remote_ip}:{args.command_port}")
    state_socket = context.socket(zmq.PULL)
    state_socket.setsockopt(zmq.CONFLATE, 1)
    state_socket.setsockopt(zmq.RCVHWM, 1)
    state_socket.setsockopt(zmq.LINGER, 0)
    state_socket.connect(f"tcp://{args.remote_ip}:{args.state_port}")

    mujoco_mirror = None
    if args.mujoco:
        mujoco_mirror = MujocoMirror(
            args.mujoco_xml.resolve() if args.mujoco_xml is not None else default_mujoco_xml(),
            physics_steps=args.mujoco_physics_steps,
            config_path=args.config,
        )

    planner = PosePlanner(
        linear_speed_m_s=args.linear_speed_m_s,
        angular_speed_rad_s=args.angular_speed_rad_s,
        max_delta_xyz_m=(
            abs(args.max_delta_x_mm) / 1000.0,
            abs(args.max_delta_y_mm) / 1000.0,
            abs(args.max_delta_z_mm) / 1000.0,
        ),
        max_delta_rxz_rad=(abs(args.max_delta_rx_rad), abs(args.max_delta_rz_rad)),
    )
    kbd = KeyboardDevice()
    sequence = 0
    current = zero_action()
    log_path = args.log_csv or f"teleop_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    fields = ["time", "keys", *[f"target_{a}" for a in ("x","y","z","rx","ry","rz")],
              *[f"feedback_p{i}" for i in range(1, 6)], *[f"target_p{i}" for i in range(1, 6)],
              *[f"pulse_err{i}" for i in range(1, 6)]]
    log_file = open(log_path, "w", newline="", encoding="utf-8")
    log_writer = csv.DictWriter(log_file, fieldnames=fields)
    log_writer.writeheader()
    interval = 1.0 / args.rate_hz
    print("Keyboard XYZ+RX/RZ teleop started (world frame: +X right, +Y insertion, +Z up).")
    print("A/D: X-,X+ | Q/E: Y+,Y- | W/S: Z+,Z-")
    print("U/J: RX+,RX- | I/K: RZ+,RZ- | world Ry: disabled | Esc/Ctrl+C: return to startup position")
    print(
        f"Limits: xyz=[{args.max_delta_x_mm:g}, {args.max_delta_y_mm:g}, "
        f"{args.max_delta_z_mm:g}] mm, "
        f"rxz=[{args.max_delta_rx_rad:g}, {args.max_delta_rz_rad:g}] rad"
    )

    try:
        kbd.start()
        next_call = time.perf_counter()
        while not kbd.stop_requested and (
            mujoco_mirror is None or mujoco_mirror.is_running()
        ):
            delta = planner.update(kbd.pressed, interval)
            current = action_from_delta(delta)
            send_action(command_socket, sequence, current)
            sequence += 1
            state_message = latest_message(state_socket)
            row = {"time": time.time(), "keys": "+".join(sorted(kbd.pressed))}
            for axis in ("x", "y", "z", "rx", "ry", "rz"):
                row[f"target_{axis}"] = current[f"tip_delta_{axis}"]
            if isinstance(state_message, Mapping):
                for name in fields:
                    if name not in row:
                        row[name] = state_message.get(name, "")
            log_writer.writerow(row)
            log_file.flush()
            if mujoco_mirror is not None:
                mujoco_mirror.update(state_message)
            if sequence == 1 or sequence % max(1, int(args.rate_hz)) == 0:
                print(
                    f"target xyz=[{delta[0] * 1000:+.1f}, {delta[1] * 1000:+.1f}, "
                    f"{delta[2] * 1000:+.1f}]mm "
                    f"rxz=[{delta[3]:+.3f}, {delta[4]:+.3f}]rad | "
                    f"{state_text(state_message)}"
                )
            next_call += interval
            remaining = next_call - time.perf_counter()
            if remaining > 0.0:
                time.sleep(remaining)
            else:
                next_call = time.perf_counter()
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received.")
    finally:
        log_file.close()
        kbd.stop()
        try:
            sequence = ramp_to_zero(
                command_socket,
                state_socket,
                sequence,
                current,
                args.return_s,
                args.rate_hz,
                mujoco_mirror,
            )
            command_socket.send_json(build_hold_message(sequence))
            print("Returned to startup target and sent hold command.")
        finally:
            if mujoco_mirror is not None:
                mujoco_mirror.close()
            command_socket.close()
            state_socket.close()
            context.term()


if __name__ == "__main__":
    main()
