from __future__ import annotations

import ctypes
import threading
import time
from typing import Any

import numpy as np

from lerobot.teleoperators.teleoperator import Teleoperator
from lerobot.types import RobotAction
from lerobot.utils.errors import DeviceNotConnectedError

from .config_omega_continuum import OmegaContinuumConfig
from .mapping import ACTION_FIELDS, OmegaContinuumMapper


class OmegaContinuum(Teleoperator):
    """Force Dimension Omega translation source for continuum tip control."""

    config_class = OmegaContinuumConfig
    name = "omega_continuum"

    def __init__(self, config: OmegaContinuumConfig):
        super().__init__(config)
        self.config = config
        self._dhd = None
        self._drd = None
        if not config.simulate:
            try:
                import forcedimension_core.dhd as dhd
                import forcedimension_core.drd as drd
            except ImportError as exc:
                raise ImportError(
                    "forcedimension-core is required for omega_continuum. "
                    "Install the plugin with its dependencies."
                ) from exc
            self._dhd = dhd
            self._drd = drd
        self._position = np.zeros(3, dtype=float)
        self._orientation = np.eye(3, dtype=float)
        self._gripper = ctypes.pointer(ctypes.c_double(0.0))
        self._is_connected = False
        self._clutch_pressed = False
        self._clutch_active = False
        self._clutch_listener = None
        self._clutch_lock = threading.Lock()
        self._action_anchor = np.zeros(len(ACTION_FIELDS), dtype=float)
        self._last_action = np.zeros(len(ACTION_FIELDS), dtype=float)
        self._last_axis_debug_at = 0.0
        self._last_axis_debug_signature: tuple[str, str] | None = None
        self._last_axis_debug_omega_mm = 0.0
        self._last_axis_debug_tip_mm = 0.0
        self._last_rotation_debug_at = 0.0
        self._last_rotation_debug_signature: tuple[str, str] | None = None
        self._last_rotation_debug_omega_rad = 0.0
        self._last_rotation_debug_tip_rad = 0.0
        self._mapper = OmegaContinuumMapper(
            scale_xyz=(config.scale_x, config.scale_y, config.scale_z),
            max_delta_xyz=(config.max_delta_x, config.max_delta_y, config.max_delta_z),
            deadband_m=config.deadband_m,
            omega_map=config.omega_map,
            rotation_map=config.rotation_map,
            position_offset_xyz=(
                config.position_offset_x,
                config.position_offset_y,
                config.position_offset_z,
            ),
            rotation_scale_xyz=(
                config.rotation_scale_x,
                config.rotation_scale_y,
                config.rotation_scale_z,
            ),
            max_rotation_xyz=(
                config.max_rotation_x,
                config.max_rotation_y,
                config.max_rotation_z,
            ),
            rotation_deadband_rad=config.rotation_deadband_rad,
        )

    @property
    def action_features(self) -> dict[str, type]:
        return dict.fromkeys(ACTION_FIELDS, float)

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        return None

    def configure(self) -> None:
        return None

    def _reset_clutch_state(self) -> None:
        with self._clutch_lock:
            self._clutch_pressed = False
        self._clutch_active = False
        self._action_anchor.fill(0.0)
        self._last_action.fill(0.0)
        self._last_axis_debug_at = 0.0
        self._last_axis_debug_signature = None
        self._last_axis_debug_omega_mm = 0.0
        self._last_axis_debug_tip_mm = 0.0
        self._last_rotation_debug_at = 0.0
        self._last_rotation_debug_signature = None
        self._last_rotation_debug_omega_rad = 0.0
        self._last_rotation_debug_tip_rad = 0.0

    def connect(self, calibrate: bool = True) -> None:
        if self._is_connected:
            raise RuntimeError(f"{self} is already connected.")
        self._reset_clutch_state()

        if self.config.simulate:
            self._mapper.set_zero(np.zeros(3, dtype=float), np.eye(3, dtype=float))
            self._start_clutch_listener()
            self._is_connected = True
            return

        assert self._dhd is not None
        assert self._drd is not None
        self._dhd.close()
        if self._drd.open() < 0:
            raise DeviceNotConnectedError(
                "Could not open the Force Dimension DRD device: "
                f"{self._dhd.errorGetLastStr()}"
            )

        try:
            if not self._drd.isInitialized() and self._drd.autoInit() < 0:
                raise DeviceNotConnectedError("Force Dimension Omega automatic initialization failed.")

            self._dhd.enableForce(True)
            self._drd.stop(True)
            self._is_connected = True
            zero_positions = []
            zero_orientations = []
            for _ in range(self.config.zero_samples):
                position, orientation = self._read_pose()
                zero_positions.append(position)
                zero_orientations.append(orientation)
                if self.config.zero_sample_period_s > 0.0:
                    time.sleep(self.config.zero_sample_period_s)
            mean_orientation = np.mean(zero_orientations, axis=0)
            u, _, vt = np.linalg.svd(mean_orientation)
            zero_orientation = u @ vt
            if np.linalg.det(zero_orientation) < 0.0:
                u[:, -1] *= -1.0
                zero_orientation = u @ vt
            self._mapper.set_zero(np.mean(zero_positions, axis=0), zero_orientation)
            self._start_clutch_listener()
        except Exception:
            try:
                self._stop_clutch_listener()
                self._drd.stop(False)
                self._drd.close()
                self._dhd.close()
            finally:
                self._is_connected = False
            raise

    def _read_pose(self) -> tuple[np.ndarray, np.ndarray]:
        if self.config.simulate:
            return np.zeros(3, dtype=float), np.eye(3, dtype=float)

        assert self._dhd is not None
        result = self._dhd.getPositionAndOrientationFrame(self._position, self._orientation)
        if isinstance(result, int) and result < 0:
            raise DeviceNotConnectedError("Failed to read the Force Dimension Omega pose.")
        self._dhd.getGripperAngleDeg(self._gripper)
        return self._position.copy(), self._orientation.copy()

    def _read_position(self) -> np.ndarray:
        position, _ = self._read_pose()
        return position

    def _key_name(self, key: Any) -> str | None:
        try:
            from pynput import keyboard
        except ImportError:
            return None

        if key == keyboard.Key.space:
            return "space"
        try:
            return key.char.lower() if key.char else None
        except AttributeError:
            return None

    def _start_clutch_listener(self) -> None:
        if not self.config.clutch_enabled or self._clutch_listener is not None:
            return
        try:
            from pynput import keyboard
        except ImportError as exc:
            raise ImportError(
                "pynput is required for omega_continuum clutch support. "
                "Install pynput or run with --teleop.clutch_enabled=false."
            ) from exc

        clutch_key = self.config.clutch_key

        def on_press(key: Any) -> None:
            if self._key_name(key) == clutch_key:
                with self._clutch_lock:
                    self._clutch_pressed = True

        def on_release(key: Any) -> None:
            if self._key_name(key) == clutch_key:
                with self._clutch_lock:
                    self._clutch_pressed = False

        self._clutch_listener = keyboard.Listener(on_press=on_press, on_release=on_release)
        self._clutch_listener.start()

    def _stop_clutch_listener(self) -> None:
        if self._clutch_listener is None:
            return
        self._clutch_listener.stop()
        self._clutch_listener = None
        with self._clutch_lock:
            self._clutch_pressed = False
        self._clutch_active = False

    def _is_clutch_pressed(self) -> bool:
        with self._clutch_lock:
            return self._clutch_pressed

    def _action_array_to_dict(self, action: np.ndarray) -> RobotAction:
        return {
            key: float(value)
            for key, value in zip(ACTION_FIELDS, action, strict=True)
        }

    def _action_dict_to_array(self, action: RobotAction) -> np.ndarray:
        return np.asarray([float(action[key]) for key in ACTION_FIELDS], dtype=float)

    def _clip_action(self, action: np.ndarray) -> np.ndarray:
        limits = np.asarray(
            [
                self.config.max_delta_x,
                self.config.max_delta_y,
                self.config.max_delta_z,
                self.config.max_rotation_x,
                self.config.max_rotation_y,
                self.config.max_rotation_z,
            ],
            dtype=float,
        )
        return np.clip(action, -limits, limits)

    @staticmethod
    def _signed_axis_label(value: float, axis_index: int) -> str:
        sign = "+" if value >= 0.0 else "-"
        return f"{sign}{'XYZ'[axis_index]}"

    @staticmethod
    def _signed_rotation_label(value: float, axis_index: int) -> str:
        sign = "+" if value >= 0.0 else "-"
        return f"{sign}R{'XYZ'[axis_index]}"

    def _maybe_print_axis_debug(
        self,
        position: np.ndarray,
        orientation: np.ndarray,
        action: RobotAction,
    ) -> None:
        if not self.config.axis_debug_enabled:
            return

        now = time.perf_counter()
        omega_delta = self._mapper.omega_position_delta(position, orientation)
        tip_delta = np.asarray(
            [
                action["tip_delta_x"],
                action["tip_delta_y"],
                action["tip_delta_z"],
            ],
            dtype=float,
        )

        if now - self._last_axis_debug_at >= 1.0 / self.config.axis_debug_hz:
            omega_abs = np.abs(omega_delta)
            tip_abs = np.abs(tip_delta)
            omega_index = int(np.argmax(omega_abs))
            tip_index = int(np.argmax(tip_abs))
            omega_mm = float(omega_delta[omega_index] * 1000.0)
            tip_mm = float(tip_delta[tip_index] * 1000.0)
            omega_mag_mm = abs(omega_mm)
            tip_mag_mm = abs(tip_mm)

            if (
                omega_mag_mm >= self.config.axis_debug_min_omega_mm
                or tip_mag_mm >= self.config.axis_debug_min_tip_mm
            ):
                omega_label = self._signed_axis_label(omega_mm, omega_index)
                tip_label = self._signed_axis_label(tip_mm, tip_index)
                signature = (omega_label, tip_label)
                changed_enough = (
                    signature != self._last_axis_debug_signature
                    or abs(omega_mag_mm - self._last_axis_debug_omega_mm)
                    >= self.config.axis_debug_change_omega_mm
                    or abs(tip_mag_mm - self._last_axis_debug_tip_mm)
                    >= self.config.axis_debug_change_tip_mm
                )
                if changed_enough:
                    self._last_axis_debug_at = now
                    self._last_axis_debug_signature = signature
                    self._last_axis_debug_omega_mm = omega_mag_mm
                    self._last_axis_debug_tip_mm = tip_mag_mm

                    line = (
                        "axis | "
                        f"omega {omega_label} {omega_mag_mm:5.1f} mm "
                        f"-> tip {tip_label} {tip_mag_mm:5.1f} mm"
                    )
                    if self.config.axis_debug_show_xyz:
                        line += (
                            " | "
                            f"omegaXYZ=[{omega_delta[0] * 1000:+5.1f},"
                            f"{omega_delta[1] * 1000:+5.1f},"
                            f"{omega_delta[2] * 1000:+5.1f}]mm "
                            f"tipXYZ=[{tip_delta[0] * 1000:+5.1f},"
                            f"{tip_delta[1] * 1000:+5.1f},"
                            f"{tip_delta[2] * 1000:+5.1f}]mm"
                        )
                    print(line)

        if now - self._last_rotation_debug_at < 1.0 / self.config.axis_debug_hz:
            return

        omega_rotation = self._mapper.omega_rotation_delta(orientation)
        tip_rotation = np.asarray(
            [
                action["tip_delta_rx"],
                action["tip_delta_ry"],
                action["tip_delta_rz"],
            ],
            dtype=float,
        )
        omega_rot_index = int(np.argmax(np.abs(omega_rotation)))
        tip_rot_index = int(np.argmax(np.abs(tip_rotation)))
        omega_rot = float(omega_rotation[omega_rot_index])
        tip_rot = float(tip_rotation[tip_rot_index])
        omega_rot_mag = abs(omega_rot)
        tip_rot_mag = abs(tip_rot)

        if (
            omega_rot_mag < self.config.axis_debug_min_omega_rot_rad
            and tip_rot_mag < self.config.axis_debug_min_tip_rot_rad
        ):
            return

        omega_rot_label = self._signed_rotation_label(omega_rot, omega_rot_index)
        tip_rot_label = self._signed_rotation_label(tip_rot, tip_rot_index)
        rotation_signature = (omega_rot_label, tip_rot_label)
        rotation_changed_enough = (
            rotation_signature != self._last_rotation_debug_signature
            or abs(omega_rot_mag - self._last_rotation_debug_omega_rad)
            >= self.config.axis_debug_change_omega_rot_rad
            or abs(tip_rot_mag - self._last_rotation_debug_tip_rad)
            >= self.config.axis_debug_change_tip_rot_rad
        )
        if not rotation_changed_enough:
            return

        self._last_rotation_debug_at = now
        self._last_rotation_debug_signature = rotation_signature
        self._last_rotation_debug_omega_rad = omega_rot_mag
        self._last_rotation_debug_tip_rad = tip_rot_mag
        line = (
            "rot  | "
            f"omega {omega_rot_label} {omega_rot_mag:5.3f} rad "
            f"-> tip {tip_rot_label} {tip_rot_mag:5.3f} rad"
        )
        if self.config.axis_debug_show_xyz:
            line += (
                " | "
                f"omegaR=[{omega_rotation[0]:+5.3f},"
                f"{omega_rotation[1]:+5.3f},"
                f"{omega_rotation[2]:+5.3f}] "
                f"tipR=[{tip_rotation[0]:+5.3f},"
                f"{tip_rotation[1]:+5.3f},"
                f"{tip_rotation[2]:+5.3f}]"
            )
        print(line)

    def get_action(self) -> RobotAction:
        if not self._is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        position, orientation = self._read_pose()
        if self.config.clutch_enabled and self._is_clutch_pressed():
            if not self._clutch_active:
                self._action_anchor = self._last_action.copy()
                self._clutch_active = True
                print("Omega clutch engaged: hold robot target and recenter Omega.")
            self._mapper.set_zero(position, orientation)
            self._last_action = self._clip_action(self._action_anchor)
            return self._action_array_to_dict(self._last_action)

        if self._clutch_active:
            self._clutch_active = False
            print("Omega clutch released: continue from recentered Omega pose.")

        mapped_action = self._mapper.map_pose(position, orientation)
        self._maybe_print_axis_debug(position, orientation, mapped_action)
        mapped = self._action_dict_to_array(mapped_action)
        self._last_action = self._clip_action(self._action_anchor + mapped)
        return self._action_array_to_dict(self._last_action)

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        return None

    def disconnect(self) -> None:
        if not self._is_connected:
            return
        if self.config.simulate:
            self._stop_clutch_listener()
            self._is_connected = False
            return

        assert self._dhd is not None
        assert self._drd is not None
        try:
            self._stop_clutch_listener()
            self._drd.stop(False)
            self._drd.close()
            self._dhd.close()
        finally:
            self._is_connected = False
