from __future__ import annotations

import ctypes
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
        self._mapper = OmegaContinuumMapper(
            scale_xyz=(config.scale_x, config.scale_y, config.scale_z),
            max_delta_xyz=(config.max_delta_x, config.max_delta_y, config.max_delta_z),
            deadband_m=config.deadband_m,
            omega_map=config.omega_map,
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

    def connect(self, calibrate: bool = True) -> None:
        if self._is_connected:
            raise RuntimeError(f"{self} is already connected.")

        if self.config.simulate:
            self._mapper.set_zero(np.zeros(3, dtype=float))
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
            for _ in range(self.config.zero_samples):
                zero_positions.append(self._read_position())
                if self.config.zero_sample_period_s > 0.0:
                    time.sleep(self.config.zero_sample_period_s)
            self._mapper.set_zero(np.mean(zero_positions, axis=0))
        except Exception:
            try:
                self._drd.stop(False)
                self._drd.close()
                self._dhd.close()
            finally:
                self._is_connected = False
            raise

    def _read_position(self) -> np.ndarray:
        if self.config.simulate:
            return np.zeros(3, dtype=float)

        assert self._dhd is not None
        result = self._dhd.getPositionAndOrientationFrame(self._position, self._orientation)
        if isinstance(result, int) and result < 0:
            raise DeviceNotConnectedError("Failed to read the Force Dimension Omega pose.")
        self._dhd.getGripperAngleDeg(self._gripper)
        return self._position.copy()

    def get_action(self) -> RobotAction:
        if not self._is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        return self._mapper.map_position(self._read_position())

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        return None

    def disconnect(self) -> None:
        if not self._is_connected:
            return
        if self.config.simulate:
            self._is_connected = False
            return

        assert self._dhd is not None
        assert self._drd is not None
        try:
            self._drd.stop(False)
            self._drd.close()
            self._dhd.close()
        finally:
            self._is_connected = False
