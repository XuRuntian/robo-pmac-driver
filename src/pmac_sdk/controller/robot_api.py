from ..core.config_model import PMACConfig
from ..comms.modbus_client import ModbusClient32Bit
from ..hardware.ssh_manager import PMACHardwareManager
import logging
import math
from .protocol import PMACProtocol

logger = logging.getLogger(__name__)

class PMACRobotController:
    """Five-axis PMAC control using acknowledged protocol-v2 mailboxes."""
    def __init__(self, config: PMACConfig):
        self.config = config
        self.modbus = ModbusClient32Bit(config.ip, config.modbus_port, config.slave_id, config.modbus_timeout_s)
        self.hw_manager = PMACHardwareManager(config.ip, config.ssh_user, config.ssh_pass)
        self.base_positions = [0, 0, 0, 0, 0]
        self.protocol = PMACProtocol(self.modbus, config.handshake_timeout_s, config.poll_interval_s)
        self._session_active = False
        self._boot_requested = False
        self._stream_faulted = False
        self.pvt_axis5_max_step = None

    def hardware_boot(self):
        """Request the shared PLC reset; connect_and_home waits for completion."""
        if not self.modbus.connect():
            raise ConnectionError("Cannot connect to PMAC Modbus.")
        self._session_active = True
        self.protocol.write(216, [0])
        self.hw_manager.reset_with_plc4()
        self._boot_requested = True

    def connect_and_home(self):
        """Capture a fresh encoder reference. This does not physically home axes."""
        if not self._boot_requested:
            self.hardware_boot()
        self.protocol.wait(
            lambda: self.modbus.read_int32_array(216, 1)[0] == 1,
            "PLC reset completion", timeout_s=10.0,
        )
        self.protocol.status()  # Reject mismatched controller firmware.
        self.pvt_axis5_max_step = self.modbus.read_int32_array(218, 1)[0]
        if self.pvt_axis5_max_step <= 0:
            raise RuntimeError("PMAC reported an invalid axis-5 PVT step limit.")
        current = self.read_positions()
        if not any(current):
            raise RuntimeError("PMAC returned an all-zero startup reference.")
        self.protocol.write(0, current)
        self.protocol.write(40, [200000])
        self.protocol.write(50, [0] * 5)
        self.base_positions = current
        self.hw_manager.start_prog()
        self.protocol.wait(
            lambda: self.protocol.status().ready == 1,
            "PVT program startup", timeout_s=self.config.startup_timeout_s,
        )
        self._stream_faulted = False
        self._boot_requested = False
        print(f"PMAC ready; fresh encoder reference: {self.base_positions}")

    def safe_boot_and_home(self, use_plc4_reset: bool = False):
        """Both legacy choices now use the same acknowledged PLC reset path."""
        try:
            self.hardware_boot()
            self.connect_and_home()
        except Exception:
            try:
                self.close()
            except Exception:
                logger.exception("PMAC cleanup also failed after startup failure")
            raise

    def move_joints(self, target_pulses: list, move_time: int = 500, accel: int = 100, scurve: int = 50):
        raise NotImplementedError(
            "The paired PLC does not implement the legacy point-to-point protocol. "
            "Use a timed trajectory through move_pvt_stream()."
        )

    def move_single_joint_angle(self, joint_idx: int, angle: float, move_time: int = 500, accel: int = 100, scurve: int = 50):
        """按照你的原版逻辑换算角度，增加速度控制和【方向系数】"""
        targets = list(self.base_positions)
        
        # 引入方向系数 (如果你在 config 没配，默认给 1)
        direction = getattr(self.config, 'joint_directions', [1, 1, 1, 1, 1])[joint_idx]
        
        # 计算脉冲时乘以方向系数
        target_pulses = int(self.base_positions[joint_idx] + (angle * self.config.pulses_per_degree * direction))
        targets[joint_idx] = target_pulses
        
        print(f"🎯 正在向电机 {joint_idx+1} 发送指令: 目标角度 {angle}° (方向:{direction}), 绝对脉冲 {target_pulses}")
        print(f"⏱️  期望耗时: {move_time}ms, 加减速: {accel}ms，s型时间{scurve}")
        
        self.move_joints(targets, move_time=move_time, accel=accel, scurve=scurve)
        
    def set_current_as_absolute_zero(self):
        current_pos = self.read_positions()
        self.config.zero_offsets = current_pos
        self.base_positions = current_pos
        print(f"✅ 已标定绝对零点偏置: {self.config.zero_offsets}")

    def read_positions(self) -> list[int]:
        positions, _ = self.protocol.snapshot()
        return positions

    def move_to_absolute_angle(self, joint_idx: int, absolute_angle: float, move_time: int = 500, accel: int = 100, scurve: int = 50):
        """绝对控制：增加【方向系数】"""
        current_pos = self.read_positions()
        targets = list(current_pos)
        
        # 引入方向系数
        direction = getattr(self.config, 'joint_directions', [1, 1, 1, 1, 1])[joint_idx]
        
        # 计算脉冲时乘以方向系数
        target_pulses = int(self.config.zero_offsets[joint_idx] + (absolute_angle * self.config.pulses_per_degree * direction))
        targets[joint_idx] = target_pulses
        
        print(f"🎯 绝对控制 -> 电机 {joint_idx+1} 目标角度 {absolute_angle}°, 对应脉冲 {target_pulses}")
        
        self.move_joints(targets, move_time=move_time, accel=accel, scurve=scurve)
        
    def move_pvt_stream(self, target_pulses: list, velocities: list, move_time: float):
        """Submit one acknowledged frame: counts, counts/ms and milliseconds."""
        if self._stream_faulted:
            raise RuntimeError("PVT is faulted; a complete restart is required.")
        if len(target_pulses) != 5 or len(velocities) != 5:
            raise ValueError("PVT requires exactly five positions and velocities.")
        if not all(math.isfinite(v) for v in [*target_pulses, *velocities, move_time]):
            raise ValueError("PVT values must be finite.")
        if not 5.0 < move_time <= 500.0:
            raise ValueError("PVT segment duration must be >5 and <=500 ms.")
        positions = [int(p) for p in target_pulses]
        speeds = [int(v * 10000.0) for v in velocities]
        duration = int(move_time * 10000.0)
        if duration <= 50000:
            raise ValueError("PVT duration rounds to <=5 ms in the wire format.")
        for value in [*positions, *speeds, duration]:
            if not -(2**31) <= value < 2**31:
                raise ValueError("Scaled PVT value exceeds signed int32 range.")
        try:
            self.protocol.submit_pvt(positions, speeds, duration)
        except Exception:
            self._stream_faulted = True
            try:
                self.stop_motion()
            except Exception:
                logger.exception("PVT failed and the coordinate abort could not be confirmed")
            raise

    def stop_motion(self):
        with self.modbus.transaction_lock:
            self.hw_manager.stop_pvt()
            self._session_active = False

    def close(self):
        try:
            if self._session_active:
                self.stop_motion()
                self._session_active = False
        finally:
            self.modbus.disconnect()


class VisualHomingManager:
    """Acknowledged manual homing commands and protocol-v2 snapshots."""

    def __init__(self, modbus_client):
        self.modbus = modbus_client
        self.protocol = PMACProtocol(modbus_client)
        self.CMD_ADDRESS = 220

    def _command(self, command: int):
        with self.modbus.transaction_lock:
            self.protocol.status()
            self.protocol.wait(
                lambda: self.modbus.read_int32_array(self.CMD_ADDRESS, 1)[0] == 0,
                "previous homing command",
            )
            self.protocol.write(self.CMD_ADDRESS, [command])
            self.protocol.wait(
                lambda: self.modbus.read_int32_array(self.CMD_ADDRESS, 1)[0] == 0,
                "homing command acknowledgement",
            )

    def start_homing(self):
        _, status = self.protocol.snapshot()
        if status[0] != 0:
            raise RuntimeError("Cancel the previous homing session before starting again.")
        self._command(1)
        state, *_ = self.read_status()
        if state not in (1, 2):
            raise RuntimeError(f"PMAC rejected homing start (state={state}).")

    def stop_movement(self):
        """Stop and retain the state that permits an explicit zero confirmation."""
        self._command(2)
        self.protocol.wait(lambda: self.read_status()[0] == 3, "homing stop")

    def cancel_homing(self):
        """Stop, restore temporary current limits and leave without setting zero."""
        self._command(4)
        self.protocol.wait(lambda: self.read_status()[0] == 0, "homing cancellation")

    def confirm_and_set_zero(self):
        if self.read_status()[0] != 3:
            raise RuntimeError("Zero confirmation requires a stopped homing session.")
        self._command(3)

        def zeroed():
            positions, status = self.protocol.snapshot()
            if status[0] == 3 and status[1] == 10:
                raise RuntimeError("PMAC could not confirm encoder zero.")
            return status[0] == 0 and status[4] == 1 and positions[4] == 0

        self.protocol.wait(zeroed, "verified encoder zero", timeout_s=3.0)

    def read_status(self):
        _, status = self.protocol.snapshot()
        return status[:4]
