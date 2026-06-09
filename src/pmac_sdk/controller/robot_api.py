from ..core.config_model import PMACConfig
from ..comms.modbus_client import ModbusClient32Bit
from ..hardware.ssh_manager import PMACHardwareManager
import time # 新增 time 导入，供休眠使用

class PMACRobotController:
    """具身智能上层控制接口 (严格遵循原机通信逻辑)"""
    def __init__(self, config: PMACConfig):
        self.config = config
        self.modbus = ModbusClient32Bit(config.ip, config.modbus_port, config.slave_id)
        self.hw_manager = PMACHardwareManager(config.ip, config.ssh_user, config.ssh_pass)
        self.base_positions = [0, 0, 0, 0, 0]

    def hardware_boot(self):
        """执行硬件级别的上电和复位"""
        self.hw_manager.init_motors()

    def connect_and_home(self):
        if not self.modbus.connect():
            raise ConnectionError("❌ 无法连接到 PMAC，请检查网络设置。")
        
        # 1. 强制清空运动触发寄存器 (Modbus 地址 100 对应 PMAC P123)
        self.modbus.write_int32_array(address=100, values=[0])
        
        # 2. 读取当前真实位置 (已删去你原代码中重复粘贴的冗余部分)
        res = self.modbus.client.read_holding_registers(address=10, count=10, unit=self.config.slave_id)
        if not res.isError():
            regs = res.registers
            for i in range(5):
                self.base_positions[i] = self.modbus._registers_to_int32(regs[i*2], regs[i*2+1])
                
        # 3. 将当前位置立刻作为"目标位置"写回 Modbus 地址 0，防止 PLC 读到默认的 0
        self.modbus.write_int32_array(address=0, values=self.base_positions)
        
        print(f"✅ 系统就绪，基准位置已锁定: {self.base_positions}")
        
    def safe_boot_and_home(self, use_plc4_reset: bool = True):
        """
        安全的整合启动序列：
        1. SSH 电机上电
        2. Modbus 连接获取真实位置
        3. 清洗 PVT 缓冲区（防止暴走）
        4. 启动 PMAC 运动程序
        """
        import time
        
        # 1. 硬件准备
        if use_plc4_reset:
            self.hw_manager.reset_with_plc4()
        else:
            self.hw_manager.prepare_motors()
        time.sleep(1.0)
        
        # 2. 连接 Modbus，并读取电机的真实物理位置
        # 注意：这里调用的是下面那个底层的 connect_and_home 函数
        self.connect_and_home() 
        current_positions = self.base_positions.copy()
        
        # 3. 清洗 Modbus 缓冲区
        print("🧽 [阶段2] 正在清洗 PVT 数据缓冲区...")
        self.modbus.write_int32_array(address=0, values=current_positions)
        self.modbus.write_int32_array(address=50, values=[0, 0, 0, 0, 0])
        self.modbus.write_int32_array(address=200, values=[0])
        print(f"✅ 缓冲区已同步至安全位置: {current_positions}")
        
        # 4. 启动 PMAC 内的运动程序
        if not use_plc4_reset:
            self.hw_manager.start_prog()
    def move_joints(self, target_pulses: list, move_time: int = 500, accel: int = 100, scurve: int = 50):
        """核心底层：只下发原版的地址 0 和 地址 100，并新增动态时间参数"""
        self.modbus.write_int32_array(address=0, values=target_pulses)
        self.modbus.write_int32_array(address=20, values=[move_time, accel, scurve])
        self.modbus.write_int32_array(address=100, values=[1])

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
        current_pos = self.modbus.read_int32_array(address=10, count=5)
        self.config.zero_offsets = current_pos
        self.base_positions = current_pos
        print(f"✅ 已标定绝对零点偏置: {self.config.zero_offsets}")

    def read_positions(self) -> list[int]:
        return self.modbus.read_int32_array(address=10, count=5)

    def move_to_absolute_angle(self, joint_idx: int, absolute_angle: float, move_time: int = 500, accel: int = 100, scurve: int = 50):
        """绝对控制：增加【方向系数】"""
        current_pos = self.modbus.read_int32_array(address=10, count=5)
        targets = list(current_pos)
        
        # 引入方向系数
        direction = getattr(self.config, 'joint_directions', [1, 1, 1, 1, 1])[joint_idx]
        
        # 计算脉冲时乘以方向系数
        target_pulses = int(self.config.zero_offsets[joint_idx] + (absolute_angle * self.config.pulses_per_degree * direction))
        targets[joint_idx] = target_pulses
        
        print(f"🎯 绝对控制 -> 电机 {joint_idx+1} 目标角度 {absolute_angle}°, 对应脉冲 {target_pulses}")
        
        self.move_joints(targets, move_time=move_time, accel=accel, scurve=scurve)
        
    def move_pvt_stream(self, target_pulses: list, velocities: list, move_time: float):
        """
        专门适配 PVT 环形缓冲区的流式下发接口
        :param target_pulses: 5个轴的目标绝对脉冲列表
        :param velocities: 5个轴的目标瞬时速度 (脉冲/ms)
        :param move_time: 本段轨迹执行的时间 (ms)
        """
        pos_scale = 1.0 # 必须与 PMAC global definitions.pmh 一致[cite: 6]
        vel_time_scale = 10000.0
        # 1. 缩放并转换数据为 32位整数
        scaled_pos = [int(p * pos_scale) for p in target_pulses]
        scaled_vel = [int(v * vel_time_scale) for v in velocities]
        scaled_time = int(move_time * vel_time_scale)
        
        # 2. 写入位置 (地址 0, 4, 8, 12, 16)[cite: 4]
        self.modbus.write_int32_array(address=0, values=scaled_pos)
        
        # 3. 写入时间 (地址 40)[cite: 4]
        self.modbus.write_int32_array(address=40, values=[scaled_time])
        
        # 4. 写入速度 (地址 50, 54, 58, 62, 66)[cite: 4]
        self.modbus.write_int32_array(address=50, values=scaled_vel)
        
        # 5. 发送触发信号 (地址 200)[cite: 4]
        # 注意：PMAC PLC 2 处理完后会自动将其置零[cite: 4]
        self.modbus.write_int32_array(address=200, values=[1])
    
    def close(self):
        self.modbus.disconnect()
        
class VisualHomingManager:
    """视觉引导回零托管类"""
    def __init__(self, modbus_client):
        self.modbus = modbus_client
        self.CMD_ADDRESS = 220

    def start_homing(self):
        """下发指令 1：启动回零 (PLC将执行 #5j-)"""
        self.modbus.write_int32_array(address=self.CMD_ADDRESS, values=[1])

    def stop_movement(self):
        """下发指令 2：强制停止 (PLC将执行 #5k)"""
        self.modbus.write_int32_array(address=self.CMD_ADDRESS, values=[2])

    def confirm_and_set_zero(self):
        """下发指令 3：确认停稳并设零 (PLC将执行 #5hmz)"""
        self.modbus.write_int32_array(address=self.CMD_ADDRESS, values=[3])

    def read_status(self):
        """读取底层状态 (状态, 停止原因, 电流, 跟随误差)"""
        # PLC 中我们存放在 444, 446, 448, 450
        state_reason = self.modbus.read_int32_array(address=444, count=2)
        iq_fe = self.modbus.read_int32_array(address=448, count=2)
        return state_reason[0], state_reason[1], iq_fe[0], iq_fe[1]
