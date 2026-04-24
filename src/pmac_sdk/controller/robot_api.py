from ..core.config_model import PMACConfig
from ..comms.modbus_client import ModbusClient32Bit
from ..hardware.ssh_manager import PMACHardwareManager

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
        """连接 Modbus 并获取当前真实位置作为基准"""
        if not self.modbus.connect():
            raise ConnectionError("❌ 无法连接到 PMAC，请检查网络设置。")
        
        # 按照原代码逻辑，直接去读地址 10 的 10个寄存器
        res = self.modbus.client.read_holding_registers(address=10, count=10, unit=self.config.slave_id)
        if not res.isError():
            regs = res.registers
            for i in range(5):
                self.base_positions[i] = self.modbus._registers_to_int32(regs[i*2], regs[i*2+1])
        print(f"✅ 系统就绪，基准位置: {self.base_positions}")

    def move_joints(self, target_pulses: list, move_time: int = 500, accel: int = 100, scurve: int = 50):
        """核心底层：只下发原版的地址 0 和 地址 100，并新增动态时间参数"""
        # 1. 写入 5 个电机的目标坐标 (地址 0)
        self.modbus.write_int32_array(address=0, values=target_pulses)
        
        # 2. 写入动态时间参数 (地址 20，对应 PMAC 字节 40, 44, 48)
        self.modbus.write_int32_array(address=20, values=[move_time, accel, scurve])
        
        # 3. 扣动扳机 (地址 100)
        self.modbus.write_int32_array(address=100, values=[1])

    def move_single_joint_angle(self, joint_idx: int, angle: float, move_time: int = 500, accel: int = 100, scurve: int = 50):
        """按照你的原版逻辑换算角度，增加速度控制"""
        targets = list(self.base_positions)
        target_pulses = int(self.base_positions[joint_idx] + (angle * self.config.pulses_per_degree))
        targets[joint_idx] = target_pulses
        
        print(f"🎯 正在向电机 {joint_idx+1} 发送指令: 目标角度 {angle}°, 对应绝对脉冲 {target_pulses}")
        print(f"⏱️  期望耗时: {move_time}ms, 加减速: {accel}ms，s型时间{scurve}")
        
        # 透传参数到底层
        self.move_joints(targets, move_time=move_time, accel=accel, scurve=scurve)
        
    def set_current_as_absolute_zero(self):
        """
        【标定模式】：将机器人手动摆到标准的“零点姿态”后调用此方法。
        把当前的物理脉冲记录下来，作为绝对零点偏置。
        (实际工程中，这个偏置应该被保存到 default_pmac.yaml 里持久化)
        """
        current_pos = self.modbus.read_int32_array(address=10, count=5)
        self.config.zero_offsets = current_pos
        # 更新当前的相对基准，防止乱跳
        self.base_positions = current_pos
        print(f"✅ 已标定绝对零点偏置: {self.config.zero_offsets}")

    def move_to_absolute_angle(self, joint_idx: int, absolute_angle: float, move_time: int = 500, accel: int = 100, scurve: int = 50):
        """
        【绝对控制】：基于标定好的物理零点进行精确到角度的控制。
        无论在哪上电，指令发 0°，机械臂就一定会回到物理固定的那个 0° 姿态。
        """
        # 1. 获取当前所有轴的最新绝对脉冲，防止其他轴被误归零
        current_pos = self.modbus.read_int32_array(address=10, count=5)
        targets = list(current_pos)
        
        # 2. 计算目标脉冲：零点偏置 + 目标角度对应的脉冲
        target_pulses = int(self.config.zero_offsets[joint_idx] + (absolute_angle * self.config.pulses_per_degree))
        targets[joint_idx] = target_pulses
        
        print(f"🎯 绝对控制 -> 电机 {joint_idx+1} 目标角度 {absolute_angle}°, 对应脉冲 {target_pulses}")
        
        # 3. 发送运动指令
        self.move_joints(targets, move_time=move_time, accel=accel, scurve=scurve)
        
    def close(self):
        self.modbus.disconnect()