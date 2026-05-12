import time
import sys
from pathlib import Path
from typing import List

# 确保能导入 src 下的模块
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))

from omega_sdk.haptic_device import OmegaDevice, HapticState
from pmac_sdk.core.config_model import PMACConfig
from pmac_sdk.controller.robot_api import PMACRobotController

# ==========================================
# [解耦的算法层] XYZ 直映关节映射器
# ==========================================
class SimpleXYZMapper:
    def __init__(self, scale_x: float = 150.0, scale_y: float = 150.0, scale_z: float = 150.0):
        # 放大系数：0.1米的主手位移 -> 对应多少度关节角
        self.scale = [scale_x, scale_y, scale_z]
        # 弹性牵引绳的长度（度）。主手可以走很远，但目标点最多只能领先电机真实位置 15 度
        self.max_lead_deg = 30.0 

    def solve(self, haptic_state: HapticState, current_angles: List[float]) -> List[float]:
        x, y, z = haptic_state.pos[0], haptic_state.pos[1], haptic_state.pos[2]
        
        # 1. 计算主手的“理想目标角度” (绝对角度偏移量)
        ideal_target_0 = x * self.scale[0]
        ideal_target_1 = y * self.scale[1]
        ideal_target_2 = z * self.scale[2]
        
        # 2. 复制当前真实角度，准备进行限位加工
        targets = current_angles.copy()
        
        # 3. 施加弹性牵引绳逻辑 (防猛拉、防过冲)
        def apply_leash(ideal_angle, current_angle):
            max_allowed = current_angle + self.max_lead_deg
            min_allowed = current_angle - self.max_lead_deg
            # 将理想角度强制夹在这个安全范围内
            return max(min_allowed, min(ideal_angle, max_allowed))
            
        targets[0] = apply_leash(ideal_target_0, current_angles[0])
        targets[1] = apply_leash(ideal_target_1, current_angles[1])
        targets[2] = apply_leash(ideal_target_2, current_angles[2])
        
        return targets

# ==========================================
# [应用层] 主控循环
# ==========================================
def main():
    print("初始化系统中...")
    omega = OmegaDevice()
    # scale 可根据实际手感调大或调小
    mapper = SimpleXYZMapper(scale_x=1000.0, scale_y=1000.0, scale_z=1000.0) 
    
    pmac_config = PMACConfig(ip='192.168.0.200')
    robot = PMACRobotController(pmac_config)
    
    try:
        # 1. 硬件连接与初始化
        if not omega.connect():
            print("❌ 主手连接失败，退出。")
            return
            
        robot.hardware_boot()
        time.sleep(2)
        robot.connect_and_home()
        print("\n✅ 系统就绪！当前进入 XYZ -> Joint1,2,3 直接映射测试模式。")
        
        # 获取系统启动时的物理脉冲基准，作为 0 度参考点
        base_pulses = list(robot.base_positions)
        
        # 2. 遥操作主循环
        update_interval = 0.03  # 约 33Hz 控制频率
        loop_count = 0
        total_modbus_time = 0
        
        print("\n🚀 开始 Omega 主手遥操作 (按 Ctrl+C 退出)...")
        print("⚠️ 警告：请先稳住主手柄再开始移动！直接松开手柄会导致机械臂快速回零点！")
        
        while True:
            loop_start = time.perf_counter()
            
            # [步骤 A] 读取机械臂真实位置
            current_pulses = robot.modbus.read_int32_array(address=10, count=5)
            current_angles_deg = [(p - base_pulses[i]) / robot.config.pulses_per_degree for i, p in enumerate(current_pulses)]
            
            # [步骤 B] 读取主手状态并映射
            haptic_data = omega.get_state()
            target_angles_deg = mapper.solve(haptic_data, current_angles_deg)
            
            # [步骤 C] 转换为底层脉冲并下发
            targets_pulses = []
            for idx, angle_deg in enumerate(target_angles_deg):
                pulse = int(base_pulses[idx] + (angle_deg * robot.config.pulses_per_degree))
                targets_pulses.append(pulse)
                
            modbus_start = time.perf_counter()
            robot.move_joints(
                target_pulses=targets_pulses,
                move_time=int(update_interval * 1000), 
                accel=10, 
                scurve=0
            )
            modbus_cost = time.perf_counter() - modbus_start
            
            # [步骤 D] 维持控制周期并打印 Debug
            loop_cost = time.perf_counter() - loop_start
            sleep_time = update_interval - loop_cost
            if sleep_time > 0:
                time.sleep(sleep_time)
                
            loop_count += 1
            total_modbus_time += modbus_cost
            if loop_count % 15 == 0:
                avg_modbus = (total_modbus_time / 15) * 1000 
                real_hz = 1.0 / (time.perf_counter() - loop_start) 
                
                # 打印主手 X 坐标 和 机械臂 1 轴 目标角度的对比
                print(f"📊 Debug | 主手 X: {haptic_data.pos[0]:.3f}m | "
                      f"目标角度[0]: {target_angles_deg[0]:.2f}° | "
                      f"真实角度[0]: {current_angles_deg[0]:.2f}° | "
                      f"Modbus: {avg_modbus:.1f}ms | 频率: {real_hz:.1f}Hz")
                total_modbus_time = 0
                
    except KeyboardInterrupt:
        print("\n⏹️ 接收到中断信号，正在退出...")
    except Exception as e:
        print(f"\n❌ 运行时异常: {e}")
    finally:
        omega.close()
        robot.close()
        print("🔌 系统已安全关闭。")

if __name__ == "__main__":
    main()