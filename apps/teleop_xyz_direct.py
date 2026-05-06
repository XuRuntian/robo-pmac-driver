# apps/teleop_xyz_direct.py
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
    """
    纯逻辑类：负责将主手的笛卡尔位移映射为指定的关节角度。
    它不依赖任何具体的硬件连接，便于后续单独写单元测试。
    """
    def __init__(self, scale_x: float = 200.0, scale_y: float = 200.0, scale_z: float = 200.0):
        # 放大系数：主手的移动范围通常在 ±0.1米 左右
        # 如果 scale 是 200，则 0.1m 的移动会转化为 20° 的关节旋转
        self.scale = [scale_x, scale_y, scale_z]
        
        # 定义要控制的机械臂轴索引 (比如控制 0, 1, 2 也就是前三个轴)
        self.target_joints = [0, 1, 2]

    def map_to_angles(self, haptic_state: HapticState) -> List[float]:
        """
        输入：主手的纯数据状态
        输出：对应 self.target_joints 的目标角度列表
        """
        # 提取主手的 x, y, z
        x, y, z = haptic_state.pos[0], haptic_state.pos[1], haptic_state.pos[2]
        
        # 线性映射为角度
        angle_1 = x * self.scale[0]
        angle_2 = y * self.scale[1]
        angle_3 = z * self.scale[2]
        
        return [angle_1, angle_2, angle_3]

# ==========================================
# [应用层] 主控循环
# ==========================================
def main():
    # 1. 实例化各个解耦的模块
    omega = OmegaDevice()
    # 调整 scale 可以控制主手的“灵敏度”
    mapper = SimpleXYZMapper(scale_x=50.0, scale_y=50.0, scale_z=50.0) 
    
    pmac_config = PMACConfig(ip='192.168.0.200')
    robot = PMACRobotController(pmac_config)
    
    try:
        # 2. 硬件连接与初始化
        if not omega.connect():
            print("❌ 主手连接失败，退出。")
            return
            
        robot.hardware_boot()
        time.sleep(2)
        robot.connect_and_home()
        print("\n✅ 系统就绪！当前进入 XYZ -> Joint1,2,3 直接映射测试模式。")
        
        # 记录机械臂当前的基准绝对脉冲，避免突变
        base_pulses = list(robot.base_positions)
        
        # 3. 遥操作主循环
        update_interval = 0.05  # 20Hz 控制频率
        
        print("\n🚀 开始遥操作 (按 Ctrl+C 退出)...")
        print("请缓慢移动主手 XYZ 观察前三轴的转动。")
        
        while True:
            start_time = time.time()
            
            # [步骤 A] 读取输入源
            haptic_data = omega.get_state()
            
            # [步骤 B] 算法层计算 (纯数据流转)
            # 获取算出的 3 个目标角度
            target_angles = mapper.map_to_angles(haptic_data) 
            
            # [步骤 C] 转换为底层执行器所需的协议并下发
            # 初始化一个包含 5 个轴当前基准位置的列表
            current_targets = list(base_pulses) 
            
            # 仅修改我们需要控制的前三个轴的脉冲
            for i, joint_idx in enumerate(mapper.target_joints):
                # 目标脉冲 = 基准脉冲 + (目标角度 * 转换率)
                pulse_offset = target_angles[i] * robot.config.pulses_per_degree
                current_targets[joint_idx] = int(base_pulses[joint_idx] + pulse_offset)
            
            # 批量同步下发给 PMAC，时间与控制周期对齐，保证平滑度
            robot.move_joints(
                target_pulses=current_targets,
                move_time=int(update_interval * 1000), 
                accel=10,   # 小加速度拟合连续轨迹
                scurve=0
            )
            
            # [步骤 D] 维持严谨的控制频率
            elapsed = time.time() - start_time
            if elapsed < update_interval:
                time.sleep(update_interval - elapsed)
                
    except KeyboardInterrupt:
        print("\n⏹️ 接收到中断信号，正在退出...")
    except Exception as e:
        print(f"\n❌ 运行时异常: {e}")
    finally:
        # 4. 安全退出清理
        omega.close()
        robot.close()

if __name__ == "__main__":
    main()