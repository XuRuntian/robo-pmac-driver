# apps/teleop_cartesian.py
import time
import sys
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))

from omega_sdk.haptic_device import OmegaDevice, HapticState
from pmac_sdk.core.config_model import PMACConfig
from pmac_sdk.controller.robot_api import PMACRobotController

class CartesianToJointAlgorithm:
    """
    逆运动学 (IK) 算法层的占位类。
    职责：将 Omega 返回的 HapticState (笛卡尔空间) 映射为 PMAC 的目标关节角度。
    """
    def __init__(self):
        # 这里可以加载 URDF、设定基座标系偏置等
        pass

    def solve(self, haptic_state: HapticState) -> list[float]:
        """
        输入: 主手的笛卡尔状态
        输出: 包含 5 个关节目标角度的列表
        """
        # TODO: 在这里实现你真正的逆解逻辑
        # 下面是一个伪造的映射逻辑，仅作演示：
        # 比如将主手的 Z 轴平移映射到机械臂的 1 轴角度，夹爪映射到 5 轴
        j1 = haptic_state.pos[2] * 100.0  
        j2 = 0.0
        j3 = 0.0
        j4 = haptic_state.euler[0] * 0.5  
        j5 = haptic_state.gripper_deg
        
        return [j1, j2, j3, j4, j5]

def main():
    # 1. 实例化各个解耦的模块
    omega = OmegaDevice()
    ik_solver = CartesianToJointAlgorithm()
    
    pmac_config = PMACConfig(ip='192.168.0.200')
    robot = PMACRobotController(pmac_config)
    
    try:
        # 2. 硬件连接
        if not omega.connect():
            return
            
        robot.hardware_boot()
        time.sleep(2)
        robot.connect_and_home()
        
        print("\n🚀 开始笛卡尔遥操作 (按 Ctrl+C 退出)...")
        
        # 3. 遥操作主循环
        update_interval = 0.05  # 20Hz 控制频率
        
        while True:
            start_time = time.time()
            
            # [Input] 获取主手当前状态
            haptic_data = omega.get_state()
            
            # [Algorithm] 通过预留算法层进行逆解计算
            target_joint_angles = ik_solver.solve(haptic_data)
            
            # [Output] 转换为 PMAC 脉冲并下发
            # 获取基于零点偏置的绝对脉冲
            targets_pulses = []
            for idx, angle in enumerate(target_joint_angles):
                pulse = int(robot.config.zero_offsets[idx] + (angle * robot.config.pulses_per_degree))
                targets_pulses.append(pulse)
            
            # 批量下发给底层 (动态匹配 move_time 以保证轨迹平滑)
            robot.move_joints(
                target_pulses=targets_pulses,
                move_time=int(update_interval * 1000), 
                accel=10,   # 小加速度拟合连续轨迹
                scurve=0
            )
            
            # 维持稳定的控制周期
            elapsed = time.time() - start_time
            if elapsed < update_interval:
                time.sleep(update_interval - elapsed)
                
    except KeyboardInterrupt:
        print("\n⏹️ 接收到中断信号，正在退出遥操作...")
    except Exception as e:
        print(f"\n❌ 运行时异常: {e}")
    finally:
        omega.close()
        robot.close()

if __name__ == "__main__":
    main()