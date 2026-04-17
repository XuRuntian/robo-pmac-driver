import time
import sys
# 确保能导入当前目录的包
sys.path.append('.') 

from pmac_sdk.core.config_model import PMACConfig
from pmac_sdk.controller.robot_api import PMACRobotController

def main():
    # 1. 实例化配置与控制器
    config = PMACConfig(ip='192.168.0.200')
    robot = PMACRobotController(config)
    
    try:
        robot.hardware_boot()
        print("等待 2 秒让 PLC 完全启动...")
        time.sleep(2)
        
        robot.connect_and_home()
        
        input("\n[交互] 按下回车键开始测试：电机 1 将旋转输出轴 180 度...")
        
        # ==========================================
        # 4. 执行运动 (加入速度控制)
        # 设定 move_time=1000 毫秒，加减速时间为 200 毫秒
        # ==========================================
        robot.move_single_joint_angle(
            joint_idx=0, 
            angle=180.0,
            move_time=100, 
            accel=200,
            scurve=50
        )
        
        # 5. 等待运动完成并观察 (等待时间 = move_time/1000 + 0.5秒缓冲)
        time.sleep(0.5) 
        
        # 获取最新位置对比
        current_pos = robot.modbus.read_int32_array(address=10, count=5)
        print(f"🏁 移动结束。最新五轴位置: {current_pos}")
        
        # 打印真实的物理稳态误差
        target_pulses = robot.base_positions[0] + int(180.0 * robot.config.pulses_per_degree)
        actual_pulses = current_pos[0]
        print(f"📉 稳态误差 (Target - Actual): {target_pulses - actual_pulses}")
        
    except Exception as e:
        print(f"❌ 运行报错: {e}")
    finally:
        robot.close()
        print("🔌 连接已关闭。")

if __name__ == "__main__":
    main()