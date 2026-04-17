import time
import sys
# 确保能导入当前目录的包 (如果你已经 pip install -e . 了，这句可以省掉)
sys.path.append('.') 

from pmac_sdk.core.config_model import PMACConfig
from pmac_sdk.controller.robot_api import PMACRobotController

def main():
    # 1. 实例化配置与控制器
    config = PMACConfig(ip='192.168.0.200')
    robot = PMACRobotController(config)
    
    try:
        # 2. 硬件上电 (按需执行，如果已经上电可以注释掉)
        robot.hardware_boot()
        print("等待 2 秒让 PLC 完全启动...")
        time.sleep(2)
        
        # 3. 连接并获取基准
        robot.connect_and_home()
        
        input("\n[交互] 按下回车键开始测试：电机 1 将旋转输出轴 90 度...")
        
        # 4. 执行运动
        # 让 0号电机（即电机1）移动 90 度，耗时 2000ms
        robot.move_single_joint_angle(
            joint_idx=0, 
            angle=180.0, 
        )
        
        # 5. 等待运动完成并观察
        time.sleep(0.1) 
        
        # 获取最新位置对比
        current_pos = robot.modbus.read_int32_array(address=10, count=5)
        print(f"🏁 移动结束。最新五轴位置: {current_pos}")
        print(f"📈 脉冲差值: {current_pos[0] - robot.base_positions[0]}")
        
    except Exception as e:
        print(f"❌ 运行报错: {e}")
    finally:
        robot.close()
        print("🔌 连接已关闭。")

if __name__ == "__main__":
    main()