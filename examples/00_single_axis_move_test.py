import time
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))

from pmac_sdk.core.config_model import PMACConfig
from pmac_sdk.controller.robot_api import PMACRobotController

def main():
    # 1. 实例化配置与控制器 (请根据实际情况修改 IP)
    config = PMACConfig(ip='192.168.0.200')
    robot = PMACRobotController(config)
    
    try:
        # 2. 硬件启动与连接
        robot.hardware_boot()
        print("正在等待系统启动...")
        time.sleep(2)
        
        robot.connect_and_home()
        print("✅ 系统已就绪。")

        # ==========================================
        # 运动参数配置
        # ==========================================
        joint_idx = 1         # 要操作的轴 (0-4)
        target_angle = 5.0   # 目标角度 (度)
        move_time = 200      # 运动耗时 (毫秒)，2000ms = 2秒
        
        print(f"\n🚀 准备移动轴 {joint_idx} 到 {target_angle}°")
        confirm = input("确认执行运动？(按回车键开始 / 输入 n 退出): ")
        
        if confirm.lower() == 'n':
            return

        # 3. 执行单轴运动
        # move_single_joint_angle 是绝对位置运动
        robot.move_single_joint_angle(
            joint_idx=joint_idx, 
            angle=target_angle,
            move_time=move_time,
            accel=150,        # 加速度
            scurve=20         # S曲线平滑度 (0-100)，越大越丝滑
        )
        
        # 等待运动完成
        print(f"正在执行运动，预计耗时 {move_time/1000} 秒...")
        time.sleep(move_time / 1000.0 + 0.5)
        
        # 4. 读取当前位置确认
        # 假设 Modbus 地址 10 开始存放 5 个轴的当前位置
        pos_array = robot.modbus.read_int32_array(address=10, count=5)
        print(f"\n📊 运动完成！当前五轴位置: {pos_array}")

    except Exception as e:
        print(f"❌ 运行中出现错误: {e}")
        
    finally:
        robot.close()
        print("🔌 已关闭连接。")

if __name__ == "__main__":
    main()