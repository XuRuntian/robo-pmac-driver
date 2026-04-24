import time
import sys
import math # 引入数学库计算正弦曲线
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))

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
        
        input("\n[交互] 按下回车键开始测试：3 轴和 4 轴将按照正弦曲线连续运动 (按 Ctrl+C 随时停止)...")
        
        # ==========================================
        # 正弦曲线运动参数设置 (可随时调节)
        # ==========================================
        amplitude = 10.0          # 幅值 (度)：轴将在 -15° 到 +15° 之间来回摆动
        frequency = 1           # 频率 (Hz)：0.5Hz 代表 2 秒完成一次完整的往复运动
        update_interval = 0.05    # 控制周期/下发频率：0.05秒 (即 20Hz 更新率)
        test_duration = 60.0      # 总测试时长 (秒)，设为 float('inf') 可无限运行
        
        # 确认你代码里对应的轴索引
        joint_idx_3 = 2
        joint_idx_4 = 3
        
        print(f"\n-> 开始双轴正弦运动: 幅值 ±{amplitude}°, 频率 {frequency}Hz")
        start_time = time.time()
        
        # 4. 执行连续的循环运动
        while True:
            elapsed_time = time.time() - start_time
            
            # 如果到达测试总时长，退出循环
            if elapsed_time > test_duration:
                break
                
            # 计算当前时刻的目标角度: A * sin(2 * π * f * t)
            target_angle = amplitude * math.sin(2 * math.pi * frequency * elapsed_time)
            
            # 依次下发指令 (将运动时间与下发周期匹配，加减速设小以拟合连续轨迹)
            # --- 下发 3 轴 ---
            robot.move_single_joint_angle(
                joint_idx=joint_idx_3, 
                angle=target_angle,
                move_time=int(update_interval * 1000), 
                accel=10, 
                scurve=0
            )
            
            # --- 下发 4 轴 ---
            robot.move_single_joint_angle(
                joint_idx=joint_idx_4, 
                angle=target_angle,
                move_time=int(update_interval * 1000), 
                accel=10, 
                scurve=0
            )
            
            # 延时以维持控制周期
            time.sleep(update_interval)
            
        print("\n🏁 设定的测试时间结束，运动停止。")
        
    except KeyboardInterrupt:
        # 捕获 Ctrl+C，安全退出
        print("\n⏹️ 检测到 Ctrl+C，已手动停止正弦运动测试。")
        
    except Exception as e:
        print(f"❌ 运行报错: {e}")
        
    finally:
        # 获取最终位置对比
        try:
            # 稍微等电机停稳
            time.sleep(0.5)
            current_pos = robot.modbus.read_int32_array(address=10, count=5)
            print(f"📊 最终停止时五轴位置: {current_pos}")
        except Exception as e:
            print(f"⚠️ 读取最终位置失败: {e}")
            
        robot.close()
        print("🔌 连接已关闭。")

if __name__ == "__main__":
    main()