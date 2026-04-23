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
        # 2. 硬件初始化与回零
        robot.hardware_boot()
        print("等待 2 秒让 PLC 完全启动...")
        time.sleep(2)
        
        robot.connect_and_home()
        
        # ==========================================
        # 往复运动参数设置
        # ==========================================
        joint_to_test = 3          # 控制的轴索引 (0 代表 1 轴)
        target_angle_a = 20.0      # 目标位置 A (度)
        target_angle_b = -20.0     # 目标位置 B (度)
        
        move_time_ms = 1500        # 运动耗时 (毫秒)：值越小速度越快
        dwell_time = 0.5           # 到达位置后的停顿时间 (秒)
        test_duration = 60.0       # 总测试时长 (秒)
        
        print(f"\n-> 开始往复运动测试: 轴 {joint_to_test}")
        print(f"-> 运动范围: {target_angle_a}° <-> {target_angle_b}°")
        print(f"-> 运行速度: 每段耗时 {move_time_ms}ms")
        
        input("\n[交互] 按下回车键开始运动 (按 Ctrl+C 随时停止)...")
        
        start_time = time.time()
        current_target = target_angle_a # 初始目标设为 A
        
        # 3. 往复运动循环
        while True:
            elapsed_time = time.time() - start_time
            if elapsed_time > test_duration:
                break
            
            print(f"正在移动至: {current_target}°")
            
            # 下发单轴运动指令
            robot.move_single_joint_angle(
                joint_idx=joint_to_test, 
                angle=current_target,
                move_time=move_time_ms,   # 指定运动完成的时间
                accel=200,                # 往复运动可以设置较大的加速度
                scurve=10                 # 开启平滑 S 曲线，防止机械冲击
            )
            
            # 等待运动完成 + 停顿时间
            # 总等待时间 = (运动时间 / 1000) + 停顿时间
            time.sleep((move_time_ms / 1000.0) + dwell_time)
            
            # 切换目标位置 (A -> B, B -> A)
            if current_target == target_angle_a:
                current_target = target_angle_b
            else:
                current_target = target_angle_a
            
        print("\n🏁 设定的测试时间结束，运动停止。")
        
    except KeyboardInterrupt:
        print("\n⏹️ 检测到 Ctrl+C，已手动停止往复运动测试。")
        
    except Exception as e:
        print(f"❌ 运行报错: {e}")
        
    finally:
        # 停止并清理连接
        try:
            current_pos = robot.modbus.read_int32_array(address=10, count=5)
            print(f"📊 最终停止时五轴位置: {current_pos}")
        except:
            pass
            
        robot.close()
        print("🔌 连接已关闭。")

if __name__ == "__main__":
    main()