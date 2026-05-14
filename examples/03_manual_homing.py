import time
import sys
from pathlib import Path
from pynput import keyboard

# 保证能找到 src 目录
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))

from pmac_sdk.core.config_model import PMACConfig
from pmac_sdk.controller.robot_api import PMACRobotController, VisualHomingManager

# 全局标志位，用于捕捉按键动作
user_triggered_stop = False

def on_press(key):
    global user_triggered_stop
    if key == keyboard.Key.space or key == keyboard.Key.enter:
        user_triggered_stop = True

def main():
    print("========================================")
    print("👁️  增量轴 (电机 5) 人工视觉引导回零程序")
    print("========================================")

    config = PMACConfig(ip='192.168.0.200')
    robot = PMACRobotController(config)
    homer = VisualHomingManager(robot.modbus)

    try:
        # 1. 初始化硬件 (此时会同时使能 PLC 2 和 PLC 3)
        robot.hardware_boot()
        time.sleep(2)
        robot.connect_and_home()

        print("\n⚠️ 准备就绪！")
        print("操作说明：")
        print("1. 按下回车键，电机将开始向负方向移动 (#5j-)。")
        print("2. 观察电机位置。")
        print("3. 当到达你期望的零点时，按下【空格键】或【回车键】，电机将立即停止并设为绝对零点。")
        input("\n👉 确认安全后，按回车键开始移动...")

        # 2. 下发启动指令给 PLC 3
        homer.start_homing()
        
        # 启动键盘监听器，非阻塞捕捉刹车信号
        listener = keyboard.Listener(on_press=on_press)
        listener.start()

        print("\n🚀 电机运行中... (按【空格键】刹车)")

        # 3. 监控循环
        global user_triggered_stop
        while True:
            # 读取 PMAC 电机 5 的当前位置 (Modbus 地址 18)
            # 注意：在你的 robot_api.py 中，read_int32_array(10, 5) 对应 5 个电机
            # 电机 5 的地址是 10 + 4*2 = 18
            try:
                curr_pos = robot.modbus.read_int32_array(address=18, count=1)[0]
                state, reason, iq, fe = homer.read_status()
                
                # 动态刷新终端显示
                sys.stdout.write(f"\r📊 当前位置: {curr_pos} | 电流: {iq} | PLC状态: {state}      ")
                sys.stdout.flush()

            except Exception as e:
                pass # 忽略单次通讯抖动

            # 4. 判断用户是否按下了停止键
            if user_triggered_stop:
                print("\n\n🛑 接收到手动刹车信号！正在急停...")
                homer.stop_movement()
                time.sleep(0.5) # 给电机一点时间彻底停稳
                
                print("🏠 正在将当前位置设为绝对零点...")
                homer.confirm_and_set_zero()
                time.sleep(0.5)
                
                # 重新读一次位置确认
                final_pos = robot.modbus.read_int32_array(address=18, count=1)[0]
                print(f"✅ 设零完成！当前电机 5 读数: {final_pos}")
                break

            # 5. 如果底层 PLC 意外触发了硬件保护（例如真撞墙了或者电流过大）
            if state == 3:
                print(f"\n\n⚠️ 底层 PLC 触发物理保护停车！(原因码: {reason})")
                print("可能是提前撞到了硬挡块。")
                homer.confirm_and_set_zero()
                print("✅ 已就地设零。")
                break

            time.sleep(0.05) # 20Hz 刷新率

    except KeyboardInterrupt:
        print("\n⏹️ 程序被强行中断，发送紧急停止指令...")
        homer.stop_movement()
    finally:
        if 'listener' in locals():
            listener.stop()
        robot.close()
        print("🔌 连接已安全关闭。")

if __name__ == "__main__":
    main()