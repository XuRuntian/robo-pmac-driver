import time
import math
import sys
from pathlib import Path

# 保证能找到 src 目录
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))

from pmac_sdk.core.config_model import PMACConfig
from pmac_sdk.controller.robot_api import PMACRobotController

def main():
    print("🧬 启动 1-4 轴 PVT 同步性协调测试...")
    
    pmac_config = PMACConfig(ip='192.168.0.200')
    robot = PMACRobotController(pmac_config)
    
    # 测试参数
    # 1-4 轴分别运动不同的幅度，但周期完全一致
    amplitudes = [10.0, 20.0, -15.0, 30.0, 10.0]  # 5轴不动
    freq = 0.2  # 0.2Hz (5秒一个来回)
    
    update_hz = 50
    update_interval = 1.0 / update_hz
    move_time_ms = update_interval * 1000 # 严格 20ms
    
    try:
        # robot.hardware_boot()
        # time.sleep(1)
        robot.axi_syn_boot()
        time.sleep(2)
        robot.connect_and_home()    
        base_pulses = robot.base_positions.copy()
        
        print("\n🚀 开始同步运动！")
        print("💡 观察重点：虽然各轴摆动幅度不同，但它们应该同时到达最高点和最低点。")

        # 使用虚拟时钟保证轨迹点的数学连续性
        virtual_time = 0.0
        next_call = time.perf_counter()

        while True:
            # 1. 计算 5 个轴的 P 和 V
            target_pulses = []
            velocities_p_ms = []
            
            for i in range(5):
                amp = amplitudes[i]
                # P(t) = A * sin(2*pi*f*t)
                angle = amp * math.sin(2 * math.pi * freq * virtual_time)
                # V(t) = A * 2*pi*f * cos(2*pi*f*t)
                v_deg_s = amp * (2 * math.pi * freq) * math.cos(2 * math.pi * freq * virtual_time)
                
                # 转换脉冲
                p = int(base_pulses[i] + (angle * robot.config.pulses_per_degree))
                v = (v_deg_s * robot.config.pulses_per_degree) / 1000.0
                
                target_pulses.append(p)
                velocities_p_ms.append(v)

            # 2. 一次性下发 5 轴数据 (同步触发)
            # 在 robot_api 内部，地址 0, 4, 8, 12, 16 会被依次写入
            # 然后写入时间地址 40，最后写入触发地址 200
            robot.move_pvt_stream(
                target_pulses=target_pulses,
                velocities=velocities_p_ms,
                move_time=move_time_ms
            )

            # 3. 更新虚拟时间
            virtual_time += update_interval
            
            # 4. 频率对齐
            next_call += update_interval
            sleep_time = next_call - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_call = time.perf_counter()

    except KeyboardInterrupt:
        print("\n⏹️ 测试结束。")
    finally:
        robot.close()

if __name__ == "__main__":
    main()