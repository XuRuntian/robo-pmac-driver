import time
import sys
import math
from pathlib import Path

# 保证能找到 src 目录
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))

from pmac_sdk.core.config_model import PMACConfig
from pmac_sdk.controller.robot_api import PMACRobotController

# ==========================================
# 第二层替换：正弦波生成器 (Generator Layer)
# ==========================================
class SineWaveGenerator:
    """
    代替键盘映射，生成平滑的正弦波轨迹
    """
    def __init__(self, amplitude_deg=10.0, frequency_hz=0.5):
        self.amplitude = amplitude_deg  # 振幅（度）
        self.freq = frequency_hz        # 频率（Hz，即一秒钟往返多少次）
        self.start_time = None

    def get_target(self, elapsed_time) -> tuple[float, float]:
        """
        根据当前时间计算目标角度和目标角速度
        返回: (target_angle_deg, target_velocity_deg_per_s)
        """
        # P(t) = A * sin(2 * pi * f * t)
        angle = self.amplitude * math.sin(2 * math.pi * self.freq * elapsed_time)
        
        # V(t) = A * 2 * pi * f * cos(2 * pi * f * t)
        # 注意：这是 度/秒
        velocity = self.amplitude * (2 * math.pi * self.freq) * math.cos(2 * math.pi * self.freq * elapsed_time)
        
        return angle, velocity

# ==========================================
# 第三层：测试调度 (Test Loop)
# ==========================================
def main():
    print("🧪 启动 PVT 正弦波金标准测试...")
    
    # 设置测试参数：幅值15度，频率0.2Hz（5秒一个来回，非常丝滑）
    gen = SineWaveGenerator(amplitude_deg=15.0, frequency_hz=0.2)
    
    pmac_config = PMACConfig(ip='192.168.0.200')
    robot = PMACRobotController(pmac_config)
    
    try:
        robot.hardware_boot()
        time.sleep(2)
        robot.connect_and_home()
        
        base_pulses = robot.base_positions.copy()
        
        print("\n✅ 系统就绪！正在进行正弦波平滑运动...")
        print("💡 提示：如果此运动抖动，请检查 PMAC 缓存大小或网络延迟。")

        update_interval = 0.02  # 50Hz
        move_time_ms = update_interval * 1000
        virtual_elapsed_time = 0.0
        start_real_time = time.perf_counter()
        next_call = start_real_time
        while True:
            loop_start = time.perf_counter()
            elapsed = loop_start - start_real_time
            
            # 1. 计算当前时间点理论上的 P 和 V (度/秒)
            target_angle, v_deg_s = gen.get_target(elapsed)
            
            # 2. 转换为 PMAC 脉冲单位
            # 我们只测试第 1 关节 (Index 0)，其他关节保持不动
            target_pulses = []
            velocities_p_ms = [] # 脉冲 / ms
            
            for i in range(5):
                if i == 0: # 只有一轴动，方便观察
                    p = int(base_pulses[i] + (target_angle * robot.config.pulses_per_degree))
                    # 速度换算：(度/s) * (脉冲/度) / 1000 = 脉冲/ms
                    v = (v_deg_s * robot.config.pulses_per_degree) / 1000.0
                else:
                    p = base_pulses[i]
                    v = 0.0
                
                target_pulses.append(p)
                velocities_p_ms.append(v)
            
            # 3. PVT 下发
            robot.move_pvt_stream(
                target_pulses=target_pulses,
                velocities=velocities_p_ms,
                move_time=move_time_ms
            )
            virtual_elapsed_time += update_interval
            next_call += update_interval
            # 维持频率控制
            sleep_time = next_call - time.perf_counter()
            
            if sleep_time > 0:
                time.sleep(sleep_time)
            
            # 打印调试信息（可选，不要在高频循环里打印太多）
            if int(elapsed * 50) % 50 == 0:
                print(f"Time: {elapsed:.2f}s, Pos: {target_angle:.2f}°, Vel: {v_deg_s:.2f}°/s")

    except KeyboardInterrupt:
        print("\n⏹️ 测试停止。")
    finally:
        robot.close()

if __name__ == "__main__":
    main()