import time
import sys
from pathlib import Path
from pynput import keyboard

# 保证能找到 src 目录
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))

from pmac_sdk.core.config_model import PMACConfig
from pmac_sdk.controller.robot_api import PMACRobotController

# ==========================================
# 第一层：输入捕获 (Input Layer)
# ==========================================
class KeyboardDevice:
    def __init__(self):
        self.pressed_keys = set()
        self.listener = keyboard.Listener(on_press=self._on_press, on_release=self._on_release)

    def _on_press(self, key):
        try:
            self.pressed_keys.add(key.char.lower())
        except AttributeError: pass

    def _on_release(self, key):
        try:
            self.pressed_keys.discard(key.char.lower())
        except AttributeError: pass

    def start(self): self.listener.start()
    def stop(self): self.listener.stop()
    def get_state(self) -> set: return self.pressed_keys

# ==========================================
# 第二层：轨迹规划 (PVT Planner)
# ==========================================
class KeyboardPVTPlanner:
    def __init__(self, update_hz=50, speed_deg_s=20.0):
        self.dt = 1.0 / update_hz
        self.speed_deg_s = speed_deg_s # 移动速度：度/秒
        self.target_angles = None
        
        # 定义按键映射：轴索引, 方向
        self.key_map = {
            'w': (0, 1), 's': (0, -1),   # 1轴
            'a': (1, 1), 'd': (1, -1),   # 2轴
            'q': (2, 1), 'e': (2, -1),   # 3轴
            'r': (3, 1), 'f': (3, -1),   # 4轴
            't': (4, 1), 'g': (4, -1),   # 5轴
        }

    def compute_next_step(self, active_keys, current_angles):
        if self.target_angles is None:
            self.target_angles = list(current_angles)

        next_velocities_deg_s = [0.0] * 5
        
        # 计算每一轴的期望移动
        for key, (axis_idx, direction) in self.key_map.items():
            if key in active_keys:
                # 瞬时速度 = 设定速度 * 方向
                next_velocities_deg_s[axis_idx] = direction * self.speed_deg_s
                # 位置增量 = 速度 * 时间
                self.target_angles[axis_idx] += next_velocities_deg_s[axis_idx] * self.dt

        return self.target_angles, next_velocities_deg_s

# ==========================================
# 第三层：主控制循环
# ==========================================
def main():
    print("🎮 启动 PVT 键盘遥操作 (高平滑模式)...")
    
    update_hz = 50
    update_interval = 1.0 / update_hz
    move_time_ms = update_interval * 1000  # 严格 20ms
    
    kbd = KeyboardDevice()
    planner = KeyboardPVTPlanner(update_hz=update_hz, speed_deg_s=25.0) # 速度设为 25度/秒
    
    pmac_config = PMACConfig(ip='192.168.0.200')
    robot = PMACRobotController(pmac_config)
    
    try:
        robot.hardware_boot()
        time.sleep(2)
        robot.connect_and_home()
        base_pulses = robot.base_positions.copy()
        
        # 初始化当前角度状态
        current_pulses = robot.modbus.read_int32_array(address=10, count=5)
        curr_angles = [(p - base_pulses[i]) / robot.config.pulses_per_degree for i, p in enumerate(current_pulses)]
        
        kbd.start()
        print("\n✅ 控制就绪！WASD/QE/RF/TG 控制轴。按 Ctrl+C 退出。")

        next_call = time.perf_counter()

        while True:
            loop_start = time.perf_counter()
            
            # 1. 获取输入并计算下一步 PVT
            active_keys = kbd.get_state()
            target_angles, v_degs = planner.compute_next_step(active_keys, curr_angles)
            
            # 2. 转换为脉冲和速度单位
            target_pulses = []
            velocities_p_ms = []
            for i in range(5):
                p = int(base_pulses[i] + (target_angles[i] * robot.config.pulses_per_degree))
                v = (v_degs[i] * robot.config.pulses_per_degree) / 1000.0
                target_pulses.append(p)
                velocities_p_ms.append(v)
            
            # 3. PVT 下发
            robot.move_pvt_stream(
                target_pulses=target_pulses,
                velocities=velocities_p_ms,
                move_time=move_time_ms
            )
            
            # 4. 严格频率对齐
            next_call += update_interval
            sleep_time = next_call - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                # 容错：如果 Python 调度慢了，追齐时钟防止误差累积
                next_call = time.perf_counter()

    except KeyboardInterrupt:
        print("\n⏹️ 停止操作。")
    finally:
        kbd.stop()
        robot.close()

if __name__ == "__main__":
    main()