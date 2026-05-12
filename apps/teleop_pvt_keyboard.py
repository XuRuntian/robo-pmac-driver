import time
import sys
import math
from pathlib import Path
from pynput import keyboard

# 保证能找到 src 目录
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))

from pmac_sdk.core.config_model import PMACConfig
from pmac_sdk.controller.robot_api import PMACRobotController

# ==========================================
# 第一层：输入捕获 (支持特殊功能键)
# ==========================================
class KeyboardDevice:
    def __init__(self):
        self.pressed_keys = set()
        self.listener = keyboard.Listener(on_press=self._on_press, on_release=self._on_release)
        self.on_speed_change = None # 回调函数

    def _on_press(self, key):
        try:
            k = key.char.lower() if hasattr(key, 'char') and key.char else str(key)
            self.pressed_keys.add(k)
            
            # 触发速度调节回调
            if self.on_speed_change:
                if k == 'z': self.on_speed_change(-2.0) # 减速
                if k == 'x': self.on_speed_change(2.0)  # 加速
        except Exception: pass

    def _on_release(self, key):
        try:
            k = key.char.lower() if hasattr(key, 'char') and key.char else str(key)
            self.pressed_keys.discard(k)
        except Exception: pass

    def start(self): self.listener.start()
    def stop(self): self.listener.stop()
    def get_state(self) -> set: return self.pressed_keys

# ==========================================
# 第二层：轨迹规划 (带动态速度管理)
# ==========================================
class KeyboardPVTPlanner:
    def __init__(self, update_hz=50, init_speed=20.0):
        self.dt = 1.0 / update_hz
        self.current_max_speed = init_speed 
        self.target_angles = None
        
        # 限制范围，防止速度过快损坏硬件
        self.min_speed = 2.0
        self.max_speed = 160.0
        
        self.key_map = {
            'w': (0, 1), 's': (0, -1),
            'a': (1, 1), 'd': (1, -1),
            'q': (2, 1), 'e': (2, -1),
            'r': (3, 1), 'f': (3, -1),
            't': (4, 1), 'g': (4, -1),
        }

    def adjust_speed(self, delta):
        self.current_max_speed = max(self.min_speed, min(self.max_speed, self.current_max_speed + delta))
        print(f"🚀 当前设定的轴运行速度: {self.current_max_speed:.1f} °/s")

    def compute_next_step(self, active_keys, current_angles):
        if self.target_angles is None:
            self.target_angles = list(current_angles)

        next_velocities_deg_s = [0.0] * 5
        
        for key, (axis_idx, direction) in self.key_map.items():
            if key in active_keys:
                next_velocities_deg_s[axis_idx] = direction * self.current_max_speed
                self.target_angles[axis_idx] += next_velocities_deg_s[axis_idx] * self.dt

        return self.target_angles, next_velocities_deg_s

# ==========================================
# 第三层：主循环
# ==========================================
def main():
    print("🎮 启动 PVT 键盘遥操作 (动态调速版)...")
    
    update_hz = 50
    update_interval = 1.0 / update_hz
    move_time_ms = update_interval * 1000 
    
    kbd = KeyboardDevice()
    planner = KeyboardPVTPlanner(update_hz=update_hz, init_speed=20.0)
    
    # 绑定调速快捷键到 Planner
    kbd.on_speed_change = planner.adjust_speed
    
    pmac_config = PMACConfig(ip='192.168.0.200')
    robot = PMACRobotController(pmac_config)
    
    try:
        robot.hardware_boot()
        time.sleep(2)
        robot.connect_and_home()
        base_pulses = robot.base_positions.copy()
        
        current_pulses = robot.modbus.read_int32_array(address=10, count=5)
        curr_angles = [(p - base_pulses[i]) / robot.config.pulses_per_degree for i, p in enumerate(current_pulses)]
        
        kbd.start()
        print("\n✅ 控制就绪！")
        print("👉 移动: WASD / QE / RF / TG")
        print("👉 调速: Z (减速) / X (加速)")
        print(f"👉 初始速度: {planner.current_max_speed} °/s\n")

        next_call = time.perf_counter()

        while True:
            # 1. 计算
            active_keys = kbd.get_state()
            target_angles, v_degs = planner.compute_next_step(active_keys, curr_angles)
            
            # 2. 转换
            target_pulses = []
            velocities_p_ms = []
            for i in range(5):
                p = int(base_pulses[i] + (target_angles[i] * robot.config.pulses_per_degree))
                v = (v_degs[i] * robot.config.pulses_per_degree) / 1000.0
                target_pulses.append(p)
                velocities_p_ms.append(v)
            
            # 3. 下发
            robot.move_pvt_stream(target_pulses, velocities_p_ms, move_time_ms)
            
            # 4. 对齐
            next_call += update_interval
            sleep_time = next_call - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_call = time.perf_counter()

    except KeyboardInterrupt:
        print("\n⏹️ 停止操作。")
    finally:
        kbd.stop()
        robot.close()

if __name__ == "__main__":
    main()