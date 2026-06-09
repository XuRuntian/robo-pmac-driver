import time
import sys
from pathlib import Path

# 保证能找到 src 目录
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))

from pmac_sdk.core.config_model import PMACConfig
from pmac_sdk.controller.robot_api import PMACRobotController
from pynput import keyboard

# ==========================================
# 第一层：输入解耦 (Input Layer)
# ==========================================
class KeyboardDevice:
    """
    键盘输入监听类 (替代 OmegaDevice)
    非阻塞地捕获按键状态，解耦具体硬件。
    """
    def __init__(self):
        self.pressed_keys = set()
        self.listener = keyboard.Listener(
            on_press=self._on_press,
            on_release=self._on_release
        )

    def _on_press(self, key):
        try:
            self.pressed_keys.add(key.char.lower())
        except AttributeError:
            pass # 忽略非字母按键

    def _on_release(self, key):
        try:
            self.pressed_keys.discard(key.char.lower())
        except AttributeError:
            pass

    def start(self):
        self.listener.start()
        print("⌨️  键盘监听已启动...")

    def stop(self):
        self.listener.stop()

    def get_state(self) -> set:
        """返回当前被按下的所有键"""
        return self.pressed_keys

# ==========================================
# 第二层：算法与映射解耦 (Algorithm Layer)
# ==========================================
class SimpleJointMapper:
    def __init__(self, step_size_deg=3.0): 
        # 每次循环的角度步进量（在 16Hz 下，1度代表期望速度 16度/秒）
        self.step_size = step_size_deg 
        # 维护一个纯软状态 (我们的“胡萝卜”)
        self.target_angles = None 
        # 牵引绳的最大长度（度）。这个值决定了“手感”：
        # 太大 -> 松手后会有惯性（继续转一会）
        # 太小 -> 电机跑不快（蠕动）
        self.max_lead_deg = 15.0 

    def solve(self, active_keys: set, current_angles: list[float]) -> list[float]:
        # 第一次运行，将胡萝卜对齐到电机的真实位置
        if self.target_angles is None:
            self.target_angles = current_angles.copy()

        # 根据按键移动“胡萝卜”
        if 'a' in active_keys: self.target_angles[0] -= self.step_size
        if 'd' in active_keys: self.target_angles[0] += self.step_size
        if 'w' in active_keys: self.target_angles[1] += self.step_size
        if 's' in active_keys: self.target_angles[1] -= self.step_size
        if 'q' in active_keys: self.target_angles[2] += self.step_size
        if 'e' in active_keys: self.target_angles[2] -= self.step_size
        if 'r' in active_keys: self.target_angles[3] += self.step_size
        if 'f' in active_keys: self.target_angles[3] -= self.step_size

        # 【核心逻辑：弹性牵引绳】
        # 限制“胡萝卜”不能离“电机真实位置”太远，防止指令积压
        for i in range(5):
            max_allowed = current_angles[i] + self.max_lead_deg
            min_allowed = current_angles[i] - self.max_lead_deg
            
            if self.target_angles[i] > max_allowed:
                self.target_angles[i] = max_allowed
            elif self.target_angles[i] < min_allowed:
                self.target_angles[i] = min_allowed

        return self.target_angles

# ==========================================
# 第三层：调度与执行 (Application Loop)
# ==========================================
def main():
    print("初始化系统中...")
    
    # 1. 实例化各个解耦模块
    kbd = KeyboardDevice()
    mapper = SimpleJointMapper(step_size_deg=10) # 降低步进，让运动更平滑
    
    pmac_config = PMACConfig(ip='192.168.0.200')
    robot = PMACRobotController(pmac_config)
    
    try:
        # 2. 硬件连接 (与原本逻辑保持一致)
        robot.hardware_boot()
        time.sleep(2)
        robot.connect_and_home()
        
        # 获取系统启动时的物理脉冲基准，作为 0 度参考点
        base_pulses = robot.base_positions.copy()
        
        kbd.start()
        print("\n🚀 开始键盘遥操作测试 (按 Ctrl+C 退出)...")
        print("🕹️  控制说明:")
        print("   轴1: A / D")
        print("   轴2: W / S")
        print("   轴3: Q / E")
        print("   轴4: R / F")
        
        # 3. 遥操作主循环 (固定频率 20Hz)
       # 3. 遥操作主循环 (固定频率 20Hz)
        update_interval = 0.03
        
        # [Debug 专用变量]
        loop_count = 0
        total_modbus_time = 0
        
        while True:
            loop_start = time.perf_counter() # 使用高精度时钟
            current_pulses = robot.modbus.read_int32_array(address=10, count=5)
            current_angles_deg = [(p - base_pulses[i]) / robot.config.pulses_per_degree for i, p in enumerate(current_pulses)]
            # 1. 捕捉输入与计算目标
            keys = kbd.get_state()
            target_angles_deg = mapper.solve(keys, current_angles_deg)
            
            targets_pulses = []
            for idx, angle_deg in enumerate(target_angles_deg):
                pulse = int(base_pulses[idx] + (angle_deg * robot.config.pulses_per_degree))
                targets_pulses.append(pulse)
            
            # 2. 测量 Modbus 通信耗时
            modbus_start = time.perf_counter()
            robot.move_joints(
                target_pulses=targets_pulses,
                move_time=int(update_interval * 1000), 
                accel=10, 
                scurve=0
            )
            modbus_cost = time.perf_counter() - modbus_start
            
            # 3. 计算循环耗时与自适应休眠
            loop_cost = time.perf_counter() - loop_start
            sleep_time = update_interval - loop_cost
            if sleep_time > 0:
                time.sleep(sleep_time)
            
            # [Debug 打印] 每 10 次循环打印一次诊断信息 (约 0.5 秒)
            loop_count += 1
            total_modbus_time += modbus_cost
            if loop_count % 10 == 0:
                avg_modbus = (total_modbus_time / 10) * 1000 # 换算为毫秒
                real_hz = 1.0 / (time.perf_counter() - loop_start) # 实际运行频率
                
                print(f"📊 Debug Info | 按键: {list(keys)} | "
                      f"目标角度[0]: {target_angles_deg[0]:.2f}° | "
                      f"Modbus 耗时: {avg_modbus:.1f}ms | "
                      f"实际循环频率: {real_hz:.1f}Hz | "
                      f"是否超时: {'❌' if sleep_time <= 0 else '✅'}")
                total_modbus_time = 0
                
    except KeyboardInterrupt:
        print("\n⏹️ 接收到退出信号。")
    except Exception as e:
        print(f"\n❌ 运行时异常: {e}")
    finally:
        kbd.stop()
        robot.close()
        print("🔌 系统已安全关闭。")

if __name__ == "__main__":
    main()