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
    print("🚀 启动 PVT 模式键盘遥操作测试...")
    
    kbd = KeyboardDevice()
    mapper = SimpleJointMapper(step_size_deg=5) # 步进
    
    pmac_config = PMACConfig(ip='192.168.0.200')
    robot = PMACRobotController(pmac_config)
    
    try:
        # 1. 硬件初始化
        robot.hardware_boot()
        time.sleep(2)
        robot.connect_and_home()
        
        # 获取初始基准脉冲
        base_pulses = robot.base_positions.copy()
        # 【关键】：初始化“上一次目标”，用于速度计算
        last_target_pulses = base_pulses.copy()
        
        kbd.start()
        print("\n✅ 系统就绪！请确保 PMAC 端已运行 '&1 b1 r'") # 运行 prog 1
        
        # 2. 设定控制频率 (建议 20Hz - 50Hz)
        update_interval = 0.02  # 50Hz
        move_time_ms = update_interval * 1000
        
        while True:
            loop_start = time.perf_counter()
            
            # 读取当前物理角度 (用于 Mapper 算法)
            current_pulses = robot.modbus.read_int32_array(address=10, count=5)
            current_angles_deg = [(p - base_pulses[i]) / robot.config.pulses_per_degree for i, p in enumerate(current_pulses)]
            
            # 计算新的目标角度
            keys = kbd.get_state()
            target_angles_deg = mapper.solve(keys, current_angles_deg)
            
            # 转换为目标脉冲
            target_pulses = []
            for idx, angle_deg in enumerate(target_angles_deg):
                p = int(base_pulses[idx] + (angle_deg * robot.config.pulses_per_degree))
                target_pulses.append(p)
            
            # --- 【核心新增】：计算瞬时速度 (脉冲/ms) ---
            # V = (P_new - P_old) / T
            velocities = []
            for curr, last in zip(target_pulses, last_target_pulses):
                vel = (curr - last) / move_time_ms
                velocities.append(vel)
            
            # 更新历史记录
            last_target_pulses = target_pulses.copy()
            
            # --- 【核心调用】：PVT 流式下发 ---
            # 内部会自动缩放 10000 倍并写入地址 0, 40, 50, 200[cite: 4, 6]
            robot.move_pvt_stream(
                target_pulses=target_pulses,
                velocities=velocities,
                move_time=move_time_ms
            )
            
            # 维持频率
            loop_cost = time.perf_counter() - loop_start
            sleep_time = update_interval - loop_cost
            if sleep_time > 0:
                time.sleep(sleep_time)

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