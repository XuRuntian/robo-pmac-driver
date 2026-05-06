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
    """
    简易的 Joint-to-Joint 映射器 (替代复杂的 IK 算法)
    根据键盘输入，输出 5 个轴的目标增量角度。
    """
    def __init__(self, step_size_deg=0.5):
        self.step_size = step_size_deg # 每次循环的角度步进量
        # 维护一个纯软状态，记录目标角度 (相对于系统启动时的 0 度)
        self.target_angles = [0.0, 0.0, 0.0, 0.0, 0.0]

    def solve(self, active_keys: set) -> list[float]:
        """
        按键映射规则：
        Axis 1 (底座旋转): A(-) / D(+)
        Axis 2 (大臂俯仰): W(+) / S(-)
        Axis 3 (小臂俯仰): Q(+) / E(-)
        Axis 4 (末端旋转): R(+) / F(-)
        """
        if 'a' in active_keys: self.target_angles[0] -= self.step_size
        if 'd' in active_keys: self.target_angles[0] += self.step_size
        
        if 'w' in active_keys: self.target_angles[1] += self.step_size
        if 's' in active_keys: self.target_angles[1] -= self.step_size
        
        if 'q' in active_keys: self.target_angles[2] += self.step_size
        if 'e' in active_keys: self.target_angles[2] -= self.step_size
        
        if 'r' in active_keys: self.target_angles[3] += self.step_size
        if 'f' in active_keys: self.target_angles[3] -= self.step_size

        # Axis 5 (夹爪或其它) 预留，当前不动
        
        return self.target_angles

# ==========================================
# 第三层：调度与执行 (Application Loop)
# ==========================================
def main():
    print("初始化系统中...")
    
    # 1. 实例化各个解耦模块
    kbd = KeyboardDevice()
    mapper = SimpleJointMapper(step_size_deg=0.2) # 降低步进，让运动更平滑
    
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
        update_interval = 0.05
        
        while True:
            start_time = time.time()
            
            # [Input] 读取键盘输入状态
            keys = kbd.get_state()
            
            # 如果没有按键按下，我们依然维持当前角度下发，或者跳过
            # 为了维持系统刚度和避免 PMAC 报 timeout，持续下发是好习惯
            
            # [Algorithm] 映射为目标角度列表
            target_angles_deg = mapper.solve(keys)
            
            # [Output] 转化为绝对脉冲并下发
            targets_pulses = []
            for idx, angle_deg in enumerate(target_angles_deg):
                # 目标脉冲 = 基准位置 + 角度对应的增量脉冲
                pulse = int(base_pulses[idx] + (angle_deg * robot.config.pulses_per_degree))
                targets_pulses.append(pulse)
                
            # 批量下发给底层 (一次性写完 5 个轴，避免阻塞 Modbus)
            # time=50ms，拟合 20Hz 的控制周期
            robot.move_joints(
                target_pulses=targets_pulses,
                move_time=int(update_interval * 1000), 
                accel=10, 
                scurve=0
            )
            
            # 维持稳定的 20Hz 控制周期
            elapsed = time.time() - start_time
            if elapsed < update_interval:
                time.sleep(update_interval - elapsed)
                
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