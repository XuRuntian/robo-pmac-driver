import numpy as np
import forcedimension_core.dhd as dhd
import forcedimension_core.drd as drd
import time
import ctypes
from scipy.spatial.transform import Rotation as R  # 用于旋转转换

def init_device():
    dhd.close()
    res = drd.open()
    if res < 0:
        print(f"❌ 错误: 无法打开 DRD 模块 (返回码: {res})")
        return False

    print(f"✅ DRD 模式开启，设备 ID: {res}")

    if not drd.isInitialized():
        print("正在进行自动校准 (Auto-Init)，请松开主手...")
        if drd.autoInit() < 0:
            print("❌ 校准失败!")
            return False
    
    dhd.enableForce(True)
    drd.stop(True) # 切换到透明模式，消除震动
    
    print("✨ 设备已进入就绪状态 (自由移动模式)")
    return True

def main_loop():
    if not init_device():
        return

    # 准备容器
    pos = np.zeros(3)
    matrix = np.eye(3) # 旋转矩阵
    gripper_ptr = ctypes.pointer(ctypes.c_double(0.0))
    last_display_time = time.time()

    try:
        print(">>> 开始读取数据，按 'q' 退出...")
        
        while True:
            # --- A. 读取物理状态 ---
            # 这个函数会同时更新 pos 和 matrix
            dhd.getPositionAndOrientationFrame(pos, matrix)
            dhd.getGripperAngleDeg(gripper_ptr)
            gripper_deg = gripper_ptr.contents.value

            # --- B. 旋转数据处理 ---
            # 将 3x3 旋转矩阵转换为欧拉角 (单位：度)
            # 'xyz' 表示旋转顺序，你可以根据仿真环境的需求调整为 'zyx' 等
            try:
                r = R.from_matrix(matrix)
                euler = r.as_euler('xyz', degrees=True) 
            except ValueError:
                # 偶尔在极速运动时矩阵可能由于数值精度失去正交性，做个保险
                euler = np.zeros(3)

            # --- C. 打印调试 ---
            current_time = time.time()
            if current_time - last_display_time > 0.1:
                # 更新打印：加入 R(Roll), P(Pitch), Y(Yaw)
                print(f"Pos: {pos[0]:+.3f} {pos[1]:+.3f} {pos[2]:+.3f} | "
                      f"Ori: R{euler[0]:+06.1f} P{euler[1]:+06.1f} Y{euler[2]:+06.1f} | "
                      f"Grip: {gripper_deg:+.1f}°", end='\r')
                last_display_time = current_time

            # --- D. 映射逻辑 (Mujoco 仿真) ---
            # action_quat = R.from_matrix(matrix).as_quat() # 如果仿真需要四元数

            if dhd.os_independent.kbHit():
                if dhd.os_independent.kbGet() == 'q':
                    break
            
            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\n用户中断...")
    finally:
        drd.stop(False) 
        drd.close()
        dhd.close()
        print("\n✅ 设备已安全关闭")

if __name__ == "__main__":
    main_loop()
    
    # Pos: -0.021 +0.013 -0.042 | Ori: R+049.7 P+008.6 Y+070.9 | Grip: +4.1°