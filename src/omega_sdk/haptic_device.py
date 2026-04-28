# src/omega_sdk/haptic_device.py
import numpy as np
import forcedimension_core.dhd as dhd
import forcedimension_core.drd as drd
import ctypes
from scipy.spatial.transform import Rotation as R
from dataclasses import dataclass

@dataclass
class HapticState:
    """定义主手状态的标准化数据结构"""
    pos: np.ndarray       # [x, y, z]
    rot_matrix: np.ndarray # 3x3 旋转矩阵 (如果逆解需要)
    euler: np.ndarray     # [Roll, Pitch, Yaw] (度)
    gripper_deg: float    # 夹爪角度

class OmegaDevice:
    """Force Dimension Omega/Sigma 主手硬件抽象类"""
    def __init__(self):
        self._gripper_ptr = ctypes.pointer(ctypes.c_double(0.0))
        self._pos = np.zeros(3)
        self._matrix = np.eye(3)
        self._is_connected = False

    def connect(self) -> bool:
        """初始化 DRD/DHD 设备并进入自由移动模式"""
        dhd.close()
        res = drd.open()
        if res < 0:
            print(f"❌ 无法打开 DRD 模块 (错误码: {res})")
            return False

        if not drd.isInitialized():
            print("正在进行自动校准，请松开主手...")
            if drd.autoInit() < 0:
                print("❌ 校准失败!")
                return False
        
        dhd.enableForce(True)
        # 显式传入 force_on 位置参数，避免 drd 模块抛出 TypeError
        drd.stop(True) 
        
        self._is_connected = True
        print("✅ Omega 主手已就绪 (透明模式)")
        return True

    def get_state(self) -> HapticState:
        """读取最新物理状态，对外部隐藏 ctypes 和底层 API 细节"""
        if not self._is_connected:
            raise RuntimeError("设备未连接，无法读取状态")

        dhd.getPositionAndOrientationFrame(self._pos, self._matrix)
        dhd.getGripperAngleDeg(self._gripper_ptr)
        
        try:
            r = R.from_matrix(self._matrix)
            euler = r.as_euler('xyz', degrees=True)
        except ValueError:
            euler = np.zeros(3)

        return HapticState(
            pos=self._pos.copy(),
            rot_matrix=self._matrix.copy(),
            euler=euler,
            gripper_deg=self._gripper_ptr.contents.value
        )

    def close(self):
        """安全释放硬件资源"""
        if self._is_connected:
            drd.stop(False) # 关闭 force_on
            drd.close()
            dhd.close()
            self._is_connected = False
            print("🔌 Omega 主手已安全关闭")