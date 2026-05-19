import math
from dataclasses import dataclass, field
from typing import List

@dataclass
class PMACConfig:
    # --- 网络与通信 ---
    ip: str = '192.168.0.200'
    modbus_port: int = 502
    slave_id: int = 1
    ssh_user: str = 'root'
    ssh_pass: str = 'deltatau'
    
    # --- 物理电机参数 ---
    # 旋转轴 (1-4轴) 参数
    encoder_resolution: int = 131072
    gear_ratio: float = 97.34
    # 直线轴 (5轴) 参数
    pulses_per_meter: float = 62781744.0  
    
    # 基准偏置
    zero_offsets: List[int] = field(default_factory=lambda: [0, 0, 0, 0, 0])
    
    # --- 轴映射关系 (逻辑算法 -> 物理电机) ---
    # 逻辑轴顺序为: [a_x, a_y, c_x, c_y, d]
    # 如果物理接线变了，只需改下面这两个列表
    axis_order: List[int] = field(default_factory=lambda: [1, 0, 2, 3, 4])
    axis_signs: List[int] = field(default_factory=lambda: [1, -1, -1, -1, 1]) # 暂定为符号给到调换之前

    @property
    def pulses_per_degree(self) -> float:
        return (self.encoder_resolution * self.gear_ratio) / 360.0

    @property
    def pulses_per_rad(self) -> float:
        """根据电机分辨率自动计算弧度脉冲比"""
        return (self.encoder_resolution * self.gear_ratio) / (2 * math.pi)