import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class ContinuumGeometry:
    s_s: float = 0.002
    s_a: float = 0.048
    s_c: float = 0.024
    h_bc: float = 0.0064
    h_de: float = 0.009
    theta_a_max: float = math.radians(180.0)
    theta_c_max: float = math.radians(90.0)
    d_min: float = 0.0
    d_max: float = 0.255
    use_sheath: bool = False
    base_offset: tuple[float, float, float] = (0.0, 0.4402, 0.207)
