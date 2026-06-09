import numpy as np

class ContinuumAxisMapper:
    """唯一的单位转换层：将逻辑目标 (rad, m) 转换为物理脉冲"""
    def __init__(self, pulses_per_rad: float, pulses_per_meter: float, axis_order: list[int], axis_signs: list[int]):
        self.pulses_per_rad = pulses_per_rad
        self.pulses_per_meter = pulses_per_meter
        self.axis_order = axis_order
        self.axis_signs = axis_signs

    def logical_to_pulses(self, base_pulses: list[int], logical_targets: list[float]) -> list[int]:
        out = [0] * 5

        for logical_idx, value in enumerate(logical_targets):
            physical_idx = self.axis_order[logical_idx]
            sign = self.axis_signs[logical_idx]

            scale = self.pulses_per_rad if logical_idx < 4 else self.pulses_per_meter
            out[physical_idx] = int(base_pulses[physical_idx] + sign * value * scale)
            
        return out

    def pulses_to_logical(self, base_pulses: list[int], physical_pulses: list[int]) -> list[float]:
        """Convert physical PMAC feedback to [alpha1..alpha4, d] in rad and meters."""
        if len(base_pulses) != 5 or len(physical_pulses) != 5:
            raise ValueError("base_pulses and physical_pulses must each contain five values.")

        out = [0.0] * 5
        for logical_idx in range(5):
            physical_idx = self.axis_order[logical_idx]
            sign = self.axis_signs[logical_idx]
            scale = self.pulses_per_rad if logical_idx < 4 else self.pulses_per_meter
            out[logical_idx] = (
                (physical_pulses[physical_idx] - base_pulses[physical_idx]) / (sign * scale)
            )
        return out

    def logical_velocity_to_pulses_per_ms(self, logical_velocity: list[float]) -> list[float]:
        out = [0.0] * 5

        for logical_idx, value_per_s in enumerate(logical_velocity):
            physical_idx = self.axis_order[logical_idx]
            sign = self.axis_signs[logical_idx]
            
            scale = self.pulses_per_rad if logical_idx < 4 else self.pulses_per_meter
            out[physical_idx] = sign * value_per_s * scale / 1000.0

        return out

    def diff_velocity(self, prev_targets: list[float], next_targets: list[float], dt_s: float) -> list[float]:
        return ((np.asarray(next_targets) - np.asarray(prev_targets)) / dt_s).tolist()
