from dataclasses import dataclass

@dataclass(frozen=True)
class ContinuumGeometry:
    s_s: float = 0.002
    s_a: float = 0.048
    s_c: float = 0.024
    h_bc: float = 0.0064
    h_de: float = 0.009
    theta_a_max: float = 3.141592653589793
    theta_c_max: float = 1.5707963267948966
    d_min: float = 0.0
    d_max: float = 0.255
    use_sheath: bool = False
    base_offset: tuple[float, float, float] = (0.0, 0.4402, 0.207)

    def insertion_bounds(self) -> tuple[float, float]:
        return self.d_min, self.d_max

    def sheath_partition_lengths(self, insertion: float) -> tuple[float, float]:
        inside_total = max(0.0, -float(insertion))

        segment_a_inside = min(inside_total, self.s_a)
        inside_total -= segment_a_inside

        inside_total = max(0.0, inside_total - self.h_bc)
        segment_c_inside = min(inside_total, self.s_c)
        return segment_a_inside, segment_c_inside

    def curvature_caps(self, insertion: float) -> tuple[float, float]:
        if not self.use_sheath:
            return self.theta_a_max, self.theta_c_max

        segment_a_inside, segment_c_inside = self.sheath_partition_lengths(insertion)
        segment_a_effective = self.s_a - segment_a_inside
        segment_c_effective = self.s_c - segment_c_inside

        theta_a_cap = self.theta_a_max * (segment_a_effective / max(self.s_a, 1e-9))
        theta_c_cap = self.theta_c_max * (segment_c_effective / max(self.s_c, 1e-9))
        return float(theta_a_cap), float(theta_c_cap)
