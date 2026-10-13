"""Stateful insertion-only PVT profile. Does not solve IK or remap axes.

Limits are checked on the cubic interpolant after integer position and wire
velocity quantization. Other axes are intentionally outside this component.
"""
from dataclasses import dataclass
import math


def cubic_extrema(p0, p1, v0, v1, dt):
    """Absolute peak velocity/acceleration in counts/s and counts/s²."""
    a = 2 * (p0 - p1) + dt * (v0 + v1)
    b = 3 * (p1 - p0) - dt * (2 * v0 + v1)
    c = dt * v0
    points = [0., 1.]
    if a and 0 < -b / (3 * a) < 1:
        points.append(-b / (3 * a))
    return (max(abs((3*a*s*s+2*b*s+c)/dt) for s in points),
            max(abs(2*b), abs(6*a+2*b))/dt**2)


@dataclass(frozen=True)
class InsertionSegment:
    position: int
    velocity_counts_ms: float
    peak_speed_counts_s: float
    peak_accel_counts_s2: float
    remaining_counts: float


class InsertionProfile:
    def __init__(self, position, dt, max_speed, max_accel, max_step):
        for value in (dt, max_speed, max_accel, max_step):
            if not math.isfinite(value) or value <= 0:
                raise ValueError("Insertion profile limits must be finite and positive.")
        self.dt, self.max_speed, self.max_accel, self.max_step = dt, max_speed, max_accel, max_step
        # Worst-case cubic perturbation from two rounded positions (±0.5 ct)
        # and two truncated velocities (<0.1 ct/s). Leave a little extra slack.
        self.speed = min(max_speed, max_step/dt) - 1.5/dt - .2
        self.accel = max_accel - 6/dt**2 - 1.2/dt
        if self.speed <= 0 or self.accel <= 0:
            raise ValueError("Insertion limits are below the quantization margin at this period.")
        self.p = float(position)
        self.v = 0.
        self.wire_p = int(position)
        self.wire_v = 0.

    def _stopping_distance(self, speed):
        """Distance when braking at cycle boundaries, including the last cycle.

        A final fractional-speed cycle still occupies a full PVT period, so
        v²/(2a) alone underestimates this generator's braking distance.
        """
        decrement = self.accel * self.dt
        full = math.floor(speed / decrement)
        remainder = speed - full * decrement
        return self.dt * (full * speed - .5 * decrement * full * full + .5 * remainder)

    def _arrival_segment(self, target):
        """Accept a one-cycle rest endpoint only if its entire cubic is valid."""
        end = round(target)
        if not -(2**31) <= end < 2**31:
            return None
        distance = end - self.wire_p
        pv, pa = cubic_extrema(self.wire_p, end, self.wire_v, 0., self.dt)
        if abs(distance) > self.max_step or pv > self.max_speed or pa > self.max_accel:
            return None
        # With end velocity zero, this Hermite cubic is monotone exactly when
        # the initial signed velocity lies between 0 and 3*distance/dt.
        if distance == 0:
            if self.wire_v != 0.:
                return None
        elif not 0 <= self.wire_v * (1 if distance > 0 else -1) <= 3 * abs(distance) / self.dt:
            return None
        return end, pv, pa

    def step(self, target, *, holding=False):
        if not math.isfinite(target) or not -(2**31) <= target < 2**31:
            raise ValueError("Invalid insertion target.")
        h, a = self.dt, self.accel
        if not holding:
            arrival = self._arrival_segment(target)
            if arrival is not None:
                end, pv, pa = arrival
                self.p, self.v, self.wire_p, self.wire_v = float(end), 0., end, 0.
                return InsertionSegment(end, 0., pv, pa, target-end)
        if holding:
            desired = 0.
        else:
            error = target - self.p
            sign = 1. if error >= 0 else -1.
            # Include the next segment and all subsequent discrete braking
            # segments. The expression is monotone in the next signed speed.
            low, high = 0., self.speed
            for _ in range(40):
                candidate = .5 * (low + high)
                distance = .5 * (sign*self.v + candidate) * h + self._stopping_distance(candidate)
                if distance <= abs(error):
                    low = candidate
                else:
                    high = candidate
            desired = sign * low
        v = max(self.v-a*h, min(self.v+a*h, desired))
        p = self.p + .5*(self.v+v)*h
        # A resting hold must keep the identical last commanded integer pulse.
        wire_p = round(p)
        wire_v_ms = math.trunc(v*10) / 10000.
        wire_v = wire_v_ms*1000
        pv, pa = cubic_extrema(self.wire_p, wire_p, self.wire_v, wire_v, h)
        if (not -(2**31) <= wire_p < 2**31 or abs(wire_p-self.wire_p) > self.max_step
                or pv > self.max_speed+1e-6 or pa > self.max_accel+1e-6):
            raise RuntimeError("Insertion profile segment failed quantized PVT validation.")
        self.p, self.v, self.wire_p, self.wire_v = p, v, wire_p, wire_v
        return InsertionSegment(wire_p, wire_v_ms, pv, pa, target-wire_p)
