"""Joint-space straightening ramp: unbend both segments, hold insertion.

Reset means returning the tip to the straight pose, not returning the motors to
a stored reference: the recorded reference pulses are not trusted. Insertion d
is held exactly, so the tube never advances or retracts during a reset, and the
bending-plane angles phi are left alone because phi is meaningless once theta
reaches zero. Only the four rotary axes move.

No hardware access, no IK: the target pose is known in closed form.
"""
import math

import numpy as np

# u = [d, theta_a, phi_a, theta_c, phi_c]
D, THETA_A, PHI_A, THETA_C, PHI_C = range(5)


def smoothstep(value):
    value = max(0., min(1., value))
    return value*value*(3. - 2.*value)


class StraightenRamp:
    """Advance a held pose toward zero bend at a bounded joint rate."""

    def __init__(self, start_u, update_interval_s, max_theta_rate_rad_s):
        start = np.asarray(start_u, dtype=float)
        if start.shape != (5,) or not np.all(np.isfinite(start)):
            raise ValueError('start_u must be five finite joint values.')
        if not math.isfinite(update_interval_s) or update_interval_s <= 0.:
            raise ValueError('update_interval_s must be finite and positive.')
        if not math.isfinite(max_theta_rate_rad_s) or max_theta_rate_rad_s <= 0.:
            raise ValueError('max_theta_rate_rad_s must be finite and positive.')

        self.start = start.copy()
        self.update_interval_s = float(update_interval_s)
        # Hold d and both phi; drive only the two bending magnitudes to zero.
        self.target = start.copy()
        self.target[THETA_A] = 0.
        self.target[THETA_C] = 0.

        travel = max(abs(start[THETA_A]), abs(start[THETA_C]))
        # smoothstep peaks at 1.5x its average rate, so size the duration for
        # the peak rather than the average or the ramp would overshoot the cap.
        self.duration_s = 1.5*travel/max_theta_rate_rad_s if travel > 0. else 0.
        self.steps = max(1, int(math.ceil(self.duration_s/self.update_interval_s)))
        self.index = 0

    @property
    def already_straight(self):
        return self.duration_s == 0.

    @property
    def done(self):
        return self.index >= self.steps

    def peak_theta_rate_rad_s(self):
        """Fastest bend rate the plan will request, for pre-flight checks."""
        if self.already_straight:
            return 0.
        travel = max(abs(self.start[THETA_A]), abs(self.start[THETA_C]))
        return 1.5*travel/(self.steps*self.update_interval_s)

    def plan(self):
        """Every joint vector this ramp will command, without advancing it."""
        return [self._at(i) for i in range(1, self.steps + 1)]

    def _at(self, index):
        alpha = smoothstep(index/self.steps)
        return self.start + alpha*(self.target - self.start)

    def step(self):
        """Next joint vector; returns the straight target once exhausted."""
        if self.done:
            return self.target.copy()
        self.index += 1
        return self._at(self.index)
