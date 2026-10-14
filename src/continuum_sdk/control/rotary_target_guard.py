"""Reject anomalous PVT segments; not a motor capability model or smoother."""
import math

from continuum_sdk.control.insertion_profile import cubic_extrema


class RotaryTargetJump(RuntimeError):
    def __init__(self, evidence):
        self.evidence = evidence
        super().__init__(
            f"ROTARY TARGET STOP: physical axis {evidence['physical_axis']}, "
            f"PVT peak {evidence['peak_rad_s']:.3f} rad/s exceeds software "
            f"stop threshold {evidence['threshold_rad_s']:.3f} rad/s. "
            "Candidate not submitted; ending session, no automatic resume.")


class RotaryTargetGuard:
    def __init__(self, axis_mapper, threshold_rad_s):
        if threshold_rad_s is None or not math.isfinite(threshold_rad_s) or threshold_rad_s <= 0:
            raise ValueError('Set execution.rotary_target_stop_rad_s to a finite positive software threshold.')
        self.mapper = axis_mapper
        self.threshold = float(threshold_rad_s)

    def check(self, previous, candidate, previous_velocity, candidate_velocity, dt):
        """Physical positions in counts; endpoint velocities in counts/ms.

        Check the entire cubic segment, including endpoint velocity overshoot.
        Conversion uses the same mapping as control. No mutation or hardware I/O.
        """
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError('Invalid PVT period.')
        if any(len(v) != 5 or not all(math.isfinite(x) for x in v)
               for v in (previous, candidate, previous_velocity, candidate_velocity)):
            raise ValueError('Expected five finite PVT positions and velocities.')
        # Mirror wire conversion performed by PMACRobotController.
        p0 = [int(x) for x in previous]
        p1 = [int(x) for x in candidate]
        delta = self.mapper.pulses_to_logical(p0, p1)
        velocities = [self.mapper.pulses_to_logical(
            [0]*5, [math.trunc(x*10000)/10000*1000 for x in v])
            for v in (previous_velocity, candidate_velocity)]
        peaks = [cubic_extrema(0., delta[i], velocities[0][i], velocities[1][i], dt)[0]
                 for i in range(4)]
        worst = max(range(4), key=peaks.__getitem__)
        if peaks[worst] > self.threshold:
            raise RotaryTargetJump(dict(
                kind='rotary_target_stop', physical_axis=self.mapper.axis_order[worst]+1,
                logical_axis=worst+1, peak_rad_s=peaks[worst],
                threshold_rad_s=self.threshold, logical_delta_rad=delta[:4],
                peaks_rad_s=peaks, previous_pulses=p0, rejected_pulses=p1,
                previous_velocity_counts_ms=list(previous_velocity),
                rejected_velocity_counts_ms=list(candidate_velocity), period_s=dt))
        return peaks
