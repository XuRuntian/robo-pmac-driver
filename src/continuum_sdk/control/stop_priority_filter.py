"""Optional teleoperation policy: discard unexecuted input instead of chasing it.

Wire commands remain reference-relative pose offsets. Differences are taken
internally, so retransmitting a fixed offset never adds motion. This changes
position correspondence intentionally; it is not a trajectory capability model.
"""
import numpy as np

from continuum_sdk.control.cartesian_frame import apply_axis_transform
from continuum_sdk.control.tip_command_filter import TipCommandFilter


class StopPriorityCommandFilter(TipCommandFilter):
    def __init__(self, config, update_interval_s):
        super().__init__(config, update_interval_s)
        if not 0 < config.smooth_alpha <= 1:
            raise ValueError('Stop priority requires 0 < smooth_alpha <= 1.')
        self._last_input = None
        self._motion_anchor = None
        self._motion_seen = False
        self._idle_elapsed = 0.
        self.input_holding = True
        self.input_discarded = False
        self._epsilon = np.array([config.input_translation_epsilon_m]*3
                                 + [config.input_rotation_epsilon_rad]*3)
        self._speed = np.array([*config.max_speed_m_s, *config.max_angular_speed_rad_s])
        self._caps = np.array([*config.max_delta_m, *self._rotation_caps])

    def set_command(self, command):
        old_target = np.r_[self._target_delta, self._target_rotation]
        # Preserve existing input validation, deadband, disabled Ry, and caps.
        normalized = super().set_command(command)
        raw = np.r_[self._target_delta, self._target_rotation]
        self._target_delta, self._target_rotation = old_target[:3], old_target[3:]
        if self._last_input is None:
            # First command after startup/explicit hold establishes a reference.
            self._last_input = raw.copy()
            self._motion_anchor = raw.copy()
            return normalized
        moved = np.any(np.abs(raw-self._motion_anchor) > self._epsilon)
        delta = raw - (self._motion_anchor if self.input_holding else self._last_input)
        if moved:
            self._motion_anchor = raw.copy()
            self._motion_seen = True
            self.input_holding = False
        if not self.input_holding:
            candidate = old_target + delta
            applied = np.r_[self._applied_delta, self._applied_rotation]
            # Permit gradual return if model reconstruction is outside the
            # command envelope; never jump back to its edge during reanchoring.
            candidate = np.clip(candidate, np.minimum(-self._caps, applied),
                                np.maximum(self._caps, applied))
            self._target_delta, self._target_rotation = candidate[:3], candidate[3:]
        self._last_input = raw.copy()
        return normalized

    def step(self):
        self._idle_elapsed = 0. if self._motion_seen else self._idle_elapsed+self.update_interval_s
        self._motion_seen = False
        if self._idle_elapsed + 1e-12 >= self.config.input_idle_timeout_s:
            if not self.input_holding and self._last_input is not None:
                self._motion_anchor = self._last_input.copy()
            self.input_holding = True
        applied = np.r_[self._applied_delta, self._applied_rotation]
        target = np.r_[self._target_delta, self._target_rotation]
        if self.input_holding:
            self.input_discarded = bool(np.any(np.abs(target-applied) > 1e-12))
            target = applied.copy()
        else:
            # Limit stored lead to at most one speed-limited filtered step.
            # Saturated excess is discarded and is not restored by repeats.
            lead = self._speed*self.update_interval_s/self.config.smooth_alpha
            bounded = np.clip(target, applied-lead, applied+lead)
            self.input_discarded = bool(np.any(np.abs(target-bounded) > 1e-12))
            target = bounded
            applied = applied + self.config.smooth_alpha*(target-applied)
        self._applied_delta, self._applied_rotation = applied[:3], applied[3:]
        self._target_delta, self._target_rotation = target[:3], target[3:]
        return self.applied_delta

    def hold(self):
        super().hold()
        self._last_input = None
        self._motion_anchor = None
        self._motion_seen = False
        self._idle_elapsed = 0.
        self.input_holding = True

    def reanchor_from_tip(self, position, rotation, center_p, center_r, frame):
        """Cancel IK/execution debt while held, using committed-target FK.

        Keep the original reference frame and configured WORLD mappings. Only
        tip direction matters: the shortest tilt has zero local roll, so WORLD
        Ry stays disabled. This is command-state reanchoring, not encoder reset.
        """
        if not self.input_holding:
            raise RuntimeError('Reanchor only while input is held.')
        z = np.asarray(center_r).T @ np.asarray(rotation)[:, 2]
        z = z/np.linalg.norm(z)
        cross = np.array([-z[1], z[0], 0.])
        length = np.linalg.norm(cross)
        if length > 1e-12:
            tilt = cross/length*np.arctan2(length, z[2])
        elif z[2] < 0:
            tilt = np.array([np.pi, 0., 0.])
        else:
            tilt = np.zeros(3)
        def inverse(vector, mapping, signs):
            basis = np.column_stack([apply_axis_transform(v, mapping, signs) for v in np.eye(3)])
            return basis.T @ vector
        delta = inverse(np.asarray(position)-center_p, frame.translation_map, frame.translation_signs)
        rot = inverse(tilt, frame.rotation_map, frame.rotation_signs)
        if not self.config.orientation_enabled:
            rot = np.zeros(3)
        if np.any(np.abs(rot[self._rotation_caps == 0]) > 1e-8):
            raise ValueError('Stop priority requires the disabled WORLD rotation to map to local roll.')
        self._applied_delta = delta.copy()
        self._target_delta = delta.copy()
        self._applied_rotation = rot.copy()
        self._target_rotation = rot.copy()
