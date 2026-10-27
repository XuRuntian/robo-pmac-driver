"""Offline checks for the motor-level recovery jog. No PMAC connection is made."""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'apps'))

from keyboard_reset_motor import (MOTOR_KEYS, KeyboardDevice, clamp_travel, describe,
                                  inferred_shape, parse_args, requested_step)
from continuum_sdk.control.axis_mapper import ContinuumAxisMapper
from continuum_sdk.control.straighten_ramp import StraightenRamp
from continuum_sdk.core.config_loader import load_continuum_config
from continuum_sdk.core.factory import build_tendon_mapper
from pmac_sdk.core.config_model import PMACConfig

TENDON = build_tendon_mapper(load_continuum_config(str(ROOT/'config/continuum.yaml')))
PMAC = PMACConfig()
ARGS = parse_args([])


# --- the reason this script exists ---

def test_a_bent_startup_pose_reads_as_zero_bend():
    """Motivation: joint-space recovery is impossible from a bent startup.

    safe_boot_and_home records the current encoders as the reference, so the
    model reports zero bend no matter how the robot is actually shaped, and a
    joint-space straighten ramp correctly concludes there is nothing to do.
    """
    axis = ContinuumAxisMapper(pulses_per_rad=PMAC.pulses_per_rad,
                               pulses_per_meter=PMAC.pulses_per_meter,
                               axis_order=PMAC.axis_order, axis_signs=PMAC.axis_signs)
    bent = [68887403 + int(math.radians(12.)*PMAC.pulses_per_rad),
            502449050, -138990801, 404264370, 957431]
    logical = axis.pulses_to_logical(bent, bent)   # captured as its own reference
    np.testing.assert_allclose(logical, np.zeros(5), atol=1e-12)
    assert StraightenRamp(np.zeros(5), .02, .5).already_straight


# --- key mapping ---

def test_no_keys_commands_no_motion():
    np.testing.assert_array_equal(requested_step(set(), ARGS, .02), np.zeros(5))


def test_each_key_moves_only_its_own_motor():
    for key, (index, sign) in MOTOR_KEYS.items():
        step = requested_step({key}, ARGS, .02)
        moved = [i for i, value in enumerate(step) if abs(value) > 0.]
        assert moved == [index], f'{key} moved {moved}, expected {index}'
        assert np.sign(step[index]) == sign


def test_all_five_logical_motors_are_reachable():
    assert {index for index, _ in MOTOR_KEYS.values()} == {0, 1, 2, 3, 4}


def test_opposite_keys_cancel():
    pairs = {}
    for key, (index, _) in MOTOR_KEYS.items():
        pairs.setdefault(index, []).append(key)
    for index, keys in pairs.items():
        assert len(keys) == 2, f'motor {index} needs exactly two directions'
        np.testing.assert_allclose(requested_step(set(keys), ARGS, .02), np.zeros(5),
                                   atol=1e-18)


def test_unknown_keys_are_ignored():
    np.testing.assert_array_equal(requested_step({'k', '1', 'j'}, ARGS, .02), np.zeros(5))


# --- rates ---

def test_motor_rate_matches_the_request():
    total = sum(requested_step({'w'}, ARGS, .02)[0] for _ in range(50))
    assert math.degrees(total) == pytest.approx(ARGS.deg_per_s, rel=1e-9)


def test_insertion_rate_matches_the_request():
    total = sum(requested_step({'v'}, ARGS, .02)[4] for _ in range(50))
    assert total*1000. == pytest.approx(ARGS.mm_per_s, rel=1e-9)


def test_shift_slows_and_ctrl_speeds_without_flipping_direction():
    normal = requested_step({'w'}, ARGS, .02)[0]
    slow = requested_step({'w', 'shift'}, ARGS, .02)[0]
    fast = requested_step({'w', 'ctrl'}, ARGS, .02)[0]
    assert slow == pytest.approx(normal*ARGS.fine_scale)
    assert fast == pytest.approx(normal*ARGS.coarse_scale)
    assert np.sign(slow) == np.sign(fast) == np.sign(normal)


def test_shift_overrides_ctrl():
    both = requested_step({'w', 'shift', 'ctrl'}, ARGS, .02)[0]
    slow = requested_step({'w', 'shift'}, ARGS, .02)[0]
    assert both == pytest.approx(slow)


def test_default_rate_stays_under_the_drivers_rotary_stop_threshold():
    """A manual jog must not be able to demand a guard-tripping motor rate."""
    peak = math.radians(ARGS.deg_per_s*ARGS.coarse_scale)
    assert peak < 2.5*.5, f'{peak:.3f} rad/s is too close to the 2.5 rad/s guard'


# --- travel clamp ---

def test_travel_clamp_bounds_every_motor_both_ways():
    far = np.array([10.]*4 + [1.])
    clamped = clamp_travel(far, ARGS)
    for i in range(4):
        assert math.degrees(clamped[i]) == pytest.approx(ARGS.max_deg)
    assert clamped[4]*1000. == pytest.approx(ARGS.max_mm)
    clamped = clamp_travel(-far, ARGS)
    for i in range(4):
        assert math.degrees(clamped[i]) == pytest.approx(-ARGS.max_deg)
    assert clamped[4]*1000. == pytest.approx(-ARGS.max_mm)


def test_in_range_targets_are_untouched():
    inside = np.array([math.radians(5.)]*4 + [.002])
    np.testing.assert_allclose(clamp_travel(inside.copy(), ARGS), inside)


def test_sustained_hold_saturates_at_the_limit():
    logical = np.zeros(5)
    for _ in range(2000):
        logical = clamp_travel(logical + requested_step({'w'}, ARGS, .02), ARGS)
    assert math.degrees(logical[0]) == pytest.approx(ARGS.max_deg)


# --- readout ---

def test_zero_motion_infers_zero_bend():
    theta_a, theta_c = inferred_shape(TENDON, np.zeros(5))
    assert theta_a == pytest.approx(0., abs=1e-12)
    assert theta_c == pytest.approx(0., abs=1e-12)


def test_readout_shows_coupling_into_both_segments():
    """One tendon motor changes both segments; the display must reflect that."""
    logical = np.zeros(5)
    logical[0] = math.radians(14.)
    theta_a, theta_c = inferred_shape(TENDON, logical)
    assert theta_a > math.radians(1.)
    assert theta_c > math.radians(1.)


def test_describe_reports_every_motor_and_the_inferred_bend():
    text = describe(np.array([math.radians(3.), 0., 0., 0., .001]), TENDON)
    assert '+3.00' in text and '+1.00mm' in text and 'inferred bend' in text


def test_describe_handles_the_singular_zero_pose():
    assert 'inferred bend' in describe(np.zeros(5), TENDON)


# --- keyboard plumbing ---

def test_keyboard_tracks_press_and_release_case_insensitively():
    device = KeyboardDevice()

    class Char:
        def __init__(self, char):
            self.char = char

    device._on_press(Char('W'))
    assert device.pressed == {'w'}
    device._on_release(Char('w'))
    assert device.pressed == set()
    assert not device.stop_requested


def test_esc_requests_stop():
    from pynput import keyboard as kb
    device = KeyboardDevice()
    device._on_press(kb.Key.esc)
    assert device.stop_requested


def test_modifier_keys_are_named():
    from pynput import keyboard as kb
    assert KeyboardDevice.key_name(kb.Key.shift) == 'shift'
    assert KeyboardDevice.key_name(kb.Key.ctrl_l) == 'ctrl'
    assert KeyboardDevice.key_name(kb.Key.space) == 'space'


# --- arguments ---

@pytest.mark.parametrize('argv', [
    ['--rate-hz', '0'],
    ['--rate-hz', '1'],            # 1000 ms segment exceeds the PVT 500 ms cap
    ['--deg-per-s', '-1'],
    ['--mm-per-s', '0'],
    ['--max-deg', '0'],
    ['--fine-scale', '0'],
    ['--fine-scale', '1.5'],
    ['--coarse-scale', '0.5'],
])
def test_invalid_arguments_are_rejected(argv):
    with pytest.raises(SystemExit):
        parse_args(argv)


def test_defaults_give_a_valid_pvt_segment():
    assert 5. < 1000./ARGS.rate_hz <= 500.


def test_dry_run_is_the_default():
    assert not ARGS.execute
