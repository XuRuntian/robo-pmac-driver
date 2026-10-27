"""Straighten reset: unbend both segments while holding insertion depth."""
import math

import numpy as np
import pytest

from continuum_sdk.control.straighten_ramp import (D, PHI_A, PHI_C, THETA_A, THETA_C,
                                                   StraightenRamp, smoothstep)
from continuum_sdk.core.config_loader import load_continuum_config
from continuum_sdk.core.factory import build_continuum_ik, build_tendon_mapper
from continuum_sdk.transport.zmq_protocol import (build_straighten_message,
                                                  parse_control_message)

INTERVAL = .02
RATE = .5
BENT = np.array([.080, math.radians(60.), .7, math.radians(40.), -1.2])
TENDON = build_tendon_mapper(load_continuum_config('config/continuum.yaml'))
GEOMETRY = build_continuum_ik(load_continuum_config('config/continuum.yaml')).geometry


# --- the defining property: insertion must not move ---

def test_insertion_is_held_exactly_through_the_whole_ramp():
    """The reset must never advance or retract the tube."""
    ramp = StraightenRamp(BENT, INTERVAL, RATE)
    for joint in ramp.plan():
        assert joint[D] == BENT[D]


def test_insertion_is_held_in_the_motor_targets_too():
    """Holding d in joint space is useless if the logical axis still moves."""
    ramp = StraightenRamp(BENT, INTERVAL, RATE)
    start_d = TENDON.to_axis_targets(BENT)[4]
    for joint in ramp.plan():
        assert TENDON.to_axis_targets(joint)[4] == pytest.approx(start_d)


def test_bending_planes_are_held():
    """phi is meaningless at theta=0, so moving it would be wasted motion."""
    ramp = StraightenRamp(BENT, INTERVAL, RATE)
    for joint in ramp.plan():
        assert joint[PHI_A] == BENT[PHI_A]
        assert joint[PHI_C] == BENT[PHI_C]


# --- reaching straight ---

def test_ramp_ends_with_both_segments_straight():
    ramp = StraightenRamp(BENT, INTERVAL, RATE)
    final = ramp.plan()[-1]
    assert final[THETA_A] == pytest.approx(0., abs=1e-12)
    assert final[THETA_C] == pytest.approx(0., abs=1e-12)


def test_both_bend_magnitudes_decrease_monotonically():
    ramp = StraightenRamp(BENT, INTERVAL, RATE)
    plan = [BENT] + ramp.plan()
    for index in (THETA_A, THETA_C):
        values = [row[index] for row in plan]
        assert all(b <= a + 1e-15 for a, b in zip(values, values[1:]))


def test_step_reaches_straight_and_stays_there():
    ramp = StraightenRamp(BENT, INTERVAL, RATE)
    while not ramp.done:
        joint = ramp.step()
    assert joint[THETA_A] == pytest.approx(0., abs=1e-12)
    # Extra calls past the end must not drift or raise.
    for _ in range(5):
        repeat = ramp.step()
        assert repeat[THETA_A] == pytest.approx(0., abs=1e-12)
        assert repeat[D] == BENT[D]


def test_step_matches_plan():
    ramp = StraightenRamp(BENT, INTERVAL, RATE)
    planned = StraightenRamp(BENT, INTERVAL, RATE).plan()
    for expected in planned:
        np.testing.assert_allclose(ramp.step(), expected)


# --- rate bounding ---

def test_peak_rate_respects_the_configured_cap():
    """smoothstep peaks at 1.5x average, so the duration must size for the peak."""
    ramp = StraightenRamp(BENT, INTERVAL, RATE)
    assert ramp.peak_theta_rate_rad_s() <= RATE*1.001


def test_measured_step_rate_never_exceeds_the_cap():
    ramp = StraightenRamp(BENT, INTERVAL, RATE)
    plan = [BENT] + ramp.plan()
    worst = max(max(abs(b[THETA_A] - a[THETA_A]), abs(b[THETA_C] - a[THETA_C]))/INTERVAL
                for a, b in zip(plan, plan[1:]))
    assert worst <= RATE*1.001


def test_a_larger_bend_takes_proportionally_longer():
    small = StraightenRamp([.05, .2, 0., 0., 0.], INTERVAL, RATE)
    large = StraightenRamp([.05, .8, 0., 0., 0.], INTERVAL, RATE)
    assert large.duration_s > small.duration_s*3.5


def test_slower_cap_gives_a_longer_ramp():
    fast = StraightenRamp(BENT, INTERVAL, 1.)
    slow = StraightenRamp(BENT, INTERVAL, .25)
    assert slow.duration_s > fast.duration_s


def test_duration_is_driven_by_the_larger_bend():
    a_dominant = StraightenRamp([.05, .9, 0., .1, 0.], INTERVAL, RATE)
    c_dominant = StraightenRamp([.05, .1, 0., .9, 0.], INTERVAL, RATE)
    assert a_dominant.duration_s == pytest.approx(c_dominant.duration_s)


# --- already straight ---

def test_already_straight_pose_is_detected():
    ramp = StraightenRamp([.1, 0., .5, 0., -.5], INTERVAL, RATE)
    assert ramp.already_straight
    assert ramp.peak_theta_rate_rad_s() == 0.


def test_already_straight_ramp_commands_no_change():
    start = np.array([.1, 0., .5, 0., -.5])
    ramp = StraightenRamp(start, INTERVAL, RATE)
    np.testing.assert_allclose(ramp.step(), start)


# --- validation ---

@pytest.mark.parametrize('bad', [
    np.zeros(4), np.zeros(6), np.array([0., np.nan, 0., 0., 0.]),
    np.array([0., np.inf, 0., 0., 0.]),
])
def test_invalid_start_is_rejected(bad):
    with pytest.raises(ValueError):
        StraightenRamp(bad, INTERVAL, RATE)


@pytest.mark.parametrize('interval', [0., -.02, math.nan, math.inf])
def test_invalid_interval_is_rejected(interval):
    with pytest.raises(ValueError):
        StraightenRamp(BENT, interval, RATE)


@pytest.mark.parametrize('rate', [0., -1., math.nan, math.inf])
def test_invalid_rate_is_rejected(rate):
    with pytest.raises(ValueError):
        StraightenRamp(BENT, INTERVAL, rate)


def test_smoothstep_is_clamped_and_monotone():
    assert smoothstep(-1.) == 0.
    assert smoothstep(2.) == 1.
    values = [smoothstep(i/20.) for i in range(21)]
    assert all(b >= a for a, b in zip(values, values[1:]))


# --- protocol ---

def test_straighten_message_round_trips():
    kind, action = parse_control_message(build_straighten_message(4))
    assert kind == 'straighten'
    assert action is None


def test_straighten_message_carries_no_action():
    assert 'action' not in build_straighten_message(1)


# --- workspace sanity ---

def test_ramp_stays_inside_the_model_joint_limits():
    """Unbending can only reduce theta, so limits must hold throughout."""
    cap_a, cap_c = GEOMETRY.curvature_caps(BENT[D])
    low, high = GEOMETRY.insertion_bounds()
    for joint in StraightenRamp(BENT, INTERVAL, RATE).plan():
        assert 0. <= joint[THETA_A] <= cap_a
        assert 0. <= joint[THETA_C] <= cap_c
        assert low <= joint[D] <= high


def test_extreme_bend_still_produces_a_finite_plan():
    extreme = np.array([.2, GEOMETRY.theta_a_max, 2.5, GEOMETRY.theta_c_max, -2.5])
    plan = StraightenRamp(extreme, INTERVAL, RATE).plan()
    assert len(plan) > 1
    assert all(np.all(np.isfinite(joint)) for joint in plan)
    assert all(np.all(np.isfinite(TENDON.to_axis_targets(joint))) for joint in plan)
