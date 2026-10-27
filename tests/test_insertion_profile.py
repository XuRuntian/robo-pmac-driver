import math
import random
import pytest
from continuum_sdk.control.insertion_profile import InsertionProfile, cubic_extrema

PPM = 62781744

def make(start=634591):
    return InsertionProfile(start, .02, .004*PPM, .00195*PPM, 5023)

@pytest.mark.parametrize('sign', [-1, 1])
def test_step_and_return_converge_with_bounded_quantized_cubics(sign):
    profile = make()
    for target in [634591+sign*round(.015*PPM), 634591]:
        previous_p, previous_v = profile.wire_p, profile.wire_v
        for _ in range(1000):
            segment = profile.step(target)
            v, a = cubic_extrema(previous_p, segment.position, previous_v,
                                 segment.velocity_counts_ms*1000, .02)
            assert v <= .004*PPM+1e-6 and a <= .00195*PPM+1e-6
            assert abs(segment.position-previous_p) <= 5023
            previous_p, previous_v = segment.position, segment.velocity_counts_ms*1000
        assert abs(segment.position-target) <= 1
        assert abs(segment.velocity_counts_ms) < .001

def test_hold_brakes_then_stays_without_return_or_restart():
    profile = make()
    for _ in range(180):
        profile.step(634591+round(.05*PPM))
    start = profile.wire_p
    positions = []
    for _ in range(200):
        segment = profile.step(start, holding=True)
        positions.append(segment.position)
    assert positions[-1] > start
    assert len(set(positions[-50:])) == 1
    assert segment.velocity_counts_ms == 0
    assert (positions[-1]-start)/PPM < .005

def test_reversals_jitter_and_large_absolute_reference():
    rng = random.Random(42)
    profile = make(1500000000)
    for i in range(5000):
        target = 1500000000 + round(.03*PPM*math.sin(i/31)) + rng.randint(-100,100)
        profile.step(target, holding=(i % 377 < 12))

def test_rejected_target_does_not_advance_profile():
    profile = make()
    state = vars(profile).copy()
    with pytest.raises(ValueError):
        profile.step(float('nan'))
    assert vars(profile) == state


def assert_sampled_cubic(profile, previous_p, previous_v, segment):
    # Independently sample the polynomial rather than calling cubic_extrema.
    h = profile.dt
    p1, v1 = segment.position, segment.velocity_counts_ms * 1000
    delta = p1 - previous_p
    a = -2 * delta + h * (previous_v + v1)
    b = 3 * delta - h * (2 * previous_v + v1)
    c = h * previous_v
    for i in range(101):
        s = i / 100
        assert abs((3*a*s*s + 2*b*s + c)/h) <= profile.max_speed + 1e-6
        assert abs((6*a*s + 2*b)/h**2) <= profile.max_accel + 1e-6
    assert abs(delta) <= profile.max_step


@pytest.mark.parametrize('accel', [.00195, .005])
@pytest.mark.parametrize('sign', [-1, 1])
def test_rest_to_rest_small_and_large_moves_do_not_cross_fixed_target(accel, sign):
    for counts in [1, 20, 1000, round(.015*PPM)]:
        start = 1500000000
        profile = InsertionProfile(start, .02, .004*PPM, accel*PPM, 5023)
        target = start + sign*counts
        for _ in range(1500):
            p0, v0 = profile.wire_p, profile.wire_v
            seg = profile.step(target)
            assert_sampled_cubic(profile, p0, v0, seg)
            assert 0 <= sign*(seg.position-start) <= counts
            assert sign*(seg.position-p0) >= 0
            if seg.position == target and seg.velocity_counts_ms == 0:
                break
        else:
            pytest.fail('Fixed target did not settle')
        for _ in range(10):
            assert profile.step(target).position == target


@pytest.mark.parametrize('sign', [-1, 1])
def test_full_speed_reverse_and_hold_preserve_cubic_limits(sign):
    profile = InsertionProfile(634591, .02, .004*PPM, .005*PPM, 5023)
    for _ in range(100):
        profile.step(634591 + sign*round(.05*PPM))
    # An abrupt reversal cannot erase existing velocity. Braking initially
    # continues in the old direction; every cubic must still meet the budget.
    p0, v0 = profile.wire_p, profile.wire_v
    seg = profile.step(634591 - sign*round(.05*PPM))
    assert sign*(seg.position-p0) > 0
    assert_sampled_cubic(profile, p0, v0, seg)
    for _ in range(150):
        p0, v0 = profile.wire_p, profile.wire_v
        seg = profile.step(634591-sign*round(.05*PPM))
        assert_sampled_cubic(profile, p0, v0, seg)
    stop_start = profile.wire_p
    for _ in range(100):
        p0, v0 = profile.wire_p, profile.wire_v
        seg = profile.step(stop_start, holding=True)
        assert_sampled_cubic(profile, p0, v0, seg)
        assert -sign*(seg.position-p0) >= 0
    assert seg.velocity_counts_ms == 0
    assert abs(seg.position-stop_start)/PPM < .0018
