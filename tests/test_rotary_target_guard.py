import numpy as np
import pytest
from continuum_sdk.control.axis_mapper import ContinuumAxisMapper
from continuum_sdk.control.rotary_target_guard import RotaryTargetGuard, RotaryTargetJump

@pytest.fixture
def guard():
    return RotaryTargetGuard(ContinuumAxisMapper(2e6,6e7,[1,0,2,3,4],[1,-1,-1,-1,1]),2.5)

def test_normal_segment_unchanged_and_physical_mapping(guard):
    base=[100,200,300,400,500]
    target=guard.mapper.logical_to_pulses(base,[.005,-.005,.005,-.005,.01])
    before=target.copy()
    assert max(guard.check(base,target,[0]*5,[0]*5,.02))<2.5
    assert target==before
    target=guard.mapper.logical_to_pulses(base,[.1,0,0,0,0])
    with pytest.raises(RotaryTargetJump) as error:
        guard.check(base,target,[0]*5,[0]*5,.02)
    assert error.value.evidence['physical_axis']==2
    assert error.value.evidence['logical_axis']==1

def test_velocity_overshoot_rejected_even_without_position_jump(guard):
    # 3 rad/s in logical axis 1, physical axis 2. Equal endpoints do not ensure hold.
    velocity=guard.mapper.logical_velocity_to_pulses_per_ms([3,0,0,0,0])
    with pytest.raises(RotaryTargetJump):
        guard.check([0]*5,[0]*5,velocity,[0]*5,.02)

@pytest.mark.parametrize('bad',[None,0,-1,float('nan'),float('inf')])
def test_invalid_threshold(guard,bad):
    with pytest.raises(ValueError):RotaryTargetGuard(guard.mapper,bad)

def test_cubic_extremum_against_dense_samples(guard):
    from continuum_sdk.control.insertion_profile import cubic_extrema
    rng=np.random.default_rng(8)
    for _ in range(100):
        delta,v0,v1=rng.uniform(-.05,.05),rng.uniform(-2,2),rng.uniform(-2,2)
        h=.02;s=np.linspace(0,1,10001)
        a=-2*delta+h*(v0+v1);b=3*delta-h*(2*v0+v1);c=h*v0
        sampled=np.max(np.abs((3*a*s*s+2*b*s+c)/h))
        peak,_=cubic_extrema(0,delta,v0,v1,h)
        assert sampled<=peak+1e-12
        assert peak-sampled<1e-6
