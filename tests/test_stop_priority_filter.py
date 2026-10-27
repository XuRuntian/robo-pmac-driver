from dataclasses import replace

import numpy as np
import pytest

from continuum_sdk.core.interface_config import load_robot_interface_config
from continuum_sdk.control.stop_priority_filter import StopPriorityCommandFilter
from continuum_sdk.control.cartesian_frame import apply_axis_transform
from continuum_sdk.kinematics.dls_ik import rotvec_to_matrix


def action(x=0., rz=0., ry=0.):
    return dict(tip_delta_x=x, tip_delta_y=0., tip_delta_z=0.,
                tip_delta_rx=0., tip_delta_ry=ry, tip_delta_rz=rz)


def make():
    cfg=load_robot_interface_config('config/robot_interface_stop_priority_trial.yaml')
    return cfg, StopPriorityCommandFilter(cfg.command,.02)


def test_initial_and_repeated_absolute_offset_do_not_invent_motion():
    _,f=make()
    for _ in range(100):
        f.set_command(action(.03,1.)); f.step()
        assert f.input_holding
        assert np.all(f.applied_delta==0)
        assert np.all(f.applied_rotation==0)


@pytest.mark.parametrize('sign',[-1,1])
def test_large_input_does_not_accumulate_seconds_of_catchup(sign):
    _,f=make(); f.set_command(action()); f.step()
    f.set_command(action(sign*.03,sign*2)); f.step()
    assert f.input_discarded
    assert f.applied_delta[0]==pytest.approx(sign*.0002)
    assert f.applied_rotation[2]==pytest.approx(sign*.01)
    for _ in range(6):
        f.set_command(action(sign*.03,sign*2)); f.step()
    assert f.input_holding
    end=np.r_[f.applied_delta,f.applied_rotation]
    assert abs(end[0])<=.0004
    assert abs(end[5])<=.02
    for _ in range(100):
        f.set_command(action(sign*.03,sign*2)); f.step()
        np.testing.assert_array_equal(np.r_[f.applied_delta,f.applied_rotation],end)


def test_small_continuous_motion_is_preserved_and_slow_motion_accumulates():
    _,f=make(); f.set_command(action()); f.step()
    for n in range(1,101):
        f.set_command(action(n*.0001,n*.004)); f.step()
    assert not f.input_holding
    assert not f.input_discarded
    assert f.applied_delta[0]==pytest.approx(.0099,abs=1e-7)
    assert f.applied_rotation[2]==pytest.approx(.396,abs=1e-7)
    _,f=make(); f.set_command(action()); f.step()
    for n in range(1,201):
        f.set_command(action(0,n*.0001)); f.step()
    assert f.applied_rotation[2]>.01


def test_jitter_below_motion_anchor_threshold_does_not_reactivate_hold():
    _,f=make(); f.set_command(action()); f.step()
    for n in range(100):
        f.set_command(action(0,(-1)**n*.0003)); f.step()
        assert f.input_holding


def test_hold_rearm_ignores_old_absolute_target_and_allows_reversal():
    _,f=make(); f.set_command(action());f.step()
    f.set_command(action(.03,1));f.step();f.hold()
    old=f.applied_rotation.copy()
    f.set_command(action(-.03,-1));f.step()
    np.testing.assert_array_equal(old,f.applied_rotation)
    f.set_command(action(-.029,-1.01));f.step()
    assert f.applied_rotation[2]<old[2]


def test_reanchor_reproduces_committed_pose_in_original_world_frame():
    cfg,f=make()
    center_p=np.array([.1,.2,.3]);center_r=rotvec_to_matrix(np.array([.1,.2,.05]))
    rotation=center_r@rotvec_to_matrix(np.array([.4,-.2,.0]))
    position=center_p+np.array([.003,-.002,.01])
    f.set_command(action(.03,1));f.step()
    f.reanchor_from_tip(position,rotation,center_p,center_r,cfg.frame)
    dp=apply_axis_transform(f.applied_delta,cfg.frame.translation_map,cfg.frame.translation_signs)
    rv=apply_axis_transform(f.applied_rotation,cfg.frame.rotation_map,cfg.frame.rotation_signs)
    np.testing.assert_allclose(center_p+dp,position,atol=1e-12)
    np.testing.assert_allclose((center_r@rotvec_to_matrix(rv))[:,2],rotation[:,2],atol=1e-12)
    assert f.applied_rotation[1]==0
    previous=np.r_[f.applied_delta,f.applied_rotation]
    f.set_command(action(.03,1));f.step()
    np.testing.assert_array_equal(np.r_[f.applied_delta,f.applied_rotation],previous)
    f.set_command(action(.0301,1.002));f.step()
    assert np.linalg.norm(f.applied_delta-previous[:3])<.0002
    assert np.linalg.norm(f.applied_rotation-previous[3:])<.002


def test_reanchor_outside_command_envelope_cannot_snap_to_boundary():
    cfg,f=make(); f.set_command(action());f.step()
    f.reanchor_from_tip(np.array([0.,-.1,0.]),np.eye(3),np.zeros(3),np.eye(3),cfg.frame)
    assert f.applied_delta[0]==pytest.approx(.1)
    f.set_command(action(-.001));f.step()
    assert .0998-1e-12<=f.applied_delta[0]<=.1


@pytest.mark.parametrize('field,value', [('input_idle_timeout_s',0),('input_rotation_epsilon_rad',float('nan')),
    ('input_translation_epsilon_m',-1),('stop_priority','yes')])
def test_config_rejects_invalid_stop_policy(tmp_path,field,value):
    import yaml
    from pathlib import Path
    raw=yaml.safe_load(Path('config/robot_interface_stop_priority_trial.yaml').read_text(encoding='utf-8'))
    raw['command'][field]=value
    p=tmp_path/'bad.yaml';p.write_text(yaml.safe_dump(raw))
    with pytest.raises(ValueError):load_robot_interface_config(p)
