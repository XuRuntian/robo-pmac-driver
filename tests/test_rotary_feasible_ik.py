"""Real rejected segments; fixtures contain no feedback or hardware access."""
import json
from pathlib import Path
import numpy as np
import pytest

from continuum_sdk.core.config_loader import load_continuum_config
from continuum_sdk.core.interface_config import load_robot_interface_config
from continuum_sdk.core.factory import build_continuum_ik, build_tendon_mapper
from continuum_sdk.control.axis_mapper import ContinuumAxisMapper
from continuum_sdk.control.pvt_mapper import ContinuumPVTMapper
from continuum_sdk.control.rotary_target_guard import RotaryTargetGuard, RotaryTargetJump
from continuum_sdk.control.cartesian_frame import apply_axis_transform
from continuum_sdk.kinematics.dls_ik import rotvec_to_matrix

CASES=json.loads((Path(__file__).parent/'fixtures/rotary_stop_cases.json').read_text(encoding='utf-8'))


def setup(case,tmp_path):
    (tmp_path/'c.yaml').write_text(case['continuum_yaml'],encoding='utf-8')
    (tmp_path/'i.yaml').write_text(case['interface_yaml'],encoding='utf-8')
    cfg=load_continuum_config(tmp_path/'c.yaml');iface=load_robot_interface_config(tmp_path/'i.yaml')
    axis=ContinuumAxisMapper(131072*97.34/(2*np.pi),62781744,case['axis_order'],case['axis_signs'])
    ik=build_continuum_ik(cfg);tendon=build_tendon_mapper(cfg)
    mapper=ContinuumPVTMapper(ik,tendon,axis,case['base_pulses'],.02,8)
    mapper.commit_pulses(case['initial_pulses']);cp,cr=ik.fk_tip()
    E=case['event'];a=E['applied_command']
    pg=cp+apply_axis_transform(np.array([a['tip_delta_'+k] for k in 'xyz']),iface.frame.translation_map,iface.frame.translation_signs)
    zg=(cr@rotvec_to_matrix(apply_axis_transform(np.array([a['tip_delta_'+k] for k in ['rx','ry','rz']]),iface.frame.rotation_map,iface.frame.rotation_signs)))[:,2]
    mapper.commit_pulses(E['previous_pulses'])
    return mapper,RotaryTargetGuard(axis,2.5),pg,zg


@pytest.mark.parametrize('case',CASES,ids=lambda c:c['source'])
def test_old_mode_exactly_reproduces_recorded_rejection(case,tmp_path):
    mp,g,pg,zg=setup(case,tmp_path);E=case['event']
    cmd=mp.build_command(pg,z_goal=zg)
    assert cmd.target_pulses[:4]==E['rejected_pulses'][:4]
    np.testing.assert_array_equal(cmd.ik_result.u,E['ik_u'])
    with pytest.raises(RotaryTargetJump) as caught:
        g.check(E['previous_pulses'],cmd.target_pulses,E['previous_velocity_counts_ms'],cmd.velocities,.02)
    assert caught.value.evidence['peak_rad_s']==E['peak_rad_s']


@pytest.mark.parametrize('case',CASES,ids=lambda c:c['source'])
def test_guarded_solve_makes_task_progress_without_rejecting_final_segment(case,tmp_path):
    mp,g,pg,zg=setup(case,tmp_path);E=case['event']
    initial=float(np.linalg.norm(mp.ik._task_u(mp.ik.u,pg,None,zg)))
    cmd=mp.build_command(pg,z_goal=zg,rotary_guard=g,previous_pulses=E['previous_pulses'],previous_velocities=E['previous_velocity_counts_ms'])
    peaks=g.check(E['previous_pulses'],cmd.target_pulses,E['previous_velocity_counts_ms'],cmd.velocities,.02)
    assert max(peaks)<=2.5
    assert np.linalg.norm(cmd.ik_result.error)<initial
    assert cmd.feasibility_rejections>0
    assert cmd.target_pulses[:4]!=E['previous_pulses'][:4]
    assert cmd.feasibility_checks<=8*9


def test_execution_constraint_applies_to_total_cycle_not_each_inner_step(tmp_path):
    case=CASES[-1];mp,g,pg,zg=setup(case,tmp_path);E=case['event']
    anchors=[];original=g.check
    def inspect(p0,p1,v0,v1,dt):
        anchors.append((p0.copy(),v0.copy()))
        return original(p0,p1,v0,v1,dt)
    g.check=inspect
    mp.build_command(pg,z_goal=zg,rotary_guard=g,previous_pulses=E['previous_pulses'],previous_velocities=E['previous_velocity_counts_ms'])
    assert len(anchors)>1
    assert all(p==E['previous_pulses'] and v==E['previous_velocity_counts_ms'] for p,v in anchors)


def test_all_trials_rejected_leave_solver_at_start(tmp_path):
    mp,_,pg,zg=setup(CASES[-1],tmp_path);before=mp.ik.u.copy()
    r=mp.ik.solve(pg,z_goal=zg,max_steps=8,candidate_validator=lambda _:False)
    np.testing.assert_allclose(r.u,before,atol=1e-12)


def test_invalid_guard_context_fails_before_solver_changes(tmp_path):
    mp,g,pg,zg=setup(CASES[-1],tmp_path);before=mp.ik.u.copy()
    with pytest.raises(ValueError,match='previous committed'):
        mp.build_command(pg,z_goal=zg,rotary_guard=g)
    np.testing.assert_array_equal(mp.ik.u,before)


@pytest.mark.parametrize('case',CASES,ids=lambda c:c['source'])
def test_permissive_validator_preserves_original_solution(case,tmp_path):
    mp,_,pg,zg=setup(case,tmp_path)
    expected=mp.ik.solve(pg,z_goal=zg,max_steps=8)
    mp,_,pg,zg=setup(case,tmp_path)
    actual=mp.ik.solve(pg,z_goal=zg,max_steps=8,candidate_validator=lambda _:True)
    np.testing.assert_array_equal(actual.u,expected.u)


def test_guarded_ik_config_is_opt_in_and_strict_boolean(tmp_path):
    assert not load_robot_interface_config('config/robot_interface.yaml').rotary_feasible_ik
    assert load_robot_interface_config('config/robot_interface_stop_priority_trial.yaml').rotary_feasible_ik
    path=tmp_path/'invalid.yaml'
    source=Path('config/robot_interface_stop_priority_trial.yaml').read_text(encoding='utf-8')
    path.write_text(source.replace('rotary_feasible_ik: true','rotary_feasible_ik: "false"'),encoding='utf-8')
    with pytest.raises(ValueError,match='rotary_feasible_ik must be a boolean'):
        load_robot_interface_config(path)
