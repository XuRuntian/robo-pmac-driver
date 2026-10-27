from pathlib import Path
import numpy as np
import pytest

from continuum_sdk.control.visual_state import visual_snapshot
from continuum_sdk.kinematics.joint_motor_model import TDRCJointMotorModel


def test_snapshot_distinguishes_target_and_feedback_without_hardware():
    result = visual_snapshot(12,[0,0,0,0,.01],[0,0,0,0,.02],TDRCJointMotorModel(.00215,.012),
                             execute=True,feedback_age_s=.1)
    assert result['source']=='encoder_model'
    assert result['feedback_joint']==[.01,0,0,0,0]
    assert result['target_joint']==[.02,0,0,0,0]
    assert result['feedback_age_s']==.1


def test_existing_mjcf_kinematics_without_physics(monkeypatch):
    mj=pytest.importorskip('mujoco')
    from continuum_sdk.control.mujoco_visual import KinematicMirror
    xml=Path(__file__).resolve().parents[2]/'surgical_continuum_robot/model/continuum_robot/mjcf/scene_main.xml'
    if not xml.exists():
        pytest.skip('Sibling model is not installed')
    def forbidden(*args,**kwargs):
        raise AssertionError('Dynamics must not run in the visual mirror')
    monkeypatch.setattr(mj,'mj_step',forbidden)
    monkeypatch.setattr(mj,'mj_forward',forbidden)
    mirror=KinematicMirror(xml)
    for values in [[.01,.5,.3,.2,-.4],[.02,.1,-.7,.4,.9],[0,0,0,0,0]]:
        mirror.update(values)
        # Compute fixed tendon lengths independently from the rendered qpos.
        mj.mj_tendon(mirror.model,mirror.data)
        d,ta,pa,tc,pc=values
        for name,expected in dict(ax=-ta*np.cos(pa),ay=-ta*np.sin(pa),
                                  cy=-tc*np.cos(pc),cx=-tc*np.sin(pc)).items():
            tid=mj.mj_name2id(mirror.model,mj.mjtObj.mjOBJ_TENDON,name)
            assert mirror.data.ten_length[tid]==pytest.approx(expected)
        assert mirror.data.qpos[mirror.linear_address]==d
        assert np.all(np.isfinite(mirror.data.geom_xpos))
        assert mirror.data.time==0
    before=mirror.data.qpos.copy()
    with pytest.raises(ValueError):
        mirror.update([0,0,float('nan'),0,0])
    np.testing.assert_array_equal(before,mirror.data.qpos)


@pytest.mark.parametrize('initial_pose', ['source', 'zero'])
def test_display_gain_preserves_start_and_scales_subsequent_motion(initial_pose):
    pytest.importorskip('mujoco')
    from continuum_sdk.control.mujoco_visual import KinematicMirror
    xml=Path(__file__).resolve().parents[2]/'surgical_continuum_robot/model/continuum_robot/mjcf/scene_main.xml'
    if not xml.exists():
        pytest.skip('Sibling model is not installed')
    plain=KinematicMirror(xml)
    amplified=KinematicMirror(xml,initial_pose=initial_pose)
    neutral=amplified.data.qpos.copy()
    start=[.016,.1,np.pi-.01,.2,-.3]
    plain.update(start)
    q0=plain.data.qpos.copy()
    amplified.update(start,display_gain=5)
    origin=neutral if initial_pose=='zero' else q0
    np.testing.assert_allclose(amplified.data.qpos,origin,atol=1e-12)
    for pose in [[.017,.11,-np.pi+.01,.22,-.32],start]:
        plain.update(pose)
        amplified.update(pose,display_gain=5)
        np.testing.assert_allclose(amplified.data.qpos,origin+5*(plain.data.qpos-q0),atol=1e-12)
