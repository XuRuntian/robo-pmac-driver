import numpy as np
import pytest

from continuum_sdk.kinematics.dls_ik import DLSIK, u_to_qu
from continuum_sdk.kinematics.geometry import ContinuumGeometry


@pytest.mark.parametrize('task_mode', ['position', 'pos_z'])
def test_insertion_can_retreat_from_upper_limit(task_mode):
    ik = DLSIK(task_mode=task_mode)
    ik.reset(np.array([.255, 0., 0., 0., 0.]))
    goal, rotation = ik.fk_tip(np.array([.254, 0., 0., 0., 0.]))
    result = ik.solve(goal, z_goal=rotation[:, 2] if task_mode == 'pos_z' else None, max_steps=8)
    assert result.converged
    assert result.u[0] < .255
    assert np.linalg.norm(ik.fk_tip()[0] - goal) < ik.pos_tol


def test_fixed_insertion_still_has_zero_jacobian_column():
    ik = DLSIK(geometry=ContinuumGeometry(d_min=.1, d_max=.1))
    ik.reset(np.array([.1, .2, .3, .4, .5]))
    p, _ = ik.fk_tip()
    jacobian, _ = ik._jac_fd_qu(u_to_qu(ik.u), p, None, None)
    assert np.array_equal(jacobian[:, 0], np.zeros(3))

@pytest.mark.parametrize('phi', [0., np.pi/2, np.pi/2+.0006, np.pi, -np.pi/2])
@pytest.mark.parametrize('sheath', [False, True])
def test_bending_boundary_jacobian_matches_small_central_difference(phi, sheath):
    from continuum_sdk.kinematics.dls_ik import qu_to_u
    ik = DLSIK(ContinuumGeometry(use_sheath=sheath), task_mode='pos_z')
    caps = ik.geometry.curvature_caps(.1)
    ik.reset(np.array([.1, min(.7,caps[0]), .4, caps[1], phi]))
    q = u_to_qu(ik.u)
    p,r = ik.fk_tip()
    jac,err = ik._jac_fd_qu(q,p,None,r[:,2])
    for k in range(1,5):
        plus=q.copy();minus=q.copy();plus[k]+=1e-6;minus[k]-=1e-6
        ref=(ik._task_qu(plus,p,None,r[:,2],geometry_only=True)-
             ik._task_qu(minus,p,None,r[:,2],geometry_only=True))/(2e-6)
        np.testing.assert_allclose(jac[:,k],ref,atol=.003,rtol=.02)
    # Public FK and accepted iterates remain constrained.
    outside=ik.u.copy();outside[3]+=1.
    bounded=outside.copy();bounded[3]=caps[1]
    np.testing.assert_allclose(ik.fk_tip(outside)[0],ik.fk_tip(bounded)[0])
    result=ik.solve(p+np.array([.02,0,.01]),z_goal=r[:,0],max_steps=8)
    ca,cc=ik.geometry.curvature_caps(result.u[0])
    assert result.u[1]<=ca+1e-10 and result.u[3]<=cc+1e-10

def test_interior_jacobian_preserves_existing_forward_difference():
    from continuum_sdk.kinematics.dls_ik import clamp_qu
    ik = DLSIK(ContinuumGeometry(use_sheath=False),task_mode='pos_z')
    ik.reset(np.array([.1,.7,.4,.6,-.5]))
    q=u_to_qu(ik.u);p,r=ik.fk_tip();j,e=ik._jac_fd_qu(q,p,None,r[:,2])
    caps=ik.geometry.curvature_caps(q[0])
    for k,h in enumerate(ik.eps_j):
        probe=q.copy();probe[k]+=h
        probe=clamp_qu(probe,(ik.geometry.d_min,ik.geometry.d_max),*caps)
        expected=(ik._task_qu(probe,p,None,r[:,2])-e)/(probe[k]-q[k])
        np.testing.assert_allclose(j[:,k],expected,atol=1e-11,rtol=1e-10)
