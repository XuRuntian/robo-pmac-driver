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
