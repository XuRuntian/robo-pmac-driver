import numpy as np

from continuum_sdk.control.ik_limit_monitor import IKLimitMonitor
from continuum_sdk.kinematics.dls_ik import DLSIK, IKResult


def test_idle_at_lower_bound_does_not_report_blockage():
    ik = DLSIK(task_mode="position")
    goal, _ = ik.fk_tip()
    result = ik.solve(goal)
    assert result.converged
    assert IKLimitMonitor().update(result, 0) is None


def test_outward_attempt_at_upper_bound_is_recorded():
    ik = DLSIK(task_mode="position")
    ik.reset(np.array([.255, 0, 0, 0, 0]))
    goal, _ = ik.fk_tip()
    goal[2] += .01
    result = ik.solve(goal)
    assert not result.converged
    assert "d.upper" in result.limit_attempts
    attempted, bound = result.limit_attempts["d.upper"]
    assert attempted > bound == .255
    assert "d.upper" in IKLimitMonitor().update(result, 0)


def test_inward_retreat_does_not_report_blockage():
    ik = DLSIK(task_mode="position")
    ik.reset(np.array([.255, 0, 0, 0, 0]))
    goal, _ = ik.fk_tip(np.array([.254, 0, 0, 0, 0]))
    result = ik.solve(goal, max_steps=8)
    assert result.converged
    assert IKLimitMonitor().update(result, 0) is None


def test_repeat_clear_and_hold_dont_reuse_stale_attempts():
    result = IKResult(np.zeros(5), np.ones(3), np.zeros(5), False,
                      {"theta_a.upper": (3.2, 3.14)})
    monitor = IKLimitMonitor()
    assert "theta_a.upper" in monitor.update(result, 0)
    assert monitor.update(result, .02) is None
    assert monitor.update(result, 2) is not None
    assert "cleared" in monitor.update(None, 2.1)
    result.converged = True
    assert monitor.update(result, 3) is None


def test_diagnostic_records_theta_attempts_without_mutating_candidate():
    ik = DLSIK()
    candidate = np.array([-.01, 4., 0., 2., 0.])
    original = candidate.copy()
    ik._record_limit_attempt(candidate, (0., .255), 3.14, 1.57)
    assert set(ik._limit_attempts) == {"d.lower", "theta_a.upper", "theta_c.upper"}
    assert np.array_equal(candidate, original)


def test_observation_does_not_change_solver_result():
    observed = DLSIK(task_mode="pos_z")
    plain = DLSIK(task_mode="pos_z")
    plain._record_limit_attempt = lambda *args: None
    goal, rotation = observed.fk_tip(np.array([.02, .5, .3, .2, -.1]))
    a = observed.solve(goal, z_goal=rotation[:, 2])
    b = plain.solve(goal, z_goal=rotation[:, 2])
    assert np.array_equal(a.u, b.u)
    assert np.array_equal(a.error, b.error)
