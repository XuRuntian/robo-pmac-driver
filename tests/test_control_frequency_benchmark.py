"""Offline benchmark regression checks; no driver or hardware imports."""
import importlib.util
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("benchmark", ROOT / "apps/benchmark_control_frequency.py")
benchmark = importlib.util.module_from_spec(spec)
spec.loader.exec_module(benchmark)


def test_same_physical_trajectory_at_common_times():
    ik = benchmark.build_continuum_ik(benchmark.load_continuum_config(ROOT / "config/continuum.yaml"))
    for scenario in benchmark.SCENARIOS:
        p50, z50 = benchmark.reachable_goal(ik, 75 / 50, scenario)
        p200, z200 = benchmark.reachable_goal(ik, 300 / 200, scenario)
        np.testing.assert_array_equal(p50, p200)
        np.testing.assert_array_equal(z50, z200)
        assert benchmark.world_action(75 / 50, scenario) == benchmark.world_action(300 / 200, scenario)
        assert benchmark.world_action(1.5, scenario)["tip_delta_ry"] == 0


def test_error_is_physical_angle_not_weighted_solver_residual():
    class FakeIK:
        def fk_tip(self, u):
            return np.zeros(3), np.eye(3)
    position, angle = benchmark.residuals(FakeIK(), None, np.array([0.001, 0, 0]), np.array([1, 0, 0]))
    assert position == pytest.approx(1)
    assert angle == pytest.approx(90)


@pytest.mark.parametrize("mode", ["ik", "pipeline"])
def test_run_without_network_and_timing_accounting(monkeypatch, mode):
    import socket
    def forbidden(*args, **kwargs):
        raise AssertionError("Offline benchmark attempted network access")
    monkeypatch.setattr(socket, "socket", forbidden)
    cfg = benchmark.load_continuum_config(ROOT / "config/continuum.yaml")
    interface = benchmark.load_robot_interface_config(ROOT / "config/robot_interface.yaml")
    rows = benchmark.run_case(cfg, interface, 100, 0.03, "normal", mode)
    assert len(rows) == 3
    for row in rows:
        assert np.isfinite(row["position_mm"])
        assert row["work_ms"] == pytest.approx(sum(row[k] for k in (
            "prepare_ms", "ik_ms", "mapping_ms", "commit_ms")), abs=1e-8)
        assert row["completion_late_ms"] == 0
    result = benchmark.summarize(rows, 100)
    assert result["budget_ms"] == 10
    assert result["work_max_ms"] >= result["work_p99_ms"]


def test_deadline_misses_and_reserve_are_not_mean_throughput():
    row = dict(work_ms=1, converged=True, position_mm=0, direction_deg=0,
               command_position_mm=0, completion_late_ms=0,
               prepare_ms=0, ik_ms=1, mapping_ms=0, commit_ms=0)
    rows = [row.copy() for _ in range(4)]
    rows[1]["work_ms"] = 11
    rows[2]["work_ms"] = 12
    result = benchmark.summarize(rows, 100)
    assert result["compute_miss_pct"] == 50
    assert result["longest_compute_miss_run"] == 2
    assert not result["compute_pass"]
    assert not result["reserve_30pct_pass"]


def test_repeats_do_not_merge_consecutive_miss_runs():
    row = dict(work_ms=11, converged=False, position_mm=1, direction_deg=1,
               command_position_mm=1, completion_late_ms=2, paced=True,
               prepare_ms=0, ik_ms=11, mapping_ms=0, commit_ms=0)
    rows = [{**row, "repeat": repeat} for repeat in (0, 0, 1, 1)]
    result = benchmark.summarize(rows, 100)
    assert result["longest_compute_miss_run"] == 2
    assert result["schedule_miss_pct"] == 100
