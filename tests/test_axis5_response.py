import csv
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

spec = importlib.util.spec_from_file_location(
    "axis5_response", Path(__file__).parents[1] / "apps/test_axis5_response.py"
)
app = importlib.util.module_from_spec(spec)
spec.loader.exec_module(app)


def test_roundtrip_preserves_other_axes_and_sign():
    cfg = app.PMACConfig()
    cfg.axis_signs[4] = -1
    base = [123456, 222333, -432100, 445566, 700000]
    frames, summary = app.plan(cfg, base)
    assert all(r["positions"][:4] == base[:4] for r in frames)
    assert frames[-1]["positions"] == base
    assert min(r["positions"][4] for r in frames) == int(
        base[4] - 0.001 * cfg.pulses_per_meter
    )
    assert max(r["positions"][4] for r in frames) == base[4]
    # Ramp duration rounds UP to full 20 ms segments; include pulse quantization.
    assert summary["interpolant_peak_mm_s"] == pytest.approx(
        1.875 / summary["ramp_s"], abs=0.002
    )
    assert summary["max_step_counts"] < 3000
    for r in frames:
        if r["phase"] in ("prime", "hold", "settle"):
            assert r["velocities"] == [0.0] * 5


def test_segment_peak_is_not_just_endpoint_speed():
    # Zero endpoint velocities still have a 1.5 count/ms interior peak.
    v, a = app.segment_extrema(0, 20, 0, 0, 20)
    assert v == pytest.approx(1.5)
    assert a == pytest.approx(0.3)


@pytest.mark.parametrize('distance', [-5, 5])
@pytest.mark.parametrize('axis_sign', [-1, 1])
@pytest.mark.parametrize('accel', [5, 6, 7.5])
def test_candidate_events_are_bounded_reverse_brake_and_return(distance, axis_sign, accel):
    cfg = app.PMACConfig()
    cfg.axis_signs[4] = axis_sign
    base = [123456, 222333, -432100, 445566, 700000]
    frames, summary = app.plan(cfg, base, distance, 4, step_cap_counts=6500,
                               accel_mm_s2=accel, profile_events=True)
    assert frames[-1]['positions'] == base
    assert all(r['positions'][:4] == base[:4] for r in frames)
    assert summary['interpolant_peak_mm_s'] <= 4
    assert summary['interpolant_peak_accel_mm_s2'] <= accel
    assert summary['total_s'] == 20
    reverse = [r for r in frames if r['phase'] == 'reverse']
    sign = axis_sign * (1 if distance > 0 else -1)
    assert reverse[0]['velocities'][4]*sign > 0
    assert any(r['velocities'][4]*sign < 0 for r in reverse)
    hold = [r for r in frames if r['phase'] == 'brake_hold']
    assert hold[0]['velocities'][4]*sign > 0
    assert hold[-1]['velocities'][4] == 0
    assert all(r['velocities'][4]*sign >= 0 for r in hold)
    assert len({r['positions'][4] for r in hold[-40:]}) == 1


def test_candidate_cannot_bypass_fixed_experiment_budget():
    for options in [dict(preview_unlimited=True), dict(step_cap_counts=3000),
                    dict(accel_mm_s2=1.95), dict(accel_mm_s2=10), dict(move_duration_s=10)]:
        kwargs = dict(step_cap_counts=6500, accel_mm_s2=5, profile_events=True)
        kwargs.update(options)
        with pytest.raises(ValueError, match='Profile events require'):
            app.plan(app.PMACConfig(), [0]*5, 5, 4, **kwargs)


@pytest.mark.parametrize('distance', [-5, 5])
@pytest.mark.parametrize('axis_sign', [-1, 1])
def test_repeated_reversals_remain_bounded_and_return(distance, axis_sign):
    cfg = app.PMACConfig()
    cfg.axis_signs[4] = axis_sign
    base = [100000, 200000, -300000, 400000, 634591]
    frames, summary = app.plan(cfg, base, distance, 4, step_cap_counts=6500,
                               accel_mm_s2=5, profile_events=True, repeat_reversals=True)
    assert summary['total_s'] == 23
    assert summary['interpolant_peak_mm_s'] <= 4
    assert summary['interpolant_peak_accel_mm_s2'] <= 5
    assert frames[-1]['positions'] == base
    assert frames[-1]['velocities'] == [0.]*5
    sign = axis_sign*(1 if distance > 0 else -1)
    for frame in frames:
        assert frame['positions'][:4] == base[:4]
        assert 0 <= sign*(frame['positions'][4]-base[4]) <= .005*cfg.pulses_per_meter+1
    for cycle in range(1, 7):
        out = [r for r in frames if r['phase'] == f'out_{cycle}']
        reverse = [r for r in frames if r['phase'] == f'reverse_{cycle}']
        assert out[-1]['velocities'][4]*sign > 0
        assert reverse[-1]['velocities'][4]*sign < 0


def test_repeated_reversals_reject_higher_acceleration():
    with pytest.raises(ValueError, match='Repeated reversals require'):
        app.plan(app.PMACConfig(), [0]*5, 5, 4, step_cap_counts=6500,
                 accel_mm_s2=6, profile_events=True, repeat_reversals=True)


def test_three_mm_s_preview_reports_required_limit():
    base = [12345, 23456, 34567, 45678, 693329]
    frames, summary = app.plan(app.PMACConfig(), base, 5, 3, preview_unlimited=True)
    assert summary["preview_only"] and not summary["within_recorded_limits"]
    assert 3700 < summary["minimum_step_cap_counts"] < 3800
    assert summary["interpolant_peak_mm_s"] == pytest.approx(3, abs=0.02)
    assert frames[-1]["positions"] == base
    assert all(row["positions"][:4] == base[:4] for row in frames)


def test_explicit_duration_reduces_speed_and_acceleration():
    cfg = app.PMACConfig()
    _, normal = app.plan(cfg, [0] * 5, 5, 2.3)
    _, slower = app.plan(cfg, [0] * 5, 5, 2.3, move_duration_s=6)
    assert slower["ramp_s"] == 6
    assert slower["interpolant_peak_mm_s"] < normal["interpolant_peak_mm_s"]
    assert (
        slower["interpolant_peak_accel_mm_s2"] < normal["interpolant_peak_accel_mm_s2"]
    )
    for duration in (1, -1, float("nan"), float("inf"), 121):
        with pytest.raises(ValueError, match="move-duration-s"):
            app.plan(cfg, [0] * 5, 5, 2.3, move_duration_s=duration)


def test_preview_cannot_execute_or_create_log(tmp_path):
    args = app.arguments(
        [
            "--execute",
            "--preview-unlimited",
            "--peak-mm-s",
            "3",
            "--csv",
            str(tmp_path / "forbidden.csv"),
        ]
    )
    with pytest.raises(ValueError, match="cannot be combined"):
        app.run(args, controller_factory=lambda _: pytest.fail("Hardware instantiated"))
    assert not args.csv.exists()


@pytest.mark.parametrize("distance", [-5, 5])
def test_three_mm_s_requires_explicit_experimental_budget(distance):
    cfg = app.PMACConfig()
    base = [123, 456, 789, 1011, 157466]
    frames, info = app.plan(cfg, base, distance, 3, step_cap_counts=4000)
    assert 3000 < info["max_step_counts"] <= 4000
    assert not info["within_recorded_limits"]
    assert frames[-1]["positions"] == base
    assert all(frame["positions"][:4] == base[:4] for frame in frames)
    with pytest.raises(ValueError):
        app.plan(cfg, base, distance, 3)
    with pytest.raises(ValueError):
        app.plan(cfg, base, distance, 3.1, step_cap_counts=4000)


@pytest.mark.parametrize("distance", [-15, 15])
@pytest.mark.parametrize("sign", [-1, 1])
def test_five_mm_s_budget_keeps_acceleration_near_previous_trial(distance, sign):
    cfg = app.PMACConfig()
    cfg.axis_signs[4] = sign
    base = [123, 456, 789, 1011, 157467]
    frames, info = app.plan(cfg, base, distance, 5, step_cap_counts=6500)
    assert 6200 < info["max_step_counts"] <= 6500
    assert 4.98 < info["interpolant_peak_mm_s"] <= 5.01
    assert info["interpolant_peak_accel_mm_s2"] <= 3.1
    assert info["interpolant_peak_counts_ms"] <= 400
    assert frames[-1]["positions"] == base
    assert all(r["positions"][:4] == base[:4] for r in frames)
    assert (
        max(abs(r["positions"][4] - base[4]) for r in frames)
        <= 0.015 * cfg.pulses_per_meter + 1
    )


@pytest.mark.parametrize("distance,speed", [(16, 5), (15, 5.1), (5, 5)])
def test_five_mm_s_rejects_excess_distance_speed_or_acceleration(distance, speed):
    with pytest.raises(ValueError):
        app.plan(app.PMACConfig(), [0] * 5, distance, speed, step_cap_counts=6500)


@pytest.mark.parametrize("distance", [-15, 15])
@pytest.mark.parametrize("sign", [-1, 1])
def test_matched_acceleration_pair(distance, sign):
    cfg = app.PMACConfig()
    cfg.axis_signs[4] = sign
    base = [123, 456, 789, 1011, 634591]
    results = []
    for accel in (1.95, 5.0):
        frames, info = app.plan(
            cfg, base, distance, 4, step_cap_counts=6500, accel_mm_s2=accel
        )
        assert frames[-1]["positions"] == base
        assert all(row["positions"][:4] == base[:4] for row in frames)
        assert info["max_step_counts"] <= 5023
        assert info["interpolant_peak_counts_ms"] < 252
        assert info["interpolant_peak_accel_mm_s2"] < (2.3 if accel == 1.95 else 5.5)
        assert info["trajectory_profile"] == "cosine_velocity"
        out = [r for r in frames if r["phase"] == "out"]
        for prev, row in zip(out, out[1:]):
            assert (row["positions"][4] - prev["positions"][4]) * sign * distance >= 0
        assert out[-1]["velocities"][4] == 0
        assert abs(out[-1]["positions"][4] - base[4]) == pytest.approx(
            0.015 * cfg.pulses_per_meter, abs=1
        )
        results.append(info)
    low, high = results
    assert low["interpolant_peak_mm_s"] == pytest.approx(
        high["interpolant_peak_mm_s"], abs=0.002
    )
    assert high["acceleration_ramp_s"] < low["acceleration_ramp_s"]
    assert high["cruise_s"] > low["cruise_s"]


@pytest.mark.parametrize(
    "overrides",
    [
        dict(peak_mm_s=5),
        dict(distance_mm=5),
        dict(step_cap_counts=4000),
        dict(accel_mm_s2=10),
        dict(move_duration_s=8),
    ],
)
def test_acceleration_comparison_rejects_unvalidated_combinations(overrides):
    kwargs = dict(distance_mm=15, peak_mm_s=4, step_cap_counts=6500, accel_mm_s2=5)
    kwargs.update(overrides)
    with pytest.raises(ValueError):
        app.plan(app.PMACConfig(), [0] * 5, **kwargs)


@pytest.mark.parametrize(
    "limit,error,expected",
    [
        (3000, 0, "Live controller"),
        (4000, 4000, "following_error_threshold"),
        (4000, -1, "invalid_error_feedback"),
    ],
)
def test_experimental_limit_and_error_abort(
    tmp_path, monkeypatch, limit, error, expected
):
    from pmac_sdk.core import waiting

    events = []

    class Timer:
        def sleep(self, _):
            pass

        def close(self):
            events.append("timer_close")

    class Robot:
        def __init__(self, _):
            self.base_positions = [1000] * 5
            self.pvt_axis5_max_step = limit
            self.protocol = SimpleNamespace(
                last_pvt_status=SimpleNamespace(queued=5),
                snapshot=lambda: ([1000] * 5, [0, 0, 0, error]),
            )

        def safe_boot_and_home(self):
            events.append("boot")

        def move_pvt_stream(self, positions, *args):
            events.append("submit")
            assert positions == [1000] * 5  # First prime point only; never continue.

        def close(self):
            events.append("robot_close")

    monkeypatch.setattr(waiting, "WindowsHighResolutionWait", Timer)
    args = app.arguments(
        [
            "--execute",
            "--distance-mm",
            "5",
            "--peak-mm-s",
            "3",
            "--step-cap-counts",
            "4000",
            "--confirmed-clearance-mm",
            "50",
            "--csv",
            str(tmp_path / "guard.csv"),
        ]
    )
    with pytest.raises((ValueError, RuntimeError), match=expected):
        app.run(args, Robot)
    assert events[-2:] == ["robot_close", "timer_close"]
    assert events.count("submit") == (0 if limit == 3000 else 1)
    if limit == 4000:
        with args.csv.open() as log:
            metadata = json.loads(log.readline()[2:])
            rows = list(csv.DictReader(log))
        assert metadata["live_step_cap_counts"] == 4000
        assert len(rows) == 1 and rows[0]["stop_reason"] == expected


@pytest.mark.parametrize("threshold", ["0", "nan", "inf", "0.1", "-1"])
def test_invalid_stop_threshold_before_hardware(tmp_path, threshold):
    args = app.arguments(
        [
            "--execute",
            "--stop-error-mm",
            threshold,
            "--csv",
            str(tmp_path / "invalid.csv"),
        ]
    )
    with pytest.raises(ValueError, match="stop-error-mm"):
        app.run(args, controller_factory=lambda _: pytest.fail("Hardware instantiated"))
    assert not args.csv.exists()


@pytest.mark.parametrize(
    "distance,speed",
    [(0, 0.5), (6, 0.5), (float("nan"), 0.5), (1, 0), (1, float("inf")), (1, 3)],
)
def test_invalid_plan(distance, speed):
    with pytest.raises(ValueError):
        app.plan(app.PMACConfig(), [0] * 5, distance, speed)


@pytest.mark.parametrize("distance", [-5.0, 5.0])
@pytest.mark.parametrize("speed", [1.0, 2.3])
def test_five_mm_roundtrip_within_existing_limits(distance, speed):
    base = [12345, 23456, 34567, 45678, 693329]
    frames, summary = app.plan(app.PMACConfig(), base, distance, speed)
    assert frames[-1]["positions"] == base
    assert all(r["positions"][:4] == base[:4] for r in frames)
    assert summary["max_step_counts"] <= 3000
    assert summary["interpolant_peak_counts_ms"] <= 400
    assert summary["interpolant_peak_mm_s"] == pytest.approx(
        1.875 * abs(distance) / summary["ramp_s"], abs=0.002
    )


def test_dry_run_has_no_controller_or_fake_feedback(tmp_path):
    args = app.arguments(["--csv", str(tmp_path / "preview.csv")])

    def forbidden(_):
        raise AssertionError("Hardware instantiated")

    app.run(args, controller_factory=forbidden)
    with args.csv.open() as f:
        metadata = json.loads(f.readline()[2:])
        rows = list(csv.DictReader(f))
    assert metadata["execute"] is False
    assert rows and all(r["feedback_p5"] == "" and r["t_s"] == "" for r in rows)
    with pytest.raises(FileExistsError):
        app.run(args, controller_factory=forbidden)


def test_missing_clearance_rejected_before_controller(tmp_path):
    args = app.arguments(["--execute", "--csv", str(tmp_path / "run.csv")])
    with pytest.raises(ValueError, match="clearance"):
        app.run(args, controller_factory=lambda _: pytest.fail("Hardware instantiated"))
    assert not args.csv.exists()


def test_insufficient_clearance_rejected_before_controller(tmp_path):
    args = app.arguments(
        [
            "--execute",
            "--confirmed-clearance-mm",
            ".5",
            "--csv",
            str(tmp_path / "run.csv"),
        ]
    )
    with pytest.raises(ValueError, match="clearance"):
        app.run(args, controller_factory=lambda _: pytest.fail("Hardware instantiated"))
    assert not args.csv.exists()


def test_interrupt_aborts_without_return_and_closes_timer(tmp_path, monkeypatch):
    from pmac_sdk.core import waiting

    events = []

    class Timer:
        def sleep(self, _):
            pass

        def close(self):
            events.append("timer_close")

    class Robot:
        def __init__(self, _):
            self.protocol = SimpleNamespace()
            self.base_positions = [1000] * 5
            self.pvt_axis5_max_step = 3000

        def safe_boot_and_home(self):
            events.append("boot")

        def move_pvt_stream(self, *args):
            events.append("submit")
            raise KeyboardInterrupt

        def close(self):
            events.append("robot_close")

    monkeypatch.setattr(waiting, "WindowsHighResolutionWait", Timer)
    args = app.arguments(
        [
            "--execute",
            "--confirmed-clearance-mm",
            "1",
            "--csv",
            str(tmp_path / "run.csv"),
        ]
    )
    with pytest.raises(KeyboardInterrupt):
        app.run(args, Robot)
    assert events == ["boot", "submit", "robot_close", "timer_close"]
