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
