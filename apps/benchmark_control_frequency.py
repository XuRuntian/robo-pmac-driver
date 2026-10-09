"""Offline IK / driver-compute frequency sweep. Never opens hardware or sockets.

This measures a computational subset of the driver, NOT end-to-end control.
See docs/control_frequency_benchmark.md for scope and interpretation.
"""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import os
import platform
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from continuum_sdk.control.axis_mapper import ContinuumAxisMapper
from continuum_sdk.control.cartesian_frame import apply_axis_transform
from continuum_sdk.control.pvt_mapper import ContinuumPVTMapper
from continuum_sdk.control.tip_command_filter import TipCommandFilter
from continuum_sdk.core.config_loader import load_continuum_config
from continuum_sdk.core.factory import build_continuum_ik, build_tendon_mapper
from continuum_sdk.core.interface_config import load_robot_interface_config
from continuum_sdk.kinematics.dls_ik import rotvec_to_matrix
from pmac_sdk.core.config_model import PMACConfig

SCENARIOS = ("normal", "near_straight", "boundary", "unreachable")
SEEDS = {
    "normal": [0.12, 0.5, 0.4, 0.3, -0.5],
    "near_straight": [0.12, 0.001, 0.4, 0.001, -0.5],
    "boundary": [0.254, 3.10, 0.4, 1.53, -0.5],
    "unreachable": [0.12, 0.5, 0.4, 0.3, -0.5],
}


def world_action(t: float, scenario: str) -> dict[str, float]:
    # Same continuous-time input at every sampling frequency. Ry stays disabled.
    s = np.sin(2 * np.pi * 0.2 * t)
    c = 1 - np.cos(2 * np.pi * 0.2 * t)
    values = [0.002 * s, 0.001 * c, 0.002 * s, 0.03 * s, 0.0, 0.02 * c]
    if scenario == "unreachable":
        values = [0.08, 0.05, 0.05, 1.0, 0.0, 1.0]
    return dict(zip(("tip_delta_x", "tip_delta_y", "tip_delta_z",
                     "tip_delta_rx", "tip_delta_ry", "tip_delta_rz"), values))


def reachable_goal(ik, t: float, scenario: str):
    u = np.array(SEEDS[scenario], dtype=float)
    s = np.sin(2 * np.pi * 0.2 * t)
    c = 1 - np.cos(2 * np.pi * 0.2 * t)
    if scenario == "boundary":
        u += [-0.0004 * c, -0.02 * c, 0.03 * s, -0.015 * c, 0.03 * s]
    elif scenario == "near_straight":
        u += [0.001 * s, 0.02 * c, 0.1 * s, 0.01 * c, -0.1 * s]
    else:
        u += [0.001 * s, 0.03 * s, 0.05 * s, 0.02 * s, -0.05 * s]
    p, r = ik.fk_tip(u)
    if scenario == "unreachable":
        p = p + np.array([0.5, 0.5, 0.5])
    return p, r[:, 2]


def residuals(ik, u, p, z):
    actual_p, actual_r = ik.fk_tip(u)
    a = actual_r[:, 2] / np.linalg.norm(actual_r[:, 2])
    b = z / np.linalg.norm(z)
    angle = np.arctan2(np.linalg.norm(np.cross(a, b)), np.dot(a, b))
    return float(np.linalg.norm(actual_p - p) * 1000), float(np.degrees(angle))


def summarize(rows, hz):
    work = np.array([r["work_ms"] for r in rows])
    budget = 1000 / hz
    misses = work > budget
    longest = current = 0
    previous_repeat = None
    for row, miss in zip(rows, misses):
        if row.get("repeat") != previous_repeat:
            current = 0
        previous_repeat = row.get("repeat")
        current = current + 1 if miss else 0
        longest = max(longest, current)
    result = {
        "samples": len(rows), "budget_ms": budget,
        "work_p50_ms": float(np.percentile(work, 50)),
        "work_p95_ms": float(np.percentile(work, 95)),
        "work_p99_ms": float(np.percentile(work, 99)),
        "work_max_ms": float(work.max()),
        "compute_miss_pct": float(misses.mean() * 100),
        "longest_compute_miss_run": longest,
        "converged_pct": float(np.mean([r["converged"] for r in rows]) * 100),
        "position_max_mm": max(r["position_mm"] for r in rows),
        "direction_max_deg": max(r["direction_deg"] for r in rows),
        "command_position_max_mm": max(r["command_position_mm"] for r in rows),
        "schedule_miss_pct": (float(np.mean([r["completion_late_ms"] > 0 for r in rows]) * 100)
                              if any(r.get("paced", False) for r in rows) else None),
        "completion_late_max_ms": max(r["completion_late_ms"] for r in rows),
        "compute_pass": bool(not misses.any()),
        "reserve_30pct_pass": bool(work.max() <= 0.7 * budget),
    }
    for stage in ("prepare_ms", "ik_ms", "mapping_ms", "commit_ms"):
        result[stage] = float(np.mean([r[stage] for r in rows]))
    return result


def run_case(cfg, interface, hz, seconds, scenario, mode, paced=False):
    ik = build_continuum_ik(cfg)
    ik.reset(np.array(SEEDS[scenario]))
    tendon = build_tendon_mapper(cfg)
    pc = PMACConfig()  # Data only. Never serialize credentials or network settings.
    axis = ContinuumAxisMapper(pc.pulses_per_rad, pc.pulses_per_meter,
                               pc.axis_order, pc.axis_signs)
    base = [1000, -2000, 3000, -4000, 5000]  # Synthetic reference, never encoder data.
    mapper = ContinuumPVTMapper(ik, tendon, axis, base, 1 / hz, cfg.ik.max_inner_steps)
    last = axis.logical_to_pulses(base, tendon.to_axis_targets(ik.u))
    if mode == "pipeline":
        mapper.commit_pulses(last)
    center_p, center_r = ik.fk_tip()
    command_filter = TipCommandFilter(interface.command, 1 / hz)
    n = max(1, int(round(seconds * hz)))
    # Generation and later residual analysis are outside the measured workload.
    inputs = [world_action(i / hz, scenario) if mode == "pipeline"
              else reachable_goal(ik, i / hz, scenario) for i in range(n)]
    captured = []
    solve_original = ik.solve
    solve_ns = 0

    def timed_solve(*args, **kwargs):
        nonlocal solve_ns
        start = time.perf_counter_ns()
        result = solve_original(*args, **kwargs)
        solve_ns = time.perf_counter_ns() - start
        return result

    ik.solve = timed_solve
    epoch = time.perf_counter()
    previous_start = None
    for i, value in enumerate(inputs):
        release = epoch + i / hz
        if paced:
            remaining = release - time.perf_counter()
            if remaining > 0:
                time.sleep(remaining)
        start = time.perf_counter_ns()
        if mode == "pipeline":
            command_filter.set_command(value)
            delta = command_filter.step()
            p = center_p + apply_axis_transform(delta, interface.frame.translation_map,
                                                interface.frame.translation_signs)
            rotation = apply_axis_transform(command_filter.applied_rotation,
                                            interface.frame.rotation_map,
                                            interface.frame.rotation_signs)
            z = (center_r @ rotvec_to_matrix(rotation))[:, 2]
            prepared = time.perf_counter_ns()
            command = mapper.build_command(p, z_goal=z)
            mapped = time.perf_counter_ns()
            result = command.ik_result
            # Match the driver's software axis-5 clipping. PLC limits are unknown offline.
            physical = axis.axis_order[4]
            max_step = abs(pc.pulses_per_meter * interface.command.max_speed_m_s[2] / hz)
            delta_p = float(np.clip(command.target_pulses[physical] - last[physical],
                                    -max_step, max_step))
            command.target_pulses[physical] = int(round(last[physical] + delta_p))
            command.velocities[physical] = delta_p / (1000 / hz)
            mapper.commit_pulses(command.target_pulses)
            ik.fk_tip()  # Driver's command FK computation.
            last = command.target_pulses
        else:
            p, z = value
            prepared = time.perf_counter_ns()
            result = ik.solve(p, z_goal=z, max_steps=cfg.ik.max_inner_steps)
            mapped = time.perf_counter_ns()
        end = time.perf_counter_ns()
        captured.append((i, start, end, prepared, mapped, solve_ns, result,
                         ik.u.copy(), p.copy(), z.copy(), previous_start,
                         max(0.0, (end / 1e9 - release - 1 / hz) * 1000) if paced else 0.0))
        previous_start = start
    rows = []
    for i, start, end, prepared, mapped, solve_ns, result, accepted_u, p, z, prev, late in captured:
        pos, direction = residuals(ik, result.u, p, z)
        command_pos, command_dir = residuals(ik, accepted_u, p, z)
        row = dict(mode=mode, scenario=scenario, hz=hz, paced=paced, t_s=i / hz,
                   work_ms=(end - start) / 1e6, prepare_ms=(prepared - start) / 1e6,
                   ik_ms=solve_ns / 1e6, mapping_ms=max(0, mapped - prepared - solve_ns) / 1e6,
                   commit_ms=(end - mapped) / 1e6, completion_late_ms=late,
                   interval_ms=0 if prev is None else (start - prev) / 1e6,
                   converged=result.converged, limited=bool(result.limit_attempts),
                   position_mm=pos, direction_deg=direction,
                   command_position_mm=command_pos, command_direction_deg=command_dir)
        rows.append(row)
    return rows


def write_csv(path, rows):
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def plots(out, summaries, samples=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    fields = [("work_p99_ms", "P99 computation (ms)"),
              ("compute_miss_pct", "Computation over budget (%)"),
              ("position_max_mm", "Maximum model position error (mm)"),
              ("direction_max_deg", "Maximum tool-axis error (deg)")]
    for ax, (field, label) in zip(axes.flat, fields):
        for mode in ("ik", "pipeline"):
            for scenario in SCENARIOS:
                rows = sorted([r for r in summaries if r["mode"] == mode and r["scenario"] == scenario],
                              key=lambda r: r["hz"])
                if rows:
                    ax.plot([r["hz"] for r in rows], [r[field] for r in rows],
                            marker=".", label=f"{mode}/{scenario}")
        ax.set(xlabel="Requested software frequency (Hz)", ylabel=label)
        if field in ("position_max_mm", "direction_max_deg"):
            ax.set_yscale("symlog", linthresh=1e-5)
            ax.set_ylabel(label + " [symlog]")
        ax.grid(alpha=0.25)
    rates = sorted({r["hz"] for r in summaries})
    axes[0, 0].plot(rates, [1000 / f for f in rates], "k--", label="Cycle budget")
    axes[0, 0].legend(fontsize=7)
    fig.suptitle("Offline computation only - no PMAC / no measured motor feedback")
    fig.savefig(out / "frequency.png", dpi=160)
    plt.close(fig)
    rows = [r for r in summaries if r["mode"] == "pipeline" and r["scenario"] == "normal"]
    if rows:
        fig, ax = plt.subplots(figsize=(9, 4), constrained_layout=True)
        bottom = np.zeros(len(rows))
        for stage in ("prepare_ms", "ik_ms", "mapping_ms", "commit_ms"):
            values = np.array([r[stage] for r in rows])
            ax.bar([str(r["hz"]) for r in rows], values, bottom=bottom, label=stage)
            bottom += values
        ax.set(xlabel="Requested frequency (Hz)", ylabel="Mean computation (ms)",
               title="Normal WORLD replay: computation breakdown")
        ax.legend()
        fig.savefig(out / "stages.png", dpi=160)
        plt.close(fig)
    if samples and any(r["mode"] == "pipeline" for r in samples):
        fig, axes = plt.subplots(2, 2, figsize=(12, 7), constrained_layout=True)
        for scenario in SCENARIOS:
            selected = [r for r in samples if r["mode"] == "pipeline" and r["scenario"] == scenario]
            if not selected:
                continue
            hz = max(r["hz"] for r in selected)
            selected = [r for r in selected if r["hz"] == hz and r["repeat"] == 0]
            label = f"{scenario} ({hz:g} Hz)"
            t = [r["t_s"] for r in selected]
            work = sorted(r["work_ms"] for r in selected)
            axes[0, 0].plot(work, np.arange(1, len(work) + 1) / len(work), label=label)
            axes[0, 1].plot(t, [r["work_ms"] for r in selected], label=label, alpha=0.7)
            axes[1, 0].plot(t, [r["position_mm"] for r in selected], label=label)
            if selected[0]["paced"]:
                axes[1, 1].plot(t[1:], [r["interval_ms"] for r in selected[1:]], label=label, alpha=0.7)
        for ax, xlabel, ylabel in zip(axes.flat,
                ["Computation (ms)", "Trajectory time (s)", "Trajectory time (s)", "Trajectory time (s)"],
                ["Empirical CDF", "Computation (ms)", "Model position error (mm)", "Paced interval (ms)"]):
            ax.set(xlabel=xlabel, ylabel=ylabel)
            ax.grid(alpha=0.25)
        axes[1, 0].set_yscale("symlog", linthresh=1e-4)
        axes[0, 0].legend(fontsize=8)
        fig.suptitle("Highest tested pipeline frequency, first repeat; stress cases separate from accuracy acceptance")
        fig.savefig(out / "traces.png", dpi=160)
        plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frequencies", type=float, nargs="+", default=[50, 75, 100, 125, 150, 200, 300, 500])
    parser.add_argument("--seconds", type=float, default=10)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--scenarios", nargs="+", choices=SCENARIOS, default=list(SCENARIOS))
    parser.add_argument("--modes", nargs="+", choices=["ik", "pipeline"], default=["ik", "pipeline"])
    parser.add_argument("--paced", action="store_true", help="Sleep against fixed deadlines; no catch-up resets.")
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    if args.seconds <= 0 or not np.isfinite(args.seconds) or args.repeats < 1 or any(
            not np.isfinite(f) or f <= 0 for f in args.frequencies):
        parser.error("Frequencies, duration and repeats must be positive finite values.")
    if args.out.exists():
        parser.error("Output directory already exists; choose a new path to preserve evidence.")
    cfg = load_continuum_config(ROOT / "config/continuum.yaml")
    interface = load_robot_interface_config(ROOT / "config/robot_interface.yaml")
    if cfg.ik.task_mode != "pos_z":
        parser.error("This benchmark requires the current pos_z task.")
    args.out.mkdir(parents=True)
    def git(*command):
        return subprocess.check_output(["git", "-C", str(ROOT), *command], text=True).strip()
    metadata = {
        "scope": "Offline computation subset; no ZMQ, Modbus, PLC, motor or measured feedback.",
        "python": sys.version, "numpy": np.__version__, "platform": platform.platform(),
        "processor": platform.processor(), "cpu_count": os.cpu_count(),
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "commit": git("rev-parse", "HEAD"), "git_status": git("status", "--short"),
        "arguments": {**vars(args), "out": str(args.out)},
        "seeds": SEEDS, "sample_order_seed": 20261008,
        "configs": {name: (ROOT / "config" / name).read_text(encoding="utf-8")
                    for name in ("continuum.yaml", "robot_interface.yaml")},
        "source_sha256": {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                          for p in [Path(__file__), *sorted((ROOT / "src/continuum_sdk").rglob("*.py")),
                                    ROOT / "src/pmac_sdk/core/config_model.py"]},
        "criteria": "compute_pass: no observed work > period; reserve: max work <= 70% period. Not hard realtime.",
    }
    pc = PMACConfig()
    metadata["axis_mapping"] = dict(order=pc.axis_order, signs=pc.axis_signs,
                                    pulses_per_rad=pc.pulses_per_rad, pulses_per_meter=pc.pulses_per_meter)
    (args.out / "metadata.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    jobs = [(mode, scenario, hz, repeat) for repeat in range(args.repeats)
            for mode in args.modes for scenario in args.scenarios for hz in args.frequencies]
    np.random.default_rng(20261008).shuffle(jobs)
    all_rows = []
    for mode, scenario, hz, repeat in jobs:
        # Discarded warm-up on an independent state; measured startup remains included.
        run_case(cfg, interface, hz, 3 / hz, scenario, mode)
        rows = run_case(cfg, interface, hz, args.seconds, scenario, mode, args.paced)
        for row in rows:
            row["repeat"] = repeat
        all_rows.extend(rows)
        result = summarize(rows, hz)
        print(f"{mode}/{scenario} {hz:g} Hz repeat {repeat + 1}: "
              f"p99={result['work_p99_ms']:.2f} ms miss={result['compute_miss_pct']:.2f}% "
              f"converged={result['converged_pct']:.1f}%", flush=True)
    summaries = []
    for mode in args.modes:
        for scenario in args.scenarios:
            for hz in sorted(set(args.frequencies)):
                rows = [r for r in all_rows if r["mode"] == mode and r["scenario"] == scenario and r["hz"] == hz]
                summaries.append(dict(mode=mode, scenario=scenario, hz=hz, **summarize(rows, hz)))
    write_csv(args.out / "samples.csv", all_rows)
    write_csv(args.out / "summary.csv", summaries)
    plots(args.out, summaries, all_rows)
    print(f"Saved offline evidence to {args.out.resolve()}", flush=True)


if __name__ == "__main__":
    main()
