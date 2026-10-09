"""Analyze a driver --latency-csv trace offline; never connects to hardware."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    with args.csv.open(encoding="utf-8") as f:
        metadata = json.loads(f.readline().removeprefix("# "))
        rows = list(csv.DictReader(f))
    if not rows:
        parser.error("Trace has no complete samples")
    args.out.mkdir(parents=True, exist_ok=False)
    def values(key):
        return np.array([float(r[key]) if r.get(key) not in (None, "") else np.nan for r in rows])
    summary = {"execute": metadata["execute"], "samples": len(rows),
               "feedback_reads": sum(r["feedback_new"] == "True" for r in rows),
               "note": "No PMAC interpolator setpoint or synchronized controller clock. Target-feedback difference is NOT servo following error."}
    metric_keys = ["period_ms", "work_before_log_ms", "previous_log_ms", "prepare_ms",
                   "ik_mapping_ms", "submit_ms", "feedback_ms", "queued_at_ack"]
    if "pvt_read_ms" in rows[0]:
        metric_keys += [f"{prefix}_{field}" for prefix in ("pvt", "snapshot")
                       for field in ("total_ms", "read_ms", "write_ms", "sleep_ms", "sleep_requested_ms",
                                     "reads", "writes", "polls", "sleeps")]
        metric_keys += ["previous_cycle_sleep_requested_ms", "previous_cycle_sleep_actual_ms"]
    for key in metric_keys:
        v = values(key)
        v = v[np.isfinite(v)]
        summary[key] = None if not len(v) else dict(p50=float(np.percentile(v, 50)),
            p95=float(np.percentile(v, 95)), p99=float(np.percentile(v, 99)), maximum=float(v.max()))
    (args.out / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    t = values("t_s")
    if "pvt_read_ms" in rows[0]:
        fig, axes = plt.subplots(3, 1, figsize=(12, 9), constrained_layout=True)
        for ax, prefix in zip(axes[:2], ("pvt", "snapshot")):
            for field in ("read_ms", "write_ms", "sleep_ms", "sleep_requested_ms"):
                v = values(f"{prefix}_{field}")
                good = np.isfinite(v)
                ax.plot(t[good], v[good], ".-", markersize=2, label=field, alpha=.8)
            ax.set_ylabel(f"{prefix} time (ms)")
        for key in ("previous_cycle_sleep_requested_ms", "previous_cycle_sleep_actual_ms"):
            axes[2].plot(t, values(key), label=key, alpha=.8)
        axes[2].set_ylabel("Previous cycle sleep (ms)")
        for ax in axes:
            ax.legend(fontsize=8)
            ax.grid(alpha=.25)
            ax.set_xlabel("Host time (s)")
        fig.suptitle("Protocol I/O calls vs explicit sleeps (I/O includes OS/library scheduling)")
        fig.savefig(args.out / "protocol_timing.png", dpi=150)
        plt.close(fig)
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), constrained_layout=True)
    for key in ("period_ms", "ik_mapping_ms", "submit_ms", "feedback_ms"):
        v = values(key)
        good = np.isfinite(v)
        axes[0].plot(t[good], v[good], "." if key == "feedback_ms" else "-", label=key)
    axes[0].axhline(1000 / metadata["control_hz"], color="k", linestyle="--", label="period budget")
    axes[0].set_ylabel("Host elapsed time (ms)")
    axes[1].plot(t, values("queued_at_ack"), label="Software queue at acceptance (not total backlog)")
    axes[1].set_ylabel("Queued points")
    axes[2].plot(t, values("feedback_age_since_read_ms"), label="Age since host completed last read")
    axes[2].set_ylabel("Feedback read age (ms)")
    for ax in axes:
        ax.legend(fontsize=8)
        ax.grid(alpha=.25)
        ax.set_xlabel("Host monotonic time since loop start (s)")
    fig.suptitle("Driver latency trace" if metadata["execute"] else "DRY RUN - no hardware feedback")
    fig.savefig(args.out / "timing.png", dpi=150)
    plt.close(fig)
    fig, axes = plt.subplots(3, 2, figsize=(12, 9), constrained_layout=True)
    for ax, suffix in zip(axes.flat, ("x", "y", "z", "rx", "ry", "rz")):
        scale = 1000 if len(suffix) == 1 else 180 / np.pi
        for prefix in ("raw", "applied"):
            ax.plot(t, values(f"{prefix}_tip_delta_{suffix}") * scale, label=prefix)
        ax.set(title=suffix, xlabel="Time (s)", ylabel="mm" if scale == 1000 else "deg")
        ax.legend()
        ax.grid(alpha=.25)
    fig.suptitle("Last accepted raw target vs filtered target; inspect holding flag separately")
    fig.savefig(args.out / "commands.png", dpi=150)
    plt.close(fig)
    fig, axes = plt.subplots(5, 1, figsize=(12, 10), constrained_layout=True)
    fresh = np.array([r["feedback_new"] == "True" for r in rows])
    for i, ax in enumerate(axes, 1):
        target = values(f"target_p{i}")
        origin = target[0]
        sent_t = values("submit_end_s") if metadata["execute"] else t
        ax.plot(sent_t, target - origin, label="Submitted target (host acknowledgement time)")
        if fresh.any():
            # Acquisition lies within request start/end, not exactly at completion.
            lo, hi = values("feedback_request_start_s")[fresh], values("feedback_request_end_s")[fresh]
            ax.errorbar((lo + hi) / 2, values(f"feedback_p{i}")[fresh] - origin,
                        xerr=(hi - lo) / 2, fmt=".", label="Encoder; horizontal bars bound acquisition")
        ax.set(ylabel=f"Physical axis {i}\nrelative counts", xlabel="Time (s)")
        ax.grid(alpha=.25)
        ax.legend(fontsize=7)
    fig.suptitle("Not servo following error: submitted targets include downstream buffering")
    fig.savefig(args.out / "axes.png", dpi=150)
    plt.close(fig)
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
