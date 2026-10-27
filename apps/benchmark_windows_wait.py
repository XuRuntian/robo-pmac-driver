"""Standalone Windows wait-mechanism experiment. Stdlib only, NO robot imports.

Does not change timer resolution, priority, affinity, Python version or driver.
The timer wrappers below are laboratory probes, not production control helpers.
"""
import argparse
import csv
import ctypes
from ctypes import wintypes
import hashlib
import json
import math
from pathlib import Path
import platform
import random
import statistics
import sys
import time


class WindowsWaits:
    def __init__(self):
        self.k = ctypes.WinDLL("kernel32", use_last_error=True)
        declarations = {
            "CreateEventW": ([ctypes.c_void_p, wintypes.BOOL, wintypes.BOOL, wintypes.LPCWSTR], wintypes.HANDLE),
            "CreateWaitableTimerExW": ([ctypes.c_void_p, wintypes.LPCWSTR, wintypes.DWORD, wintypes.DWORD], wintypes.HANDLE),
            "SetWaitableTimer": ([wintypes.HANDLE, ctypes.POINTER(ctypes.c_longlong), wintypes.LONG,
                                   ctypes.c_void_p, ctypes.c_void_p, wintypes.BOOL], wintypes.BOOL),
            "WaitForSingleObjectEx": ([wintypes.HANDLE, wintypes.DWORD, wintypes.BOOL], wintypes.DWORD),
            "CloseHandle": ([wintypes.HANDLE], wintypes.BOOL),
            "Sleep": ([wintypes.DWORD], None),
        }
        for name, (args, result) in declarations.items():
            fn = getattr(self.k, name)
            fn.argtypes, fn.restype = args, result
        self.handles = []
        try:
            self.event = self._keep(self.k.CreateEventW(None, True, False, None))
            # TIMER_MODIFY_STATE | SYNCHRONIZE. Unnamed, non-inheritable handles.
            self.normal = self._keep(self.k.CreateWaitableTimerExW(None, None, 0, 0x100002))
            self.high = self._keep(self.k.CreateWaitableTimerExW(None, None, 2, 0x100002))
        except BaseException:
            self.close()
            raise

    def _keep(self, handle):
        if not handle:
            raise ctypes.WinError(ctypes.get_last_error())
        self.handles.append(handle)
        return handle

    def close(self):
        for handle in reversed(self.handles):
            self.k.CloseHandle(handle)
        self.handles.clear()

    def wait(self, method, seconds):
        if seconds <= 0:
            return
        if method == "python_sleep":
            time.sleep(seconds)
        elif method == "win_sleep":
            self.k.Sleep(math.ceil(seconds * 1000))
        elif method == "event_timeout":
            rc = self.k.WaitForSingleObjectEx(self.event, math.ceil(seconds * 1000), False)
            if rc != 258:  # WAIT_TIMEOUT, event is never signaled.
                raise RuntimeError(f"Unexpected event result {rc}")
        else:
            handle = self.high if method == "high_res_timer" else self.normal
            due = ctypes.c_longlong(-max(1, math.ceil(seconds * 10_000_000)))
            if not self.k.SetWaitableTimer(handle, ctypes.byref(due), 0, None, None, False):
                raise ctypes.WinError(ctypes.get_last_error())
            # Bounded wait: probe failure must never leave an infinite blocking wait.
            rc = self.k.WaitForSingleObjectEx(handle, math.ceil(seconds * 1000) + 2000, False)
            if rc != 0:
                raise RuntimeError(f"Unexpected timer result {rc}")


def percentile(values, p):
    values = sorted(values)
    index = (len(values) - 1) * p / 100
    lo, hi = math.floor(index), math.ceil(index)
    return values[lo] + (values[hi] - values[lo]) * (index - lo)


def write_csv(path, rows):
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--samples", type=int, default=100)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--period-cycles", type=int, default=100)
    args = parser.parse_args()
    if sys.platform != "win32":
        parser.error("This experiment requires Windows")
    if min(args.samples, args.repeats, args.period_cycles) < 1:
        parser.error("Sample counts must be positive")
    args.out.mkdir(parents=True, exist_ok=False)
    metadata = dict(python=sys.version, executable=sys.executable, platform=platform.platform(),
                    perf_counter=vars(time.get_clock_info("perf_counter")),
                    arguments={**vars(args), "out": str(args.out)},
                    source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                    methods="same process/main thread; randomized blocks; 3 discarded warmups per block",
                    limitations="Background load not controlled. No hardware or real Modbus. Not a hard-realtime guarantee.",
                    unchanged="No timeBeginPeriod, priority/affinity changes, busy-spin wait, or driver modifications.")
    (args.out / "metadata.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    methods = ["python_sleep", "win_sleep", "event_timeout", "normal_timer", "high_res_timer"]
    jobs = [(repeat, method, delay) for repeat in range(args.repeats)
            for method in methods for delay in (1, 5, 10, 20)]
    random.Random(20261008).shuffle(jobs)
    waits = WindowsWaits()
    rows, blocks, periods = [], [], []
    try:
        for repeat, method, delay in jobs:
            for _ in range(3):
                waits.wait(method, delay / 1000)
            wall, cpu = time.perf_counter(), time.process_time()
            for i in range(args.samples):
                start = time.perf_counter()
                waits.wait(method, delay / 1000)
                actual = (time.perf_counter() - start) * 1000
                rows.append(dict(repeat=repeat, method=method, requested_ms=delay, index=i,
                                 actual_ms=actual, excess_ms=actual-delay))
            blocks.append(dict(repeat=repeat, method=method, requested_ms=delay,
                               wall_s=time.perf_counter()-wall, cpu_s=time.process_time()-cpu))
            print(f"{method} {delay} ms repeat {repeat + 1}", flush=True)
        # Same scheduler policy as driver: advance by 20 ms, reset on overrun.
        jobs = [(r, m) for r in range(args.repeats) for m in methods]
        random.Random(20261009).shuffle(jobs)
        for repeat, method in jobs:
            deadline = time.perf_counter()
            previous = None
            for i in range(args.period_cycles):
                start = time.perf_counter()
                sum(j*j for j in range(10000))  # Fixed small CPU workload; no simulated hardware.
                deadline += .02
                remaining = deadline-time.perf_counter()
                periods.append(dict(repeat=repeat, method=method, index=i,
                    interval_ms="" if previous is None else (start-previous)*1000,
                    requested_sleep_ms=max(0, remaining)*1000))
                previous = start
                if remaining > 0:
                    waits.wait(method, remaining)
                else:
                    deadline = time.perf_counter()
            print(f"periodic {method} repeat {repeat + 1}", flush=True)
    finally:
        waits.close()
        if rows:
            write_csv(args.out / "samples.csv", rows)
            write_csv(args.out / "blocks.csv", blocks)
        if periods:
            write_csv(args.out / "periods.csv", periods)
    summary = []
    for method in methods:
        for delay in (1, 5, 10, 20):
            v = [r["actual_ms"] for r in rows if r["method"] == method and r["requested_ms"] == delay]
            b = [r for r in blocks if r["method"] == method and r["requested_ms"] == delay]
            summary.append(dict(method=method, requested_ms=delay, samples=len(v),
                p50_ms=percentile(v,50), p95_ms=percentile(v,95), p99_ms=percentile(v,99), max_ms=max(v),
                cpu_one_core_pct=100*sum(r["cpu_s"] for r in b)/sum(r["wall_s"] for r in b)))
    write_csv(args.out / "summary.csv", summary)
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
