#!/usr/bin/env python3
"""Read-only host readiness report. Does not open EtherCAT or raw sockets."""
import argparse
import datetime
import json
import os
from pathlib import Path
import platform
import resource
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def read(path):
    try:
        return Path(path).read_text().strip()
    except (OSError, ValueError):
        return None


def run(*command):
    try:
        r = subprocess.run(command, capture_output=True, text=True, timeout=10)
        return {"exit_code": r.returncode, "output": (r.stdout + r.stderr).strip()}
    except (OSError, subprocess.TimeoutExpired) as e:
        return {"exit_code": None, "output": str(e)}


def report():
    kernel = platform.release()
    config = read(f"/boot/config-{kernel}") or ""
    prefix = ROOT / "build/igh-install"
    interfaces = []
    for path in sorted(Path("/sys/class/net").iterdir()):
        if path.name == "lo":
            continue
        interfaces.append({"name": path.name, "mac": read(path / "address"),
                           "operstate": read(path / "operstate"), "carrier": read(path / "carrier"),
                           "wireless": (path / "wireless").exists(),
                           "device_driver": (path / "device/driver").resolve().name})
    compiled = read(ROOT / "build/kernel-release.txt")
    modules = [ROOT / "build/igh-source/master/ec_master.ko",
               ROOT / "build/igh-source/devices/ec_generic.ko"]
    data = {
        "timestamp_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "kernel": kernel, "architecture": platform.machine(),
        "preempt_rt_configured": "CONFIG_PREEMPT_RT=y" in config,
        "kernel_headers_present": Path(f"/lib/modules/{kernel}/build/Makefile").is_file(),
        "compiled_kernel": compiled,
        "compiled_modules_present": all(p.is_file() for p in modules),
        "compiled_kernel_matches_running": compiled == kernel,
        "master_module_loaded": Path("/sys/module/ec_master").exists(),
        "ethercat_device_nodes": [str(p) for p in Path("/dev").glob("EtherCAT*")],
        "local_cli": run(str(prefix / "bin/ethercat"), "version"),
        "local_abi": run(str(ROOT / "build/offline/igh_abi_check")),
        "interfaces": interfaces,
        "process_scheduler_policy": os.sched_getscheduler(0),
        "rt_priority_limits": resource.getrlimit(resource.RLIMIT_RTPRIO),
        "locked_memory_limits_bytes": resource.getrlimit(resource.RLIMIT_MEMLOCK),
        "hardware_motion_ready": False,
        "not_validated": ["master loading and Secure Boot policy", "real slave identity and PDOs",
                          "drive DC synchronization and WKC under load", "physical units and travel",
                          "homing and coordinated stop on every fault", "full control-loop realtime performance"],
    }
    return data


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    data = report()
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps(data, indent=2, ensure_ascii=False))
    if data["local_cli"]["exit_code"] != 0 or data["local_abi"]["exit_code"] != 0 \
            or not data["compiled_modules_present"] or not data["compiled_kernel_matches_running"]:
        raise SystemExit("offline environment incomplete or kernel has changed; rebuild with prepare_igh.sh")


if __name__ == "__main__":
    main()
