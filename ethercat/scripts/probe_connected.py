#!/usr/bin/env python3
"""Collect diagnostics using a temporary idle IgH master; never command motion.

Discovery initializes EtherCAT/mailboxes, normally reaching PREOP. No application
PDO configuration, SDO download, fault reset or enable command is issued. Exit 0
means collection/cleanup succeeded; drive faults are reported as warnings.
"""
import argparse
import datetime
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
CLI = ROOT / "build/igh-install/bin/ethercat"


def run(*args, timeout=5, required=True):
    try:
        p = subprocess.run([str(a) for a in args], capture_output=True, text=True, timeout=timeout)
        result = {"exit_code": p.returncode, "stdout": p.stdout.strip(), "stderr": p.stderr.strip()}
    except subprocess.TimeoutExpired:
        result = {"exit_code": 124, "stdout": "", "stderr": "diagnostic command timed out"}
    if required and result["exit_code"]:
        raise RuntimeError(f"{args[0]} failed: {result['stderr']}")
    return result


def cli(*args, required=True):
    return run(CLI, "--master", "0", *args, timeout=3, required=required)


def number(result):
    if result["exit_code"]:
        return None
    try:
        return int(result["stdout"].split()[0], 0)
    except (ValueError, IndexError):
        return None


def read_array(pos, index, datatype, limit):
    """Read a bounded diagnostic array, preserving raw replies and failures."""
    count = cli("--position", pos, "--type", "uint8", "upload", index, 0, required=False)
    result = {"count": count, "entries": {}, "complete": False}
    size = number(count)
    if size is None or not 0 <= size <= limit:
        return result
    for subindex in range(1, size + 1):
        value = cli("--position", pos, "--type", datatype, "upload", index, subindex, required=False)
        result["entries"][str(subindex)] = value
        if number(value) is None:
            return result
    result["complete"] = True
    return result


def interrupted(signum, _):
    raise RuntimeError(f"interrupted by signal {signum}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("interface")
    args = parser.parse_args()
    if args.interface not in os.listdir("/sys/class/net"):
        raise SystemExit("unknown interface")
    net = Path("/sys/class/net") / args.interface
    if os.geteuid() != 0:
        raise SystemExit("temporary module loading requires root")
    if not (net / "device").exists() or (net / "wireless").exists():
        raise SystemExit("a physical wired interface is required")
    if (net / "carrier").read_text().strip() != "1":
        raise SystemExit("no wired link; check cable and drive control power")
    if any(Path(f"/sys/module/{name}").exists() for name in ("ec_master", "ec_generic")):
        raise SystemExit("existing EtherCAT modules found; refusing to disturb an existing master")
    mac = (net / "address").read_text().strip()
    if not re.fullmatch(r"(?:[0-9a-f]{2}:){5}[0-9a-f]{2}", mac) or mac in (
            "00:00:00:00:00:00", "ff:ff:ff:ff:ff:ff"):
        raise SystemExit("invalid/wildcard MAC")
    modules = [ROOT / "build/igh-source/master/ec_master.ko",
               ROOT / "build/igh-source/devices/ec_generic.ko"]
    for module in modules:
        if run("modinfo", "-F", "vermagic", module)["stdout"].split()[0] != os.uname().release:
            raise SystemExit("kernel/module mismatch; rebuild first")
    expected = json.loads((ROOT / "config/pmac_drives.json").read_text())["slaves"][0]
    loaded = []
    report = {"timestamp_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
              "interface": args.interface, "mac": mac, "slave_count": 0,
              "slaves": [], "motion_commands_sent": False, "errors": [], "warnings": [],
              "diagnostic_collection_complete": False, "motion_readiness_validated": False}
    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGINT, interrupted)
    try:
        run("insmod", modules[0], f"main_devices={mac}")
        loaded.append("ec_master")
        run("udevadm", "settle", "--timeout=5", timeout=6)
        run("insmod", modules[1])
        loaded.append("ec_generic")
        deadline = time.monotonic() + 15
        previous, stable_since = "", time.monotonic()
        while time.monotonic() < deadline:
            listing = cli("slaves")["stdout"]
            if listing != previous:
                previous, stable_since = listing, time.monotonic()
            if listing and time.monotonic() - stable_since >= 2:
                break
            time.sleep(0.25)
        report["slave_listing"] = listing
        report["master_before"] = cli("master")["stdout"]
        if "Active: no" not in report["master_before"] or f"Main: {mac} (attached)" not in report["master_before"]:
            raise RuntimeError("expected inactive master attached to the selected MAC")
        positions = [int(m[1]) for line in listing.splitlines()
                     if (m := re.match(r"^\s*(\d+)\s+", line))]
        report["slave_count"] = len(positions)
        print(f"Detected {len(positions)} slave(s); reading identities and diagnostics.", file=sys.stderr, flush=True)
        if not positions:
            raise RuntimeError("link is up but no EtherCAT slave responded")
        if len(positions) > 5:
            raise RuntimeError("more than five slaves found; inspect topology before further reads")
        objects = [("statusword", "0x6041", "uint16"), ("error_code", "0x603f", "uint16"),
                   ("mode_requested", "0x6060", "int8"), ("mode_display", "0x6061", "int8"),
                   ("actual_position", "0x6064", "int32"), ("actual_torque", "0x6077", "int16"),
                   ("digital_inputs", "0x60fd", "uint32")]
        for pos in positions:
            detail = cli("--position", pos, "--verbose", "slaves")["stdout"]
            drive = {"position": pos, "detail": detail, "sdo": {}}
            report["slaves"].append(drive)
            vendor = re.search(r"Vendor Id:\s*(0x[0-9a-fA-F]+)", detail)
            product = re.search(r"Product code:\s*(0x[0-9a-fA-F]+)", detail)
            revision = re.search(r"Revision number:\s*(0x[0-9a-fA-F]+)", detail)
            drive["identity_matches"] = bool(vendor and product and revision
                    and int(vendor[1], 16) == expected["vendor_id"]
                    and int(product[1], 16) == expected["product_code"]
                    and int(revision[1], 16) == expected["revision"])
            if not drive["identity_matches"]:
                report["errors"].append(f"slave {pos}: identity differs from PMAC contract; SDO reads skipped")
                continue
            drive["pdo_listing"] = cli("--position", pos, "pdos", required=False)
            drive["pdo_listing_note"] = "May originate from SII defaults; not proof of current CoE assignment."
            failures = 0
            for name, index, datatype in objects:
                value = cli("--position", pos, "--type", datatype, "upload", index, 0, required=False)
                drive["sdo"][name] = value
                if value["exit_code"]:
                    report["warnings"].append(f"slave {pos}: {index}:0 upload failed")
                failures = failures + 1 if value["exit_code"] else 0
                if failures >= 3:
                    report["errors"].append(f"slave {pos}: three consecutive SDO failures; remaining reads skipped")
                    break
            status = number(drive["sdo"]["statusword"])
            if status is not None and status & 0x004f == 0x0008:
                report["warnings"].append(f"slave {pos}: CiA402 Fault; see error_code")
            if failures < 3:
                drive["pdo_objects"] = {}
                for index, datatype, limit in (("0x1c12", "uint16", 4), ("0x1c13", "uint16", 4),
                                               ("0x1600", "uint32", 16), ("0x1a00", "uint32", 16)):
                    values = read_array(pos, index, datatype, limit)
                    drive["pdo_objects"][index] = values
                    if not values["complete"]:
                        report["warnings"].append(f"slave {pos}: {index} array read incomplete")
                    elif index in ("0x1c12", "0x1c13") and any(
                            number(entry) == 0 for entry in values["entries"].values()):
                        report["warnings"].append(f"slave {pos}: {index} assignment contains PDO index 0")
            print(f"Slave {pos}: diagnostic reads complete.", file=sys.stderr, flush=True)
        report["master_after"] = cli("master")["stdout"]
        report["slave_listing_after"] = cli("slaves")["stdout"]
        report["diagnostic_collection_complete"] = not report["errors"]
    except Exception as error:
        report["errors"].append(str(error))
    finally:
        report["cleanup"] = []
        for name in reversed(loaded):
            result = run("rmmod", name, required=False)
            report["cleanup"].append({"module": name, **result})
            if result["exit_code"]:
                report["errors"].append(f"could not unload {name}")
        report["modules_unloaded"] = all(not Path(f"/sys/module/{name}").exists() for name in loaded)
        print(json.dumps(report, indent=2, ensure_ascii=False), flush=True)
    return 1 if report["errors"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
