#!/usr/bin/env python3
"""Single Diamond: configure volatile PDO/CSP/DC, observe with 6040 always 0.

Uses the ordinary diagnostic wrapper's temporary modules and dictionary wait.
Requires one healthy, disabled drive. No enable, reset, motion or flash save.
"""
import json
import subprocess
import time

import probe_connected as probe
from reset_fault_once import read_snapshot, require_single_preop
from check_esi import audit, ESI
import xml.etree.ElementTree as ET


def add_arguments(parser):
    parser.add_argument("--seconds", type=int, choices=range(5, 121), default=30)
    parser.add_argument("--period-us", type=int, choices=(1000, 2000, 4000), default=1000)
    parser.add_argument("--dc", choices=("esi", "pmac"), default="esi")
    parser.add_argument("--cpu", type=int)
    parser.add_argument("--ordinary-scheduler", action="store_true")


def check_pdo_readback(pdo, assigned, mapped):
    """Validate payload prefix; return count conformance separately for diagnostics.

    This permits disabled timing measurements with the observed count anomaly;
    it is not an approval gate for any enabled operation.
    """
    expected = [(e["index"] << 16) | (e["subindex"] << 8) | e["bits"] for e in pdo["entries"]]
    values = [probe.number(v) for v in mapped["entries"].values()]
    assignments = [probe.number(v) for v in assigned["entries"].values()]
    if not assigned["complete"] or not mapped["complete"] or assignments[:1] != [pdo["index"]] \
            or any(assignments[1:]) or values[:len(expected)] != expected:
        raise RuntimeError("live PDO payload/assignment differs from selected profile")
    return probe.number(assigned["count"]) == 1 and probe.number(mapped["count"]) == len(expected)


def run_disabled(report):
    result = report["disabled_cyclic"] = {"enable_commands_sent": False, "started": False}
    if report["errors"] or len(report["slaves"]) != 1:
        raise RuntimeError("disabled cyclic test requires exactly one successfully diagnosed slave")
    drive = report["slaves"][0]
    if drive["position"] != 0 or not drive["identity_matches"]:
        raise RuntimeError("disabled cyclic test requires matching Diamond identity at position 0")
    contract = json.loads((probe.ROOT / "config/pmac_drives.json").read_text())
    audit(ET.parse(ESI).getroot(), contract)
    probe.run("python3", probe.ROOT / "scripts/check_esi.py", "--check")
    require_single_preop()
    before = result["before"] = read_snapshot()
    if before["controlword"] != 0 or before["statusword"] & 0x4f != 0x40 or before["error_code"] != 0:
        raise RuntimeError("drive must be disabled with controlword 0 and no error")
    options = report.get("arguments", {})
    period_us = options.get("period_us", 1000)
    seconds = options.get("seconds", 30)
    sync1_ns = 0 if options.get("dc", "esi") == "esi" else 500000
    binary = probe.ROOT / "build/commissioning/igh_disabled_probe"
    if not binary.is_file():
        raise RuntimeError("run scripts/build_commissioning.sh first")
    command = [str(binary), "--disabled-single-axis", "--period-us", str(period_us),
               "--sync1-ns", str(sync1_ns), "--seconds", str(seconds)]
    if options.get("cpu") is not None:
        command.extend(("--cpu", str(options["cpu"])))
    if options.get("ordinary_scheduler", False):
        command.append("--ordinary-scheduler")
    result["command"] = command
    result["started"] = True
    # Automatic dictionary diagnostics have already restored debug level zero.
    # Per-transaction kernel logging is excluded from timing measurements.
    # A hard bound also covers blocking API calls before the cyclic deadline.
    # The subprocess context reaps the child before wrapper module cleanup.
    with subprocess.Popen(command, stdout=subprocess.PIPE,
                          stderr=None, text=True) as child:
        try:
            stdout, _ = child.communicate(timeout=seconds + 90)
        except BaseException:
            child.terminate()
            try:
                child.communicate(timeout=3)
            except subprocess.TimeoutExpired:
                child.kill()
                child.communicate()
            raise
        result["exit_code"] = child.returncode
        result["stdout"] = stdout
        lines = [line for line in stdout.splitlines() if line.startswith('{"activated":')]
        result["observation"] = json.loads(lines[-1]) if lines else None
    observation = result["observation"]
    # On AL/cycle failure IgH may be in automatic reconfiguration, even if the
    # advertised AL state is already PREOP. Do not race it with CLI uploads.
    if result["exit_code"] or not observation or not observation["observation_complete"]:
        result["post_sdo_skipped"] = "cyclic failure may leave configuration mailbox work in flight"
        raise RuntimeError("disabled cyclic observation failed; post-release SDOs skipped; inspect kernel log")
    if observation["non_op_cycles_after_op"]:
        result["post_sdo_skipped"] = "OP was lost; configuration completion is not established"
        raise RuntimeError("OP was lost during observation; post-release SDOs skipped")
    result["slave_listing_after_release"] = probe.cli("slaves")["stdout"]
    # Release returns the idle master to PREOP; serialize reads after transition.
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        listing = probe.cli("slaves")["stdout"].splitlines()
        if len(listing) == 1 and listing[0].split()[:4] == ["0", "0:0", "PREOP", "+"]:
            break
        time.sleep(0.1)
    require_single_preop()
    result["after"] = read_snapshot()
    result["pdo_objects_after"] = {index: probe.read_array(0, index, datatype, limit)
                                   for index, datatype, limit in (("0x1c12", "uint16", 4),
                                       ("0x1c13", "uint16", 4), ("0x1600", "uint32", 16),
                                       ("0x1a00", "uint32", 16))}
    result["interpolation_after"] = {
        str(sub): probe.cli("--position", 0, "--type", dtype, "upload", "0x60c2", sub)
        for sub, dtype in ((1, "uint8"), (2, "int8"))}
    result["pdo_payload_matches"] = True
    result["pdo_counts_match"] = True
    for tag, assignment in (("RxPdo", "0x1c12"), ("TxPdo", "0x1c13")):
        pdo = contract["pdos"][tag]
        assigned = result["pdo_objects_after"][assignment]
        mapped = result["pdo_objects_after"][f"0x{pdo['index']:04x}"]
        try:
            counts_match = check_pdo_readback(pdo, assigned, mapped)
        except RuntimeError:
            result["pdo_payload_matches"] = False
            raise
        if not counts_match:
            result["pdo_counts_match"] = False
            report["warnings"].append(f"{tag}: payload prefix matches, but PDO counts remain inconsistent; "
                                      "disabled timing measurement only, no motion qualification")
    interpolation = result["interpolation_after"]
    if probe.number(interpolation["1"]) != period_us // 1000 \
            or int(interpolation["2"]["stdout"].split()[-1]) != -3:
        raise RuntimeError("drive interpolation period readback does not match master")
    after = result["after"]
    if after["controlword"] != 0 or after["statusword"] & 0x4f != 0x40 or after["error_code"] != 0:
        raise RuntimeError("post-test drive is not healthy and disabled")
    if observation["incomplete_cycles_after_op"] or observation["non_op_cycles_after_op"]:
        report["warnings"].append("cyclic gaps observed; this run does not validate reliable cyclic timing")
    result["dc_within_diagnostic_bound"] = (
        observation.get("observation_samples", 0) > 0
        and observation.get("dc_max_difference_ns_observation", 0xffffffff) <= 100000
        and observation["dc_missing_after_op"] == 0)
    if not result["dc_within_diagnostic_bound"]:
        report["warnings"].append("DC monitoring exceeds the 100 us diagnostic bound or is incomplete; "
                                  "OP/WKC success is not DC synchronization qualification")
    report["warnings"].append("disabled communication only; motion/stop behavior and real-time timing not validated")


if __name__ == "__main__":
    raise SystemExit(probe.main(after_diagnostics=run_disabled, description=__doc__, configure_parser=add_arguments))
