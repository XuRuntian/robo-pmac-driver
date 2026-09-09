#!/usr/bin/env python3
"""Single-drive commissioning: one CiA402 fault-reset pulse, never enable.

Use only with the motor/encoder connected and the original master disconnected.
The normal probe remains read-only. This separate entry point writes only
6040:00 = 0, 128, 0 in PREOP, then observes feedback for approximately 10 seconds.
No PDO configuration, mode, target, flash parameter or enable command is written.
Exit 0 means the procedure completed; inspect fault_reset for the drive result.
"""
import sys
import time

import probe_connected as probe


def read_snapshot():
    fields = (("controlword", "0x6040", "uint16"),
              ("statusword", "0x6041", "uint16"),
              ("error_code", "0x603f", "uint16"),
              ("actual_position", "0x6064", "int32"),
              ("mode_display", "0x6061", "int8"))
    snapshot = {}
    for name, index, datatype in fields:
        reply = probe.cli("--position", 0, "--type", datatype, "upload", index, 0)
        # Numeric uploads show hexadecimal first, signed decimal last.
        snapshot[name] = int(reply["stdout"].split()[-1], 10)
    return snapshot


def require_single_preop():
    lines = probe.cli("slaves")["stdout"].splitlines()
    if len(lines) != 1 or lines[0].split()[:4] != ["0", "0:0", "PREOP", "+"]:
        raise RuntimeError("fault reset requires exactly one error-free PREOP slave at position 0")


def reset_once(report):
    reset = {"writes": [], "samples": [], "enable_commands_sent": False,
             "pulse_completed": False, "observation_complete": False}
    report["fault_reset"] = reset
    if report["errors"] or len(report["slaves"]) != 1:
        raise RuntimeError("fault reset requires successful diagnostics of exactly one slave")
    drive = report["slaves"][0]
    if drive["position"] != 0 or not drive["identity_matches"]:
        raise RuntimeError("fault reset requires a matching Diamond drive at position 0")
    require_single_preop()
    before = reset["before"] = read_snapshot()
    if (before["statusword"] & 0x004f) != 0x0008:
        reset["skipped"] = "drive is not in CiA402 Fault"
        return
    if before["controlword"] & 0x000b:
        raise RuntimeError("existing controlword requests power/enable; refusing reset")

    def write(value):
        if value not in (0x0000, 0x0080):
            raise ValueError("only disabled and fault-reset controlwords are permitted")
        require_single_preop()
        record = {"index": "0x6040", "subindex": 0, "value": value}
        reset["writes"].append(record)
        reply = probe.cli("--position", 0, "--type", "uint16", "download",
                          "0x6040", 0, value, required=False)
        record.update(reply)
        if reply["exit_code"]:
            raise RuntimeError(f"controlword {value:#06x} write failed: {reply['stderr']}")

    write(0x0000)
    time.sleep(0.1)
    try:
        # A timed-out write may have reached the drive: attempt falling edge in finally.
        write(0x0080)
        time.sleep(0.1)
    finally:
        write(0x0000)
    reset["pulse_completed"] = True
    started = time.monotonic()
    for i in range(11):
        if i:
            time.sleep(max(0, started + i - time.monotonic()))
        require_single_preop()
        snapshot = read_snapshot()
        snapshot["seconds_after_pulse"] = round(time.monotonic() - started, 3)
        reset["samples"].append(snapshot)
        print(f"Reset observation {i}: status={snapshot['statusword']:#06x}, "
              f"error={snapshot['error_code']:#06x}, position={snapshot['actual_position']}",
              file=sys.stderr, flush=True)
        if snapshot["controlword"] != 0 or (snapshot["statusword"] & 0x006f) == 0x0027:
            # Only a disable value is allowed if unexpected external control is detected.
            write(0x0000)
            raise RuntimeError("unexpected controlword or enabled state during observation")
    reset["observation_complete"] = True
    reset["all_samples_disabled_and_error_free"] = all(
        (s["statusword"] & 0x004f) == 0x0040 and s["error_code"] == 0
        for s in reset["samples"])
    reset["final"] = reset["samples"][-1]


if __name__ == "__main__":
    raise SystemExit(probe.main(after_diagnostics=reset_once, description=__doc__))
