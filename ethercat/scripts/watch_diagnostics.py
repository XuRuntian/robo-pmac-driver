#!/usr/bin/env python3
"""Read one Diamond drive's diagnostic SDOs for 10 seconds; no drive writes."""
import sys
import time

import probe_connected as probe


def watch(report):
    if report["errors"] or len(report["slaves"]) != 1 or not report["slaves"][0]["identity_matches"]:
        raise RuntimeError("watch requires successful diagnostics of one matching drive")
    result = report["watch"] = {"samples": [], "complete": False}
    started = time.monotonic()
    for i in range(11):
        time.sleep(max(0, started + i - time.monotonic()))
        sample = {"seconds": round(time.monotonic() - started, 3), "sdo": {}}
        result["samples"].append(sample)
        for name, index, datatype in (("controlword", "0x6040", "uint16"),
                                     ("statusword", "0x6041", "uint16"),
                                     ("error_code", "0x603f", "uint16"),
                                     ("actual_position", "0x6064", "int32"),
                                     ("mode_display", "0x6061", "int8")):
            reply = probe.cli("--position", 0, "--type", datatype, "upload", index, 0, required=False)
            sample["sdo"][name] = reply
            if reply["exit_code"]:
                raise RuntimeError(f"watch SDO {index}:0 failed: {reply['stderr']}")
        print(f"Read-only sample {i}: " + ", ".join(
            f"{key}={value['stdout']}" for key, value in sample["sdo"].items()),
            file=sys.stderr, flush=True)
    result["complete"] = True


if __name__ == "__main__":
    raise SystemExit(probe.main(after_diagnostics=watch, description=__doc__))
