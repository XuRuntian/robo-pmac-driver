#!/usr/bin/env python3
"""Read actual scaling, limits and stop configuration before any enable test."""
import re
import sys

import probe_connected as probe
from reset_fault_once import read_snapshot, require_single_preop


def inspect(report):
    if report["errors"] or len(report["slaves"]) != 1 or not report["slaves"][0]["identity_matches"]:
        raise RuntimeError("motion setup inspection requires one matching drive")
    require_single_preop()
    result = report["motion_setup"] = {"before": read_snapshot(), "reads": [], "writes": []}
    if result["before"]["controlword"] != 0 or result["before"]["statusword"] & 0x4f != 0x40:
        raise RuntimeError("motion setup inspection requires a disabled drive")
    dictionary = probe.cli("--position", 0, "sdos")["stdout"]
    indices = {0x100a, 0x10f1, 0x605a, 0x605b, 0x605c, 0x605d,
               0x6065, 0x6066, 0x6072, 0x6073, 0x6075, 0x6076,
               0x607b, 0x607d, 0x607f, 0x6080, 0x6083, 0x6084, 0x6085,
               0x608f, 0x6091, 0x6092, 0x6093, 0x60e0, 0x60e1,
               0x60b0, 0x60b1, 0x60b2, 0x60fd}
    found = set()
    for line in dictionary.splitlines():
        entry = re.match(r'\s*(0x[0-9a-fA-F]{4}):([0-9a-fA-F]{2}), ([rw-]{6}), '
                         r'(\w+), (\d+) bit, "(.*)"', line)
        if not entry:
            continue
        index, subindex, access, dtype, bits, name = entry.groups()
        idx, sub = int(index, 16), int(subindex, 16)
        selected = idx in indices or (idx == 0x2100 and sub in (3, 6, 0x11, 0x12, 0x13))
        if not selected or access[0] != "r" or dtype not in ("uint8", "uint16", "uint32", "int8", "int16", "int32", "string"):
            continue
        if len(result["reads"]) >= 64:
            raise RuntimeError("too many motion setup objects; review dictionary")
        reply = probe.cli("--position", 0, "--type", dtype, "upload", index, sub)
        result["reads"].append({"index": index, "subindex": sub, "name": name, "datatype": dtype,
                                "bits": int(bits), "access": access, **reply})
        found.add(idx)
        print(f"{index}:{sub:02x} {name}: {reply['stdout']}", file=sys.stderr, flush=True)
    result["objects_absent_from_dictionary"] = [f"0x{i:04x}" for i in sorted(indices - found)]
    result["after"] = read_snapshot()
    result["complete"] = True


if __name__ == "__main__":
    raise SystemExit(probe.main(after_diagnostics=inspect, description=__doc__))
