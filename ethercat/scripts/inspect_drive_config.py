#!/usr/bin/env python3
"""Read one Diamond drive's identity and encoder-related settings; no writes."""
import re
import sys

import probe_connected as probe


def inspect(report):
    if report["errors"] or len(report["slaves"]) != 1 or not report["slaves"][0]["identity_matches"]:
        raise RuntimeError("configuration inspection requires one matching drive")
    listing = probe.cli("--position", 0, "sdos")["stdout"]
    result = report["configuration"] = {"dictionary": listing, "reads": []}
    parent = ""
    for line in listing.splitlines():
        if line.startswith("SDO "):
            parent = line
        entry = re.match(r'\s*(0x[0-9a-fA-F]{4}):([0-9a-fA-F]{2}), ([rw-]{6}), '
                         r'(\w+), (\d+) bit, "(.*)"', line)
        if not entry:
            continue
        index, subindex, access, datatype, _, name = entry.groups()
        idx, sub = int(index, 16), int(subindex, 16)
        selected = idx in (0x1000, 0x1001, 0x1008, 0x1009, 0x100a, 0x1018, 0x2004, 0x2005)
        selected |= 0x2000 <= idx <= 0x2600 and bool(re.search(
            r'encoder|feedback.*(?:type|select)|hall.*(?:mode|select)|master.*type', parent + name, re.I))
        if not selected or access[0] != "r" or datatype not in (
                "uint8", "uint16", "uint32", "int8", "int16", "int32", "string", "bool"):
            continue
        if len(result["reads"]) >= 64:
            raise RuntimeError("more than 64 candidate settings; inspect dictionary before more reads")
        value = probe.cli("--position", 0, "--type", datatype, "upload", index, sub, required=False)
        result["reads"].append({"index": index, "subindex": sub, "name": name,
                                "object": parent, "datatype": datatype, "access": access, **value})
        if value["exit_code"]:
            report["warnings"].append(f"configuration {index}:{sub:02x} read failed")
        print(f"{index}:{sub:02x} {name}: {value['stdout'] or value['stderr']}",
              file=sys.stderr, flush=True)


if __name__ == "__main__":
    raise SystemExit(probe.main(after_diagnostics=inspect, description=__doc__))
