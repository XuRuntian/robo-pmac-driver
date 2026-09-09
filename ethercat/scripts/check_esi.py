#!/usr/bin/env python3
"""Audit the supplied Diamond ESI against the PMAC contract, without hardware.

ESI object defaults are descriptive, NOT a parameter download list. The selected
profile retains PMAC PDOs/DC and explicitly sets interpolation time to 1 ms.
"""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

from import_pmac import ROOT, number, require

ESI = ROOT / "config/esi/ISMC_STANDARD_165_默认DC_20220720.xml"
REVIEW = ROOT / "config/esi_review.json"


def audit(root, contract):
    require(root.tag == "EtherCATInfo", "expected manufacturer EtherCATInfo ESI")
    devices = root.findall("./Descriptions/Devices/Device")
    require(len(devices) == 1, "expected one unambiguous ESI device")
    device = devices[0]
    kind = device.find("Type")
    identity = {"vendor_id": number(root.findtext("./Vendor/Id")),
                "product_code": number(kind.get("ProductCode")),
                "revision": number(kind.get("RevisionNo"))}
    for slave in contract["slaves"]:
        require(all(slave[k] == v for k, v in identity.items()), "ESI/PMAC identity mismatch")
    coe = device.find("Mailbox/CoE")
    require(coe is not None and coe.get("PdoAssign") == "1" and coe.get("PdoConfig") == "1",
            "ESI does not permit required PDO assignment/remapping")
    objects = {number(o.findtext("Index")): o
               for o in device.findall("./Profile/Dictionary/Objects/Object")}
    types = {t.findtext("Name"): t for t in device.findall("./Profile/Dictionary/DataTypes/DataType")}
    defaults = {}
    checked = []
    assignments = {}
    for tag, direction, assignment in (("RxPdo", "R", 0x1c12), ("TxPdo", "T", 0x1c13)):
        selected = contract["pdos"][tag]
        pdos = [p for p in device.findall(tag) if p.get("Sm") == str(selected["sm"])]
        require(len(pdos) == 1, f"ambiguous {tag} SM assignment")
        pdo = pdos[0]
        require(pdo.get("Fixed") == "0" and number(pdo.findtext("Index")) == selected["index"],
                f"{tag} is fixed or has a different mapping index")
        default_entries = [{"index": number(e.findtext("Index")),
                            "subindex": number(e.findtext("SubIndex", "0")),
                            "bits": number(e.findtext("BitLen"))} for e in pdo.findall("Entry")]
        defaults[tag] = {"index": selected["index"], "sm": selected["sm"],
                         "bytes": sum(e["bits"] for e in default_entries) // 8,
                         "entries": default_entries}
        values = [int.from_bytes(bytes.fromhex(n.text), "little")
                  for n in objects[assignment].findall("Info/SubItem/Info/DefaultData")]
        require(values[:2] == [1, selected["index"]], f"unexpected {assignment:#x} ESI assignment")
        assignments[f"0x{assignment:04x}"] = {"count": values[0], "active_pdos": values[1:2],
                                             "unused_slots": values[2:]}
        for entry in selected["entries"]:
            index = entry["index"]
            if index == 0:
                require(entry["subindex"] == 0 and entry["bits"] == 8, "invalid PDO padding")
                continue
            obj = objects.get(index)
            require(obj is not None and entry["subindex"] == 0, f"missing object {index:#x}")
            require(number(obj.findtext("BitSize")) == entry["bits"], f"width mismatch {index:#x}")
            require(obj.findtext("Type") == entry["data_type"], f"signed/type mismatch {index:#x}")
            require(direction in obj.findtext("Flags/PdoMapping", ""), f"PDO direction denied {index:#x}")
            require("r" in obj.findtext("Flags/Access", "") and
                    (direction == "T" or "w" in obj.findtext("Flags/Access", "")),
                    f"object access denied {index:#x}")
            checked.append({"index": index, "direction": direction, "bits": entry["bits"],
                            "type": entry["data_type"]})
    startup = []
    for cmd in coe.findall("InitCmd"):
        startup.append({"transition": cmd.findtext("Transition"),
                        "index": number(cmd.findtext("Index")),
                        "subindex": number(cmd.findtext("SubIndex")),
                        "data_hex": bytes.fromhex(cmd.findtext("Data")).hex()})
    require(startup == [dict(transition="PS", index=0x6060, subindex=0, data_hex="08"),
                        dict(transition="PS", index=0x60c2, subindex=1, data_hex="02")],
            "ESI startup commands changed; review migration explicitly")
    period_type = types[objects[0x60c2].findtext("Type")]
    for sub, dtype in ((1, "USINT"), (2, "SINT")):
        fields = [n for n in period_type.findall("SubItem") if n.findtext("SubIdx") == str(sub)]
        require(len(fields) == 1 and fields[0].findtext("Type") == dtype
                and fields[0].findtext("BitSize") == "8"
                and fields[0].findtext("Flags/Access") == "rw", "60C2 period fields not writable bytes")
    period_defaults = [n.text.upper() for n in objects[0x60c2].findall("Info/SubItem/Info/DefaultData")]
    require(period_defaults == ["02", "04", "FD"], "unexpected interpolation defaults/exponent")
    dc_modes = []
    for mode in device.findall("Dc/OpMode"):
        item = {"name": mode.findtext("Name"), "assign_activate": number(mode.findtext("AssignActivate"))}
        for key in ("CycleTimeSync0", "CycleTimeSync1"):
            item[key] = {"value": number(mode.findtext(key)), "factor": number(mode.find(key).get("Factor"))}
        item["sync0_shift_ns"] = number(mode.findtext("ShiftTimeSync0", "0"))
        item["sync1_shift_ns"] = number(mode.findtext("ShiftTimeSync1", "0"))
        dc_modes.append(item)
    matching_dc = [m for m in dc_modes if m["assign_activate"] == contract["dc"]["assign_activate"]]
    require(len(matching_dc) == 1, "DC activation unsupported or ambiguous in ESI")
    dc = matching_dc[0]
    require(all(dc[k] == {"value": 0, "factor": 1} for k in ("CycleTimeSync0", "CycleTimeSync1"))
            and dc["sync0_shift_ns"] == dc["sync1_shift_ns"] == 0,
            "ESI DC factors/shifts changed; review conversion")
    period = contract["application_period_ns"]
    require(period == contract["dc"]["sync0_cycle_ns"] and period % 1000000 == 0
            and 1 <= period // 1000000 <= 4, "master/Sync0/interpolation period mismatch or outside 1-4 ms")
    require(contract["operation_mode"] == 8, "expected CSP mode")
    return {"schema_version": 1, "identity": identity, "coe_capabilities": dict(coe.attrib),
            "esi_default_pdos": defaults, "esi_default_assignments": assignments,
            "pmac_pdo_objects_checked": checked, "esi_startup_sdos": startup,
            "esi_interpolation_default_ms": 4, "esi_startup_interpolation_ms": 2,
            "esi_dc_modes": dc_modes,
            "esi_dc_resolved_at_selected_period": {"assign_activate": dc["assign_activate"],
                "sync0_cycle_ns": period, "sync0_shift_ns": 0, "sync1_register_ns": 0,
                "derivation": "Pinned ecrt.h: T1 = Sync0 * Factor(1); API sync1 = T1 - Sync0 + shift(0). "
                              "ESI default differs from PMAC's 500000 ns register value; "
                              "the initial disabled probe preserves PMAC for comparison."},
            "selected_profile": {"pdo_source": "pmac_drives.json", "dc": contract["dc"],
                "application_period_ns": period,
                "startup_sdos": [dict(index=0x6040, subindex=0, bits=16, value=0),
                                 dict(index=0x6060, subindex=0, bits=8, value=8),
                                 dict(index=0x60c2, subindex=1, bits=8, value=period // 1000000),
                                 dict(index=0x60c2, subindex=2, bits=8, value=-3)],
                "note": "Retain PMAC PDO/DC register values; override ESI 2 ms startup to match master. "
                        "Only volatile communication parameters; no encoder/motor defaults or flash save."}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--header", type=Path)
    args = parser.parse_args()
    contract_file = ROOT / "config/pmac_drives.json"
    review = audit(ET.parse(ESI).getroot(), json.loads(contract_file.read_text()))
    review["source_sha256"] = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                               for p in (ESI, contract_file)}
    encoded = json.dumps(review, indent=2, ensure_ascii=False) + "\n"
    if args.check:
        require(REVIEW.read_text() == encoded, "ESI review changed; regenerate and inspect")
    else:
        REVIEW.write_text(encoded)
    if args.header:
        args.header.parent.mkdir(parents=True, exist_ok=True)
        period = review["selected_profile"]["application_period_ns"] // 1000000
        args.header.write_text("// Generated by check_esi.py; reviewed ESI/PMAC profile.\n#pragma once\n"
                              "namespace continuum::esi_config {\n"
                              f"inline constexpr unsigned period_ms = {period};\n"
                              "inline constexpr unsigned time_index_byte = 0xfd;\n}\n")
    print(f"ESI matches Diamond identity; 9 PMAC PDO objects compatible; "
          f"explicit {review['selected_profile']['application_period_ns'] // 1000000} ms startup selected")


if __name__ == "__main__":
    main()
