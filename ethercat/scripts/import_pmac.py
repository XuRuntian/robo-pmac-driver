#!/usr/bin/env python3
"""Extract the five-drive contract from the PMAC ENI; never access hardware."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PMAC = ROOT.parent.parent / "surgical_continuum_robot_pmac" / "PowerPMAC"


def number(text):
    if text is None:
        raise ValueError("missing numeric XML field")
    return int(text[2:], 16) if text.startswith("#x") else int(text)


def require(ok, message):
    if not ok:
        raise ValueError(message)


def import_contract(pmac):
    eni = pmac / "Configuration/eni.xml"
    cfg = pmac / "Configuration/ECATConfig.cfg"
    modes = pmac / "PMAC Script Language/Global Includes/global definitions.pmh"
    root = ET.parse(eni).getroot()
    slaves = root.findall("./Config/Slave")
    require(len(slaves) == 5, "expected exactly five ENI slaves")
    # The cfg Position fields are all zero: use ENI AutoIncAddr for ring positions.
    text = cfg.read_text()
    io = {}
    for slot, key, value in re.findall(r"ECAT\[0\]\.IO\[(\d+)\]\.(\w+)=(\d+)", text):
        io.setdefault(int(slot), {})[key] = int(value)
    require(len(io) == 45, "expected 45 non-padding process entries")
    mode_text = modes.read_bytes().decode("gb18030")
    cycle = root.findtext("./Config/Cyclic/CycleTime")
    require(cycle is not None, "missing ENI application period")
    out = {
        "schema_version": 1,
        "hardware_validated": False,
        "source_sha256": {
            str(p.relative_to(pmac)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in (eni, cfg)
        },
        "application_period_ns": number(cycle) * 1000,
        "operation_mode": 8,
        "slaves": [],
        "live_validation_required": [
            "actual slave order, identity and revision",
            "ESI, supported PDO mapping and startup SDO requirements",
            "DC Sync0/Sync1 meaning, phase and measured synchronization",
            "drive-side watchdog, stop behavior and physical emergency stop",
            "raw counts, axis directions, offsets and travel limits",
        ],
    }
    common_pdos = common_dc = None
    for pos, slave in enumerate(slaves):
        info = slave.find("Info")
        require((-number(info.findtext("AutoIncAddr"))) % 65536 == pos,
                "unexpected ENI physical order")
        identity = {"position": pos, "alias": 0,
                    "vendor_id": number(info.findtext("VendorId")),
                    "product_code": number(info.findtext("ProductCode")),
                    "revision": number(info.findtext("RevisionNo")),
                    "pmac_station_address": number(info.findtext("PhysAddr")),
                    "name": info.findtext("Name")}
        for field, key in (("VendorID", "vendor_id"), ("ProductCode", "product_code"),
                           ("Alias", "alias"), ("StationAddress", "pmac_station_address")):
            match = re.search(rf"ECAT\[0\]\.Slave\[{pos}\]\.{field}=(\$[0-9a-fA-F]+|\d+)", text)
            require(match is not None, f"missing cfg {field}")
            value = match[1]
            require((int(value[1:], 16) if value.startswith("$") else int(value)) == identity[key],
                    f"ENI/cfg identity mismatch on slave {pos}: {field}")
        pdos = {}
        for tag, direction, sm, area in (("RxPdo", 0, 2, "Send"), ("TxPdo", 1, 3, "Recv")):
            proc = slave.find("ProcessData")
            active = [p for p in proc.findall(tag) if "Sm" in p.attrib]
            require(len(active) == 1 and int(active[0].attrib["Sm"]) == sm,
                    f"expected one {tag} on SM{sm}")
            pdo = active[0]
            entries = []
            offset = 0
            base = number(proc.findtext(f"{area}/BitStart"))
            for e in pdo.findall("Entry"):
                index = number(e.findtext("Index"))
                sub = number(e.findtext("SubIndex", "0"))
                bits = number(e.findtext("BitLen"))
                require(bits in (8, 16, 32) and offset % 8 == 0, "unsupported bit layout")
                entries.append({"index": index, "subindex": sub, "bits": bits,
                                "byte_offset": offset // 8,
                                "name": e.findtext("Name", "padding"),
                                "data_type": e.findtext("DataType", "padding")})
                if index:
                    matches = [(slot, v) for slot, v in io.items()
                               if v.get("Slave") == pos and v.get("Input") == direction
                               and v.get("Index") == index and v.get("SubIndex") == sub]
                    require(len(matches) == 1, f"missing/duplicate cfg object {index:#x} on {pos}")
                    slot, v = matches[0]
                    require(v["BitLength"] == bits and v["BitPosition"] == 0
                            and v["Offset"] * 8 == base + offset,
                            f"ENI/cfg layout mismatch: slave {pos} object {index:#x}")
                    if index == 0x6060:
                        require(re.search(rf"ECAT\[0\]\.IO\[{slot}\]\.Data\s*=\s*8\b", mode_text),
                                "expected CSP mode 8 on each drive")
                offset += bits
            require(offset == number(proc.findtext(f"{area}/BitLength")), "PDO length mismatch")
            pdos[tag] = {"sm": sm, "index": number(pdo.findtext("Index")),
                         "bytes": offset // 8, "entries": entries}
        require(common_pdos is None or common_pdos == pdos, "drive PDO layouts differ")
        common_pdos = pdos
        # Preserve ESC register values, not a guessed ESI Sync1 period.
        writes = [c for c in slave.findall("./InitCmds/InitCmd")
                  if "PS" in [t.text for t in c.findall("Transition")]
                  and c.findtext("Cmd") == "5"]
        def register(address, length):
            values = [bytes.fromhex(c.findtext("Data", "")) for c in writes
                      if number(c.findtext("Ado")) == address]
            require(len(values) == 1 and len(values[0]) == length,
                    f"missing/ambiguous DC register {address:#x}")
            return values[0]
        cycles = register(0x9A0, 8)
        dc = {"assign_activate": int.from_bytes(register(0x980, 2), "little"),
              "sync0_cycle_ns": int.from_bytes(cycles[:4], "little"),
              "sync1_register_ns": int.from_bytes(cycles[4:], "little"),
              "sync0_shift_ns": number(slave.findtext("DC/ShiftTime"))}
        require(dc["sync0_cycle_ns"] == number(slave.findtext("DC/CycleTime0"))
                and dc["sync1_register_ns"] == number(slave.findtext("DC/CycleTime1")),
                "DC XML/register mismatch")
        require(common_dc is None or common_dc == dc, "drive DC parameters differ")
        common_dc = dc
        identity["reference_clock"] = bool(number(slave.findtext("DC/ReferenceClock")))
        out["slaves"].append(identity)
        for c in slave.findall("./Mailbox/CoE/InitCmds/InitCmd"):
            idx = number(c.findtext("Index"))
            require(0x1600 <= idx <= 0x17FF or 0x1A00 <= idx <= 0x1BFF
                    or 0x1C10 <= idx <= 0x1C2F,
                    f"extra startup SDO {idx:#x} requires explicit migration")
    require(sum(s["reference_clock"] for s in out["slaves"]) == 1, "expected one DC reference")
    out["pdos"] = common_pdos
    out["dc"] = common_dc
    return out


def header(contract):
    lines = ["// Generated by import_pmac.py from the checked PMAC ENI contract.",
             "#pragma once", "#include <ecrt.h>", "#include <array>",
             "namespace continuum::drive_config {",
             f"inline constexpr unsigned period_ns = {contract['application_period_ns']};",
             f"inline constexpr int mode = {contract['operation_mode']};",
             "struct Identity { unsigned short alias, position; unsigned vendor, product, revision; bool reference; };",
             "inline constexpr std::array<Identity, 5> slaves{{"]
    for s in contract["slaves"]:
        lines.append(f"  {{{s['alias']}, {s['position']}, {s['vendor_id']:#x}, {s['product_code']:#x}, "
                     f"{s['revision']}, {str(s['reference_clock']).lower()}}},")
    lines.append("}};")
    for name, pdo in contract["pdos"].items():
        var = "rx" if name == "RxPdo" else "tx"
        lines.append(f"inline ec_pdo_entry_info_t {var}_entries[] = {{")
        for e in pdo["entries"]:
            lines.append(f"  {{{e['index']:#06x}, {e['subindex']}, {e['bits']}}},")
        lines += ["};", f"inline ec_pdo_info_t {var}_pdo[] = {{{{{pdo['index']:#06x}, "
                  f"{len(pdo['entries'])}, {var}_entries}}}};"]
    lines += ["inline ec_sync_info_t syncs[] = {",
              "  {0, EC_DIR_OUTPUT, 0, nullptr, EC_WD_DISABLE},",
              "  {1, EC_DIR_INPUT, 0, nullptr, EC_WD_DISABLE},",
              "  {2, EC_DIR_OUTPUT, 1, rx_pdo, EC_WD_ENABLE},",
              "  {3, EC_DIR_INPUT, 1, tx_pdo, EC_WD_DISABLE},",
              "  {0xff, EC_DIR_INVALID, 0, nullptr, EC_WD_DEFAULT}", "};"]
    for k, v in contract["dc"].items():
        lines.append(f"inline constexpr auto {k} = {v};")
    lines += ["} // namespace continuum::drive_config", ""]
    return "\n".join(lines)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--pmac", type=Path, default=DEFAULT_PMAC)
    p.add_argument("--output", type=Path, default=ROOT / "config/pmac_drives.json")
    p.add_argument("--check", action="store_true")
    p.add_argument("--header", type=Path)
    args = p.parse_args()
    contract = import_contract(args.pmac)
    encoded = json.dumps(contract, indent=2, ensure_ascii=False) + "\n"
    if args.check:
        require(args.output.read_text() == encoded, "saved contract differs; regenerate and review")
    else:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded)
    if args.header:
        args.header.parent.mkdir(parents=True, exist_ok=True)
        args.header.write_text(header(contract))
    print("PMAC contract: 5 slaves, RxPDO 10 bytes, TxPDO 14 bytes; ENI/cfg consistent")


if __name__ == "__main__":
    main()
