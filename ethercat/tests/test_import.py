#!/usr/bin/env python3
import importlib.util
from pathlib import Path
import shutil
import tempfile
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("import_pmac", ROOT / "scripts/import_pmac.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class ImportTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.pmac = Path(self.tmp.name)
        for relative in ("Configuration/eni.xml", "Configuration/ECATConfig.cfg",
                         "PMAC Script Language/Global Includes/global definitions.pmh"):
            target = self.pmac / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(module.DEFAULT_PMAC / relative, target)
        self.eni = self.pmac / "Configuration/eni.xml"
        self.cfg = self.pmac / "Configuration/ECATConfig.cfg"

    def mutate_xml(self, change):
        tree = ET.parse(self.eni)
        change(tree.getroot())
        tree.write(self.eni, encoding="utf-8", xml_declaration=True)

    def reject(self):
        with self.assertRaises(ValueError):
            module.import_contract(self.pmac)

    def test_known_layout_padding_and_esc_register_values(self):
        result = module.import_contract(self.pmac)
        self.assertEqual([s["position"] for s in result["slaves"]], list(range(5)))
        self.assertEqual([s["alias"] for s in result["slaves"]], [0]*5)
        self.assertEqual(result["pdos"]["RxPdo"]["bytes"], 10)
        self.assertEqual(result["pdos"]["TxPdo"]["bytes"], 14)
        self.assertEqual(result["pdos"]["TxPdo"]["entries"][-1]["byte_offset"], 10)
        self.assertEqual(result["dc"]["assign_activate"], 0x0700)
        self.assertEqual(result["dc"]["sync1_register_ns"], 500000)
        self.assertFalse(result["hardware_validated"])

    def test_missing_slave(self):
        self.mutate_xml(lambda r: r.find("Config").remove(r.find("./Config/Slave")))
        self.reject()

    def test_wrong_identity(self):
        self.cfg.write_text(self.cfg.read_text().replace("VendorID=$418108", "VendorID=$418109", 1))
        self.reject()

    def test_wrong_eni_ring_order(self):
        self.mutate_xml(lambda r: setattr(r.find("./Config/Slave/Info/AutoIncAddr"), "text", "65535"))
        self.reject()

    def test_cfg_byte_offset_mismatch(self):
        self.cfg.write_text(self.cfg.read_text().replace("IO[4100].Offset=10", "IO[4100].Offset=9"))
        self.reject()

    def test_missing_padding(self):
        def change(r):
            pdo = r.find("./Config/Slave/ProcessData/TxPdo")
            pdo.remove(pdo.findall("Entry")[4])
        self.mutate_xml(change)
        self.reject()

    def test_dc_register_xml_mismatch(self):
        self.mutate_xml(lambda r: setattr(r.find("./Config/Slave/DC/CycleTime1"), "text", "0"))
        self.reject()

    def test_extra_startup_sdo_requires_migration(self):
        def change(r):
            commands = r.find("./Config/Slave/Mailbox/CoE/InitCmds")
            cmd = ET.SubElement(commands, "InitCmd")
            ET.SubElement(cmd, "Index").text = "24688"  # 0x6070, not PDO mapping
        self.mutate_xml(change)
        self.reject()

    def test_mode_mismatch(self):
        path = self.pmac / "PMAC Script Language/Global Includes/global definitions.pmh"
        path.write_bytes(path.read_bytes().replace(b"IO[3].Data = 8", b"IO[3].Data = 9"))
        self.reject()


if __name__ == "__main__":
    unittest.main(verbosity=2)
