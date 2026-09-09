"""Reject ESI/ENI combinations that would silently change wire semantics."""
import copy
import json
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from check_esi import audit, ESI, ROOT


class EsiTests(unittest.TestCase):
    def setUp(self):
        self.root = ET.parse(ESI).getroot()
        self.contract = json.loads((ROOT / "config/pmac_drives.json").read_text())
        self.device = self.root.find("./Descriptions/Devices/Device")

    def object(self, index):
        return self.device.find(f"./Profile/Dictionary/Objects/Object[Index='{index}']")

    def reject(self):
        with self.assertRaises(ValueError):
            audit(self.root, self.contract)

    def test_supplied_esi_permits_custom_digital_input_mapping(self):
        r = audit(self.root, self.contract)
        self.assertEqual(r["esi_default_pdos"]["RxPdo"]["bytes"], 20)
        self.assertEqual(len(r["pmac_pdo_objects_checked"]), 9)
        self.assertEqual(r["selected_profile"]["startup_sdos"][-2]["value"], 1)

    def test_wrong_revision(self):
        self.device.find("Type").set("RevisionNo", "2")
        self.reject()

    def test_duplicate_device(self):
        self.root.find("./Descriptions/Devices").append(copy.deepcopy(self.device))
        self.reject()

    def test_mapping_disabled(self):
        self.device.find("Mailbox/CoE").set("PdoConfig", "0")
        self.reject()

    def test_fixed_mapping(self):
        self.device.find("RxPdo").set("Fixed", "1")
        self.reject()

    def test_missing_digital_inputs(self):
        self.object("#x60FD").find("Flags/PdoMapping").text = "R"
        self.reject()

    def test_wrong_signed_position(self):
        self.object("#x6064").find("Type").text = "UDINT"
        self.reject()

    def test_wrong_width(self):
        self.object("#x607A").find("BitSize").text = "16"
        self.reject()

    def test_extra_startup_sdo(self):
        self.device.find("Mailbox/CoE").append(copy.deepcopy(self.device.find("Mailbox/CoE/InitCmd")))
        self.reject()

    def test_period_mismatch(self):
        self.contract["application_period_ns"] = 2000000
        self.reject()

    def test_period_outside_drive_range(self):
        self.contract["application_period_ns"] = self.contract["dc"]["sync0_cycle_ns"] = 500000
        self.reject()

    def test_no_zero_pdo_assignment(self):
        self.object("#x1C12").find("Info/SubItem/Info/DefaultData").text = "02"
        self.reject()

    def test_changed_dc_factor_needs_review(self):
        self.device.find("Dc/OpMode/CycleTimeSync1").set("Factor", "2")
        self.reject()


if __name__ == "__main__":
    unittest.main()
