"""Preflight failures must never start the process that configures the drive."""
import copy
import json
import unittest
from unittest.mock import patch

import probe_disabled as subject


class DisabledScopeTests(unittest.TestCase):
    def setUp(self):
        self.report = {"errors": [], "slaves": [{"position": 0, "identity_matches": True}]}
        self.snapshot = {"controlword": 0, "statusword": 0x0240, "error_code": 0}

    def reject_without_starting(self):
        with patch.object(subject, "require_single_preop"), \
                patch.object(subject, "read_snapshot", return_value=self.snapshot), \
                patch.object(subject.probe, "run"), patch.object(subject.subprocess, "Popen") as start:
            with self.assertRaises(RuntimeError):
                subject.run_disabled(self.report)
            start.assert_not_called()
            self.assertFalse(self.report["disabled_cyclic"]["started"])

    def test_multiple_drives(self):
        self.report["slaves"].append(copy.deepcopy(self.report["slaves"][0]))
        self.reject_without_starting()

    def test_diagnostic_failure(self):
        self.report["errors"].append("mailbox failed")
        self.reject_without_starting()

    def test_identity_mismatch(self):
        self.report["slaves"][0]["identity_matches"] = False
        self.reject_without_starting()

    def test_existing_enable_request(self):
        self.snapshot["controlword"] = 0x000f
        self.reject_without_starting()

    def test_existing_enabled_state(self):
        self.snapshot["statusword"] = 0x0027
        self.reject_without_starting()

    def test_fault_code_even_if_status_says_disabled(self):
        self.snapshot["error_code"] = 0x738a
        self.reject_without_starting()

    def test_cyclic_failure_never_starts_post_release_uploads(self):
        observation = {"activated": 1, "observation_complete": 0}
        with patch.object(subject, "require_single_preop"), \
                patch.object(subject, "read_snapshot", return_value=self.snapshot) as snapshot, \
                patch.object(subject.probe, "run"), patch.object(subject.probe, "cli") as cli, \
                patch("pathlib.Path.is_file", return_value=True), \
                patch.object(subject.subprocess, "Popen") as start:
            child = start.return_value.__enter__.return_value
            child.communicate.return_value = (json.dumps(observation) + "\n", None)
            child.returncode = 1
            with self.assertRaisesRegex(RuntimeError, "post-release SDOs skipped"):
                subject.run_disabled(self.report)
            self.assertEqual(snapshot.call_count, 1)
            cli.assert_not_called()


class PdoReadbackTests(unittest.TestCase):
    @staticmethod
    def array(values):
        def reply(value):
            return {"exit_code": 0, "stdout": f"{value:#x} {value}", "stderr": ""}
        return {"complete": True, "count": reply(len(values)),
                "entries": {str(i): reply(v) for i, v in enumerate(values, 1)}}

    def setUp(self):
        self.pdo = json.loads((subject.probe.ROOT / "config/pmac_drives.json").read_text())["pdos"]["RxPdo"]
        self.entries = [(e["index"] << 16) | e["bits"] for e in self.pdo["entries"]]

    def test_standard_counts(self):
        self.assertTrue(subject.check_pdo_readback(self.pdo, self.array([0x1600]), self.array(self.entries)))

    def test_observed_count_anomaly_is_never_marked_conformant(self):
        self.assertFalse(subject.check_pdo_readback(self.pdo, self.array([0x1600, 0]),
                                                   self.array(self.entries + [0x60710010, 0x60600008, 8, 0, 0])))

    def test_extra_active_pdo_is_rejected(self):
        with self.assertRaises(RuntimeError):
            subject.check_pdo_readback(self.pdo, self.array([0x1600, 0x1601]), self.array(self.entries))

    def test_changed_payload_is_rejected(self):
        self.entries[0], self.entries[1] = self.entries[1], self.entries[0]
        with self.assertRaises(RuntimeError):
            subject.check_pdo_readback(self.pdo, self.array([0x1600]), self.array(self.entries))

    def test_incomplete_read_is_rejected(self):
        mapped = self.array(self.entries); mapped["complete"] = False
        with self.assertRaises(RuntimeError):
            subject.check_pdo_readback(self.pdo, self.array([0x1600]), mapped)


if __name__ == "__main__":
    unittest.main()
