"""Verify reset scope and failure behavior without a master or hardware."""
import contextlib
import copy
import io
import unittest
from unittest.mock import patch

import reset_fault_once as reset


class FakeDrive:
    def __init__(self):
        self.controlword = 0
        self.statusword = 0x0208
        self.error_code = 0x738a
        self.downloads = []
        self.fail_high = False
        self.clear_on_reset = True
        self.listing = "0  0:0  PREOP  +  Diamond EtherCAT Drive"
        self.fail_observation = False

    def cli(self, *args, **kwargs):
        if args == ("slaves",):
            return {"stdout": self.listing, "exit_code": 0}
        if args[:4] not in (("--position", 0, "--type", "uint16"),
                           ("--position", 0, "--type", "int32"),
                           ("--position", 0, "--type", "int8")):
            raise AssertionError(f"unexpected target/type: {args}")
        if args[4] == "download":
            assert args[5:7] == ("0x6040", 0)
            value = args[7]
            self.downloads.append(value)
            self.controlword = value
            if value == 128:
                if self.fail_high:
                    return {"exit_code": 124, "stdout": "", "stderr": "timeout after write"}
                if self.clear_on_reset:
                    self.statusword, self.error_code = 0x0240, 0
            return {"exit_code": 0, "stdout": "", "stderr": ""}
        assert args[4] == "upload" and args[6] == 0
        if self.fail_observation and len(self.downloads) == 3:
            raise RuntimeError("SDO upload failed during observation")
        value = {"0x6040": self.controlword, "0x6041": self.statusword,
                 "0x603f": self.error_code, "0x6064": -1234, "0x6061": 0}[args[5]]
        return {"exit_code": 0, "stdout": f"hex {value}", "stderr": ""}


class ResetTests(unittest.TestCase):
    def setUp(self):
        self.drive = FakeDrive()
        self.report = {"errors": [], "slaves": [{"position": 0, "identity_matches": True}]}

    def run_reset(self):
        with patch.object(reset.probe, "cli", self.drive.cli), \
                patch.object(reset.time, "sleep"), patch.object(reset.time, "monotonic", return_value=0), \
                contextlib.redirect_stderr(io.StringIO()):
            reset.reset_once(self.report)

    def test_single_pulse_and_signed_feedback(self):
        self.run_reset()
        self.assertEqual(self.drive.downloads, [0, 128, 0])
        result = self.report["fault_reset"]
        self.assertTrue(result["all_samples_disabled_and_error_free"])
        self.assertEqual(len(result["samples"]), 11)
        self.assertEqual(result["final"]["actual_position"], -1234)

    def test_multiple_or_unknown_drives_never_written(self):
        for slaves in ([self.report["slaves"][0]] * 2,
                       [{"position": 0, "identity_matches": False}]):
            with self.subTest(slaves=slaves):
                self.report["slaves"] = copy.deepcopy(slaves)
                with self.assertRaises(RuntimeError):
                    self.run_reset()
                self.assertEqual(self.drive.downloads, [])

    def test_no_fault_never_written(self):
        self.drive.statusword = 0x0240
        self.run_reset()
        self.assertEqual(self.drive.downloads, [])

    def test_existing_enable_request_never_written(self):
        self.drive.controlword = 15
        with self.assertRaises(RuntimeError):
            self.run_reset()
        self.assertEqual(self.drive.downloads, [])

    def test_uncertain_high_write_still_clears_pulse(self):
        self.drive.fail_high = True
        with self.assertRaises(RuntimeError):
            self.run_reset()
        self.assertEqual(self.drive.downloads, [0, 128, 0])
        self.assertEqual(self.drive.controlword, 0)

    def test_persistent_fault_not_retried(self):
        self.drive.clear_on_reset = False
        self.run_reset()
        self.assertEqual(self.drive.downloads, [0, 128, 0])
        self.assertFalse(self.report["fault_reset"]["all_samples_disabled_and_error_free"])

    def test_bus_state_change_prevents_writes(self):
        self.drive.listing = "0  0:0  OP  +  Diamond EtherCAT Drive"
        with self.assertRaises(RuntimeError):
            self.run_reset()
        self.assertEqual(self.drive.downloads, [])

    def test_observation_failure_never_repeats_reset(self):
        self.drive.fail_observation = True
        with self.assertRaises(RuntimeError):
            self.run_reset()
        self.assertEqual(self.drive.downloads, [0, 128, 0])
        self.assertFalse(self.report["fault_reset"]["observation_complete"])


if __name__ == "__main__":
    unittest.main()
