"""The diagnostic gate must wait for completion of every slave dictionary."""
import contextlib
import io
import unittest
from unittest.mock import patch

import probe_connected as probe


class DictionaryWaitTests(unittest.TestCase):
    def setUp(self):
        self.report = {"timestamp_utc": "2026-09-09T06:23:31.123456+00:00"}

    def test_fetch_start_is_not_completion(self):
        logs = ["EtherCAT DEBUG 0-0: Fetching SDO dictionary.",
                "EtherCAT DEBUG 0-0: Fetched 251 SDOs and 600 entries."]
        with patch.object(probe, "run", side_effect=[{"stdout": log} for log in logs]) as run, \
                patch.object(probe, "cli") as cli, patch.object(probe.time, "sleep") as sleep, \
                patch.object(probe.time, "monotonic", return_value=0), \
                contextlib.redirect_stderr(io.StringIO()):
            probe.wait_dictionary(self.report, [0])
        self.assertEqual(run.call_count, 2)
        sleep.assert_called_once_with(0.25)
        cli.assert_called_once_with("debug", 0)
        self.assertTrue(self.report["dictionary_wait"]["complete"])

    def test_all_positions_required(self):
        logs = ["EtherCAT DEBUG 0-0: Fetched 251 SDOs and 600 entries.",
                "EtherCAT DEBUG 0-1: Fetched 251 SDOs and 600 entries."]
        with patch.object(probe, "run", side_effect=[{"stdout": log} for log in logs]), \
                patch.object(probe, "cli"), patch.object(probe.time, "sleep"), \
                patch.object(probe.time, "monotonic", return_value=0), \
                contextlib.redirect_stderr(io.StringIO()):
            probe.wait_dictionary(self.report, [0, 1])
        self.assertEqual(set(self.report["dictionary_wait"]["completed"]), {"0", "1"})

    def test_missing_completion_aborts_without_mailbox_calls(self):
        with patch.object(probe, "run", return_value={"stdout": ""}), \
                patch.object(probe, "cli") as cli, patch.object(probe.time, "sleep"), \
                patch.object(probe.time, "monotonic", side_effect=[0, 0, 46]):
            with self.assertRaisesRegex(RuntimeError, "external SDOs skipped"):
                probe.wait_dictionary(self.report, [0])
        cli.assert_not_called()
        self.assertFalse(self.report["dictionary_wait"]["complete"])
        self.assertEqual(self.report["dictionary_wait"]["pending_positions"], [0])


if __name__ == "__main__":
    unittest.main()
