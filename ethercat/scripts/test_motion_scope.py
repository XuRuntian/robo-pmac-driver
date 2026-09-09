#!/usr/bin/env python3
"""Exercise limit restoration and no-enable defaults without requesting a master."""
import json
import unittest
from unittest.mock import patch
from contextlib import ExitStack
import probe_motion as module


class MotionScope(unittest.TestCase):
    def setUp(self):
        self.snapshot = dict(controlword=0, statusword=0x240, error_code=0,
                             actual_position=-506000550, mode_display=8)
        self.values = {(0x6072,0):3000, (0x60e0,0):3000, (0x60e1,0):3000,
                       (0x6065,0):131072, (0x6066,0):50,
                       (0x607d,1):-(2**31), (0x607d,2):2**31-1}
        self.original = dict(self.values)
        self.writes = []
        self.command = None
        self.observation = dict(enable_commands_sent=False, completed=True, disabled_confirmed=True)
        self.returncode = 0
        self.bad_readback = False

    def cli(self, *args, **kwargs):
        if args == ('slaves',):
            return dict(stdout='0 0:0 PREOP + Diamond')
        operation, index, sub = args[4:7]
        key = (int(index,16),sub)
        self.assertIn(key, self.original)
        if operation == 'download':
            self.assertEqual(args[7], '--')
            self.values[key] = args[8]
            self.writes.append((key,args[8]))
            return dict(stdout='',exit_code=0,stderr='')
        self.assertEqual(operation,'upload')
        value = self.values[key]
        if self.bad_readback and key == (0x6072,0) and value == 30:
            value = 31
        return dict(stdout=f'0x0 {value}',exit_code=0,stderr='')

    def child(self, command, **kwargs):
        self.command = command
        outer = self
        class Child:
            returncode = outer.returncode
            def __enter__(self): return self
            def __exit__(self,*args): pass
            def communicate(self, **kwargs):
                if outer.observation is None: return '',None
                return json.dumps(dict(motion_requested=False,**outer.observation)),None
        return Child()

    def execute(self, move=False, after=None):
        report = dict(errors=[], warnings=[], slaves=[dict(identity_matches=True)],
                      arguments=dict(move_unloaded_90_counts_operator_ready=move))
        self.report = report
        with ExitStack() as stack:
            stack.enter_context(patch.object(module.probe,'cli',side_effect=self.cli))
            stack.enter_context(patch.object(module.probe,'run'))
            stack.enter_context(patch.object(module,'require_single_preop'))
            snapshots = [self.snapshot,self.snapshot, after or self.snapshot]
            stack.enter_context(patch.object(module,'read_snapshot',side_effect=snapshots))
            stack.enter_context(patch.object(module.subprocess,'Popen',side_effect=self.child))
            stack.enter_context(patch('pathlib.Path.is_file',return_value=True))
            module.run_motion(report)
        return report

    def test_default_is_disabled_and_restores_limits(self):
        report = self.execute()
        self.assertEqual(self.command[-1],'--verify-disabled')
        self.assertFalse(report['motion_commands_sent'])
        self.assertEqual(self.values,self.original)
        self.assertTrue(report['commissioning']['restored'])
        self.assertEqual(len(self.writes),14)

    def test_explicit_motion_option(self):
        self.observation['enable_commands_sent'] = True
        report = self.execute(move=True)
        self.assertEqual(self.command[-1],'--move-unloaded-90-counts-operator-ready')
        self.assertTrue(report['motion_commands_sent'])

    def test_preenabled_drive_rejected_without_writes(self):
        self.snapshot['statusword'] = 0x227
        with self.assertRaisesRegex(RuntimeError,'healthy disabled'): self.execute()
        self.assertEqual(self.writes,[])
        self.assertIsNone(self.command)

    def test_unknown_child_result_retains_conservative_limits(self):
        self.observation = None; self.returncode = 1
        with self.assertRaisesRegex(RuntimeError,'commissioning failed'): self.execute(move=True)
        self.assertEqual(len(self.writes),7)
        self.assertFalse(self.report['commissioning']['restored'])
        self.assertIsNone(self.report['motion_commands_sent'])

    def test_failed_but_disabled_child_restores_without_retry(self):
        self.observation['completed'] = False; self.returncode = 1
        with self.assertRaisesRegex(RuntimeError,'commissioning failed'): self.execute()
        self.assertEqual(self.values,self.original)
        self.assertTrue(self.report['commissioning']['restored'])

    def test_unconfirmed_disable_retains_limits(self):
        self.observation['disabled_confirmed'] = False; self.returncode = 1
        with self.assertRaisesRegex(RuntimeError,'commissioning failed'): self.execute()
        self.assertEqual(len(self.writes),7)

    def test_unhealthy_after_keeps_limits(self):
        after = dict(self.snapshot,error_code=0x8611,statusword=0x208)
        with self.assertRaisesRegex(RuntimeError,'not healthy disabled'): self.execute(after=after)
        self.assertEqual(len(self.writes),7)

    def test_bad_limit_readback_prevents_child(self):
        self.bad_readback = True
        with self.assertRaisesRegex(RuntimeError,'limit write/readback mismatch'): self.execute()
        self.assertIsNone(self.command)
        self.assertEqual(self.values,self.original)


if __name__ == '__main__': unittest.main()
