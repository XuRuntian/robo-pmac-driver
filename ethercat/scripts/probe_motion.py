#!/usr/bin/env python3
"""One unloaded Diamond: disabled echo verification, optionally 90-count return.

Motion requires --move-unloaded-90-counts-operator-ready. The operator must be
present with immediate power-off available. Applies volatile narrow limits,
verifies them, runs the bounded child, and restores limits only after confirmed
healthy disable. Never saves flash or automatically resets/re-enables on failure.
"""
import subprocess
import time
import json
import probe_connected as probe
from reset_fault_once import read_snapshot, require_single_preop


def add_arguments(parser):
    parser.add_argument('--move-unloaded-90-counts-operator-ready', action='store_true')


def healthy(snapshot):
    return snapshot['controlword'] == 0 and snapshot['statusword'] & 0x4f == 0x40 and snapshot['error_code'] == 0


def run_motion(report):
    if report['errors'] or len(report['slaves']) != 1 or not report['slaves'][0]['identity_matches']:
        raise RuntimeError('requires one matching Diamond')
    require_single_preop()
    before = read_snapshot()
    if not healthy(before):
        raise RuntimeError('requires healthy disabled drive')
    move = report['arguments']['move_unloaded_90_counts_operator_ready']
    result = report['commissioning'] = {'before': before, 'move_requested': move,
                                        'limits': [], 'restored': False, 'observation': None}
    position = before['actual_position']
    limits = [(0x6072, 0, 'uint16', 30), (0x60e0, 0, 'uint16', 30), (0x60e1, 0, 'uint16', 30),
              (0x6065, 0, 'uint32', 128), (0x6066, 0, 'uint16', 20),
              (0x607d, 1, 'int32', position-512), (0x607d, 2, 'int32', position+512)]
    if not -(2**31) <= position-512 < position+512 < 2**31:
        raise RuntimeError('position outside commissioning integer envelope')
    probe.run('python3', probe.ROOT / 'scripts/check_esi.py', '--check')
    binary = probe.ROOT / 'build/commissioning/igh_motion_probe'
    if not binary.is_file():
        raise RuntimeError('build commissioning executables first')

    def read(index, sub, dtype):
        return int(probe.cli('--position', 0, '--type', dtype, 'upload', hex(index), sub)['stdout'].split()[-1])

    def write(index, sub, dtype, value):
        require_single_preop()
        probe.cli('--position', 0, '--type', dtype, 'download', hex(index), sub, '--', value)

    for index, sub, dtype, value in limits:
        result['limits'].append({'index': hex(index), 'subindex': sub, 'datatype': dtype,
                                 'original': read(index, sub, dtype), 'temporary': value, 'write_attempted': False})
    launched = False
    try:
        for item in result['limits']:
            item['write_attempted'] = True
            index, sub, dtype = int(item['index'], 16), item['subindex'], item['datatype']
            write(index, sub, dtype, item['temporary'])
            item['readback'] = read(index, sub, dtype)
            if item['readback'] != item['temporary']:
                raise RuntimeError('drive limit write/readback mismatch')
        if not healthy(read_snapshot()):
            raise RuntimeError('drive became unhealthy during limit setup')
        option = '--move-unloaded-90-counts-operator-ready' if move else '--verify-disabled'
        command = [str(binary), option]
        result['command'] = command
        launched = True
        report['motion_commands_sent'] = None if move else False
        with subprocess.Popen(command, stdout=subprocess.PIPE, stderr=None, text=True) as child:
            try:
                stdout, _ = child.communicate(timeout=100)
            except BaseException:
                child.terminate()
                try:
                    child.communicate(timeout=3)
                except subprocess.TimeoutExpired:
                    child.kill(); child.communicate()
                raise
            result['exit_code'] = child.returncode
            result['stdout'] = stdout
            lines = [line for line in stdout.splitlines() if line.startswith('{"motion_requested":')]
            result['observation'] = json.loads(lines[-1]) if lines else None
        obs = result['observation']
        if obs:
            report['motion_commands_sent'] = bool(obs['enable_commands_sent'])
        if child.returncode or not obs or not obs['completed'] or not obs['disabled_confirmed']:
            raise RuntimeError('commissioning failed; inspect child log and stop confirmation')
    finally:
        obs = result['observation']
        # After a transport failure the master may be automatically reconfiguring.
        # Do not race that mailbox work, or lift temporary limits on unknown state.
        if launched and (not obs or not obs['disabled_confirmed']):
            result['restore_skipped'] = 'disable not confirmed; conservative limits retained'
        else:
            deadline = time.monotonic() + 8
            while time.monotonic() < deadline:
                listing = probe.cli('slaves')['stdout'].splitlines()
                if len(listing) == 1 and listing[0].split()[:4] == ['0','0:0','PREOP','+']:
                    break
                time.sleep(0.1)
            require_single_preop()
            result['after'] = read_snapshot()
            if healthy(result['after']):
                for item in reversed(result['limits']):
                    if not item['write_attempted']:
                        continue
                    index, sub, dtype = int(item['index'], 16), item['subindex'], item['datatype']
                    write(index, sub, dtype, item['original'])
                    item['restored_value'] = read(index, sub, dtype)
                    if item['restored_value'] != item['original']:
                        raise RuntimeError('original limit restoration mismatch')
                result['restored'] = True
            else:
                result['restore_skipped'] = 'drive not healthy disabled; conservative limits retained'
                raise RuntimeError('drive is not healthy disabled after commissioning')
    report['warnings'].append('single unloaded axis only; fixed PDO count replies remain a firmware compatibility limitation')


if __name__ == '__main__':
    raise SystemExit(probe.main(after_diagnostics=run_motion, description=__doc__, configure_parser=add_arguments))
