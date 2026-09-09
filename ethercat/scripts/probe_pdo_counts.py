#!/usr/bin/env python3
"""Diagnose Diamond PDO count replies in PREOP, with the drive disabled.

Temporarily clears assignment/mapping counts, writes the PMAC payload and zeros
unused slots, then commits the intended counts. No enable, target or flash save.
"""
import json
import probe_connected as probe
from reset_fault_once import read_snapshot, require_single_preop


def run_counts(report):
    if report['errors'] or len(report['slaves']) != 1 or not report['slaves'][0]['identity_matches']:
        raise RuntimeError('requires one matching Diamond')
    require_single_preop()
    before = read_snapshot()
    if before['controlword'] or before['statusword'] & 0x4f != 0x40 or before['error_code']:
        raise RuntimeError('requires healthy disabled drive')
    result = report['pdo_count_test'] = {'before': before, 'writes': [], 'reads': []}
    contract = json.loads((probe.ROOT / 'config/pmac_drives.json').read_text())

    def write(index, sub, value, dtype):
        require_single_preop()
        row = {'index': hex(index), 'subindex': sub, 'value': value}
        result['writes'].append(row)
        row.update(probe.cli('--position', 0, '--type', dtype, 'download', hex(index), sub, value))

    def read(index, label):
        row = {'index': hex(index), 'stage': label}
        row.update(probe.cli('--position', 0, '--type', 'uint8', 'upload', hex(index), 0))
        result['reads'].append(row)

    for tag, assignment, capacity in [('RxPdo', 0x1c12, 10), ('TxPdo', 0x1c13, 12)]:
        pdo = contract['pdos'][tag]
        index = pdo['index']
        entries = [(e['index'] << 16) | (e['subindex'] << 8) | e['bits'] for e in pdo['entries']]
        try:
            write(assignment, 0, 0, 'uint8')
            read(assignment, 'after_zero')
            write(index, 0, 0, 'uint8')
            read(index, 'after_zero')
            for sub in range(1, capacity + 1):
                write(index, sub, entries[sub - 1] if sub <= len(entries) else 0, 'uint32')
            write(assignment, 1, index, 'uint16')
            write(assignment, 2, 0, 'uint16')
        finally:
            # Commit the selected profile even when a diagnostic read fails.
            write(index, 0, len(entries), 'uint8')
            write(assignment, 0, 1, 'uint8')
        read(index, 'after_commit')
        read(assignment, 'after_commit')
    result['after_arrays'] = {hex(index): probe.read_array(0, hex(index), dtype, cap)
                             for index, dtype, cap in [(0x1600, 'uint32', 16), (0x1a00, 'uint32', 16),
                                                       (0x1c12, 'uint16', 4), (0x1c13, 'uint16', 4)]}
    result['after'] = read_snapshot()
    if result['after']['controlword'] or result['after']['statusword'] & 0x4f != 0x40 or result['after']['error_code']:
        raise RuntimeError('post-test drive must remain healthy and disabled')


if __name__ == '__main__':
    raise SystemExit(probe.main(after_diagnostics=run_counts, description=__doc__))
