#!/usr/bin/env python3
"""单台空载 Diamond：指定角度往返，结束后失能；默认仅做失能回读。

--move 表示电机空载、人员在场且能立即断电。角度相对使能时的位置，负数反向。
--move-seconds 是每个方向的运动时间，中间保持一秒，然后返回起点。
Applies volatile narrow limits,
verifies them, runs the bounded child, and restores limits only after confirmed
healthy disable. Never saves flash or automatically resets/re-enables on failure.
"""
import subprocess
import time
import json
import argparse
import math
import sys
import probe_connected as probe
from reset_fault_once import read_snapshot, require_single_preop


COUNTS_PER_REV = 131072


def angle_value(text):
    value = float(text)
    if not math.isfinite(value) or not 0 < abs(value) <= 10 or abs(value)*COUNTS_PER_REV/360 < 0.5:
        raise argparse.ArgumentTypeError('角度须非零、绝对值不超过 10°，且至少能换算成 1 个计数')
    return value


def seconds_value(text):
    value = float(text)
    if not math.isfinite(value) or not 1 <= value <= 5:
        raise argparse.ArgumentTypeError('单程时间须在 1–5 秒之间')
    return value


def add_arguments(parser):
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--move', action='store_true', help='真机空载往返；不加此项不会使能')
    mode.add_argument('--move-unloaded-90-counts-operator-ready', action='store_true',
                      help='兼容旧命令：固定 90 counts、单程 2 秒')
    parser.add_argument('--angle-deg', type=angle_value, help='电机轴相对角度，正负决定方向，默认 0.25°，范围 ±10°')
    parser.add_argument('--move-seconds', type=seconds_value, help='单程运动时间，默认 2 秒，范围 1–5 秒')
    parser.add_argument('--plan', action='store_true', help='只显示角度换算和限位，不连接主站，不需要 sudo')


def motion_plan(options):
    legacy = options.get('move_unloaded_90_counts_operator_ready', False)
    if legacy and (options.get('angle_deg') is not None or options.get('move_seconds') is not None):
        raise ValueError('旧命令固定 90 counts / 2 秒；指定角度请使用 --move')
    angle = 90*360/COUNTS_PER_REV if legacy else angle_value(str(options.get('angle_deg') if options.get('angle_deg') is not None else 0.25))
    seconds = seconds_value(str(options.get('move_seconds') if options.get('move_seconds') is not None else 2))
    counts = int(math.copysign(math.floor(abs(angle)*COUNTS_PER_REV/360+0.5), angle))
    host_bound = max(256, abs(counts)+128)
    return dict(move_requested=bool(legacy or options.get('move', False)), requested_angle_deg=angle,
                command_angle_deg=counts*360/COUNTS_PER_REV, displacement_counts=counts,
                move_ms=int(math.floor(seconds*1000+0.5)), counts_per_revolution=COUNTS_PER_REV,
                host_travel_bound_counts=host_bound, drive_travel_bound_counts=host_bound+256)


def healthy(snapshot):
    return snapshot['controlword'] == 0 and snapshot['statusword'] & 0x4f == 0x40 and snapshot['error_code'] == 0


def run_motion(report):
    plan = motion_plan(report['arguments'])
    if report['errors'] or len(report['slaves']) != 1 or not report['slaves'][0]['identity_matches']:
        raise RuntimeError('requires one matching Diamond')
    require_single_preop()
    before = read_snapshot()
    if not healthy(before):
        raise RuntimeError('requires healthy disabled drive')
    move = plan['move_requested']
    result = report['commissioning'] = {'before': before, 'move_requested': move,
                                        'plan': plan, 'limits': [], 'restored': False, 'observation': None}
    position = before['actual_position']
    drive_bound = plan['drive_travel_bound_counts']
    limits = [(0x6072, 0, 'uint16', 30), (0x60e0, 0, 'uint16', 30), (0x60e1, 0, 'uint16', 30),
              (0x6065, 0, 'uint32', 128), (0x6066, 0, 'uint16', 20),
              (0x607d, 1, 'int32', position-drive_bound), (0x607d, 2, 'int32', position+drive_bound)]
    if not -(2**31) <= position-drive_bound < position+drive_bound < 2**31:
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

    # Angle conversion is only valid for the measured encoder and unity factors.
    result['scaling'] = []
    for index, sub, expected in [(0x608f,1,COUNTS_PER_REV), (0x608f,2,1)] + [
            (index,sub,1) for index in (0x6091,0x6092,0x6093) for sub in (1,2)]:
        value = read(index, sub, 'uint32')
        result['scaling'].append(dict(index=hex(index),subindex=sub,value=value))
        if value != expected:
            raise RuntimeError('编码器/传动比例已改变，拒绝沿用当前角度换算')
    print(f"{'真机往返' if move else '失能校验'}：{plan['command_angle_deg']:.6f}° "
          f"({plan['displacement_counts']} counts)，单程 {plan['move_ms']/1000:g} 秒。"
          '正在检查驱动器和等待时钟稳定。', file=sys.stderr, flush=True)
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
        option = '--move-unloaded-operator-ready' if move else '--verify-disabled'
        command = [str(binary), option, '--counts', str(plan['displacement_counts']),
                   '--move-ms', str(plan['move_ms'])]
        result['command'] = command
        launched = True
        report['motion_commands_sent'] = None if move else False
        with subprocess.Popen(command, stdout=subprocess.PIPE, stderr=None, text=True) as child:
            interruption = None
            try:
                stdout, _ = child.communicate(timeout=120)
            except BaseException as error:
                interruption = error
                child.terminate()
                try:
                    stdout, _ = child.communicate(timeout=3)
                except subprocess.TimeoutExpired:
                    child.kill(); stdout, _ = child.communicate()
            result['exit_code'] = child.returncode
            result['stdout'] = stdout
            lines = [line for line in stdout.splitlines() if line.startswith('{"motion_requested":')]
            result['observation'] = json.loads(lines[-1]) if lines else None
        obs = result['observation']
        if obs:
            report['motion_commands_sent'] = bool(obs['enable_commands_sent'])
        if interruption is not None:
            raise interruption
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
    print('完成：已确认失能，临时限值已恢复。详细结果见输出 JSON。', file=sys.stderr, flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('interface', help='本机测试网卡 enp0s31f6')
    add_arguments(parser)
    args = parser.parse_args()
    try:
        plan = motion_plan(vars(args))
    except (ValueError, argparse.ArgumentTypeError) as error:
        parser.error(str(error))
    if args.plan:
        print(json.dumps(plan,ensure_ascii=False,indent=2))
        raise SystemExit(0)
    raise SystemExit(probe.main(after_diagnostics=run_motion, description=__doc__, configure_parser=add_arguments))
