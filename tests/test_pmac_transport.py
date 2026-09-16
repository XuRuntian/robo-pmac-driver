from types import SimpleNamespace

import pytest

from pmac_sdk.comms.modbus_client import ModbusClient32Bit
from pmac_sdk.hardware import ssh_manager


@pytest.mark.parametrize('value', [2**31, -(2**31)-1])
def test_wire_codec_rejects_wraparound(value):
    with pytest.raises(ValueError):
        ModbusClient32Bit._int32_to_registers(value)


def test_failed_modbus_write_raises():
    client = ModbusClient32Bit('unused', 502, 1)
    client.client = SimpleNamespace(write_registers=lambda **kw: SimpleNamespace(isError=lambda: True))
    with pytest.raises(ConnectionError, match='Write Error'):
        client.write_int32_array(0, [123])


def test_truncated_feedback_is_not_decoded_as_a_position():
    client = ModbusClient32Bit('unused', 502, 1)
    client.client = SimpleNamespace(read_holding_registers=lambda **kw: SimpleNamespace(
        isError=lambda: False, registers=[123],
    ))
    with pytest.raises(ConnectionError, match='Read Error'):
        client.read_int32_array(10, 5)


@pytest.mark.parametrize('output,error,exit_code', [('ERR003: bad command', '', 0), ('', 'failure', 0)])
def test_ssh_errors_abort_remaining_boot_commands(monkeypatch, output, error, exit_code):
    events = []
    class SSH:
        def set_missing_host_key_policy(self, policy): pass
        def connect(self, **kwargs): pass
        def close(self): events.append('close')
        def exec_command(self, command, **kwargs):
            events.append(command)
            return None, SimpleNamespace(
                read=lambda: output.encode(),
                channel=SimpleNamespace(recv_exit_status=lambda: exit_code),
            ), SimpleNamespace(read=lambda: error.encode())
    monkeypatch.setattr(ssh_manager.paramiko, 'SSHClient', SSH)
    manager = ssh_manager.PMACHardwareManager('unused', 'unused', 'unused')
    with pytest.raises(RuntimeError, match='PMAC command failed'):
        manager.send_gpascii_commands([('first', '&1A'), ('must not run', '#1..5j/')], delay=0)
    assert len(events) == 2
    assert events[-1] == 'close'


def test_gpascii_normal_piped_eof_exit_code_is_accepted(monkeypatch):
    class SSH:
        def set_missing_host_key_policy(self, policy): pass
        def connect(self, **kwargs): pass
        def close(self): pass
        def exec_command(self, command, **kwargs):
            return None, SimpleNamespace(
                read=lambda: b'OK',
                channel=SimpleNamespace(recv_exit_status=lambda: 1),
            ), SimpleNamespace(read=lambda: b'')

    monkeypatch.setattr(ssh_manager.paramiko, 'SSHClient', SSH)
    ssh_manager.PMACHardwareManager('unused', 'unused', 'unused').send_gpascii_commands(
        [('normal', '&1A')], delay=0,
    )


def test_ssh_connection_failure_is_propagated(monkeypatch):
    closed = []
    class SSH:
        def set_missing_host_key_policy(self, policy): pass
        def connect(self, **kwargs): raise ConnectionError('offline')
        def close(self): closed.append(True)
    monkeypatch.setattr(ssh_manager.paramiko, 'SSHClient', SSH)
    with pytest.raises(ConnectionError, match='offline'):
        ssh_manager.PMACHardwareManager('unused', 'unused', 'unused').start_prog()
    assert closed


def test_stop_aborts_before_clearing_queue(monkeypatch):
    commands = []
    manager = ssh_manager.PMACHardwareManager('unused', 'unused', 'unused')
    monkeypatch.setattr(manager, 'send_gpascii_commands', lambda seq: commands.extend(cmd for _, cmd in seq))
    manager.stop_pvt()
    assert commands.index('disable plc 2') < commands.index('&1A') < commands.index('PVT_WriteIdx=0')
    assert commands[-1] == 'enable plc 2'
    assert all(f'Sys.ModbusServerBuffer[{i}]=0' in commands for i in range(400, 404))
