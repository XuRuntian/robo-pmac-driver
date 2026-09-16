"""Cross-repository wire contract checks against the actual PLC sources.

These check register layout only; they do not compile or simulate Power PMAC.
"""
from pathlib import Path
import re

import pytest

from pmac_sdk.controller.protocol import (
    FEEDBACK_REQUEST, HOMING_COMMAND, HOMING_STATUS, PROTOCOL_VERSION, PVT_REQUEST,
)

PLC_ROOT = Path(__file__).resolve().parents[2] / 'surgical_continuum_robot_pmac/PowerPMAC/PMAC Script Language'
pytestmark = pytest.mark.skipif(not PLC_ROOT.exists(), reason='paired PMAC checkout is not available')


def test_feedback_and_homing_snapshot_match_host_width_and_addresses():
    source = (PLC_ROOT / 'PLC Programs/plc_communication.plc').read_text(encoding='gb18030')
    encodes = re.findall(r'call ModbusCodec\.Encode\((.+),\s*(\d+)\);', source)
    byte_address = {name: int(address) for name, address in encodes}
    for index in range(5):
        assert byte_address[f'Motor[{index + 1}].Pos'] == 2 * (10 + 2 * index)
    fields = ['Homing_State', 'Homing_Stop_Reason', 'Motor[5].IqCmd',
              'abs(Motor[5].DesPos - Motor[5].ActPos)', 'Homing_Done']
    assert [byte_address[name] for name in fields] == [2*(HOMING_STATUS + i*2) for i in range(5)]
    assert byte_address[str(PROTOCOL_VERSION)] == 2 * 212
    assert byte_address['PVT_Ack'] == 2 * (PVT_REQUEST + 2)
    assert byte_address['Feedback_Ack'] == 2 * (FEEDBACK_REQUEST + 2)
    assert f'ModbusCodec.Decode({2*PVT_REQUEST})' in source
    assert f'ModbusCodec.Decode({2*FEEDBACK_REQUEST})' in source
    home = (PLC_ROOT / 'PLC Programs/plc_homing.plc').read_text(encoding='gb18030')
    assert f'ModbusCodec.Decode({2*HOMING_COMMAND})' in home


def test_reset_publishes_limit_before_completion_without_starting_motion():
    source = (PLC_ROOT/'Libraries/clearpvt.pmc').read_text(encoding='gb18030')
    reset = source.split('sub: Reset()', 1)[1]
    assert reset.index('Encode(Axis5_MaxStep, 436)') < reset.index('Encode(Reset_Done, 432)')
    assert 'b1r' not in reset
    # The user explicitly deferred PLC 1's calibration and enable behavior.
    assert 'disable plc 1' not in reset
    assert 'enable plc 1' not in reset
