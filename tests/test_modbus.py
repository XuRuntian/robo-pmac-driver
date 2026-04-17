import pytest
from pmac_sdk.comms.modbus_client import ModbusClient32Bit

def test_int32_to_registers():
    # 测试正数
    val_positive = 50000
    regs = ModbusClient32Bit._int32_to_registers(val_positive)
    assert len(regs) == 2
    # 50000 的十六进制是 0xC350
    assert regs[0] == 0xC350
    assert regs[1] == 0x0000

    # 测试负数
    val_negative = -50000
    regs_neg = ModbusClient32Bit._int32_to_registers(val_negative)
    # -50000 的 32 位补码是 0xFFFF3CB0
    assert regs_neg[0] == 0x3CB0
    assert regs_neg[1] == 0xFFFF

def test_registers_to_int32():
    # 测试能否完美还原
    test_values = [0, 1, -1, 131072, -963200, 2147483647, -2147483648]
    
    for val in test_values:
        regs = ModbusClient32Bit._int32_to_registers(val)
        decoded = ModbusClient32Bit._registers_to_int32(regs[0], regs[1])
        assert decoded == val, f"转换失败: 原始值 {val}, 还原值 {decoded}"