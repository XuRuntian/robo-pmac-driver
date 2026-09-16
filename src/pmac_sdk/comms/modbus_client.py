from pymodbus.client.sync import ModbusTcpClient
from typing import List
from threading import RLock

class ModbusClient32Bit:
    """封装底层的 32-bit Modbus 读写操作，与具体业务无关"""
    def __init__(self, ip: str, port: int, slave_id: int, timeout_s: float = 0.1):
        self.client = ModbusTcpClient(host=ip, port=port, timeout=timeout_s, retries=0)
        self.slave_id = slave_id
        # A complete mailbox transaction must not interleave with feedback/homing.
        self.transaction_lock = RLock()

    def connect(self) -> bool:
        return self.client.connect()

    def disconnect(self):
        self.client.close()

    @staticmethod
    def _int32_to_registers(val: int) -> List[int]:
        val = int(val)
        if not -(2**31) <= val < 2**31:
            raise ValueError(f"Value outside signed int32 range: {val}")
        val &= 0xFFFFFFFF
        return [val & 0xFFFF, (val >> 16) & 0xFFFF]

    @staticmethod
    def _registers_to_int32(low: int, high: int) -> int:
        val = low + (high << 16)
        return val - 0x100000000 if val >= 0x80000000 else val

    def write_int32_array(self, address: int, values: List[int]) -> bool:
        regs = []
        for v in values:
            regs.extend(self._int32_to_registers(v))
        res = self.client.write_registers(address=address, values=regs, unit=self.slave_id)
        if res.isError():
            raise ConnectionError(f"Modbus Write Error at address {address}: {res}")
        return True

    def read_int32_array(self, address: int, count: int) -> List[int]:
        res = self.client.read_holding_registers(address=address, count=count * 2, unit=self.slave_id)
        if res.isError() or len(getattr(res, 'registers', ())) != count * 2:
            raise ConnectionError(f"Modbus Read Error at address {address}")
        return [self._registers_to_int32(res.registers[i*2], res.registers[i*2+1]) for i in range(count)]
