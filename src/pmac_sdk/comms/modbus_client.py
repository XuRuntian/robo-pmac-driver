from pymodbus.client.sync import ModbusTcpClient
from typing import List

class ModbusClient32Bit:
    """封装底层的 32-bit Modbus 读写操作，与具体业务无关"""
    def __init__(self, ip: str, port: int, slave_id: int):
        self.client = ModbusTcpClient(host=ip, port=port)
        self.slave_id = slave_id

    def connect(self) -> bool:
        return self.client.connect()

    def disconnect(self):
        self.client.close()

    @staticmethod
    def _int32_to_registers(val: int) -> List[int]:
        val = int(val) & 0xFFFFFFFF  
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
        return not res.isError()

    def read_int32_array(self, address: int, count: int) -> List[int]:
        res = self.client.read_holding_registers(address=address, count=count * 2, unit=self.slave_id)
        if res.isError():
            raise ConnectionError(f"Modbus Read Error at address {address}")
        return [self._registers_to_int32(res.registers[i*2], res.registers[i*2+1]) for i in range(count)]