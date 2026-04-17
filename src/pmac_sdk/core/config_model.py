from dataclasses import dataclass

@dataclass
class PMACConfig:
    ip: str = '192.168.0.200'
    modbus_port: int = 502
    slave_id: int = 1
    ssh_user: str = 'root'
    ssh_pass: str = 'deltatau'
    
    # 物理参数
    encoder_resolution: int = 131072
    gear_ratio: float = 97.34
    
    @property
    def pulses_per_degree(self) -> float:
        return (self.encoder_resolution * self.gear_ratio) / 360.0