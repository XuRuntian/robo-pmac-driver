from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig
from lerobot.robots.config import RobotConfig


@RobotConfig.register_subclass("continuum_pmac")
@dataclass
class ContinuumPMACConfig(RobotConfig):
    remote_ip: str = "127.0.0.1"
    command_port: int = 5555
    state_port: int = 5556
    connect_timeout_s: float = 5.0
    polling_timeout_ms: int = 20
    state_timeout_s: float = 0.5
    cameras: dict[str, CameraConfig] = field(default_factory=dict)
