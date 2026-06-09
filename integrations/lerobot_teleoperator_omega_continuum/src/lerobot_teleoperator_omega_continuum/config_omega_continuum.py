from dataclasses import dataclass

from lerobot.teleoperators.config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("omega_continuum")
@dataclass
class OmegaContinuumConfig(TeleoperatorConfig):
    simulate: bool = False
    scale_x: float = 0.25
    scale_y: float = 0.08
    scale_z: float = 0.25
    omega_map: str = "zxy"
    max_delta_x: float = 0.03
    max_delta_y: float = 0.01
    max_delta_z: float = 0.03
    deadband_m: float = 0.0003
    zero_samples: int = 20
    zero_sample_period_s: float = 0.005

    def __post_init__(self) -> None:
        self.omega_map = self.omega_map.lower()
        if sorted(self.omega_map) != ["x", "y", "z"]:
            raise ValueError("omega_map must be a permutation of xyz.")
        if any(value < 0.0 for value in (self.max_delta_x, self.max_delta_y, self.max_delta_z)):
            raise ValueError("Omega maximum deltas must be non-negative.")
        if self.deadband_m < 0.0:
            raise ValueError("deadband_m must be non-negative.")
        if self.zero_samples <= 0:
            raise ValueError("zero_samples must be positive.")
        if self.zero_sample_period_s < 0.0:
            raise ValueError("zero_sample_period_s must be non-negative.")
