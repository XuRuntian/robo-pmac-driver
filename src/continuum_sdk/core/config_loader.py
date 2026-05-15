from pathlib import Path

import yaml

from .config import (
    ContinuumAxisConfig,
    ContinuumConfig,
    ContinuumControlConfig,
    ContinuumGeometryConfig,
    ContinuumIKConfig,
)


def load_continuum_config(path: str | Path = "config/continuum.yaml") -> ContinuumConfig:
    with open(path, "r", encoding="utf-8") as f:
        raw = yaml.safe_load(f)

    return ContinuumConfig(
        geometry=ContinuumGeometryConfig(
            **{
                **raw["geometry"],
                "base_offset_m": tuple(raw["geometry"]["base_offset_m"]),
            }
        ),
        ik=ContinuumIKConfig(**raw["ik"]),
        axis=ContinuumAxisConfig(
            **{
                **raw["axis"],
                "soft_limits": [tuple(x) for x in raw["axis"]["soft_limits"]],
            }
        ),
        control=ContinuumControlConfig(**raw["control"]),
    )