from pathlib import Path
import yaml

from .config import (
    ContinuumActuationConfig,
    ContinuumConfig,
    ContinuumControlConfig,
    ContinuumGeometryConfig,
    ContinuumIKConfig,
)


def load_continuum_config(path: str | Path = "config/continuum.yaml") -> ContinuumConfig:
    with open(path, "r", encoding="utf-8") as f:
        raw = yaml.safe_load(f)

    geometry_raw = raw["geometry"]
    ik_raw = raw["ik"]

    return ContinuumConfig(
        geometry=ContinuumGeometryConfig(
            **{
                **geometry_raw,
                "base_offset_m": tuple(geometry_raw["base_offset_m"]),
                "use_sheath": bool(geometry_raw.get("use_sheath", False)),
            }
        ),
        ik=ContinuumIKConfig(
            **{
                **ik_raw,
                "jacobian_eps": tuple(
                    ik_raw.get(
                        "jacobian_eps",
                        (1e-3, 5e-3, 5e-3, 5e-3, 5e-3),
                    )
                ),
                "max_delta_u": tuple(
                    ik_raw.get(
                        "max_delta_u",
                        (0.0003, 0.0025, 0.0025, 0.0025, 0.0025),
                    )
                ),
            }
        ),
        actuation=ContinuumActuationConfig(**raw["actuation"]),
        control=ContinuumControlConfig(**raw["control"]),
    )
