"""Omega raw-device configuration (kept local so the plugin is installable alone)."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import yaml


def _triple(value: object, name: str, signs: bool = False) -> tuple[float, float, float]:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"{name} must contain exactly three values")
    result = tuple(float(item) for item in value)
    if signs and any(item not in (-1.0, 1.0) for item in result):
        raise ValueError(f"{name} values must be +1 or -1")
    return result


def _map(value: object, name: str) -> str:
    result = str(value).lower()
    if sorted(result) != ["x", "y", "z"]:
        raise ValueError(f"{name} must be a permutation of xyz")
    return result


@dataclass(frozen=True)
class OmegaAxisConfig:
    map: str
    signs: tuple[float, float, float]
    gain: tuple[float, float, float]


@dataclass(frozen=True)
class OmegaTeleopConfig:
    translation: OmegaAxisConfig
    rotation: OmegaAxisConfig


def load_omega_teleop_config(path: str | Path = "config/omega_teleop.yaml") -> OmegaTeleopConfig:
    with open(path, encoding="utf-8") as file:
        raw = yaml.safe_load(file) or {}

    def axis(name: str) -> OmegaAxisConfig:
        section = raw.get(name, {})
        return OmegaAxisConfig(
            _map(section.get("map", "xyz"), f"{name}.map"),
            _triple(section.get("signs", [1, 1, 1]), f"{name}.signs", True),
            _triple(section.get("gain", [1, 1, 1]), f"{name}.gain"),
        )
    return OmegaTeleopConfig(axis("translation"), axis("rotation"))
