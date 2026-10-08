import numpy as np

from continuum_sdk.control.cartesian_frame import apply_axis_transform
from continuum_sdk.core.omega_config import OmegaAxisConfig, load_omega_teleop_config


def test_omega_translation_mapping_reorders_signs_and_gains(tmp_path):
    path = tmp_path / "omega.yaml"
    path.write_text(
        "translation:\n  map: zxy\n  signs: [-1, 1, 1]\n  gain: [2.0, 3.0, 4.0]\n"
        "rotation:\n  map: xyz\n  signs: [1, 1, 1]\n  gain: [1, 1, 1]\n",
        encoding="utf-8",
    )
    cfg = load_omega_teleop_config(path)
    raw = np.array([1.0, 2.0, 3.0])
    world = apply_axis_transform(raw, cfg.translation.map, cfg.translation.signs) * cfg.translation.gain
    assert world.tolist() == [-6.0, 3.0, 8.0]


def test_omega_rotation_mapping_is_independent():
    cfg = OmegaAxisConfig("zxy", (-1, 1, 1), (2, 3, 4))
    raw = np.array([1.0, 2.0, 3.0])
    world = apply_axis_transform(raw, cfg.map, cfg.signs) * cfg.gain
    assert world.tolist() == [-6.0, 3.0, 8.0]


def test_two_layers_apply_once():
    omega = apply_axis_transform(np.array([1.0, 2.0, 3.0]), "zxy", (-1, 1, 1)) * np.array([2.0, 3.0, 4.0])
    ik = apply_axis_transform(omega, "xzy", (1, -1, 1))
    assert ik.tolist() == [-6.0, -8.0, 3.0]
