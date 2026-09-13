from __future__ import annotations

import numpy as np


def apply_axis_transform(
    values: np.ndarray,
    axis_map: str,
    axis_signs: tuple[int, int, int] | list[int],
) -> np.ndarray:
    """Map source XYZ values into the configured robot XYZ order and signs."""
    vector = np.asarray(values, dtype=float)
    if vector.shape != (3,):
        raise ValueError("Cartesian vector must contain exactly three values.")
    mapping = str(axis_map).lower()
    if sorted(mapping) != ["x", "y", "z"]:
        raise ValueError("Cartesian axis map must be a permutation of xyz.")
    signs = np.asarray(axis_signs, dtype=float)
    if signs.shape != (3,) or np.any(np.abs(signs) != 1.0):
        raise ValueError("Cartesian axis signs must contain exactly three values of +1 or -1.")
    indices = np.asarray(["xyz".index(axis) for axis in mapping], dtype=int)
    return vector[indices] * signs


def swap_xy_delta(delta_xyz: np.ndarray) -> np.ndarray:
    """Apply the current standard-external to legacy-internal XYZ transform.

    The transform is ``[x, y, z] -> [x, -z, y]``.  The name is retained for
    compatibility with earlier commissioning code; runtime callers should
    prefer :func:`apply_axis_transform` with the interface YAML values.
    """
    return apply_axis_transform(delta_xyz, "xzy", (1, -1, 1))


def swap_rx_ry(rotation_xyz: np.ndarray) -> np.ndarray:
    """Convert world-frame angular vector to the continuum frame.

    World axes are X right, Y insertion, Z up; continuum axes are X right,
    Y down, Z insertion.  This is the same basis transform as translation.
    """
    return apply_axis_transform(rotation_xyz, "xzy", (1, 1, -1))
