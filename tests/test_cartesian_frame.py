import numpy as np

from continuum_sdk.control.cartesian_frame import apply_axis_transform, swap_rx_ry, swap_xy_delta


def test_swap_xy_and_invert_lateral_signs() -> None:
    np.testing.assert_array_equal(
        swap_xy_delta(np.array([1.0, -2.0, 3.0])), [1.0, -3.0, -2.0]
    )


def test_rotation_transform_uses_same_basis_as_translation() -> None:
    np.testing.assert_array_equal(
        swap_rx_ry(np.array([1.0, -2.0, 3.0])), [1.0, -3.0, -2.0]
    )


def test_configured_axis_transform_matches_commissioning_frame() -> None:
    np.testing.assert_array_equal(
        apply_axis_transform(np.array([1.0, -2.0, 3.0]), "xzy", (1, -1, 1)),
        [1.0, -3.0, -2.0],
    )
