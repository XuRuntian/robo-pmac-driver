import numpy as np

from continuum_sdk.kinematics.dls_ik import DLSIK


def test_neutral_tip_uses_corrected_forward_z_axis() -> None:
    ik = DLSIK()
    position, rotation = ik.fk_tip()

    np.testing.assert_allclose(position, [0.0, -0.207, 0.5236], atol=1e-9)
    np.testing.assert_allclose(rotation, np.eye(3), atol=1e-12)


def test_positive_insertion_moves_only_robot_plus_z_at_neutral() -> None:
    ik = DLSIK()
    start, _ = ik.fk_tip(np.zeros(5))
    moved, _ = ik.fk_tip(np.array([0.01, 0.0, 0.0, 0.0, 0.0]))

    np.testing.assert_allclose(moved - start, [0.0, 0.0, 0.01], atol=1e-12)
