import numpy as np
import pytest

from apps import test_continuum_joint_params as module
from continuum_sdk.control.axis_mapper import ContinuumAxisMapper
from continuum_sdk.core.config_loader import load_continuum_config
from continuum_sdk.core.factory import build_tendon_mapper
from continuum_sdk.kinematics.joint_motor_model import MotorAngles, angle_diff
from pmac_sdk.core.config_model import PMACConfig


def test_direct_target_with_omitted_parameters_defaulting_to_zero() -> None:
    args = module.parse_args(["--theta-a", "0.5", "--phi-a", "3.14"])
    module.validate_args(args)
    assert (args.theta_a, args.phi_a, args.theta_c, args.phi_c) == (0.5, 3.14, 0, 0)


def test_all_four_parameters_accept_underscore_spelling() -> None:
    args = module.parse_args(["--theta_a", "0.1", "--phi_a", "-1", "--theta_c", "0.2", "--phi_c", "2"])
    module.validate_args(args)
    assert (args.theta_a, args.phi_a, args.theta_c, args.phi_c) == (0.1, -1, 0.2, 2)


def test_negative_theta_is_rejected() -> None:
    with pytest.raises(ValueError, match="theta_c"):
        module.validate_args(module.parse_args(["--theta-c", "-0.1"]))


@pytest.mark.parametrize(
    "target",
    [
        module.JointTarget(theta_a=0.10),
        module.JointTarget(theta_a=0.15, phi_a=0.20),
        module.JointTarget(theta_c=0.10),
        module.JointTarget(theta_c=0.15, phi_c=-0.20),
    ],
)
def test_each_joint_parameter_roundtrips_through_motor_and_physical_axis_mapping(target) -> None:
    continuum_cfg = load_continuum_config("config/continuum.yaml")
    tendon_mapper = build_tendon_mapper(continuum_cfg)
    pmac_cfg = PMACConfig()
    axis_mapper = ContinuumAxisMapper(
        pmac_cfg.pulses_per_rad,
        pmac_cfg.pulses_per_meter,
        pmac_cfg.axis_order,
        pmac_cfg.axis_signs,
    )

    logical = tendon_mapper.to_axis_targets(target.as_ik_u())
    physical_pulses = axis_mapper.logical_to_pulses([0] * 5, logical)
    recovered_logical = axis_mapper.pulses_to_logical([0] * 5, physical_pulses)
    recovered = tendon_mapper.model.motor_angles_to_joint(MotorAngles(*recovered_logical[:4]))

    assert np.allclose(recovered_logical, logical, atol=1.0 / pmac_cfg.pulses_per_rad)
    assert recovered.theta_a == pytest.approx(target.theta_a, abs=2e-6)
    assert recovered.theta_c == pytest.approx(target.theta_c, abs=2e-6)
    if target.theta_a > 0.0:
        assert angle_diff(recovered.phi_a, target.phi_a) == pytest.approx(0.0, abs=1e-5)
    if target.theta_c > 0.0:
        assert angle_diff(recovered.phi_c, target.phi_c) == pytest.approx(0.0, abs=1e-5)
