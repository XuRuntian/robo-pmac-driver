"""Command theta_a, phi_a, theta_c, phi_c directly (radians; omitted values are zero).

The default is a dry-run.  Add --execute only after checking the printed
joint -> logical motor -> physical PMAC-axis mapping.  Stop the normal driver
before executing this diagnostic.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass

import numpy as np

from continuum_sdk.control.axis_mapper import ContinuumAxisMapper
from continuum_sdk.core.config_loader import load_continuum_config
from continuum_sdk.core.factory import build_tendon_mapper
from pmac_sdk.controller.robot_api import PMACRobotController
from pmac_sdk.core.config_model import PMACConfig
try:
    from apps.test_logical_axes import LOGICAL_AXIS_NAMES, run_phase
except ModuleNotFoundError:  # Direct execution adds apps/, not the repository root, to sys.path.
    from test_logical_axes import LOGICAL_AXIS_NAMES, run_phase


PARAMETERS = ("theta_a", "phi_a", "theta_c", "phi_c")


@dataclass(frozen=True)
class JointTarget:
    theta_a: float = 0.0
    phi_a: float = 0.0
    theta_c: float = 0.0
    phi_c: float = 0.0

    def as_ik_u(self) -> np.ndarray:
        # ContinuumTendonMapper expects [d, theta_a, phi_a, theta_c, phi_c].
        return np.array([0.0, self.theta_a, self.phi_a, self.theta_c, self.phi_c])


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true", help="Connect to PMAC and execute motion.")
    parser.add_argument("--pmac-ip", default="192.168.0.200")
    parser.add_argument("--config", default="config/continuum.yaml")
    for parameter in PARAMETERS:
        parser.add_argument(
            "--" + parameter.replace("_", "-"), "--" + parameter,
            dest=parameter, type=float, default=0.0,
            help=f"Target {parameter} in radians (default: 0).",
        )
    parser.add_argument("--rate-hz", type=float, default=50.0)
    parser.add_argument("--ramp-s", type=float, default=3.0)
    parser.add_argument("--hold-s", type=float, default=2.0)
    parser.add_argument("--zero-hold-s", type=float, default=1.0)
    parser.add_argument("--feedback-hz", type=float, default=5.0)
    return parser.parse_args(argv)


def joint_label(target: JointTarget) -> str:
    return (
        f"theta_a={target.theta_a:+.4f}, phi_a={target.phi_a:+.4f}, "
        f"theta_c={target.theta_c:+.4f}, phi_c={target.phi_c:+.4f} rad"
    )


def validate_args(args: argparse.Namespace) -> None:
    for name in (*PARAMETERS, "rate_hz", "feedback_hz", "ramp_s", "hold_s", "zero_hold_s"):
        if not np.isfinite(getattr(args, name)):
            raise ValueError(f"{name} must be finite.")
    if not 2.0 <= args.rate_hz < 200.0:
        raise ValueError("--rate-hz must be within [2, 200) for PMAC PVT timing.")
    if args.rate_hz <= 0.0 or args.feedback_hz <= 0.0:
        raise ValueError("--rate-hz and --feedback-hz must be positive.")
    if args.ramp_s <= 0.0 or args.hold_s < 0.0 or args.zero_hold_s < 0.0:
        raise ValueError("--ramp-s must be positive and hold durations must be non-negative.")
    for name in ("theta_a", "theta_c"):
        if not 0.0 <= getattr(args, name) <= 2.5:
            raise ValueError(f"{name} must be within [0, 2.5] rad for this diagnostic.")


def main() -> None:
    args = parse_args()
    try:
        validate_args(args)
        joint = JointTarget(**{name: getattr(args, name) for name in PARAMETERS})
        sequence = [("target", joint)]
    except ValueError as exc:
        raise SystemExit(f"error: {exc}") from None

    continuum_cfg = load_continuum_config(args.config)
    tendon_mapper = build_tendon_mapper(continuum_cfg)
    pmac_cfg = PMACConfig(ip=args.pmac_ip)
    axis_mapper = ContinuumAxisMapper(
        pulses_per_rad=pmac_cfg.pulses_per_rad,
        pulses_per_meter=pmac_cfg.pulses_per_meter,
        axis_order=pmac_cfg.axis_order,
        axis_signs=pmac_cfg.axis_signs,
    )

    robot = PMACRobotController(pmac_cfg) if args.execute else None
    try:
        if robot is not None:
            robot.safe_boot_and_home()
            base_pulses = robot.base_positions.copy()
        else:
            base_pulses = [0, 0, 0, 0, 0]

        mode = "EXECUTE" if args.execute else "DRY-RUN"
        print(f"Continuum joint-parameter diagnostic [{mode}]")
        print(f"Target joint: {joint_label(joint)}")
        print(f"Joint -> tendon phi signs: A={tendon_mapper.model.phi_a_sign:+d}, "
              f"C={tendon_mapper.model.phi_c_sign:+d} (no zero-angle offsets)")
        print("Ramp directly to target, hold, then return to startup reference.")
        print(f"Actuation: hole_radius={tendon_mapper.model.r_hole:.6f} m, "
              f"spool_diameter={tendon_mapper.model.d_spool:.6f} m")
        print(
            "Logical to physical mapping: "
            + ", ".join(
                f"{LOGICAL_AXIS_NAMES[i]} -> PMAC #{axis_mapper.axis_order[i] + 1} "
                f"sign {axis_mapper.axis_signs[i]:+d}"
                for i in range(5)
            )
        )

        current = [0.0] * 5
        for phase_name, joint in sequence:
            logical_target = tendon_mapper.to_axis_targets(joint.as_ik_u())
            print(f"\n{phase_name}: joint [{joint_label(joint)}]")
            print("joint -> logical alpha: " + ", ".join(
                f"{name}={value:+.6f}" for name, value in zip(LOGICAL_AXIS_NAMES, logical_target)
            ))
            current = run_phase(
                name=phase_name,
                robot=robot,
                axis_mapper=axis_mapper,
                base_pulses=base_pulses,
                previous=current,
                target=logical_target,
                duration_s=args.ramp_s,
                rate_hz=args.rate_hz,
                feedback_hz=args.feedback_hz,
            )
            current = run_phase(
                name="hold",
                robot=robot,
                axis_mapper=axis_mapper,
                base_pulses=base_pulses,
                previous=current,
                target=logical_target,
                duration_s=args.hold_s,
                rate_hz=args.rate_hz,
                feedback_hz=args.feedback_hz,
            )

        print("\nReturning all logical axes to the captured startup reference.")
        run_phase(
            name="return zero",
            robot=robot,
            axis_mapper=axis_mapper,
            base_pulses=base_pulses,
            previous=current,
            target=[0.0] * 5,
            duration_s=args.ramp_s,
            rate_hz=args.rate_hz,
            feedback_hz=args.feedback_hz,
        )
        run_phase(
            name="zero hold",
            robot=robot,
            axis_mapper=axis_mapper,
            base_pulses=base_pulses,
            previous=[0.0] * 5,
            target=[0.0] * 5,
            duration_s=args.zero_hold_s,
            rate_hz=args.rate_hz,
            feedback_hz=args.feedback_hz,
        )
    except KeyboardInterrupt:
        print("\nInterrupted. PMAC session will be stopped; restart before the next test.")
    finally:
        if robot is not None:
            robot.close()


if __name__ == "__main__":
    main()
