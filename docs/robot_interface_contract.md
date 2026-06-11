# Continuum Robot Driver Interface Contract

This document defines only the framework-independent driver input, driver
output, units, coordinate references, and startup reference behavior. It does
not require LeRobot-style method names.

## Command Input

The robot accepts one tip pose offset command:

```python
{
    "tip_delta_x": float,   # m
    "tip_delta_y": float,   # m
    "tip_delta_z": float,   # m
    "tip_delta_rx": float,  # rad
    "tip_delta_ry": float,  # rad
    "tip_delta_rz": float,  # rad
}
```

Translation is an absolute offset from the configured neutral tip position:

```text
p_target = p_neutral + [dx, dy, dz]
```

Rotation is a rotation vector relative to the configured neutral tip
orientation:

```text
R_target = R_neutral * Exp([rx, ry, rz])
```

The vector direction is the rotation axis and its norm is the rotation angle in
radians. A rotation vector is used instead of Euler angles to avoid Euler
singularities and instead of a quaternion to avoid an extra normalization
constraint.

The ZMQ driver switches the IK task to `pos_z` when
`orientation_enabled: true`. Translation and rotation are filtered
independently using the configured linear and angular limits.

The robot has five generalized coordinates, so it controls three-dimensional
tip position plus the two-dimensional tip-axis direction. Independent roll
about the tip axis (`rz`) is reserved in the interface but limited to zero by
the current configuration.

## Robot State Output

The minimum robot state contains the five actuator feedback positions:

```python
{
    "axis_1_pos": float,  # rad
    "axis_2_pos": float,  # rad
    "axis_3_pos": float,  # rad
    "axis_4_pos": float,  # rad
    "axis_5_pos": float,  # m
}
```

Axes 1-4 are rotary tendon-drive axes and are reported in radians. Axis 5 is a
physical linear unit and is reported in meters. It must not be labeled radians
unless a separate motor-side angle calibration is added.

Conversion is:

```text
PMAC feedback pulses
  -> subtract configured pulse reference
  -> apply physical/logical axis order and signs
  -> axes 1-4 divide by pulses/rad
  -> axis 5 divide by pulses/m
```

Raw pulses, following error, fault flags, PVT buffer level, and timestamps are
diagnostics. They may be added separately without changing the minimum state
schema.

## Omega Input

No LeRobot-specific Omega wrapper is required. The Force Dimension SDK already
provides the required source data:

```text
position:          [x, y, z] in m
orientation frame: 3x3 rotation matrix, dimensionless
Euler debug view:  [roll, pitch, yaw] in degrees
gripper angle:     degrees
```

The existing `OmegaDevice` is only a project convenience wrapper around that
SDK. A future integration may either keep it or use the official SDK directly.
It should not create a second semantic action format.

The currently tested translation mapping is:

| Omega motion | Robot channel | Mechanism |
| --- | --- | --- |
| X | robot Y | axis 5 linear unit |
| Y | robot Z | bending through axes 1-4 |
| Z | robot X | bending through axes 1-4 |

`omega_map: zxy` means robot XYZ receives Omega ZXY.

Omega orientation is sampled at startup and converted into a relative rotation
vector in the startup handle frame. Its components currently use the same
`zxy` permutation as translation. Rotation direction and scale must be
validated with small motions before increasing the configured limits.

## Initial Position

Configuration lives in `config/robot_interface.yaml`.

Three different references must remain distinct:

1. PMAC pulse reference: encoder values corresponding to logical actuator zero.
2. Robot neutral pose: FK pose when logical actuator state is
   `[0 rad, 0 rad, 0 rad, 0 rad, 0 m]`.
3. Omega startup zero: the master pose sampled when teleoperation starts.

### Capture Current

```yaml
mode: capture_current
```

- Reject PMAC feedback `[0, 0, 0, 0, 0]`.
- Use current PMAC feedback as this run's pulse reference.
- Preserves current teleoperation behavior.
- Does not move the robot.

### Configured Reference

```yaml
mode: configured_reference
reference_pulses: [63037980, 503187440, -141762994, 404354914, 18]
require_near_reference: true
tolerance_pulses: [axis1, axis2, axis3, axis4, axis5]
```

- Use the configured pulse values as reproducible logical zero.
- Require current feedback to be within the configured per-axis tolerances.
- Reject startup if the position is outside tolerance.
- Does not automatically return the robot to reference.

Automatic homing or return-to-reference must remain an explicit operation,
separate from connecting and selecting the coordinate reference.

## Timing

- PMAC PVT production remains fixed at `50 Hz`.
- Command producers may operate at another rate.
- A future adapter must not let camera capture, dataset writing, or inference
  block the PVT producer.
