# Continuum Robot Driver Interface Contract

The physical PMAC link uses the paired [protocol-v2 mailboxes](pmac_protocol_v2.md).
Encoder reads below refer to freshly requested snapshots, not raw reads of a
continuously refreshed holding register.

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

The default ZMQ driver config uses `orientation_enabled: true` and the
`pos_z` IK task. This is the complete IK task for the current five-axis
continuum body: Cartesian tip translation plus tip-axis direction. Translation
and rotation are filtered independently using the configured linear and angular
limits.

The robot has five generalized coordinates, so it controls three-dimensional
tip position plus the two-dimensional tip-axis direction. Independent roll
about the tip axis (`rz`) is reserved in the interface but limited to zero by
the current configuration.

## Robot Variants

The current PMAC hardware path is the five-axis continuum body without a clamp
or gripper actuator. Its action space must remain the six-field tip pose offset
above, where `rz` is reserved and normally clamped to zero.

The MuJoCo project also contains clamp/gripper variants. Those models expose an
extra `clamp_pos` actuator and a haptic grip angle, but that command must not
be silently mixed into the current five-axis PMAC driver. If the clamp hardware
is added later, expose it as an explicit robot variant or an additional action
field such as `gripper_pos`, with its own unit and limits.

The portable part of the MuJoCo controller is:

```text
tip target pose
  -> DLS IK in u = [d, theta_a, phi_a, theta_c, phi_c]
  -> local tendon/motor mapping
  -> PMAC pulses
```

Only the IK target and solver behavior should be shared with the simulation.
The final tendon-to-motor mapping must stay in this repository because it owns
the physical wire routing, motor signs, encoder references, and PMAC pulse
units.

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

The existing internal IK frame is `+X=right`, `+Y=down`, `+Z=forward/insertion`.
The external command frame is standard right-handed: `+X=right`,
`+Y=forward/insertion`, `+Z=up`.

Cartesian commissioning corrections are configured in the selected
`config/robot_interface*.yaml` file rather than in the PMAC motor map:

```yaml
frame:
  translation_map: xzy
  translation_signs: [1, -1, 1]
  rotation_map: xzy
  rotation_signs: [1, -1, 1]
```

Each output axis takes the source axis named by its map and applies the
corresponding sign. The current values therefore convert the external
standard frame with `[x, y, z] -> [x, -z, y]` for the existing internal IK
frame. These frame corrections are separate from the physical PMAC
`axis_order` and `axis_signs` settings. The current mechanism supports
world-frame tip-axis tilt through `rx`/`rz`; world `ry` maps to the disabled
continuum-frame tool roll `Rz`.

The Omega device's source-axis mapping is deliberately not part of this
robot-frame configuration.  Configure it with the Omega adapter's own
`--omega-map` option.
The translation mapping is:

| Omega motion | Robot channel | Mechanism |
| --- | --- | --- |
| Z | robot X | bending through axes 1-4 |
| Y | robot Y (sign-inverted) | bending through axes 1-4 |
| X | robot Z | axis 5 linear unit |

`omega_map: zyx` means robot XYZ receives Omega ZYX.  The robot-Y scale is
negative because the corrected +Y direction is opposite the previous upward
direction.

Omega orientation is sampled at startup and converted into a relative rotation
vector in the startup handle frame. Rotation commands are tip-local, so the
validated `zxy` rotation permutation is retained; direction and scale must
still be validated with small motions before increasing the configured limits.

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

The driver service provides this explicit startup return through:

```powershell
python apps/continuum_driver_server.py --execute `
  --return-to-reference-on-start `
  --return-duration 12
```

This ramps from startup feedback to `reference_pulses` using the fixed PMAC PVT
rate before any LeRobot teleoperation client connects.

## Timing

- PMAC PVT production remains fixed at `50 Hz`.
- Command producers may operate at another rate.
- A future adapter must not let camera capture, dataset writing, or inference
  block the PVT producer.
