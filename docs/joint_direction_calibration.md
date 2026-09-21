# Joint bending-plane direction calibration

Hardware observation recorded on 2026-09-21:

- Increasing both phi_a and phi_c previously rotated the bending direction
  counterclockwise when viewed from base toward tip. Positive right-hand
  rotation about the local forward axis should appear clockwise from this view.
- A-section phi_a=0 was reported to bend toward WORLD -Z (down).
- C-section phi_c=0 was reported to bend toward WORLD +Z (up).

The two zero directions are observations only; no zero-angle offsets are
applied. WORLD directional alignment remains to be calibrated before treating
the model's predicted bend direction as the measured hardware direction.

`config/continuum.yaml` now sets actuation.phi_a_sign and phi_c_sign to -1.
Public joint angles are converted once to tendon-plane angles before computing
tendon lengths or motor angles. Feedback first removes A-section coupling in
tendon coordinates, then converts both angles back to public joint coordinates.
FK/IK, CC components, PMAC axis order/signs, and Omega/WORLD transforms retain
their existing definitions. Missing sign fields default to +1 for older configs.

The phi-sign correction alone is equivalent to negating both input phi angles.
It does not change their zero directions.

Subsequent operator hardware comparison confirmed that the proximal contribution
to distal motor commands had the wrong sign. The shared model now uses:

    alpha3/4 = -K * (C bending contribution - A pass-through contribution)

The same correction applies to tendon lengths dl5..dl8. Feedback adds back
the proximal term in tendon coordinates before decoding C bending. A's own
motor commands and C-only commands are unchanged. This is now the standard
mapping for both joint tests and teleoperation; the temporary compensation
mode switches and diagnostic target-building path have been removed.

Use apps/test_continuum_joint_params.py with --theta_a/--phi_a/--theta_c/--phi_c
to verify each section separately at 0 and +/-1.5708 radians. It defaults to
dry-run; hardware execution uses --execute. The script prints the effective
calibration signs. MuJoCo feedback mirrors accept --config and must use the
same continuum configuration as the driver.

## Retained tools and startup reference

Joint test (radians; omitted parameters are zero):

```bash
uv run python apps/test_continuum_joint_params.py --theta_a 0.2 --phi_a 0
```

Add --execute for hardware. It ramps to the target, holds, and returns to its
startup encoder reference. Ctrl+C stops the session without an automatic return.
The joint test captures its own startup reference; it does not load the stored
reference_pulses from robot_interface.yaml. Its current diagnostic theta bounds
are [0, 2.5] rad, not a validation of the physical workspace or IK geometry limits.

After straightening with Omega, stop the teleoperation/state-receiver clients
but keep the driver running, then record held, stable physical encoder feedback:

```bash
uv run python apps/record_reference.py
```

The timestamped JSON in logs/ records actual pulses, not just commanded targets.
It sends no motion commands and does not update robot_interface.yaml. The saved
reference in that YAML is currently a record only: mode=capture_current still
uses fresh startup feedback. Recording a pose does not automatically return to it.

Next calibration order: keyboard WORLD commands -> robot_interface frame;
then Omega raw input -> omega_teleop mapping. Current frame settings and Omega
gains are retained as working calibration values, not claimed as validated.
