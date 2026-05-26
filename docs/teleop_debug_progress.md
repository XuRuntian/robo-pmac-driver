# Teleoperation Debug Progress

Date: 2026-05-26

This file records the current teleoperation debugging state so a new Codex/chat window can resume quickly.

## Current Goal

Use keyboard teleoperation to tune the PMAC/PVT/IK control path first, then switch to Omega master teleoperation with the same limits and mapping strategy.

The intended shared control chain is:

```text
input device -> Cartesian p_goal -> continuum IK -> tendon/axis mapper -> PMAC PVT stream
```

For now, keyboard input is the safer test input. Omega should reuse the same scale, delta limits, and linear-axis policy after keyboard behavior is acceptable.

## Quick Resume

If starting in a new window, first reproduce the known-good keyboard commands before changing Omega parameters:

```bash
python apps/teleop_pvt_keyboard.py --execute --speed 0.02 --max-delta-x 0.03 --max-delta-y 0 --max-delta-z 0.03 --lock-linear-axis
```

Then reproduce the combined bend plus small linear-axis command:

```bash
python apps/teleop_pvt_keyboard.py --execute --speed 0.01 --max-delta-x 0.02 --max-delta-y 0.003 --max-delta-z 0.02
```

If both still feel stable, continue with Omega using conservative scale and the same delta limits.

## Scripts Changed

### `apps/teleop_pvt_keyboard.py`

Reworked as the main keyboard Cartesian teleop entrypoint.

It now supports:

- `--execute`: actually send commands to PMAC; otherwise dry-run.
- `--speed`: Cartesian target speed in m/s.
- `--max-delta-x`, `--max-delta-y`, `--max-delta-z`: workspace limits from neutral, in meters.
- `--lock-linear-axis`: keep the physical linear axis, axis 5, at startup position.
- Key controls:
  - `A / D`: X negative / X positive
  - `W / S`: Z positive / Z negative
  - `Q / E`: Y positive / Y negative
  - `Z / X`: decrease / increase speed during runtime
  - `C`: recenter target
  - `Esc`: stop
- Runtime print includes `dpulses=[axis1, axis2, axis3, axis4, axis5]`.

### `apps/test_omega_continuum_teleop.py`

Added the same safety/debug concepts:

- `--max-delta-x/y/z`
- `--lock-linear-axis`
- `dpulses` debug output

Important: Omega Y maps strongly to the physical linear axis because the model's logical `d` affects world Y. Do not use aggressive `scale-y` early.

### `apps/test_continuum_circle.py`

Added `--ramp-time` and virtual-time stepping.

Reason: the original circle test started directly on the edge of the circle, causing a startup jump and possible following error/fatal error. The ramp starts at radius 0 and grows to the requested radius.

## Verified Keyboard Parameters

These were reported as OK:

```bash
python apps/teleop_pvt_keyboard.py --execute --speed 0.02 --max-delta-x 0.03 --max-delta-y 0 --max-delta-z 0.03 --lock-linear-axis
```

This verifies the 1-4 axis bend-only path is controllable.

Also reported as OK:

```bash
python apps/teleop_pvt_keyboard.py --execute --speed 0.01 --max-delta-x 0.02 --max-delta-y 0.003 --max-delta-z 0.02
```

This verifies combined bend axes plus a small amount of linear-axis participation.

Latest interpretation: this is the current best baseline for keyboard teleop before moving to Omega.

## Interpretation

`--speed` is the keyboard-stage "gain-like" parameter.

Example:

```bash
--speed 0.02
```

means 20 mm/s Cartesian target speed.

`--max-delta-*` is not gain. It is the allowed target travel away from neutral. If motion stops while holding a key, check whether `delta=[...]` has reached the corresponding max-delta limit.

Example:

```bash
--speed 0.03 --max-delta-z 0.01
```

will hit the 10 mm Z limit in about 0.33 seconds.

## Current Recommended Test Flow

### 1. Bend-only keyboard test

Use this when checking 1-4 axes without linear axis interference:

```bash
python apps/teleop_pvt_keyboard.py --execute --speed 0.02 --max-delta-x 0.03 --max-delta-y 0 --max-delta-z 0.03 --lock-linear-axis
```

Expected behavior:

- Press key: motor starts quickly.
- Release key: motor stops quickly.
- Reverse direction: no jump, no fatal error.
- `dpulses[4]` stays at 0 because axis 5 is locked.

### 2. Small linear-axis test

Use this to test axis 5 gently:

```bash
python apps/teleop_pvt_keyboard.py --execute --speed 0.003 --max-delta-x 0 --max-delta-y 0.003 --max-delta-z 0
```

Keep this conservative. Axis 5 is the linear unit.

### 3. Combined keyboard test

Use the currently OK combined command:

```bash
python apps/teleop_pvt_keyboard.py --execute --speed 0.01 --max-delta-x 0.02 --max-delta-y 0.003 --max-delta-z 0.02
```

If this remains stable, keyboard-side PVT/IK behavior is good enough to start Omega tuning.

### 3.5. Linear-axis speed tuning

Axis 5 is the linear unit. Tune it conservatively.

Current recommendation: adjust linear-axis behavior from the Python teleop layer first, not directly in PMAC. In PVT mode, PMAC follows the streamed target position, velocity, and segment time; the Python side is the easiest place to tune operator feel.

For now, keep `--max-delta-y` small, such as:

```bash
--max-delta-y 0.003
```

If axis 5 feels too fast, lower the overall keyboard `--speed` for combined tests or use the small linear-axis-only test. A future useful improvement is to add a separate `--speed-y` parameter so the linear unit can be slower than X/Z bending motion.

### 4. Initial Omega test

Start with bend-only Omega:

```bash
python apps/test_omega_continuum_teleop.py --execute --duration 30 --scale-x 0.2 --scale-y 0 --scale-z 0.2 --max-delta-x 0.03 --max-delta-y 0 --max-delta-z 0.03 --lock-linear-axis
```

Only after that is stable, add small Y/linear-axis participation:

```bash
python apps/test_omega_continuum_teleop.py --execute --duration 30 --scale-x 0.2 --scale-y 0.05 --scale-z 0.2 --max-delta-x 0.03 --max-delta-y 0.003 --max-delta-z 0.03
```

## What Counts As "Responsive" Before the Tendon Mechanism Is Installed

At the current hardware stage, only motors are moving. So "responsive" means:

- Pressing a key starts motion quickly, roughly within 100-150 ms by feel.
- Releasing a key stops motion quickly.
- Continuous key press produces smooth speed, not obvious pulsing.
- Direction reversal is smooth.
- Short taps cause small movements; long holds cause continuous motion.
- `dpulses` changes consistently with the pressed key.

Do not judge final manipulator tip responsiveness yet. That has to wait until the tendon/continuum mechanism is installed.

## Known Model Behavior

The logical `d` coordinate maps to the physical linear axis, axis 5. In the current FK frame, `d` strongly affects world Y. Therefore:

- Keyboard `Q/E` Y motion can drive axis 5.
- Omega `scale-y` can drive axis 5.
- Even X/Z-only IK may use axis 5 for compensation if linear axis is not locked.

Use `--lock-linear-axis` when you want to inspect 1-4 axes clearly.

## PMAC/Fatal Error Caution

If a large command causes PMAC fatal/following error, simply lowering the Python radius/scale afterward may not be enough.

PMAC may still have:

- latched motor/coordinate error state
- stale PVT ring-buffer state
- stale PLC-read points
- old `PVT_WriteIdx`, `PVT_ReadIdx`, `PVT_Count`

Before continuing after a fatal error, reset PMAC state fully rather than only changing the Python command.

## Omega Axis-5 Amp Fault Update

Observed during Omega testing:

```bash
python apps/test_omega_continuum_teleop.py --execute --duration 180 --scale-x 0.25 --scale-y 0.08 --scale-z 0.25 --max-delta-x 0.03 --max-delta-y 0.005 --max-delta-z 0.03
```

If the Omega master is moved aggressively, physical axis 5 can enter amp fault. Restarting Python alone may still show amp fault because the drive/PMAC state is latched and old PVT/PLC state may remain active.

The likely cause is not `--max-delta-y` alone. `--max-delta-y` limits total Y travel, but it does not limit how fast a large master-hand motion can demand that travel. Axis 5 is especially sensitive because logical `d` maps strongly to world Y.

`apps/test_omega_continuum_teleop.py` now includes extra input protection:

- `--deadband`: ignore small Omega noise in robot-space meters.
- `--smooth-alpha`: low-pass filter the Omega target.
- `--max-speed-x/y/z`: Cartesian target slew-rate limits.
- A physical pulse-step clamp for axis 5 derived from `--max-speed-y`.
- `--feedback-hz` and `--log-csv`: sample PMAC position feedback and log target/actual/error pulses.

After an amp fault, recover PMAC before rerunning teleop. The intended manual/gpascii recovery shape is:

```text
disable plc 1,2,3
&1A
#1..5k
PVT_WriteIdx=0
PVT_ReadIdx=0
PVT_Count=0
Sys.ModbusServerBuffer[400]=0
Sys.ModbusServerBuffer[401]=0
Sys.ModbusServerBuffer[402]=0
Sys.ModbusServerBuffer[403]=0
Sys.ModbusServerBuffer[440]=0
Sys.ModbusServerBuffer[441]=0
Sys.ModbusServerBuffer[442]=0
Sys.ModbusServerBuffer[443]=0
#1..5j/
&1 #1->X #2->Y #3->Z #4->A #5->B
&1 b1r
enable plc 1,2,3
```

If `#1..5j/` cannot clear the drive fault, clear/power-cycle the axis-5 amplifier fault from the PMAC/drive side before starting Python again.

Recommended next Omega retest after recovery:

```bash
python apps/test_omega_continuum_teleop.py --execute --duration 120 --scale-x 0.25 --scale-y 0.08 --scale-z 0.25 --max-delta-x 0.03 --max-delta-y 0.005 --max-delta-z 0.03 --max-speed-x 0.02 --max-speed-y 0.0015 --max-speed-z 0.02 --deadband 0.0003 --smooth-alpha 0.25
```

For tracking analysis, add CSV logging:

```bash
python apps/test_omega_continuum_teleop.py --execute --duration 60 --scale-x 0.25 --scale-y 0.08 --scale-z 0.25 --max-delta-x 0.03 --max-delta-y 0.01 --max-delta-z 0.03 --max-speed-x 0.02 --max-speed-y 0.0015 --max-speed-z 0.02 --deadband 0.0003 --smooth-alpha 0.25 --feedback-hz 10 --log-csv logs/omega_axis5_tracking.csv
```

## Notes For The Next Window

Start by reading this file and the current scripts:

- `apps/teleop_pvt_keyboard.py`
- `apps/test_omega_continuum_teleop.py`
- `apps/test_continuum_circle.py`
- `src/pmac_sdk/controller/robot_api.py`

Do not immediately increase Omega gains. First reproduce the OK keyboard command, then migrate one parameter at a time to Omega.

Recommended next code improvement:

- Add independent `--speed-x`, `--speed-y`, `--speed-z` to `apps/teleop_pvt_keyboard.py`, or at least `--speed-y`.
- Mirror that idea in Omega with conservative `--scale-y` and small `--max-delta-y`.
