# LeRobot Integration

Current Omega + MuJoCo setup: [parallel teleoperation](parallel_teleop.md).
Keep the existing LeRobot robot path; opt into raw-pose broadcasting for an
independent reference-IK simulation. The earlier shared-IK mode is historical.

## Rotation range ownership (2026-10-13)

Current Omega plugin defaults are `max_rotation_x=None`, `max_rotation_z=None`,
and `max_rotation_y=0`. Wrist-joint mapping, gains, deadband, and clutch anchors
are unchanged. Both the mapper and post-clutch action clipping now pass Rx/Rz
through unless the user explicitly supplies a legacy device-side cap.

The main and axis-5 trial robot interface YAML files own the effective range:
`command.max_rotation_delta_rad: [0.45, 0.0, 0.45]`. This transfers the previous
effective Omega envelope to the driver; it does not expand the tested range.
It also applies to keyboard and other clients, whose driver range was formerly
6 rad. Range expansion is a separate operational decision and does not imply
that every target inside a command box is reachable.

Omit `--teleop.max_rotation_x` and `--teleop.max_rotation_z` in new commands;
legacy examples below with explicit values still opt into a device-side cap.
Restart both processes to load these changes. Driver state and latency CSV
include `rotation_command_limited`; the terminal prints `WORLD ROTATION CLAMPED`
when a command first exceeds the configured range. Existing `IK CONSTRAINED`
reports solver boundary attempts separately. Neither message proves an actual
motor limit switch has triggered. Older numerical examples below are historical;
use the on-disk Omega and robot interface YAML values for the active mapping.

The PMAC hardware link now requires the paired protocol-v2 PLC project. Read
[PMAC lifecycle and protocol](pmac_protocol_v2.md) before using the hardware commands below.
The ZMQ/LeRobot action protocol remains unchanged.

The PMAC control loop and LeRobot run as separate processes:

```text
LeRobot continuum_pmac Robot plugin
        |
        | ZMQ command and state messages
        v
continuum_driver_server.py
        |
        | fixed 50 Hz PVT production
        v
PMAC
```

LeRobot owns dataset collection, cameras, teleoperation pipelines, training, and
policy inference. The driver service owns PMAC, inverse kinematics, axis
mapping, motion limits, and the command watchdog.

## Install

Use the existing Python 3.10 driver environment for the PMAC service:

```powershell
cd D:\project\surgical_continuum_robot\robo-pmac-driver
uv sync
```

Install both plugins into the Python 3.12 LeRobot environment:

```powershell
cd D:\project\lerobot
uv pip install -e D:\project\surgical_continuum_robot\robo-pmac-driver\integrations\lerobot_robot_continuum
uv pip install -e D:\project\surgical_continuum_robot\robo-pmac-driver\integrations\lerobot_teleoperator_omega_continuum
```

The distributions use the `lerobot_robot_` and `lerobot_teleoperator_`
prefixes, so LeRobot discovers them without modifications under
`D:\project\lerobot\src\lerobot`.

## Dry-Run Test

Start the driver without touching PMAC:

```powershell
cd D:\project\surgical_continuum_robot\robo-pmac-driver
python apps/continuum_driver_server.py
```

In the LeRobot environment, verify that the registered robot can connect:

```powershell
cd D:\project\lerobot
python D:\project\surgical_continuum_robot\robo-pmac-driver\integrations\lerobot_robot_continuum\examples\smoke_test.py
```

The script sends only a zero Cartesian offset and prints the five actuator
observations plus driver status.

## Hardware Service

After the dry-run connection succeeds, start the service with PMAC execution:

```powershell
cd D:\project\surgical_continuum_robot\robo-pmac-driver
python apps/continuum_driver_server.py --execute
```

This default interface config matches the validated complete Omega
teleoperation baseline: Cartesian tip translation plus tip-axis direction.

```text
scale XYZ = [0.25, 0.08, -0.25]
max delta XYZ = [0.03, 0.01, 0.03] m
max speed XYZ = [0.05, 0.003, 0.05] m/s
rotation scale XYZ = [-0.3, 0.0, 0.3]
max rotation XYZ = [0.45, 0.0, 0.45] rad
max angular speed XYZ = [0.45, 0.0, 0.45] rad/s
smooth_alpha = 0.5
orientation_enabled = true
IK task = pos_z
```

By default, startup captures the current encoder feedback as the logical zero
for that run. To move the robot back to the configured pulse reference before
opening teleoperation, use the explicit startup return option:

```powershell
python apps/continuum_driver_server.py --execute `
  --return-to-reference-on-start `
  --return-duration 12 `
  --return-check-tolerance-pulses 50000
```

The return uses fixed-rate PVT interpolation from the startup feedback to
`initial_position.reference_pulses`, then uses that reference as the logical
zero for subsequent commands.

Default endpoints:

- command input: `tcp://127.0.0.1:5555`
- state output: `tcp://127.0.0.1:5556`
- PMAC PVT rate: `50 Hz`
- PMAC feedback read rate: `10 Hz`
- stale command hold timeout: `0.2 s`

The timeout freezes the currently applied Cartesian target. It does not return
the robot to neutral and does not continue moving toward an old target.

When a LeRobot teleoperation client reconnects while the driver is still
running, `continuum_pmac` preserves the driver's current `applied_action` as
the new session offset. This lets the operator re-grab the robot from the
current held pose instead of pulling it back toward neutral on the first zero
Omega sample. To intentionally restart from neutral, restart the driver or set:

```powershell
--robot.preserve_applied_action_on_connect=false
```

## LeRobot Types

The registered types are:

- robot: `continuum_pmac`
- teleoperator: `omega_continuum`

The current LeRobot checkout uses nested CLI arguments:

```text
--robot.type=continuum_pmac
--teleop.type=omega_continuum
```

It does not use the older `--robot_type` spelling.

## Record Without Cameras

First validate the complete software recording path without video, Hub upload,
or Omega hardware. Keep the PMAC driver in dry-run mode and add
`--teleop.simulate=true`:

```powershell
cd D:\project\lerobot
lerobot-record `
  --robot.type=continuum_pmac `
  --robot.id=continuum_robot `
  --robot.remote_ip=127.0.0.1 `
  --teleop.type=omega_continuum `
  --teleop.id=omega_master `
  --teleop.simulate=true `
  --teleop.omega_config=D:/project/surgical_continuum_robot/robo-pmac-driver/config/omega_teleop.yaml `
  --teleop.max_rotation_x=0.45 `
  --teleop.max_rotation_y=0.0 `
  --teleop.max_rotation_z=0.45 `
  --teleop.rotation_deadband_rad=0.01 `
  --dataset.repo_id=local/continuum_omega_test `
  --dataset.root=D:/project/lerobot_data/continuum_omega_test `
  --dataset.single_task="Continuum robot Omega teleoperation test" `
  --dataset.fps=30 `
  --dataset.episode_time_s=20 `
  --dataset.reset_time_s=10 `
  --dataset.num_episodes=2 `
  --dataset.video=false `
  --dataset.push_to_hub=false `
  --play_sounds=false
```

For a real Omega recording, remove `--teleop.simulate=true`. `lerobot-record`
samples the Omega and records at 30 Hz. The independent driver continues
producing PMAC PVT segments at 50 Hz.

## Record With A Camera

After the no-camera recording succeeds, add a camera:

```powershell
lerobot-record `
  --robot.type=continuum_pmac `
  --robot.id=continuum_robot `
  --robot.remote_ip=127.0.0.1 `
  --robot.cameras="{front: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}}" `
  --teleop.type=omega_continuum `
  --teleop.id=omega_master `
  --dataset.repo_id=local/continuum_omega_camera `
  --dataset.root=D:/project/lerobot_data/continuum_omega_camera `
  --dataset.single_task="Continuum robot Omega teleoperation" `
  --dataset.fps=30 `
  --dataset.episode_time_s=60 `
  --dataset.reset_time_s=20 `
  --dataset.num_episodes=10 `
  --dataset.video=true `
  --dataset.streaming_encoding=true `
  --dataset.encoder_threads=2 `
  --dataset.push_to_hub=false `
  --play_sounds=false
```

The default recorded action is the six-field Cartesian tip offset. Translation
is in meters; world-frame `rx/rz` are tip-direction rotation-vector commands in radians,
while world `ry` maps to the disabled continuum-frame `Rz`. The recorded robot
state contains axes 1-4 in radians and axis 5 in meters.

## Teleoperation Command

The default driver and LeRobot teleoperator now include rotation, so the normal
hardware command is:

```powershell
cd D:\project\surgical_continuum_robot\robo-pmac-driver
python apps/continuum_driver_server.py --execute
```

Then start LeRobot teleoperation:

```powershell
cd D:\project\lerobot
lerobot-teleoperate `
  --robot.type=continuum_pmac `
  --robot.id=continuum_robot `
  --robot.remote_ip=127.0.0.1 `
  --teleop.type=omega_continuum `
  --teleop.id=omega_master
```

The default `omega_continuum` parameters are read from
`config/omega_teleop.yaml`:

```text
WORLD frame: +X right, +Y forward/insertion, +Z up
translation map/sign/gain = zyx / [1,-1,1] / [0.5,0.08,0.25]
max delta XYZ = [0.03, 0.03, 0.005] m
rotation map/sign/gain = zxy / [-1,1,1] / [0.3,0.0,0.3]
max rotation XYZ = [0.45, 0.0, 0.45] rad
rotation deadband = 0.01 rad
```

The negative rotation direction is encoded by the rotation `signs` in the
Omega YAML. World `rz` maps through the driver frame transform into
continuum-frame `-Ry`; world `ry` maps to the disabled continuum-frame tool
roll `Rz`.

The ZMQ PUSH/PULL transport intentionally supports one active LeRobot robot
client. PMAC ownership must remain exclusive to the driver service.

## Read-only MuJoCo visual comparison

The optional visual mirror uses a separate PUB/SUB port (5557). It does not
consume LeRobot's state port (5556), send commands, initialize Omega, or connect
to PMAC. The driver remains the sole hardware owner. Start the driver with the
current trial configuration and the additional visual endpoint (Git Bash):

```bash
cd /d/project/surgical_continuum_robot/robo-pmac-driver
uv run python apps/continuum_driver_server.py \
  --interface-config config/robot_interface_axis5_profile_trial.yaml \
  --wait-mode windows-highres --live-debug \
  --visual-state-port 5557 --execute
```

Keep the existing Omega/LeRobot command in its own terminal. In another terminal:

```bash
cd /d/project/surgical_continuum_robot/robo-pmac-driver
uv run --with mujoco python apps/view_robot_mujoco.py --source target
```

The default model is the horizontal robot on a plane from the sibling project's
`surgical_continuum_robot/model/continuum_robot/mjcf/scene_main.xml`.
Use `--xml PATH` to select another copy with the same fixed tendon joint layout.
The sibling project's code and assets are not modified.

The default `--view wide` uses a 135-degree azimuth and -25-degree elevation,
with center [0, 0.32, 0.13] m and distance 0.70 m, showing the arm together with
the drive housing and platform. `--view tip` uses a fixed close-up of the bending
sections: camera
center [0, 0.49, 0.207] m, distance 0.24 m, azimuth 135 degrees, elevation -25
degrees. `--view overview` restores the full-machine framing. These presets
change only the camera, not input mappings or model motion. Both wide and tip were
checked with offscreen renders of straight, two bending directions and a
reverse bend with 40 mm insertion; it is not a full-workspace framing guarantee.
The custom XML option may need manual camera adjustment for other model layouts.

The default `--source ideal` displays the independent reference IK from
`apps/omega_reference_sim.py`. See [Parallel teleoperation](parallel_teleop.md)
for startup. It requires `--initial-pose source --display-gain 1` (the defaults).
`--source target` selects the driver's execution-limited PVT target.
In target/feedback modes, `--initial-pose zero` and `--display-gain`
remain explicit diagnostic options only: they change the displayed geometry and
must not be used for a 1:1 physical comparison. Zero mode anchors to the first
valid source sample, then scales subsequent insertion/bending-component changes.
Use `--source feedback` to display encoder-derived joint parameters using
the driver's calibrated motor model and axis mapping. These are
distinct from raw Omega commands. Multiple viewers may subscribe simultaneously.
Without `--execute`, the driver publishes a dry-run model state, not encoders.

The viewer distributes bending over the model's discrete hinge joints and
updates geometry directly. It does not integrate dynamics, simulate contacts,
or execute actuator control. This is a geometric comparison, not measured shape
or proof of dynamic tracking: tendon slack, compliance and calibration errors
can still make the real robot differ. The current trial uses `use_sheath=false`;
this viewer does not implement changing sheath constraints.

Rendering requests 60 FPS by default independently of the 50 Hz driver. Actual
refresh depends on graphics load and OS scheduling; target poses arrive at 50 Hz,
and fresh encoder poses normally arrive at about 10 Hz. This viewer does not add
pose interpolation or a smoothing buffer. Use `--fps 20`
if graphics load is excessive; camera rotation and zoom remain interactive.
The viewer defaults to `--render-quality light` (shadows and reflections off).
Use `--render-quality full` to restore those effects. `--wait-mode auto` uses the
existing high-resolution waitable timer on Windows, and system sleep elsewhere;
`--wait-mode system` explicitly restores ordinary sleep. The timer is local to
the viewer, closes on exit, and does not change the global Windows timer period.
These settings do not change robot control, feedback rate, pose mapping or
queueing. Missing high-resolution timer support reports an error rather than
silently changing the requested wait mode.

Offline measurements on the development machine (2026-10-16): a requested
16.67 ms sleep took a median 31.12 ms with ordinary Python 3.10 sleep, versus
17.04 ms with the waitable timer. At 960x640, offscreen rendering took about
201 ms/frame with shadows/reflections, versus 33.69 ms without them at identical
multisampling settings. Geometry updates took about 0.01 ms. Offscreen timings
include image readback and do not establish the interactive window's FPS;
smoothness still requires operator verification, and 60 FPS is a request, not
a guaranteed measured rate.
Only the latest received state is retained. Missing input or encoder data older
than 0.5 s freezes the last pose and prints a status change in the terminal;
`--stale-s` adjusts this threshold. Closing the viewer does not stop the robot.
No viewer is required when running the driver without `--visual-state-port`.

### Direction alignment, one axis at a time

Keep the validated LeRobot chain for hardware. The independent simulation uses
`remote_control_ex` mapping/IK with the operator-confirmed rotation adjustment:
raw local rotation increments [rx, ry, rz] become [rz, ry, -rx] before reference
mapping. Translation and hardware commands are unchanged. The simulation uses
pos_z IK, which tracks tip direction but not independent axial roll.

Compare positive and negative motion one axis at a time from a fixed camera.
The two branches do not share IK or gains, so visual differences alone do not
justify changing physical motor signs. The PMAC axis order/signs and actuation
geometry remain unchanged.


## Current input range policy

Omega translation defaults (`max_delta_x/y/z`) are now `None`: the plugin
applies mapping, gain, deadband and clutch anchoring without translation caps.
Optional explicit CLI caps remain supported for older workflows.
Both `robot_interface.yaml` and `robot_interface_axis5_profile_trial.yaml` use
`max_rotation_delta_rad: [null, 0.0, null]`: Rx/Rz have no command magnitude cap,
while Ry remains disabled. This supersedes the historical 0.45 rad values above.
Driver translation bounds, angular/linear rate limits, IK geometric bounds and
axis-5 execution protection remain active. Unreachable goals can still saturate
IK; removing an input cap does not enlarge the mechanical workspace.
Restart both driver and LeRobot to load these changes.

## Rotary target anomaly stop

Both current interface configurations explicitly set
`execution.rotary_target_stop_rad_s: 2.5`. This is a provisional software anomaly
threshold in logical/output-equivalent motor radians per second, not a measured
motor rating, tool bend rate or validated safe speed. Do not automatically raise
it to avoid a trip. Missing/nonpositive/nonfinite values are rejected before
hardware connection. Legacy startup return is rejected because it bypasses this
teleoperation guard; use capture_current.

Before each PVT submit, the driver checks the peak rotary speed of the complete
cubic segment, including both endpoint velocities with the protocol's velocity
quantization. It uses ContinuumAxisMapper to preserve axis order/signs. Normal
segments are unchanged: no extra smoothing, target backlog or altered IK weights.
A trip rejects the whole five-axis candidate and raises `ROTARY TARGET STOP`.
The session exits through the existing controller cleanup/abort path; no automatic
resume or return-to-zero occurs. Abort delivery and physical stop dynamics depend
on controller/transport behavior and have not been newly hardware-validated.
Previously queued motion is not retroactively undone. This is not a jerk- or
acceleration-bounded deceleration planner, and below-threshold behavior is not
certified safe by this check.

Latency CSV adds `rotary_peak_rad_s` and `rotary_stop_threshold_rad_s`. Rejected
segments are not represented as submitted rows. After controller cleanup, a unique
`rotary_stop_<timestamp>.json` is saved beside the latency CSV (or in outputs if
CSV logging is off). It contains the sequence, affected physical/logical axis,
previous/rejected positions and velocities, computed peaks, IK result and applied
command. Retain it together with the CSV when a stop occurs.
