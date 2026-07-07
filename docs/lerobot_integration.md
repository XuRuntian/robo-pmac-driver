# LeRobot Integration

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
rotation scale XYZ = [-0.3, 0.3, 0.0]
max rotation XYZ = [0.45, 0.45, 0.0] rad
max angular speed XYZ = [0.45, 0.45, 0.0] rad/s
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
  --teleop.scale_x=0.25 `
  --teleop.scale_y=0.08 `
  --teleop.scale_z=-0.25 `
  --teleop.omega_map=zxy `
  --teleop.max_rotation_x=0.45 `
  --teleop.max_rotation_y=0.45 `
  --teleop.max_rotation_z=0.0 `
  --teleop.rotation_scale_x=-0.3 `
  --teleop.rotation_scale_y=0.3 `
  --teleop.rotation_scale_z=0.0 `
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
is in meters; `rx/ry` are tip-direction rotation-vector commands in radians,
and `rz` is reserved at zero for the current five-axis body. The recorded robot
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

The default `omega_continuum` parameters are:

```text
scale XYZ = [0.25, 0.08, -0.25]
max delta XYZ = [0.03, 0.01, 0.03] m
rotation scale XYZ = [-0.3, 0.3, 0.0]
max rotation XYZ = [0.45, 0.45, 0.0] rad
rotation deadband = 0.01 rad
```

The sign of `rotation_scale_x` is intentionally negative because the physical
left-right bend direction was validated that way. `rotation_scale_z` remains
zero because roll about the tip axis is not an effective independent shape
input for the current five-axis continuum body.

The ZMQ PUSH/PULL transport intentionally supports one active LeRobot robot
client. PMAC ownership must remain exclusive to the driver service.
