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

Default endpoints:

- command input: `tcp://127.0.0.1:5555`
- state output: `tcp://127.0.0.1:5556`
- PMAC PVT rate: `50 Hz`
- PMAC feedback read rate: `10 Hz`
- stale command hold timeout: `0.2 s`

The timeout freezes the currently applied Cartesian target. It does not return
the robot to neutral and does not continue moving toward an old target.

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
  --teleop.scale_z=0.25 `
  --teleop.omega_map=zxy `
  --teleop.max_rotation_x=0.15 `
  --teleop.max_rotation_y=0.15 `
  --teleop.max_rotation_z=0.0 `
  --teleop.rotation_deadband_rad=0.005 `
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

The recorded action is the six-field Cartesian tip offset: translation in
meters and a startup-relative rotation vector in radians. The recorded robot
state contains axes 1-4 in radians and axis 5 in meters.

The ZMQ PUSH/PULL transport intentionally supports one active LeRobot robot
client. PMAC ownership must remain exclusive to the driver service.
