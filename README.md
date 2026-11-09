# Ranger autonomy — ROS 2 Jazzy

Adaptive Ranger Mini navigation and structured logging, split into separate ROS containers.

Current deployment helpers use `ranger_hero_<service>` container names. The drive
gateway permits other `/cmd_vel` publishers and has no elapsed-time disarm;
command timeout, odometry and E-stop checks remain. The gateway forwards accepted
controller speeds unchanged, without a commissioning maximum-speed clamp. Existing installations can
apply a pinned revision with `autonomy_stack/hardware/apply_update.sh`.
Ackermann ROS/CAN captures use `ackermann_check.sh`; see the hardware deployment
instructions for the current update and diagnostic procedures.

- Global planning: custom Smac Hybrid-A* / Lattice selection.
- Local control: custom adaptive MPPI Ackermann / DWB holonomic execution.
- Retained valid paths, planned rotations and final approach behavior.
- Separate SLAM, AMCL, map server, lifecycle, navigation and telemetry services.
- Segmented JSONL logging, dated sessions, 20 MiB rotation and costmap snapshots.

## Navigation footprint

The hardware envelope selected in the local setup is 0.738 m long and 0.500 m wide, centered
on `ranger_base_link`. During the image build, `prepare_config.py` writes that
polygon into the local and global costmap sections of
`/opt/autonomy/hardware/config/navigation_common.yaml`. Both costmaps and the
planners use zero additional footprint padding. The Gazebo configuration uses
its own 0.640 × 0.600 m model envelope.
[Hardware polygon generator](autonomy_stack/hardware/prepare_config.py) ·
[Gazebo costmaps](ugv_sim/ranger/ranger_mini/config/navigation_common.yaml) ·
[Planner settings](ugv_sim/ranger/ranger_mini/config/navigation_adaptive.yaml)

## Deploy on the robot

Download the `jazzy-r6` deployment bundle:

```bash
curl -fL https://github.com/BedaTheNic/ranger-autonomy/releases/download/jazzy-r6/ranger-hardware-deploy.tar.gz -o ranger-hardware-deploy.tar.gz
tar -xzf ranger-hardware-deploy.tar.gz
cd ranger-hardware
bash robot.sh pull
bash robot.sh external
bash robot.sh check
```

The bundle includes a non-secret `.env` preset. The image is
`ghcr.io/bedathenic/ranger-autonomy:jazzy-r6`, for the robot’s amd64 architecture.
For a private GHCR package, first run `docker login ghcr.io -u BedaTheNic` on the robot.
No source checkout or build is needed on the robot.

`external` uses the robot's existing SLAM. To run this stack's SLAM, stop only the
existing SLAM while retaining drivers and TF, then use `bash robot.sh mapping`.
For a saved map, use `bash robot.sh localization` with map assets in `maps/`.

The robot preset uses ROS domain 9, `ranger_base_link`, `/odom`, `/livox/lidar`,
`/livox/imu`, `/scan` and `/scan_obstacles`. Adaptive navigation is the default.
The confirmed driver container `ranger_hero_jazzy` uses host networking and private IPC.
The current Compose config defaults our Fast DDS processes to UDPv4 for compatibility.
The original jazzy-r3 deployment bundle needs this Compose-only update; no image rebuild is needed.

The optional drive gateway (`bash drive.sh up`) starts disarmed. The first
holonomic motion test passed on the robot; full adaptive/Ackermann navigation
still requires verified measured steering feedback and physical calibration.
The gateway permits other `/cmd_vel` publishers; arbitration is not implemented.

`jazzy-r5.1` rebuilds the latest local planning, tracking, wheel-control and
logging sources. It includes validated raw/smoothed Hybrid routes, retained-path
validation and local repair, and a separate `ranger_hero_motion-guard` container
for the developed stopping-distance checks. The gateway's former 0.05 m/s and
0.10 rad/s clamp and its hard commissioning ceilings are removed. Configured
Nav2 velocity/acceleration limits and obstacle-dependent braking remain.
The existing `jazzy-r3-gui` viewer is compatible.

`jazzy-r5.2.1` applies the local hardware envelope to both costmaps and removes
additional costmap, planner, lattice-search and shortcut footprint reserves.
Inflation costs, swept-footprint collision checks and stopping-distance checks
remain enabled. The measured CAN steering adapter publishes
`/ranger/steering_joint_states` for adaptive navigation. The configured envelope
has not been measured against the complete robot and its mounted equipment.

`jazzy-r6` packages the segment planning and retained-path behavior tested locally
on November 8–9, 2026. Candidate routes account for footprint clearance,
corridor centering, forward-facing travel and scheduled mode changes. A brief
obstruction suspends following; a second blocked check authorizes a replacement.
When the same stamped route resumes, the controller retains its completed
segments, pivots and gear progress. The Jazzy release tests cover the actual
NavigateToPose behavior tree, measured CAN feedback and all three profiles.

For an existing installation, preserve `.env` (including `NAV_PROFILE`), maps
and logs, stop/disarm driving, change `AUTONOMY_IMAGE` to the new tag, then pull
and recreate the navigation and gateway containers.

## Build and publish

The [GitHub Actions workflow](.github/workflows/publish.yaml) verifies the original
navigation/logging regressions and real inter-container planning/logging before
publishing images and a deployment archive. Push a `jazzy-*` tag or run it manually.

## Developed behavior and logging

- [Navigation behavior and original simulation results](docs/ADAPTIVE_NAVIGATION.md)
- [Structured logging and diagnostic format](docs/LOGGING.md)
- [Robot interface contract](autonomy_stack/hardware/robot-contract.yaml)

Canonical package/config inputs are embedded in each image's
`/opt/autonomy/source-manifest.json`. Local maps, logs and credentials are not
committed. Initial validation covers amd64 synthetic inputs; the real robot remains a separate validation step.
