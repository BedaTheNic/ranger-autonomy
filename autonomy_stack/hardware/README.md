# Ranger autonomy — real robot, ROS 2 Jazzy

This deployment packages the locally developed adaptive autonomy stack and its
`ranger_logging` package. The canonical sources are
`ugv_sim/ranger/ranger_nav_plugins`, `ugv_sim/ranger/ranger_mini/config`, and
`ugv_sim/ranger/ranger_logging`. The Dockerfile copies these files directly.
A SHA-256 manifest of the build inputs is included in the image at
`/opt/autonomy/source-manifest.json`.

The stack retains Hybrid-A*/Lattice route selection, MPPI for Ackermann
segments, DWB for holonomic segments, footprint checks, planned rotations,
the final holonomic approach, and reuse of a valid global plan. In `jazzy-r4`,
the shortest feasible route is selected, with ETA breaking numerical ties.
A new obstacle, an infeasible path, a new goal, or failed tracking can trigger
replanning. The Jazzy migration adapts APIs, plugin names, and the behavior-tree
format while retaining the custom controllers.

Behavior details and earlier validation results are documented in
`ranger_navigation_startup/ADAPTIVE_NAVIGATION.md` and `LOGGING.md`
(under `docs/` in the exported GitHub project). Those documents also include
known failed simulation trials; they do not establish physical navigation
performance.

## Containers

Each row represents a separate container with one application ROS process.
The navigation services share the same image; there is no separate image build
for each service. Container names include `ranger_hero_planner`,
`ranger_hero_controller`, `ranger_hero_rviz-web`, `ranger_hero_drive-guard`,
and the corresponding names for other services. Compose project names remain
the same so existing services can still be identified.

| Service | Process / role | When it runs |
|---|---|---|
| planner | planner_server, AdaptivePlanner | always |
| controller | controller_server, AdaptiveController MPPI/DWB | always |
| behaviors | behavior_server | always |
| motion-guard | navigation_motion_guard, stopping-distance checks | always |
| bt-navigator | bt_navigator, custom behavior tree | always |
| navigation-lifecycle | Nav2 lifecycle manager | always |
| telemetry | ranger_logging, passive telemetry | always |
| slam | SLAM Toolbox | mapping |
| slam-lifecycle | SLAM lifecycle manager | mapping |
| map-server | map_server | localization |
| amcl | AMCL | localization |
| localization-lifecycle | map_server/AMCL lifecycle manager | localization |
| scan-adapter | PointCloud2 → LaserScan | optional |
| velocity-smoother | velocity_smoother | standalone ackermann profile |
| command-guard | final Ackermann command limits | standalone ackermann profile |

Each service's launch/console wrapper records its processes. A single
`telemetry` container records the shared ROS topics. The existing base and
Livox drivers, robot_state_publisher, and other robot applications remain in
the existing robot deployment. This stack starts neither Gazebo nor a second
EKF.

## Confirmed robot configuration

The diagnostic archive `robot_diagnostics_20261102_104903__0bzdr18.tar.gz`
confirms:

- ROS 2 Jazzy, Ubuntu 24.04, x86_64/amd64, ROS domain **9**.
- `/livox/lidar`: PointCloud2, approximately 10 Hz, frame `livox_frame`.
- `/livox/imu`: Imu, approximately 200 Hz; units and calibration are not yet verified.
- `/odom`: Odometry, approximately 50 Hz, `odom → ranger_base_link`.
- Existing LiDAR TF: translation `(0.288, -0.007, 0.046)` m and yaw `+90°`.
- Existing `/scan`, `/scan_obstacles`, `/slam_toolbox`, `/map`, and `map → odom`.
- `/ranger_base_node` subscribes to `/cmd_vel` as Twist. Teleop,
  spray_executor_node, and kinova_state_machine_node already publish to that input.

Costmaps use `/scan_obstacles` (`range_min≈0.4 m`); localization uses `/scan`
(`range_min=1.0 m`). This deployment does not publish a duplicate LiDAR TF.

Subsequent process inspection on the robot confirmed domain 9 and the loaded
`librmw_fastrtps_cpp.so` library for the base and Livox drivers. The existing
`ranger_hero_jazzy` container uses `Network=host`, `IPC=private`. Our Fast DDS
processes therefore default to `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`; the existing
container does not need to be recreated or stopped. The variable is passed
through the common Compose environment to all our services. Adding it to
`.env` alone, without passing it through Compose, would not be sufficient.
If you use custom Fast DDS XML that disables built-in transports, check that
configuration as well.

The original `jazzy-r3` deployment archive predates this IPC check. That
installation can receive the networking fix by downloading the current
`autonomy_stack/hardware/compose.yaml` into its deployment directory. The
`jazzy-r3` image itself does not need rebuilding for this Compose environment
change. Recreate our running containers with `bash robot.sh external`
(or your selected mapping/localization mode).

**The adaptive profile is the default.** Its Ackermann controller requires
fresh measured angles for `fl_steering_wheel`, `fr_steering_wheel`,
`rl_steering_wheel`, and `rr_steering_wheel` in JointState messages.
`STEERING_FEEDBACK_TOPIC` selects the input. The presence of `/joint_states`
alone is insufficient: it may contain only Kinova joints. Command-derived
angle estimates are not substituted for measured feedback.

The robot check confirmed reception of all sensors and required TF links
inside our container. The only failed item was measured steering feedback:
`/joint_states` contains Kinova joints. The driver is configured for
`ranger_mini_v3` and publishes `/actuator_state`; all eight entries in one
sample repeated the same values. A stationary sample alone neither proves a
driver fault nor confirms four independent angles.

For a passive inspection of the existing driver source and CAN feedback, run
on the robot host:

```bash
docker exec -i ranger_hero_jazzy python3 - < inspect_drive.py > ranger-drive-details.json
```

`inspect_drive.py` runs inside the existing container, uses only Python's
standard library, and requires no ROS setup. It reads selected C++ files,
active driver paths, and up to eight seconds of standard CAN messages
`0x200..0x2ff` from the existing `can0`. It does not write inside that
container, modify the CAN interface, send commands, start another driver, or
assume a physical wheel order. Missing source files or CAN access are recorded
in the JSON report. Inspecting source files on disk does not establish that
the running binary was built from those files. The script can be downloaded
separately from this directory without rebuilding the image; the original
`jazzy-r3` archive does not include it.

**CAN diagnostic correction:** the initial script (`schema: 1`) incorrectly
included `CAN_ERR_FLAG` in its receive mask. Linux then selects the error-frame
receive list, so an empty capture says nothing about standard CAN feedback.
The corrected script (`schema: 2`) uses
`CAN_EFF_FLAG | CAN_RTR_FLAG | 0x700` and also validates flags after reception.
The previously collected driver source remains useful: code found on the
robot assigns `angle_5` and actuator 0 data to every entry. There is no build
provenance linking that source to the running binary. Independent CAN angles
still need to be confirmed with a new capture.

The regression uses real Linux SocketCAN and its own virtual CAN interface
in a fully isolated network. Do not run it directly on the robot. From the
repository root, with a locally built image:

```bash
docker run --rm --network none --cap-add NET_ADMIN \
  --mount type=bind,source="$PWD/autonomy_stack/hardware",target=/test,readonly \
  --entrypoint python3 ranger-autonomy:jazzy-r3 /test/tests/inspect_drive_can.py
```

The test refuses to run if any interface other than loopback already exists
in its network namespace. The host kernel must support `vcan`. The old
filter reproduces message loss; the corrected filter must receive all test
messages and reject other IDs, extended frames, and RTR frames. This test
does not validate physical wheel calibration.

Nav2 outputs `/cmd_vel_nav`. The optional drive gateway described below
connects it to the existing `/cmd_vel` input with command timeout handling.
Command-source arbitration, measured wheel feedback, and the physical
footprint including mounted equipment still require integration or validation.
The initial limits and footprint come from the developed stack and were not
measured on this physical robot configuration. The custom amd64 MPPI build
requires AVX2/FMA.

## Standalone source package for GitHub

The intended repository is `BedaTheNic/ranger-autonomy`, with images under
`ghcr.io/bedathenic/ranger-autonomy`. Images and deployment archives are
published to GHCR and GitHub Releases after a successful workflow.

From `autonomy_stack/hardware`:

```bash
bash release.sh source "$PWD/dist/github-source"
```

If that directory already exists, choose another name; export does not
overwrite an existing package.

The export copies only required packages, configurations, tests, and
documentation. It excludes local maps, logs, `.env`, Git metadata, and
diagnostic archives. It creates no Git submodules: the exported repository
builds independently.

For a new repository, create an empty `ranger-autonomy` repository on GitHub,
then run on this computer:

```bash
cd dist/github-source
git init -b main
git add .
git commit -m "Package Ranger Jazzy autonomy and structured logging"
git remote add origin https://github.com/BedaTheNic/ranger-autonomy.git
git push -u origin main
```

Git must be authenticated as the repository owner to push. Do not use the
existing `ugv_sim` remote, which points to the upstream AgileX repository.

### Option A: build on this computer and publish to GHCR

The host needs Docker Engine and Buildx; it does not need a ROS installation.
For a native amd64 local build:

```bash
bash release.sh build ranger-autonomy:jazzy-r5
```

Only `linux/amd64` is built, matching the robot diagnostics. This amd64
computer needs neither QEMU nor an ARM build.

Log in to the registry, using your GHCR token when prompted for a password:

```bash
docker login ghcr.io -u BedaTheNic
bash release.sh publish ghcr.io/bedathenic/ranger-autonomy:jazzy-r5
```

The default build and publication target is `linux/amd64`, the real robot's
architecture. For another registry, change the image reference to, for example,
`docker.io/USER/ranger-autonomy:jazzy-r5` or
`registry.example.com/TEAM/ranger-autonomy:jazzy-r5`, and log in to that
registry. Additional tags or registries can follow the platform argument:

```bash
bash release.sh publish ghcr.io/bedathenic/ranger-autonomy:jazzy-r5 \
  linux/amd64 docker.io/USER/ranger-autonomy:jazzy-r5
```

The build context is the parent directory containing `autonomy_stack`,
`ugv_sim`, and `ranger_ros2`. `release.sh` locates that directory automatically.

### Option B: build and publish with GitHub Actions

The exported repository includes `.github/workflows/publish.yaml`. On GitHub,
open Actions → Verify and publish Ranger autonomy → Run workflow, and enter
`jazzy-r5` as the tag. Alternatively, push the Git tag `jazzy-r5`.

The workflow first builds the amd64 image and runs the original C++/logger
regressions and isolated adaptive planning/logging integration test. If they
pass, it publishes that exact tested amd64 image to GHCR without rebuilding.
It uses the repository's `GITHUB_TOKEN` with `packages: write`; the token is
not stored in source code. Repository or organization settings must allow
Actions and the required package access. The deployment archive is available
as the `ranger-hardware-deploy` workflow artifact and, after image publication,
as a GitHub Release asset. It includes a non-secret `.env` preset referencing
the release image. This workflow does not build ARM images.

## Robot: pull and run

On the build computer, create the deployment archive:

```bash
bash release.sh bundle
```

Transfer `dist/ranger-hardware-deploy.tar.gz` to the robot, or download the CI
artifact. On the robot's Docker host:

```bash
tar -xzf ranger-hardware-deploy.tar.gz
cd ranger-hardware
# The deployment archive includes .env; robot.sh creates it for a source checkout.
# Check AUTONOMY_IMAGE, domain 9, and RMW in .env.
# For a private image, first run: docker login ghcr.io -u BedaTheNic
bash robot.sh pull
```

The image is built for the robot's amd64 architecture. The robot needs neither
source code, colcon, Buildx, nor an image build.

To keep the robot's existing SLAM running:

```bash
bash robot.sh external
bash robot.sh check
```

To use this stack's own mapping service, first stop **only the existing SLAM**
through its current startup mechanism, keeping the sensors, TF, and base
running. Save the existing map and posegraph first if you need to retain them.
Then run:

```bash
bash robot.sh mapping
bash robot.sh check
```

SLAM and AMCL must not both own `map → odom`. The helper switches only services
in its own Compose project and does not stop the external robot container.

Save a map from this stack's SLAM:

```bash
bash robot.sh ros slam ros2 run nav2_map_server map_saver_cli -f /maps/map
```

For AMCL, put the map YAML and its image in `maps/`, set `MAP_FILE` in `.env`,
and run:

```bash
bash robot.sh localization
```

Set the initial AMCL pose before sending a goal. For the standalone Ackermann
profile, set `NAV_PROFILE=ackermann`; `robot.sh` automatically enables its
separate smoother and guard containers. The adaptive profile uses its own
MPPI/DWB limits.

The optional scan adapter is unnecessary when using the existing converters.
Enable it with `ENABLE_SCAN_ADAPTER=1 bash robot.sh mapping` (or
external/localization). Its output is `/autonomy/scan`; set
`OBSTACLE_SCAN_TOPIC` and/or `LOCALIZATION_SCAN_TOPIC` in `.env` if you want
to use it instead of the existing scans.

## Logs and checks

### Drive gateway

`compose.drive.yaml`, `drive.sh`, and `drive_guard.py` use the existing Jazzy
image. The script is mounted read-only; no build is needed on the robot.
The separate `ranger-drive` project connects `/cmd_vel_nav` to `/cmd_vel`.
Controller candidates pass through `/cmd_vel_candidate` and the separate
`motion-guard` service before reaching `/cmd_vel_nav`. The guard uses
`/scan_obstacles`, `/odom`, and `/local_costmap/costmap_raw` on the robot.
The navigator uses `/is_retained_path_valid` for the new retained-route checks.
It starts **disarmed**, publishes no velocities until armed, and does not
restart automatically.

For the initial test, set `NAV_PROFILE=holonomic` and run
`bash robot.sh external` again. This selects the existing ProgressDWB profile
without requiring measured steering angles; it does not validate
Ackermann/adaptive control. The main profile remains available through
`NAV_PROFILE=adaptive` after verified angle feedback is integrated.

```bash
bash drive.sh up
# Wait a few seconds for ROS discovery and controller activation.
bash drive.sh arm
```

`arm` must return `success: true`. Then send a **NEW Nav2 Goal** in RViz.
Previously accepted goals do not enable the drive. **2D Goal Pose** remains
a planning preview. For the initial trial, select a nearby goal in free space,
with the arm stowed and the physical E-stop accessible. The gateway forwards
accepted planar commands unchanged: the former
0.05 m/s and 0.10 rad/s commissioning clamp and hard ceilings are removed.
Legacy `DRIVE_MAX_LINEAR` and `DRIVE_MAX_ANGULAR` variables are ignored.
Configured Nav2 velocity and acceleration limits remain; current profiles use
0.2 m/s translational limits, with angular limits depending on the controller.
The separate motion guard can reduce or stop commands based on stopping distance
and fresh scan, costmap, and odometry data.
There is no elapsed-time disarm: the gateway remains armed until manually
disarmed or stopped by another check.

Other `/cmd_vel` publishers appear in status reports and no longer block
arming or driving. There is no priority arbitration between simultaneous
publishers; that integration remains a separate step. Checks still cover the
known `/cmd_vel_nav` publishers, an active controller, real time, fresh odometry,
and `/system_state` with no error/E-stop and CAN control mode enabled.
Adaptive/Ackermann controllers additionally require four fresh steering
JointState values. A 0.30-second navigation command timeout stops and disarms
the gateway. Other blocking conditions appear in the `arm` response and logs.

Stop driving and cancel the goal:

```bash
bash drive.sh disarm
bash robot.sh ros bt-navigator ros2 service call \
  /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal "{}"
bash drive.sh down
```

The gateway passively records `/ranger/actuator_state` as `ACTUATOR_SAMPLE`
at 5 Hz, without resetting angles or converting encoder counts into assumed
angles. Samples and gateway status use the existing logger under
`logs/drive-test`. ROS discovery checks are not DDS authentication. Gateway
tests on an isolated Docker network do not validate the physical E-stop or
the firmware watchdog during a power loss or SIGKILL on the robot.

From the repository root, run
`python3 autonomy_stack/hardware/tests/drive_smoke.py --image IMAGE` to test
with a synthetic base and controller on an internal network. The test covers
old-goal rejection, permitted competing publishers, command timeout, E-stop
state, stale odometry, and missing steering feedback in the adaptive profile.

<a id="nadogradnja-i-ackermann-dijagnostika"></a>

### Upgrade and Ackermann diagnostics

`jazzy-r5` includes the latest local planner, tracking, wheel-control and
logging changes: validated Hybrid smoothing, retained-path validation and
repair, route clearance/alignment updates, and a separate stopping-distance
motion guard. The drive gateway no longer imposes a maximum speed. Controller
limits and obstacle-dependent braking remain. `jazzy-r3-gui` stays compatible.

To upgrade an existing deployment, first refresh its helpers with
`apply_update.sh FULL_COMMIT_SHA` as described below. This is required because
`drive_guard.py` is mounted from the robot host: pulling a new image alone does
not replace an older host script that still clamps speed. The helper refresh
also adds the `motion-guard` service to Compose. Then, while stationary:

```bash
bash drive.sh disarm
cp -p .env ".env.before-r5.$(date +%Y%m%d_%H%M%S)"
sed -i 's|^AUTONOMY_IMAGE=.*|AUTONOMY_IMAGE=ghcr.io/bedathenic/ranger-autonomy:jazzy-r5|' .env
bash robot.sh pull
bash robot.sh external
bash drive.sh up
bash robot.sh check
# Once the nodes are ready, arm the gateway and send a NEW goal:
bash drive.sh arm
```

`external` is the mode used on this robot with its existing SLAM. If you use
this stack's mapping or AMCL, retain the corresponding `mapping` or
`localization` mode. A new image requires recreating application containers:
save any edits in their writable layers to source code or an image first.
The existing `NAV_PROFILE`, maps, logs, and vendor container
`ranger_hero_jazzy` are preserved.

`apply_update.sh FULL_COMMIT_SHA` downloads deployment files only, validates
them before making changes, and backs up the previous versions under
`update_backups`. Run it while the robot is stationary. It uses
`docker rename` to rename existing containers in place, including stopped
containers, without recreating application containers. Only the drive gateway
is disarmed, stopped, and restarted to load the updated Python script.
It does not overwrite `.env`, maps, logs, or planner/controller C++ sources.
An unknown additional Compose service requires a manual configuration merge.
A later `compose up` may recreate containers if their configuration changes;
save edits from their writable layers to source code or an image first.

On the configured Mini V3, the driver selects the base mode from `Twist`:

| Input | Base mode |
|---|---|
| `linear.y != 0` | parallel, mode 1 |
| `linear.y = 0`, `linear.x != 0`, `angular.z = 0` | dual Ackermann, straight |
| `linear.y = 0`, `abs(linear.x/angular.z) >= 0.47644 m` | dual Ackermann, mode 0 |
| `linear.y = 0`, smaller radius, including rotation in place | spinning, mode 2 |

The driver compares `linear.y` to zero without a tolerance: even a tiny
nonzero value selects parallel mode. For example,
`vx=0.05 m/s, vy=0, wz=0.025 rad/s` describes an Ackermann arc with a 2 m
radius. The gateway preserves this command unchanged. This describes the driver's mode selection.
The MPPI/Ackermann Nav2 profile additionally requires verified feedback from
all four steering wheels.

Sources: [Ranger ROS2 driver](https://github.com/agilexrobotics/ranger_ros2/blob/humble/ranger_base/src/ranger_messenger.cpp),
[SDK CAN parser](https://github.com/agilexrobotics/ugv_sdk/blob/main/src/protocol_v2/agilex_msg_parser_v2.c).

To compare ROS measurements with raw CAN feedback:

```bash
bash ackermann_check.sh 20
```

The capture sends no ROS or CAN commands. During those 20 seconds, you can
repeat the previously prepared short trial in a separate window while the
wheels change steering angle. The archive includes `feedback.json`, an
inspection of the existing driver source, current parameters, and recent
planner/controller logs. CAN `0x271` contains four signed big-endian angles
for channels 5–8 in units of 0.001 rad; `0x291` contains mode and
`mode_changing`; `0x251..0x258` contain actuator RPM, current, and pulse
counts. Encoder counts are not converted into angles. The report shows
minimum, maximum, and last angles by ID, along with receive freshness.
A constant angle while stationary is not itself a fault. Physical wheel
order, signs, and zero offsets still need confirmation before implementing
a JointState adapter for MPPI.

### Windows CMD: map viewing and goals in the browser

RViz/noVNC runs in the separate amd64 image
`ghcr.io/bedathenic/ranger-autonomy:jazzy-r3-gui`, built on the verified
`jazzy-r3` navigation image. `rviz-web` is the only RViz ROS process.
The separate `goal-preview` service receives preview clicks and calls only
`/compute_path_to_pose`. Existing navigation services and the robot's
`ranger_hero_jazzy` remain in their own Compose projects. The viewer uses
`ranger-viewer`, so restarting `robot.sh external` does not remove it as an
orphan.

The deployment directory must contain `compose.gui.yaml` and `viewer.sh`
from this repository; the original `jazzy-r3` archive does not include them.
On the robot host:

```bash
cd ~/ranger_autonomy/ranger-hardware
bash viewer.sh pull
bash viewer.sh up
bash viewer.sh status
```

On the Windows computer, open CMD and start an SSH tunnel, replacing
`ROBOT_IP` with the robot's address:

```cmd
ssh -N -L 6080:127.0.0.1:6080 ranger@ROBOT_IP
```

Keep that window open. In a browser on the Windows computer, open
`http://localhost:6080/vnc.html?autoconnect=true&resize=scale`.
noVNC listens only on the robot's loopback interface; access goes through the
authenticated SSH tunnel. Windows needs neither ROS nor an X server. If local
port 6080 is occupied, use `-L 6081:127.0.0.1:6080` and open localhost:6081.

The RViz fixed frame is `map`. The view displays `/map`, the TF pose of
`ranger_base_link`, `/odom`, `/scan_obstacles`, the footprint, and a blue
preview path. Robot position is shown by the red odometry arrow and base TF
marker. The footprint comes from the costmap configuration and does not
confirm the measured outline with mounted equipment. A 3D model and dense
point cloud are unnecessary for this map view. Use the mouse wheel to zoom
and the middle mouse button to pan.

To preview a goal, select **2D Goal Pose**, left-click a free location, and
drag toward the desired final orientation. The click publishes
`/autonomy/preview_goal`; `goal-preview` calls the developed adaptive planner
with `use_start=false` and publishes the result on `/autonomy/preview_path`.
This computes a path without sending `/navigate_to_pose` or velocity commands.
A failed goal clears the previous preview path. Status is available through
`bash viewer.sh logs` and `/autonomy/preview_status`. `goal-preview` records
its console through the existing logger; `telemetry` continues to record
the original global plan and planner decisions.

Once physical drive integration is ready, set
`RANGER_GUI_NAVIGATION_TOOLS=true` in `.env` and run `bash viewer.sh up`
again. This adds the standard **Nav2 Goal** tool and Navigation 2 panel, which
use `NavigateToPose` and the existing custom behavior tree/controller.
This setting alone neither connects `/cmd_vel_nav` to the drive nor supplies
measured wheel angles.

Stop only the viewer:

```bash
bash viewer.sh down
```

The `publish-gui.yaml` workflow builds only the amd64 GUI image. Before
publication, it verifies RViz/noVNC and goal forwarding through the real ROS
action API on an isolated Docker network. Its synthetic action server does
not provide new validation of the planning algorithm or physical driving.

Each mapping/localization/external invocation receives a deployment ID.
Logs are stored on the host under
`logs/<ID>/YYYY-MM-DD/<service-session>/`:

- A command manifest, timestamps, and exit status.
- Separate sensor, odometry, TF, planning, control, and lifecycle segments.
- Loaded parameters and hashes, plans, ETA, and mode changes.
- Map/costmap snapshots on planning, failure, or cancellation.
- Rotation at 20 MiB without overwriting earlier parts.
- Summaries of dropped records and write errors.

Telemetry is sampled diagnostics rather than a full rosbag. Livox clouds are
recorded as cloud metadata; scans, IMU, and odometry are sampled as messages.
The logger does not convert unverified IMU units or publish control commands.

```bash
bash robot.sh status
bash robot.sh logs telemetry
bash robot.sh ros controller ros2 lifecycle get /controller_server
bash robot.sh ros telemetry ros2 run ranger_logging read_logs.py \
  /logs/ID/YYYY-MM-DD/SESSION --segment diagnostics --event health
bash robot.sh down
```

`check` only reads ROS data. It checks sensors, odometry frames, TF, and the
required steering joints; it does not validate physical limits or driving
safety. Maps and logs remain after shutdown. For a new release, change the
image tag in `.env`, then repeat `pull` and the selected startup mode.

The amd64 build, startup, planning, and structured logging have been validated
locally. The `jazzy-r4` image was verified in GHCR. The new `jazzy-r5` release
uses the same tested build/publish workflow. The initial physical holonomic motion test was
reported successful. Full physical adaptive/Ackermann navigation, wheel
feedback mapping, and physical limits still require validation on the robot.

Sources:

- [Docker: multi-platform images with GitHub Actions](https://docs.docker.com/build/ci/github-actions/multi-platform/)
- [GitHub: publishing Docker images](https://docs.github.com/en/actions/tutorials/publish-packages/publish-docker-images)
