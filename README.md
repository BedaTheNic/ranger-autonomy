# Ranger autonomy — ROS 2 Jazzy

Adaptive Ranger Mini navigation and structured logging, split into separate ROS containers.

- Global planning: custom Smac Hybrid-A* / Lattice selection.
- Local control: custom adaptive MPPI Ackermann / DWB holonomic execution.
- Retained valid paths, planned rotations and final approach behavior.
- Separate SLAM, AMCL, map server, lifecycle, navigation and telemetry services.
- Segmented JSONL logging, dated sessions, 20 MiB rotation and costmap snapshots.

## Deploy on the robot

After the `jazzy-r2` workflow has completed, download the release deployment bundle:

```bash
curl -fL https://github.com/BedaTheNic/ranger-autonomy/releases/download/jazzy-r2/ranger-hardware-deploy.tar.gz -o ranger-hardware-deploy.tar.gz
tar -xzf ranger-hardware-deploy.tar.gz
cd ranger-hardware
bash robot.sh pull
bash robot.sh external
bash robot.sh check
```

The bundle includes a non-secret `.env` preset. The image is
`ghcr.io/bedathenic/ranger-autonomy:jazzy-r2`, with amd64 and arm64 variants.
For a private GHCR package, first run `docker login ghcr.io -u BedaTheNic` on the robot.
No source checkout or build is needed on the robot.

`external` uses the robot's existing SLAM. To run this stack's SLAM, stop only the
existing SLAM while retaining drivers and TF, then use `bash robot.sh mapping`.
For a saved map, use `bash robot.sh localization` with map assets in `maps/`.

The robot preset uses ROS domain 9, `ranger_base_link`, `/odom`, `/livox/lidar`,
`/livox/imu`, `/scan` and `/scan_obstacles`. Adaptive navigation is the default.
The driver container must share compatible DDS/network settings.

**Physical driving is not yet integrated:** `/cmd_vel_nav` is separate from the
robot's `/cmd_vel`, which already has other command publishers. Measured Ranger
steering joint feedback, command arbitration/watchdog, footprint and limits must
be validated before connecting the drive. A successful image build does not
validate physical navigation.

## Build and publish

[Complete build/deployment instructions](autonomy_stack/hardware/README.md)
cover local Docker builds, GHCR/Docker Hub/private registries and both CPU architectures.
The [GitHub Actions workflow](.github/workflows/publish.yaml) verifies the original
navigation/logging regressions and real inter-container planning/logging before
publishing images and a deployment archive. Push a `jazzy-*` tag or run it manually.

## Developed behavior and logging

- [Navigation behavior and original simulation results](docs/ADAPTIVE_NAVIGATION.md)
- [Structured logging and diagnostic format](docs/LOGGING.md)
- [Robot interface contract](autonomy_stack/hardware/robot-contract.yaml)

Canonical package/config inputs are embedded in each image's
`/opt/autonomy/source-manifest.json`. Local maps, logs and credentials are not
committed. Initial validation covers amd64 synthetic inputs; native arm64 runtime
and the real robot remain separate validation steps.
