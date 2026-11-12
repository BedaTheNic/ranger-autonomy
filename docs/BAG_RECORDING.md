# Full-rate Ranger test recordings

Use the opt-in rosbag2 service to correlate blocked turns with Livox returns,
scan conversion, costmaps, commands and measured steering. It records the complete
serialized ROS messages it receives, including the PointCloud2 point data.

## Build or package on the development computer

Use the `ranger-autonomy-deploy` checkout. It owns the robot Jazzy image, CI
workflow and hardware bundles. The workspace `autonomy_stack` copy supplies
development inputs and simulation tooling. Package the recorder from the
deployment checkout so the add-on includes the reviewed hardware files.

```bash
bash autonomy_stack/hardware/release.sh bag-bundle
```

This creates `autonomy_stack/hardware/dist/ranger-bag-addon.tar.gz`. The add-on
can use the existing `jazzy-r6` image. The local R6 amd64 image has Jazzy rosbag2
0.26.11 and the MCAP storage plugin. For a fresh build, the release helper can
build the Jazzy image locally and include the recorder in a full robot bundle.

```bash
bash autonomy_stack/hardware/release.sh build ranger-autonomy:jazzy-bag-local linux/amd64
DEPLOY_IMAGE=ranger-autonomy:jazzy-bag-local bash autonomy_stack/hardware/release.sh bundle
```

A bundle using the local image tag requires loading that image into Docker
on the target host. The add-on is sufficient on a robot already running R6.

## Install the add-on and record

From Windows CMD, with the add-on in Downloads, substitute the robot IP.

```cmd
scp "%USERPROFILE%\Downloads\ranger-bag-addon.tar.gz" ranger@ROBOT_IP:/home/ranger/ranger_autonomy/ranger-hardware/
ssh ranger@ROBOT_IP
```

On the robot, extract the add-on alongside the existing `.env`. The helper starts
only `ranger_hero_bag-recorder` in the separate `ranger-bag` Compose project. It
uses the autonomy image and the same ROS domain, Fast DDS UDPv4 and host network.
It runs as the SSH user's UID/GID and writes to the host's `bags/` directory.

```bash
cd ~/ranger_autonomy/ranger-hardware
tar -xzf ranger-bag-addon.tar.gz
df -h .
bash bag.sh start
bash bag.sh status
bash bag.sh logs
```

Wait for the recorder to report subscriptions for the inputs relevant to your
test before starting it. Stop the recorder after the test and copy the session
name from the `Completed ranger-…` line.

```bash
bash bag.sh stop
ls -1 bags
bash bag.sh info SESSION
```

`stop` sends SIGINT and waits for rosbag2 to flush. A successful session has
`metadata.yaml`, all MCAP splits and the hidden `.recording-complete` marker.
The marker records the allowlist and stop reason. If the container exits
nonzero, check `bash bag.sh logs` for the failure. Failed sessions stay on disk
for inspection.

## Topics, QoS and completeness

- The default [allowlist](../autonomy_stack/hardware/config/bag_topics.yaml)
  includes `/livox/lidar`, `/livox/imu`, `/scan_obstacles`, `/scan`, `/tf`,
  `/tf_static`, both raw costmaps and their updates, `/map`, `/odom`, footprints,
  plans, steering/actuator data, all command stages, navigation decisions, and
  action status and feedback. The existing [robot contract](../autonomy_stack/hardware/robot-contract.yaml)
  records the verified sensor topics. Use the CAN-backed
  `/ranger/steering_joint_states` or corrected `/ranger/actuator_state` for wheel
  angle analysis. The legacy `/actuator_state` was observed repeating one wheel's
  values.
- [QoS overrides](../autonomy_stack/hardware/config/bag_qos.yaml) request reliable
  transient-local data for static TF and the raw Nav2 costmaps. Both robot scan
  publishers were observed offering best-effort, so the scan subscriptions
  request best-effort. Other topics use rosbag2 publisher QoS discovery, including
  the observed reliable Livox cloud and a transient-local map when offered.
  Compare with live endpoints using `ros2 topic info -v`. See
  [rosbag2 0.26.11 QoS selection](https://github.com/ros2/rosbag2/blob/0.26.11/rosbag2_transport/src/rosbag2_transport/recorder.cpp)
  and [Nav2 1.3.13 costmap QoS](https://github.com/ros-navigation/navigation2/blob/1.3.13/nav2_costmap_2d/src/costmap_2d_publisher.cpp).
- AMCL, diagnostics, the drive gateway, and standalone Ackermann command stages
  only appear when those publishers are active. Raw costmap updates can be absent
  when Nav2 sends full grids. Action feedback and plans require a goal. Static TF
  or a retained map can have one sample per publisher. Some controller profiles
  may publish a different local plan topic. Inspect the live graph and add it to
  a copy of the allowlist as needed.

Use the navigation container's ROS CLI over SSH for the live graph, QoS and a
short stationary rate baseline. Stop `hz` with Ctrl+C after it prints a rate.

```bash
bash robot.sh ros planner ros2 topic list -t --include-hidden-topics
bash robot.sh ros planner ros2 topic info -v /livox/lidar
bash robot.sh ros planner ros2 topic info -v /scan_obstacles
bash robot.sh ros planner ros2 topic info -v /tf_static
bash robot.sh ros planner ros2 topic info -v /map
bash robot.sh ros planner ros2 topic info -v /local_costmap/costmap_raw
bash robot.sh ros planner ros2 topic hz /livox/lidar
```

After recording, use `bash bag.sh info SESSION` to check nonzero counts on the
cloud, scan, both TF topics and costmaps. For continuously publishing sensors,
compare the count divided by recording duration with the live baseline,
accounting for startup discovery time. The original robot observations were
approximately 10 Hz for the cloud and scans and 50 Hz for odometry. Check
`bash bag.sh logs` after stopping for QoS, transport and cache-loss warnings.
Jazzy's installed CLI does
not expose the newer rosbag2 loss-statistics option. Best-effort DDS losses may
be silent, and count comparisons can flag gaps without identifying their source.
If rosbag2 reports a missing embedded message definition, use the corresponding
Jazzy message package when decoding. The local fixture recorded and decoded
`GoalStatusArray` with Jazzy despite this warning.

## Disk and configuration

The defaults stop after 120 seconds or when sampled free space reaches 2 GiB,
whichever comes first. MCAP uses the `zstd_fast` chunk-compression preset and
rolls over around 512 MiB per file. Splitting does not cap total size. The
free-space threshold is polled, so leave enough room for concurrent writes
and finalization. Start with a short stationary capture and measure disk growth
before a longer test. Export needs about one additional copy of the selected
bags. The recorder never deletes old sessions.

```bash
RANGER_BAG_MAX_SECONDS=30 bash bag.sh start
du -sh bags/*
```

Set `RANGER_BAG_MAX_SECONDS`, `RANGER_BAG_SPLIT_BYTES`, or
`RANGER_BAG_MIN_FREE_BYTES` in the shell or deployment `.env`. For a larger
disk, set `RANGER_BAG_HOST_DIR` in `.env` to an existing writable absolute
directory outside the deployment/source checkout. Pass that same path to
the exporter with `--bags-dir`. Keep custom `--export-dir` destinations and
staging roots outside the checkout too. Set
`RANGER_BAG_TOPICS_FILE` and `RANGER_BAG_QOS_FILE` to replacement YAML files
when the deployed topic names or QoS differ.

## Export and download

The robot host exporter uses Python 3 and PyYAML (Ubuntu package
`python3-yaml`). Replace `SESSION` with a completed directory name. To archive
multiple sessions together, pass their names as separate arguments.

```bash
cd ~/ranger_autonomy/ranger-hardware
python3 bag_export.py SESSION
```

The script prints the absolute paths of a TAR and its SHA-256 file inside a
timestamped `bag-exports/` folder. It packages the completion marker,
`metadata.yaml` and every declared split for each selected session. Replace
`EXPORT_NAME` below with the printed folder name. Open a new Windows CMD
window outside SSH.

```cmd
scp -r ranger@ROBOT_IP:/home/ranger/ranger_autonomy/ranger-hardware/bag-exports/EXPORT_NAME "%USERPROFILE%\Downloads"
certutil -hashfile "%USERPROFILE%\Downloads\EXPORT_NAME\EXPORT_NAME.tar" SHA256
type "%USERPROFILE%\Downloads\EXPORT_NAME\EXPORT_NAME.tar.sha256"
```

Compare the two hashes before sharing the archive. From a PC with rsync, such
as WSL, the equivalent download is:

```bash
rsync -av --partial -e ssh ranger@ROBOT_IP:/home/ranger/ranger_autonomy/ranger-hardware/bag-exports/EXPORT_NAME/ /mnt/c/Users/WINDOWS_USER/Downloads/EXPORT_NAME/
```

For an existing mounted share or web root, explicitly stage the local export
there. If your web server already maps that directory to a URL,
`--download-base-url` prints the archive and checksum links. Configure access
on your server for recordings that contain private map or sensor data.

```bash
python3 bag_export.py SESSION --stage-dir /mnt/test-results \
  --download-base-url https://downloads.example.org/test-results/
```
