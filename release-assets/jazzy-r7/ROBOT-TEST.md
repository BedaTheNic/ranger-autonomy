# Jazzy R7 robot test

This Linux/amd64 build packages the current simulator's opt-in adaptive
footprint escape. The test procedure enables it on the existing Ranger
deployment. The image is available on GHCR and the bundle is on GitHub Releases.

## Connect to the robot

Open Windows CMD and replace the robot IP.

```cmd
set "ROBOT_IP=REPLACE_WITH_ROBOT_IP"
ssh ranger@%ROBOT_IP%
```

## Update the existing R6 installation

With the robot stationary, download and run the updater in the robot SSH session.
It verifies the deployment archive and the image tested in CI. It prints the
backup directory for the original `.env`, Compose and recorder files.

```bash
curl -fsSL --retry 3 https://github.com/BedaTheNic/ranger-autonomy/releases/download/jazzy-r7/ranger-r7-update.sh -o /home/ranger/ranger-r7-update.sh &&
bash /home/ranger/ranger-r7-update.sh
```

## Check and record

Run these checks from the deployment directory. The sensor and steering
checks should report `passed: true`, the lifecycle response should report
`success=True`, and both parameters should read back `true`.

```bash
cd /home/ranger/ranger_autonomy/ranger-hardware
bash robot.sh status
bash robot.sh check
bash robot.sh steering-check
bash robot.sh ros navigation-lifecycle ros2 service call /lifecycle_manager_navigation/is_active std_srvs/srv/Trigger '{}'
bash robot.sh ros controller ros2 param get /controller_server FollowPath.footprint_escape_enabled
bash robot.sh ros controller ros2 param get /controller_server FollowPath.Holonomic.footprint_escape_enabled
RANGER_BAG_MAX_SECONDS=240 bash bag.sh start
bash bag.sh logs
```

When the recorder lists the LiDAR and obstacle scan subscriptions, open the
RViz tunnel in a second Windows CMD window. Replace the robot IP there too.
If you set `RANGER_GUI_PORT` in `.env`, use that port in both the tunnel and
the browser URL.

```cmd
ssh -N -L 6080:127.0.0.1:6080 ranger@ROBOT_IP
```

Open [RViz](http://localhost:6080/vnc.html?autoconnect=true&resize=scale).
Start with a short clear-space goal. After successful arming, send a new
**Nav2 Goal**.

```bash
bash drive.sh arm
```

For the blocked-turn test, start from a clear footprint and use a controlled
obstacle near the intended turn. The planner may choose a clear route around
it before the controller needs recovery. A nonempty
`/footprint_escape_status` message and `HOLONOMIC_FOOTPRINT_ESCAPE` in
`/navigation_mode_decision` show that the controller entered recovery. The
bag records both topics, the command stages, scans and costmaps. Some blocked
optional heading adjustments still do not trigger escape.

```bash
bash drive.sh disarm
bash bag.sh stop
```

## Export and download the bag

Copy the session name from the `Completed` line. The exporter prints the
timestamped folder containing the TAR and its checksum.
If you set `RANGER_BAG_HOST_DIR` in `.env`, pass that same directory to the
exporter with `--bags-dir /your/bag/directory`.

```bash
python3 -c 'import yaml' || sudo apt-get install -y python3-yaml
BAG_SESSION='REPLACE_WITH_COMPLETED_SESSION_NAME'
bash bag.sh info "$BAG_SESSION"
python3 bag_export.py "$BAG_SESSION"
exit
```

Back in Windows CMD, use the printed folder name and compare the two hashes.

```cmd
set "BAG_EXPORT=REPLACE_WITH_EXPORT_FOLDER_NAME"
scp -r "ranger@%ROBOT_IP%:/home/ranger/ranger_autonomy/ranger-hardware/bag-exports/%BAG_EXPORT%" "%USERPROFILE%\Downloads"
certutil -hashfile "%USERPROFILE%\Downloads\%BAG_EXPORT%\%BAG_EXPORT%.tar" SHA256
type "%USERPROFILE%\Downloads\%BAG_EXPORT%\%BAG_EXPORT%.tar.sha256"
```

To restore R6, disarm, restore the backed-up `.env` and `compose.yaml` from the
printed backup directory, then run `bash robot.sh external` and
`bash drive.sh up` from the deployment directory.
