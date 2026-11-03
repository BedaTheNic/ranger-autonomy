# Diagnostic logging

`simulation.sh` enables the logger automatically. Every invocation gets a
separate session under `ranger_navigation_startup/logs/YYYY-MM-DD/`, with
the date, time, ROS domain, command, and a unique suffix in its name. The path
is printed at startup. Restart existing processes to use the new logger.

```bash
cd /home/bedathenic/bedenic_ws/src/ranger_navigation_startup
bash simulation.sh logs
ls -l logs/latest-*
```

The `latest-navigate-d87` symlink points to the latest navigation session in
domain 87; `latest-slam-d87` points to the latest SLAM session. The
`keyboard`, `initial_pose`, validation, `build`, and `test` commands have
their own sessions. The logger launched with the simulation collects driving
telemetry. Running the keyboard command alone records its output without
starting another telemetry recorder.

## Files and rotation

- `manifest.json`: command, domain, start time, end time, and exit status.
- `segments/<segment>__<date-time>__<process>__000001.jsonl`: structured
  records grouped by segment and source (launch, ROS telemetry, or console).
- `*-summary.json`: record counts, losses, write errors, and queue-drain status.
- `telemetry-*-metadata.json`: sampling configuration.

The default limit is **20 MiB per JSONL file**. A new file opens before the
limit is exceeded; earlier files are neither overwritten nor automatically
deleted. Large individual records are split into numbered `record_chunk`
parts with SHA-256 verification. The reader described below reassembles them.
To change file size, location, or sampling, set these variables before your
usual startup command:

```bash
export RANGER_LOG_MAX_BYTES=10485760  # 10 MiB; minimum 4096 bytes
export RANGER_LOG_ROOT=/home/bedathenic/ranger_logs
export RANGER_LOG_SAMPLE_HZ=5
# Then run your existing simulation.sh sim/slam/localize/navigate command.
```

`RANGER_LOG_ENABLED=0` disables this system and leaves standard ROS logging
enabled. Direct `ros2 launch ranger_mini ...` invocations also enable the
logger, but without the console wrapper; their default location is
`~/.ros/ranger_logs`.

## Recorded data

| Segment | Data |
|---|---|
| `sensors` | LiDAR scans, IMU, covariances, and timestamps |
| `base_control` | Wheel and steering states, commanded joint positions and velocities, controller messages |
| `odometry` | Encoder, EKF, and Gazebo reference odometry; distance, speed, angle, and kinematic residual |
| `tf` | Static and sampled dynamic transforms |
| `slam`, `localization` | SLAM/AMCL messages, map, estimated pose, and initial pose with covariance |
| `planning` | Global plan, route/ETA decision, global costmap, and footprint |
| `control` | `cmd_vel` before/after smoothing where available, local plan/costmap, driving-mode changes and their reasons |
| `navigation` | Navigation action status and UUIDs, feedback, and behavior-tree state changes |
| `lifecycle` | Console messages, discovered nodes, and lifecycle transitions after the logger attaches |
| `configuration` | Launch arguments, ROS package/Python/RMW versions, selected input text files with hashes, loaded parameters, and parameter changes |
| `simulation`, `system`, `console` | Gazebo/RViz and other processes, startup/shutdown, exit status, stdout/stderr |
| `diagnostics` | Received-message frequency and age, missing expected topics, TF parents/publishers, free disk space, and logger losses |

Records contain UTC time, local Europe/Zagreb time, monotonic time, and ROS
time where available. Original messages retain their own timestamps.
ROS messages are recorded at the level emitted by the node (INFO and above
by default). Console and `/rosout` content may intentionally overlap:
console capture also covers failures before a node starts publishing ROS
messages.

Telemetry is a diagnostic sample rather than a complete rosbag. Defaults
are 5 Hz for odometry/IMU/joints, 10 Hz for velocity commands, and 1 Hz for
LiDAR and the local plan. Events, global plans, and ROS log messages are
not time-throttled. System health is checked every two seconds of wall time,
including while Gazebo is paused.

Full compressed map and costmap snapshots are saved every 30 seconds, on
a new global plan, a ROS ERROR/FATAL message, a behavior-tree failure, or a
canceled/failed navigation action. Snapshots are at least one second apart;
triggers within that interval are still recorded as events. A
`grid_snapshot` record contains metadata, age, SHA-256, and `zlib/base64`
cell data. Maps use `int8`; costmaps use `uint8`. This project's
configuration publishes full costmaps. For other systems that publish only
incremental changes, this logger does not reconstruct `costmap_updates`.

The logger only reads topics and parameters; it sends no control commands.
Parameters are fetched incrementally, from at most two nodes at once.
TF publishers are recorded at topic level: the Humble Python API does not
provide a publisher identity for each transform, so this does not prove which
node publishes a particular TF frame. `conflicting_tf_parents` identifies
different observed parents for the same frame. Action status/feedback does
not always contain the original goal request; `/goal_pose`, when published,
and the goal of a successfully generated plan provide additional context.

A separate thread writes records through a bounded queue. A full queue or
disk error can cause record loss: counters appear in `health`, `log_loss`,
and the final summary, while disk errors are also printed to stderr.
Abrupt shutdown or SIGKILL can leave an incomplete final record or snapshot;
the reader reports this. Durability during power loss is not guaranteed,
and old sessions are not deleted automatically.

Standard unbounded ROS disk logs are disabled for nodes in these launch files;
the rotating system captures their messages instead. The small initial
`launch.log` is closed when our launch is loaded. Existing logs, external
nodes started by other commands, and original colcon build artifacts are
outside this rotation policy.

## Inspecting and sharing logs for debugging

```bash
cd /home/bedathenic/bedenic_ws/src/ranger_navigation_startup
python3 ../ugv_sim/ranger/ranger_logging/scripts/read_logs.py \
  logs/latest-navigate-d87 --segment control --event ros_log

python3 ../ugv_sim/ranger/ranger_logging/scripts/read_logs.py \
  logs/latest-navigate-d87 --level ERROR

python3 ../ugv_sim/ranger/ranger_logging/scripts/read_logs.py \
  logs/latest-navigate-d87 --segment diagnostics --event health
```

The reader visits files in filename order. Use `wall_time_utc` and
`monotonic_ns` to correlate different sources in time. For analysis, share
the entire session directory, including every `segments` part, the manifest,
and summaries. Logs contain robot maps, scans, paths, and parameters.

`bash simulation.sh test` includes tests for storage, rotation, large-record
reassembly, disk errors, queue overflow, and signal forwarding.

Validation on 2026-11-01 passed 35 Python tests (11 for logging), along with
an isolated Gazebo/Nav2 drive, lifecycle-event capture, a snapshot triggered
by a deliberately marked test error, paused-clock detection, and resumed
operation. The session produced 144 rotated JSONL files, each no larger than
65,536 bytes, with no reported losses. Clean logger shutdown after repeated
SIGINT was also verified. Details:
[logging-summary-20261101.json](validation/logging-summary-20261101.json).
