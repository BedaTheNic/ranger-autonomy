#!/usr/bin/env python3
"""Record an explicit set of full ROS messages and finalize on stop."""

import datetime as dt
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import time
from uuid import uuid4

import yaml


def positive_int(name, default):
    value = int(os.environ.get(name, default))
    if value <= 0:
        raise ValueError(f"{name} must be positive")
    return value


def read_topics(path):
    with path.open() as stream:
        config = yaml.safe_load(stream)
    topics = config.get("topics") if isinstance(config, dict) else None
    if not isinstance(topics, list) or not topics:
        raise ValueError(f"{path} needs a nonempty 'topics' list")
    for topic in topics:
        if not isinstance(topic, str) or not re.fullmatch(r"(?:/[a-zA-Z_][a-zA-Z_0-9]*)+", topic):
            raise ValueError(f"Invalid absolute ROS topic in {path}: {topic!r}")
    if len(set(topics)) != len(topics):
        raise ValueError(f"Duplicate topic in {path}")
    return topics


def record():
    root = Path(os.environ.get("RANGER_BAG_ROOT", "/bags"))
    topics = read_topics(Path(os.environ["RANGER_BAG_TOPICS_PATH"]))
    qos_path = Path(os.environ["RANGER_BAG_QOS_PATH"])
    if not qos_path.is_file():
        raise ValueError(f"Missing QoS configuration: {qos_path}")
    split_bytes = positive_int("RANGER_BAG_SPLIT_BYTES", 512 * 1024**2)
    max_seconds = positive_int("RANGER_BAG_MAX_SECONDS", 120)
    min_free_bytes = positive_int("RANGER_BAG_MIN_FREE_BYTES", 2 * 1024**3)
    if shutil.disk_usage(root).free <= min_free_bytes:
        raise ValueError(f"Free space at {root} must exceed {min_free_bytes} bytes")

    utc = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    session = root / f"ranger-{utc}-{uuid4().hex[:8]}"
    storage = os.environ.get("RANGER_BAG_STORAGE", "mcap")
    command = [
        "ros2", "bag", "record", "--output", str(session), "--storage", storage,
        "--node-name", "ranger_bag_recorder", "--include-hidden-topics",
        "--qos-profile-overrides-path", str(qos_path),
        "--max-bag-size", str(split_bytes), "--max-cache-size", str(64 * 1024**2),
    ]
    if storage == "mcap":
        command += ["--storage-preset-profile", os.environ.get("RANGER_BAG_MCAP_PRESET", "zstd_fast")]
    if os.environ.get("USE_SIM_TIME") == "true":
        command.append("--use-sim-time")
    if os.environ.get("ROS_DISTRO") != "humble":
        command += ["--disable-keyboard-controls", "--topics"]
    command += topics

    stop_reason = None

    def stop(signum, _frame):
        nonlocal stop_reason
        stop_reason = stop_reason or signal.Signals(signum).name

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    print(f"Recording {session.name} for up to {max_seconds}s", flush=True)
    start = time.monotonic()
    process = subprocess.Popen(command, stdin=subprocess.DEVNULL, start_new_session=True)
    try:
        while process.poll() is None and stop_reason is None:
            if time.monotonic() - start >= max_seconds:
                stop_reason = "duration limit"
            elif shutil.disk_usage(root).free <= min_free_bytes:
                stop_reason = "free space threshold"
            else:
                time.sleep(0.25)
    finally:
        if process.poll() is None:
            print(f"Finalizing {session.name}: {stop_reason or 'recorder error'}", flush=True)
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=75)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
                raise RuntimeError(f"{session.name}: rosbag2 did not finalize within 75s") from None

    if process.returncode != 0 or stop_reason is None:
        raise RuntimeError(f"{session.name}: rosbag2 exited unexpectedly ({process.returncode})")
    with (session / "metadata.yaml").open() as stream:
        metadata = yaml.safe_load(stream)["rosbag2_bagfile_information"]
    if metadata["message_count"] <= 0 or not metadata["relative_file_paths"]:
        raise RuntimeError(f"{session.name}: rosbag2 did not record any messages")
    marker = session / ".recording-complete.tmp"
    with marker.open("x") as stream:
        json.dump({"stop_reason": stop_reason, "topics": topics,
                   "elapsed_seconds": round(time.monotonic() - start, 3)}, stream)
        stream.write("\n")
    marker.rename(session / ".recording-complete")
    print(f"Completed {session.name}: {metadata['message_count']} messages", flush=True)


if __name__ == "__main__":
    try:
        record()
    except (OSError, ValueError, KeyError, TypeError, RuntimeError, yaml.YAMLError) as error:
        print(f"Recorder failed: {error}", file=sys.stderr)
        sys.exit(1)
