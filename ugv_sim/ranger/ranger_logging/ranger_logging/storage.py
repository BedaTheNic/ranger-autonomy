"""Append-only JSONL parts, bounded asynchronous writes and session metadata."""
import base64
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import re
import sys
import threading
import time
import uuid
from zoneinfo import ZoneInfo


def clean(value):
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)  # Keep inf/nan explicit, without invalid JSON numbers.
    if isinstance(value, dict):
        return {str(k): clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(v) for v in value]
    return value


def encoded(value):
    return (json.dumps(clean(value), ensure_ascii=False, allow_nan=False,
                       separators=(',', ':'))+'\n').encode('utf-8')


def times():
    now = datetime.now(timezone.utc)
    return dict(wall_time_utc=now.isoformat(timespec='microseconds'),
                wall_time_local=now.astimezone(ZoneInfo('Europe/Zagreb')).isoformat(timespec='microseconds'),
                monotonic_ns=time.monotonic_ns())


def atomic_json(path, data):
    path = Path(path)
    temp = path.with_name(path.name+'.'+uuid.uuid4().hex+'.tmp')
    try:
        temp.write_bytes(encoded(data))
        os.replace(temp, path)
    finally:
        temp.unlink(missing_ok=True)


def create_session(root, action):
    now = datetime.now(ZoneInfo('Europe/Zagreb'))
    action = re.sub(r'[^a-zA-Z0-9_-]', '_', action)
    name = now.strftime('%Y-%m-%d_%H-%M-%S.%f')
    name += f'_d{os.environ.get("ROS_DOMAIN_ID", "0")}_{action}_{os.getpid()}_{uuid.uuid4().hex[:6]}'
    path = Path(root).expanduser().resolve()/now.strftime('%Y-%m-%d')/name
    path.mkdir(parents=True, exist_ok=False)
    return path


def max_bytes():
    value = int(os.environ.get('RANGER_LOG_MAX_BYTES', str(20*1024*1024)))
    if value < 4096:
        raise ValueError('RANGER_LOG_MAX_BYTES must be at least 4096')
    return value


class Parts:
    """Never rename/delete old parts. Oversized records use reconstructable chunks."""
    def __init__(self, directory, owner, limit):
        if limit < 4096:
            raise ValueError('Part size must be at least 4096 bytes')
        self.directory, self.owner, self.limit = Path(directory), owner, limit
        self.directory.mkdir(parents=True, exist_ok=True)
        self.stamp = datetime.now(ZoneInfo('Europe/Zagreb')).strftime('%Y-%m-%d_%H-%M-%S.%f')
        self.files, self.sizes, self.indices = {}, {}, Counter()
        self.written = Counter()

    def _line(self, segment, line):
        segment = re.sub(r'[^a-zA-Z0-9_-]', '_', segment)
        assert len(line) <= self.limit
        if segment not in self.files or self.sizes[segment]+len(line) > self.limit:
            if segment in self.files:
                self.files.pop(segment).close()
            while True:
                self.indices[segment] += 1
                name = f'{segment}__{self.stamp}__{self.owner}__{self.indices[segment]:06d}.jsonl'
                try:
                    self.files[segment] = (self.directory/name).open('xb', buffering=0)
                    break
                except FileExistsError:
                    continue
            self.sizes[segment] = 0
        stream = self.files[segment]
        offset = 0
        try:
            while offset < len(line):
                count = stream.write(line[offset:])
                if not count:
                    raise OSError('Log write made no progress')
                offset += count
        except OSError:
            # Roll back a partial JSON line if possible. Never append to it on retry.
            try:
                stream.truncate(self.sizes[segment])
            except OSError:
                pass
            self.files.pop(segment).close()
            raise
        self.sizes[segment] += len(line)
        self.written[segment] += 1

    def write(self, segment, record):
        payload = encoded(record)
        if len(payload) <= self.limit:
            self._line(segment, payload)
            return
        # Each chunk is also valid JSONL and strictly obeys the part limit.
        size = (self.limit-1024)*3//4
        count = math.ceil(len(payload)/size)
        record_id = uuid.uuid4().hex
        checksum = hashlib.sha256(payload).hexdigest()
        for i in range(count):
            self._line(segment, encoded(dict(event='record_chunk', record_id=record_id,
                index=i, count=count, sha256=checksum, encoding='json-utf8/base64',
                data=base64.b64encode(payload[i*size:(i+1)*size]).decode('ascii'))))

    def close(self):
        for stream in self.files.values():
            stream.close()
        self.files.clear()


class Writer:
    """A full/slow disk must not block a control or ROS subscription callback."""
    def __init__(self, session, owner, limit=None, capacity=4096):
        self.session = Path(session)
        self.owner = f'{owner}-{os.getpid()}'
        self.parts = Parts(self.session/'segments', self.owner, limit or max_bytes())
        self.queue = queue.Queue(maxsize=capacity)
        self.dropped = Counter()
        self.errors = 0
        self.closed = False
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._run, name='ranger-log-writer', daemon=True)
        self.thread.start()

    def emit(self, segment, event, data, *, source='', level='INFO', ros_time_ns=None):
        record = dict(schema=1, **times(), segment=segment, event=event,
                      source=source, level=level, ros_time_ns=ros_time_ns, data=data)
        if self.closed:
            return False
        try:
            self.queue.put_nowait((segment, record))
            return True
        except queue.Full:
            self.dropped[segment] += 1
            return False

    def _run(self):
        reported, next_warning = {}, 0.
        while not self.stop.is_set() or not self.queue.empty():
            try:
                segment, record = self.queue.get(timeout=.2)
            except queue.Empty:
                continue
            try:
                self.parts.write(segment, record)
                loss = dict(self.dropped)
                if loss != reported:
                    self.parts.write('diagnostics', dict(**times(), event='log_loss',
                        dropped=loss, disk_errors=self.errors, owner=self.owner))
                    reported = loss
            except (OSError, ValueError, TypeError) as exc:
                self.errors += 1
                self.dropped[segment] += 1
                if time.monotonic() >= next_warning:
                    print(f'Ranger logger cannot write: {exc}; records lost: {sum(self.dropped.values())}',
                          file=sys.stderr, flush=True)
                    next_warning = time.monotonic()+30
            finally:
                self.queue.task_done()
        self.parts.close()

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.stop.set()
        self.thread.join(timeout=5)
        summary = dict(**times(), writer=self.owner, dropped=dict(self.dropped),
                       disk_errors=self.errors, pending=self.queue.qsize(),
                       drained=not self.thread.is_alive(), written=dict(self.parts.written))
        try:
            atomic_json(self.session/(self.owner+'-summary.json'), summary)
        except OSError as exc:
            print(f'Ranger logger summary could not be saved: {exc}', file=sys.stderr)


def segment_for(logger):
    name = logger.lower()
    for keys, segment in [
        (('lifecycle', 'spawner', 'controller_manager'), 'lifecycle'),
        (('slam', 'map_server'), 'slam'), (('amcl',), 'localization'),
        (('ekf', 'odometry', 'robot_localization'), 'odometry'),
        (('global_costmap', 'planner_server', 'smac'), 'planning'),
        (('local_costmap', 'controller_server', 'mppi', 'dwb', 'velocity_smoother'), 'control'),
        (('navigator', 'behavior_server'), 'navigation'),
        (('commander', 'ackermann_command_guard', 'forward_'), 'base_control'),
        (('lidar', 'imu', 'sensor'), 'sensors'),
        (('robot_state_publisher', 'tf'), 'tf'), (('gazebo', 'gzserver', 'gzclient', 'rviz'), 'simulation')]:
        if any(k in name for k in keys):
            return segment
    return 'system'
