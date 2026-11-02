#!/usr/bin/env python3
"""Read driver provenance, selected C++ excerpts and CAN feedback; never transmit.

Run inside the already running driver container, using Python's standard library:
  docker exec -i ranger_hero_jazzy python3 - < inspect_drive.py > drive-details.json
No ROS setup, package installation or filesystem writes in that container needed.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import socket
import struct
import time


PATTERNS = {
    'ranger_messenger.cpp': (r'motor_angles|motor_speeds|actuator_[hl]s_state', 4),
    'ranger_base.hpp': (r'GetActuatorState|motor_angles|using RangerMiniV3Base', 4),
    'agilex_msg_parser_v2.c': (r'case CAN_MSG_MOTOR_ANGLE_INFO:', 23),
    'agilex_protocol_v2.h': (r'CAN_MSG_MOTOR_ANGLE_INFO|} struct16_t;|} MoterAngleFrame;', 6),
    'ranger_params.hpp': (r'struct RangerMiniV3Params', 22),
}
SKIP_DIRS = {'build', 'install', 'log', 'logs', 'node_modules', '__pycache__',
             'venv', 'env', 'proc', 'sys', 'dev'}


def driver_processes():
    result = []
    for proc in sorted(Path('/proc').iterdir()):
        if not proc.name.isdigit():
            continue
        try:
            args = proc.joinpath('cmdline').read_bytes().split(b'\0')
            if not args or Path(os.fsdecode(args[0])).name != 'ranger_base_node':
                continue
            environment = dict(item.split(b'=', 1) for item in
                               proc.joinpath('environ').read_bytes().split(b'\0') if b'=' in item)
            selected = {key: os.fsdecode(environment[key.encode()]) for key in
                        ('ROS_DISTRO', 'ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION',
                         'AMENT_PREFIX_PATH', 'COLCON_PREFIX_PATH') if key.encode() in environment}
            executable = os.readlink(proc / 'exe')
            result.append(dict(pid=int(proc.name), executable=executable,
                               cwd=os.readlink(proc / 'cwd'), environment=selected))
        except (OSError, ValueError):
            continue
    return result


def source_roots(processes, extra_roots):
    candidates = list(extra_roots)
    for proc in processes:
        paths = [proc['executable'], proc['cwd']]
        for key in ('AMENT_PREFIX_PATH', 'COLCON_PREFIX_PATH'):
            paths.extend(proc['environment'].get(key, '').split(':'))
        for value in filter(None, paths):
            path = Path(value)
            for ancestor in (path, *path.parents):
                if ancestor.name in ('install', 'build', 'src'):
                    candidates.append(str(ancestor.parent / 'src'))
    candidates.extend(['/root', '/home', '/workspace', '/workspaces', '/ws', '/app', '/opt'])
    roots = []
    for value in candidates:
        path = Path(value)
        if path.is_dir() and path not in roots and path != Path('/'):
            roots.append(path)
    return roots


def source_excerpt(path):
    if path.stat().st_size > 256 * 1024:
        return None
    content = path.read_bytes()
    source = content.decode('utf-8', errors='replace')
    lines = source.splitlines()
    pattern, context = PATTERNS[path.name]
    indexes = set()
    for index, line in enumerate(lines):
        if re.search(pattern, line):
            indexes.update(range(max(0, index-context), min(len(lines), index+context+1)))
    result = dict(path=str(path), sha256=hashlib.sha256(content).hexdigest(),
                  excerpts=[f'{index+1}: {lines[index]}' for index in sorted(indexes)])
    if path.name == 'ranger_messenger.cpp':
        # A source observation, not proof of which source produced the running binary.
        result['scalar_angle5_assignment_present'] = bool(re.search(
            r'motor_state_msg\.motor_angles\s*=\s*actuator_state\.motor_angles\.angle_5\s*;', source))
        result['unindexed_actuator_pointer_access_present'] = bool(re.search(
            r'actuator_state\.actuator_[hl]s_state\s*->', source))
    return result


def inspect_sources(roots):
    deadline = time.monotonic()+8
    seen, records = set(), []
    directories = 0
    for root in roots:
        for directory, children, filenames in os.walk(root, followlinks=False):
            if time.monotonic() >= deadline or directories >= 12000 or len(records) >= 15:
                return dict(files=records, search_limited=True)
            directories += 1
            children[:] = sorted(name for name in children if not name.startswith('.')
                                 and name not in SKIP_DIRS)
            if len(Path(directory).relative_to(root).parts) >= 10:
                children[:] = []
            for name in sorted(PATTERNS.keys() & set(filenames)):
                path = Path(directory) / name
                try:
                    identity = (path.stat().st_dev, path.stat().st_ino)
                    if identity in seen:
                        continue
                    seen.add(identity)
                    record = source_excerpt(path)
                    if record:
                        records.append(record)
                except OSError:
                    continue
    return dict(files=records, search_limited=False)


def capture_feedback(interface, seconds):
    report = dict(interface=interface, requested_seconds=seconds, frames={}, error=None)
    started = time.monotonic()
    try:
        with socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as receiver:
            # Only standard data frames 0x200..0x2ff (feedback group). No send calls.
            # CAN_ERR_FLAG in can_mask selects the kernel's error-only receive
            # list, rather than excluding error frames. Error reception is off
            # by default; check flags again after recv for defensive validation.
            mask = 0x700 | socket.CAN_EFF_FLAG | socket.CAN_RTR_FLAG
            receiver.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER,
                                struct.pack('=II', 0x200, mask))
            receiver.bind((interface,))
            started = time.monotonic()
            deadline = started+seconds
            while time.monotonic() < deadline:
                receiver.settimeout(max(.001, min(.2, deadline-time.monotonic())))
                try:
                    frame = receiver.recv(72)
                except socket.timeout:
                    continue
                if len(frame) != 16:  # Classical CAN only; never guess a CAN FD layout.
                    continue
                can_id, length, data = struct.unpack('=IB3x8s', frame)
                if can_id & ~0x7ff or not 0x200 <= can_id <= 0x2ff or length > 8:
                    continue
                entry = report['frames'].setdefault(f'0x{can_id:03x}',
                                                    dict(count=0, dlc=length, examples_hex=[]))
                value = data[:length].hex()
                entry['count'] += 1
                entry['last_hex'] = value
                entry['last_received_s'] = round(time.monotonic()-started, 3)
                if value not in entry['examples_hex'] and len(entry['examples_hex']) < 8:
                    entry['examples_hex'].append(value)
    except (OSError, AttributeError) as exc:
        report['error'] = f'{type(exc).__name__}: {exc}'
    report['observed_seconds'] = round(time.monotonic()-started, 3)
    report['motor_angle_frame_0x271_seen'] = '0x271' in report['frames']
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--can-interface', default='can0')
    parser.add_argument('--seconds', type=float, default=8)
    parser.add_argument('--source-root', action='append', default=[])
    args = parser.parse_args()
    if not 0 < args.seconds <= 30:
        parser.error('--seconds must be greater than 0 and at most 30')
    processes = driver_processes()
    roots = source_roots(processes, args.source_root)
    report = dict(schema=2, commands_transmitted=0, driver_processes=processes,
                  source_search_roots=[str(root) for root in roots],
                  sources=inspect_sources(roots),
                  can_feedback=capture_feedback(args.can_interface, args.seconds),
                  notes=[
                      'Read-only inspection: no CAN transmission, ROS publication or driver restart.',
                      'Source files on disk are not verified as the build inputs of the running binary.',
                      'Equal angles at rest alone do not prove a driver defect.',
                      'Raw CAN bytes do not establish physical wheel order, signs or calibration.',
                      'No source found or no CAN access is reported; neither enables navigation.'])
    print(json.dumps(report, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
