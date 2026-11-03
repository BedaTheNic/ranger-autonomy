"""Exercise measured CAN -> ROS -> adaptive gateway in an isolated namespace."""
import importlib.util
import math
import os
from pathlib import Path
import socket
import struct
import subprocess
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from ranger_msgs.msg import SystemState
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool


assert {name for _, name in socket.if_nameindex()} <= {'lo'}, 'Requires an empty test network namespace'


def attribute(kind, payload):
    size = 4 + len(payload)
    return struct.pack('=HH', size, kind) + payload + b'\0' * (-size % 4)


payload = struct.pack('=BBHiII', 0, 0, 0, 0, 1, 0xffffffff) + attribute(3, b'vcan_steer\0')
payload += attribute(18 | 0x8000, attribute(1, b'vcan\0'))
with socket.socket(socket.AF_NETLINK, socket.SOCK_RAW, socket.NETLINK_ROUTE) as link:
    link.bind((0, 0)); link.settimeout(2)
    link.sendto(struct.pack('=IHHII', 16 + len(payload), 16, 0x605, 1, 0) + payload, (0, 0))
    assert struct.unpack_from('=i', link.recv(4096), 16)[0] == 0, 'Virtual CAN creation failed'


def load(name, filename):
    spec = importlib.util.spec_from_file_location(name, Path(__file__).resolve().parents[1] / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


module = load('feedback', 'steering_feedback.py')
gateway = load('gateway', 'drive_guard.py')
os.environ['STEERING_CAN_INTERFACE'] = 'vcan_steer'
os.environ['STEERING_FEEDBACK_TOPIC'] = '/ranger/steering_joint_states'
rclpy.init()
adapter = module.SteeringFeedback()
guard = gateway.DriveGuard()
observer = rclpy.create_node('steering_observer')
seen = []
sub = observer.create_subscription(JointState, '/ranger/steering_joint_states',
                                  lambda msg: seen.append(msg), 1)  # Reliable MPPI-compatible subscriber
sender = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
sender.bind(('vcan_steer',))
monitor = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
monitor.bind(('vcan_steer',)); monitor.setblocking(False)


def spin(seconds, raw=None, identifier=0x271, length=8):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if raw is not None:
            sender.send(struct.pack('=IB3x8s', identifier, length, struct.pack('>4h', *raw)))
        rclpy.spin_once(adapter, timeout_sec=.002)
        # Only dispatch the subscription, avoiding graph/lifecycle fixture timers.
        guard.poll_timer.cancel(); guard.output_timer.cancel()
        rclpy.spin_once(guard, timeout_sec=.002)
        rclpy.spin_once(observer, timeout_sec=.002)


def qualify_other_inputs():
    now = time.monotonic()
    guard.stopping = False
    guard.graph_ready_since = now - 3
    guard.last_graph = now
    guard.input_source_issue = None
    guard.controller_active = True; guard.controller_stamp = now
    guard.parameters_stamp = now; guard.sim_time = False
    guard.plugin = 'ranger_nav_plugins::AdaptiveController'
    guard.odom = Odometry(); guard.odom_received = now
    guard.odom.header.frame_id = 'odom'; guard.odom.child_frame_id = 'ranger_base_link'
    guard.system = SystemState(); guard.system_received = now
    guard.system.vehicle_state = SystemState.VEHICLE_STATE_NORMAL
    guard.system.control_mode = SystemState.CONTROL_MODE_CAN


try:
    spin(.3)
    qualify_other_inputs()
    assert guard.blocked_reason(arming=True) == 'MEASURED_STEERING_MISSING'
    # Distinct signed measurements prove byte order, scale, per-wheel mapping and sign.
    raw = [-100, 200, 300, -400]  # FR, RR, RL, FL
    spin(.5, raw)
    assert len(seen) > 10, len(seen)
    msg = seen[-1]
    assert list(msg.name) == module.JOINT_NAMES
    assert all(math.isclose(a, b) for a, b in zip(msg.position, [.4, .1, -.3, -.2]))
    qualify_other_inputs()
    response = guard.enable(SetBool.Request(data=True), SetBool.Response())
    assert response.success, response.message
    guard.disarm('TEST')
    # Exercise the exact read-only readiness command delivered to the robot.
    command = subprocess.Popen([sys.executable, str(Path(__file__).resolve().parents[1] /
        'steering_feedback.py'), 'check'], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    deadline = time.monotonic() + 8
    while command.poll() is None and time.monotonic() < deadline:
        spin(.05, raw)
    if command.poll() is None:
        command.kill()
    output, _ = command.communicate(timeout=2)
    assert command.returncode == 0, output
    assert '"passed": true' in output, output
    # Stopping the base does not imply zero wheel angles: repeated unchanged
    # CAN measurements must remain valid and retain their nonzero values.
    before = len(seen)
    spin(.25, raw)
    assert len(seen) > before and list(seen[-1].position) == list(msg.position)
    # A suspension must not refresh the timestamp of packets queued before it.
    spin(.1)  # drain fixture packets
    sender.send(struct.pack('=IB3x8s', 0x271, 8, struct.pack('>4h', *raw)))
    before = adapter.samples
    time.sleep(.25)
    spin(.1)
    assert adapter.samples == before, 'Old queued packet incorrectly published as fresh'
    # No new packet means no new publication, even though the last value is cached.
    spin(.6)
    before = adapter.samples
    spin(.15)
    assert adapter.samples == before
    qualify_other_inputs()
    assert guard.blocked_reason(arming=True) == 'MEASURED_STEERING_MISSING'
    # Other CAN traffic, RTR/extended frames, wrong DLC, and impossible angles
    # cannot make steering feedback appear fresh.
    for identifier, length, values in [(0x221, 8, raw), (0x111, 8, raw),
            (0x271 | socket.CAN_RTR_FLAG, 8, raw),
            (0x271 | socket.CAN_EFF_FLAG, 8, raw), (0x271, 4, raw),
            (0x271, 8, [32767, 0, 0, 0])]:
        spin(.06, values, identifier, length)
    assert adapter.samples == before
    # Replay the user's actual captured frame: independent measured ~10 degree
    # held crab angles, not an all-zero placeholder.
    spin(.2, [178, 177, 178, 177])
    assert list(seen[-1].position) == [-.177, -.178, -.178, -.177]
    assert [pub.topic_name for pub in adapter.publishers if pub.topic_name != '/rosout'] == [
        '/parameter_events', '/ranger/steering_joint_states']
    # The receiver never adds a CAN frame of its own (monitor sees only fixture IDs).
    observed = set()
    while True:
        try: frame = monitor.recv(16)
        except BlockingIOError: break
        observed.add(struct.unpack_from('=I', frame)[0])
    assert observed <= {0x271, 0x221, 0x111, 0x271 | socket.CAN_RTR_FLAG, 0x271 | socket.CAN_EFF_FLAG}
    print('PASS: real virtual-CAN -> reliable JointState -> adaptive gateway arm; independent signed angles, '
          'readiness command, captured-frame replay, held angles, stale/queued/invalid rejection and receive-only CAN')
finally:
    sender.close(); monitor.close()
    adapter.destroy_node(); guard.destroy_node(); observer.destroy_node()
    rclpy.shutdown()
