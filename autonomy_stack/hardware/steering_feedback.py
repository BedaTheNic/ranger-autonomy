#!/usr/bin/env python3
"""Read Ranger Mini V3 CAN steering feedback and publish measured ROS joints.

AgileX Mini 3.0 manual, CAN 0x271: signed big-endian int16, 0.001 rad,
motors 5=FR, 6=RR, 7=RL, 8=FL. Feedback is clockwise-positive; ROS joints
are counterclockwise-positive. This socket never transmits or configures CAN.
"""
import hashlib
import json
import math
import os
from pathlib import Path
import socket
import struct
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


JOINT_NAMES = ['fl_steering_wheel', 'fr_steering_wheel',
               'rl_steering_wheel', 'rr_steering_wheel']
MOTOR_INDICES = (3, 0, 2, 1)  # CAN motors 8, 5, 7, 6 -> FL, FR, RL, RR
CAN_ID = 0x271
CAN_FLAGS = socket.CAN_EFF_FLAG | socket.CAN_RTR_FLAG
SO_TIMESTAMPNS = getattr(socket, 'SO_TIMESTAMPNS', 35)  # Linux 64-bit ABI
MAX_PACKET_AGE = .20


def decode_frame(frame, signs):
    if len(frame) != 16:
        return None
    identifier, length, payload = struct.unpack('=IB3x8s', frame)
    if identifier != CAN_ID or length != 8:
        return None
    angles = struct.unpack('>4h', payload)
    # All four measured angles are in one frame; never mix different samples.
    if any(abs(value) > 1700 for value in angles):
        return None
    return [signs[i] * angles[index] * .001 for i, index in enumerate(MOTOR_INDICES)]


def packet_timestamp(ancillary):
    for level, kind, payload in ancillary:
        if level == socket.SOL_SOCKET and kind == SO_TIMESTAMPNS and len(payload) >= 16:
            seconds, nanoseconds = struct.unpack_from('@ll', payload)
            return seconds + nanoseconds * 1e-9
    return None


class SteeringFeedback(Node):
    def __init__(self):
        super().__init__('ranger_steering_feedback')
        interface = os.environ.get('STEERING_CAN_INTERFACE', 'can0')
        topic = os.environ.get('STEERING_FEEDBACK_TOPIC', '/ranger/steering_joint_states')
        self.signs = [float(v) for v in os.environ.get('STEERING_FEEDBACK_SIGNS', '-1,-1,-1,-1').split(',')]
        if len(self.signs) != 4 or any(v not in (-1., 1.) for v in self.signs):
            raise ValueError('STEERING_FEEDBACK_SIGNS must contain four +/-1 values in FL,FR,RL,RR order')
        self.sample_hz = float(os.environ.get('RANGER_LOG_SAMPLE_HZ', '5'))
        if not math.isfinite(self.sample_hz) or not 0 < self.sample_hz <= 100:
            raise ValueError('RANGER_LOG_SAMPLE_HZ must be in (0,100]')
        if self.get_parameter('use_sim_time').value:
            raise ValueError('Real CAN feedback requires use_sim_time=false')
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        try:
            self.sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER,
                                 struct.pack('=II', CAN_ID, 0x7FF | CAN_FLAGS))
            # Error frames are disabled by default. CAN_ERR_FLAG must not enter
            # the filter mask: Linux interprets that as an error-only filter.
            self.sock.setsockopt(socket.SOL_SOCKET, SO_TIMESTAMPNS, 1)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 32768)
            self.sock.bind((interface,))
            self.sock.setblocking(False)
        except Exception:
            self.sock.close()
            raise
        # Reliable, volatile: compatible with MPPI's reliable JointState subscriber
        # and the drive gateway's best-effort subscriber. No latched old measurements.
        self.output = self.create_publisher(JointState, topic, 1)
        self.last_sample = None
        self.last_warning = 0.
        self.last_log = 0.
        self.samples = 0
        self.timer = self.create_timer(.01, self.poll)
        self.get_logger().info('CAN_STEERING_CONFIG ' + json.dumps(dict(
            interface=interface, can_id='0x271', topic=topic,
            names=JOINT_NAMES, motors=[8, 5, 7, 6], signs=self.signs,
            implementation_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            units='rad', max_packet_age_seconds=MAX_PACKET_AGE)))

    def poll(self):
        latest = None
        # Drain queued frames, retaining only the newest. Kernel timestamps reject
        # old queued data after a scheduling pause; cached angles are never re-stamped.
        for _ in range(256):
            try:
                frame, ancillary, flags, _ = self.sock.recvmsg(16, socket.CMSG_SPACE(16))
            except BlockingIOError:
                break
            stamp = packet_timestamp(ancillary)
            age = time.time() - stamp if stamp is not None else math.inf
            latest = None
            if flags & (socket.MSG_TRUNC | socket.MSG_CTRUNC) or not 0 <= age < MAX_PACKET_AGE:
                continue
            values = decode_frame(frame, self.signs)
            if values is not None:
                latest = (values, age)
        now = time.monotonic()
        if latest is not None:
            msg = JointState()
            # Preserve the kernel receive time in the ROS clock domain.
            sample_ns = self.get_clock().now().nanoseconds - int(latest[1] * 1e9)
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(sample_ns, 1000000000)
            msg.name = JOINT_NAMES
            msg.position = latest[0]
            self.output.publish(msg)
            if self.last_sample is None or now - self.last_sample > .5:
                self.get_logger().info('CAN_STEERING_READY ' + json.dumps(dict(position_rad=list(msg.position))))
            self.last_sample = now
            self.samples += 1
            if now - self.last_log >= 1 / self.sample_hz:
                self.last_log = now
                self.get_logger().info('CAN_STEERING_SAMPLE ' + json.dumps(dict(
                    stamp_sec=msg.header.stamp.sec, stamp_nanosec=msg.header.stamp.nanosec,
                    position_rad=list(msg.position), packet_age_seconds=latest[1])))
        elif (self.last_sample is None or now - self.last_sample > .5) and now - self.last_warning > 5:
            self.last_warning = now
            self.get_logger().warning('CAN_STEERING_STALE: waiting for fresh valid 0x271; no joints published')

    def destroy_node(self):
        self.sock.close()
        return super().destroy_node()


def check():
    """Verify fresh, uniquely published feedback without sending drive commands."""
    node = rclpy.create_node('ranger_steering_check')
    topic = os.environ.get('STEERING_FEEDBACK_TOPIC', '/ranger/steering_joint_states')
    received = []
    subscription = node.create_subscription(JointState, topic,
        lambda msg: received.append((msg, time.monotonic())), 1)
    deadline = time.monotonic() + 6
    while time.monotonic() < deadline and len(received) < 10:
        rclpy.spin_once(node, timeout_sec=.1)
    publishers = node.get_publishers_info_by_topic(topic)
    msg, when = received[-1] if received else (None, 0)
    age = ((node.get_clock().now().nanoseconds - (msg.header.stamp.sec * 1000000000 +
            msg.header.stamp.nanosec)) / 1e9) if msg else math.inf
    passed = bool(len(received) >= 10 and time.monotonic() - when < .5 and 0 <= age < .5
        and list(msg.name) == JOINT_NAMES and len(msg.position) == 4
        and all(math.isfinite(v) and abs(v) <= 1.7 for v in msg.position)
        and len(publishers) == 1 and publishers[0].node_name == 'ranger_steering_feedback')
    print(json.dumps(dict(passed=passed, topic=topic, samples=len(received),
        names=list(msg.name) if msg else [], position_rad=list(msg.position) if msg else [],
        source_age_seconds=age if math.isfinite(age) else None, commands_published=0), indent=2))
    node.destroy_node()
    return 0 if passed else 1


def main():
    rclpy.init()
    node = None
    try:
        if sys.argv[1:] == ['check']:
            return check()
        node = SteeringFeedback()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
