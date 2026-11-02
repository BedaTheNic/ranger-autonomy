#!/usr/bin/python3
"""Drive only the isolated validation simulation and measure ground-truth pose.

Run with ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1 and simulation.launch.py.
Writes JSON evidence to the path supplied on the command line.
"""
import json
import math
import os
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, LaserScan
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64MultiArray


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


class Probe(Node):
    def __init__(self):
        super().__init__('simulation_motion_probe')
        self.odom = self.state = self.scan = self.clock = self.wheels = None
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        for typ, topic, attr in ((Odometry, 'ground_truth/odom', 'odom'),
                                 (JointState, 'joint_states', 'state'),
                                 (LaserScan, 'scan', 'scan'), (Clock, 'clock', 'clock'),
                                 (Float64MultiArray, 'forward_velocity_controller/commands', 'wheels')):
            self.create_subscription(typ, topic, lambda msg, a=attr: setattr(self, a, msg),
                                     qos_profile_sensor_data)

    def pump(self, seconds, command=None):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            if command is not None:
                msg = Twist()
                msg.linear.x, msg.linear.y, msg.angular.z = command
                self.publisher.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.03)
            time.sleep(0.01)

    def pose(self):
        p = self.odom.pose.pose
        t = self.odom.header.stamp
        return [p.position.x, p.position.y, yaw(p.orientation), t.sec+t.nanosec/1e9]

    def measure(self, name, command):
        self.pump(2.5, command)  # allow steering to settle
        a = self.pose()
        self.pump(3.0, command)
        b = self.pose()
        dt = b[3]-a[3]
        if dt <= 0.5:
            raise RuntimeError('Simulation clock did not progress')
        dx, dy = b[0]-a[0], b[1]-a[1]
        body_x = (math.cos(a[2])*dx+math.sin(a[2])*dy)/dt
        body_y = (-math.sin(a[2])*dx+math.cos(a[2])*dy)/dt
        angular = math.atan2(math.sin(b[2]-a[2]), math.cos(b[2]-a[2]))/dt
        # For constant twist, compare chord displacement with its SE(2) integral.
        vx, vy, wz = command
        angle = wz*dt
        ex = vx*dt if abs(wz) < 1e-9 else (vx*math.sin(angle)-vy*(1-math.cos(angle)))/wz
        ey = vy*dt if abs(wz) < 1e-9 else (vx*(1-math.cos(angle))+vy*math.sin(angle))/wz
        error = math.hypot(body_x-ex/dt, body_y-ey/dt)
        passed = error < 0.04 and abs(angular-wz) < 0.05
        result = dict(name=name, command=list(command), start=a, end=b,
                      measured_body_chord_velocity=[body_x, body_y, angular],
                      linear_error=error, passed=passed)
        print(json.dumps(result), flush=True)
        self.pump(1.0, (0.0, 0.0, 0.0))
        return result


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '87' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise SystemExit('Use isolated ROS_DOMAIN_ID=87 and ROS_LOCALHOST_ONLY=1')
    rclpy.init(args=['--ros-args', '--disable-external-lib-logs'] if os.environ.get('RANGER_LOG_SESSION_DIR') else [])
    node = Probe()
    results = []
    try:
        node.pump(4.0)
        if any(v is None for v in (node.odom, node.state, node.scan, node.clock)):
            raise RuntimeError('Required simulation topics missing')
        if node.odom.header.frame_id != 'world':
            raise RuntimeError('Ground truth must be in world frame')
        if not any(math.isfinite(r) for r in node.scan.ranges):
            raise RuntimeError('Scan has no visible validation target')
        node.pump(1.0, (0.0, 0.0, 0.0))
        for name, command in (
                ('lateral_positive', (0.0, 0.2, 0.0)),
                ('lateral_negative', (0.0, -0.2, 0.0)),
                ('forward', (0.2, 0.0, 0.0)),
                ('reverse', (-0.2, 0.0, 0.0)),
                ('rotation', (0.0, 0.0, 0.3)),
                ('combined', (0.12, 0.08, 0.2))):
            results.append(node.measure(name, command))
        node.pump(2.0, (0.15, 0.0, 0.0))
        node.pump(1.0)  # no commands: exercise watchdog
        a = node.pose()
        node.pump(1.0)
        b = node.pose()
        drift = math.hypot(b[0]-a[0], b[1]-a[1])
        stopped = node.wheels is not None and max(abs(v) for v in node.wheels.data) < 1e-6
        results.append(dict(name='command_timeout', drift_m=drift,
                            passed=drift < 0.02 and stopped))
        Path(sys.argv[1]).write_text(json.dumps(results, indent=2)+'\n')
        if not all(r['passed'] for r in results):
            raise SystemExit(1)
    finally:
        node.pump(0.3, (0.0, 0.0, 0.0))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
