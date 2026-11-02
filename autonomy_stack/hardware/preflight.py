#!/usr/bin/env python3
"""Inspect real ROS inputs without publishing commands or changing parameters."""
import argparse
import json
import os
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState, LaserScan, PointCloud2
from tf2_ros import Buffer, TransformListener


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=12)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('autonomy_preflight', enable_rosout=False)
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    messages, received, subscriptions = {}, {}, []
    steering = os.environ.get('STEERING_FEEDBACK_TOPIC', '/joint_states')
    obstacles = os.environ.get('OBSTACLE_SCAN_TOPIC', '/scan_obstacles')
    for typ, topic in ((PointCloud2, '/livox/lidar'), (Imu, '/livox/imu'),
                       (Odometry, '/odom'), (LaserScan, obstacles), (JointState, steering)):
        def receive(msg, name=topic):
            messages[name], received[name] = msg, time.monotonic()
        subscriptions.append(node.create_subscription(typ, topic, receive, qos_profile_sensor_data))
    deadline = time.monotonic()+args.timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.1)
    checks = {}
    for topic in ('/livox/lidar', '/livox/imu', '/odom', obstacles):
        checks['fresh:'+topic] = topic in received and time.monotonic()-received[topic] < 2
    odom = messages.get('/odom')
    checks['odometry_frames'] = bool(odom and odom.header.frame_id == 'odom'
                                     and odom.child_frame_id == 'ranger_base_link')
    for parent, child in (('odom', 'ranger_base_link'), ('ranger_base_link', 'livox_frame'),
                          ('map', 'ranger_base_link')):
        checks['tf:'+parent+'->'+child] = buffer.can_transform(parent, child, rclpy.time.Time())
    if os.environ.get('NAV_PROFILE', 'adaptive') in ('adaptive', 'ackermann'):
        names = {'fl_steering_wheel', 'fr_steering_wheel', 'rl_steering_wheel', 'rr_steering_wheel'}
        msg = messages.get(steering)
        checks['measured_steering_feedback'] = bool(
            msg and names.issubset(msg.name) and len(msg.position) == len(msg.name)
            and time.monotonic()-received[steering] < .5)
    publishers = [p.node_namespace.rstrip('/')+'/'+p.node_name
                  for p in node.get_publishers_info_by_topic('/cmd_vel')]
    report = dict(passed=all(checks.values()), checks=checks,
                  ros_domain=os.environ.get('ROS_DOMAIN_ID'),
                  cmd_vel_publishers=publishers,
                  steering_joint_names=list(messages[steering].name) if steering in messages else [],
                  notes=[
                      'No commands published; /cmd_vel_nav is not connected to the drive.',
                      'A passed check does not validate footprint, dynamics, watchdog, E-stop or IMU calibration.',
                      'Adaptive/Ackermann requires measured steering angles; arm joints are insufficient.'])
    print(json.dumps(report, indent=2))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
