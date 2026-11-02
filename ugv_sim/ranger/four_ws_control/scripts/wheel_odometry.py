#!/usr/bin/python3
"""Estimate body odometry from measured wheel rotation and steering joints."""
import math
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from odometry_core import EncoderOdometry


def covariance6(planar):
    result = [0.0]*36
    for i in (2, 3, 4):
        result[6*i+i] = 1e6
    for row, i in enumerate((0, 1, 5)):
        for col, j in enumerate((0, 1, 5)):
            result[6*i+j] = float(planar[row, col])
    return result


class WheelOdometry(Node):
    def __init__(self):
        super().__init__('wheel_odometry')
        for name, value in {'wheelbase': .46, 'steering_track': .412,
                            'wheel_radius': .08, 'max_dt': .1,
                            'velocity_std': .02}.items():
            self.declare_parameter(name, value, ParameterDescriptor(read_only=True))
        value = lambda key: self.get_parameter(key).value
        self.estimator = EncoderOdometry(value('wheelbase'), value('steering_track'),
                                         value('wheel_radius'), value('max_dt'),
                                         value('velocity_std'))
        prefixes = ('fl', 'fr', 'rl', 'rr')
        self.wheel_names = [p+'_wheel' for p in prefixes]
        self.steer_names = [p+'_steering_wheel' for p in prefixes]
        self.pub = self.create_publisher(Odometry, 'wheel/odometry', 10)
        self.distance_pub = self.create_publisher(Float64, 'wheel/distance', 10)
        self.residual_pub = self.create_publisher(Float64, 'wheel/kinematic_residual', 10)
        self.create_subscription(JointState, 'joint_states', self.on_joints,
                                 qos_profile_sensor_data)
        self.get_logger().info('Encoder odometry active; TF is owned by EKF')

    def on_joints(self, msg):
        positions = dict(zip(msg.name, msg.position))
        if not all(name in positions for name in self.wheel_names+self.steer_names):
            self.get_logger().warning('Incomplete joint state; odometry skipped', throttle_duration_sec=5)
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        result = self.estimator.update(stamp, [positions[n] for n in self.wheel_names],
                                       [positions[n] for n in self.steer_names])
        if result is None:
            if self.estimator.last_event in ('gap', 'time_reset', 'invalid'):
                self.get_logger().warning('Odometry sample: '+self.estimator.last_event,
                                           throttle_duration_sec=2)
            return
        twist, covariance, residual = result
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id, odom.child_frame_id = 'odom', 'base_link'
        x, y, theta = self.estimator.pose
        odom.pose.pose.position.x, odom.pose.pose.position.y = float(x), float(y)
        odom.pose.pose.orientation.z = math.sin(theta/2)
        odom.pose.pose.orientation.w = math.cos(theta/2)
        odom.pose.covariance = covariance6(self.estimator.covariance)
        odom.twist.twist.linear.x, odom.twist.twist.linear.y = float(twist[0]), float(twist[1])
        odom.twist.twist.angular.z = float(twist[2])
        odom.twist.covariance = covariance6(covariance)
        self.pub.publish(odom)
        self.distance_pub.publish(Float64(data=self.estimator.distance))
        self.residual_pub.publish(Float64(data=residual))


def main():
    rclpy.init()
    node = WheelOdometry()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
