#!/usr/bin/env python3
"""Enforce Ackermann bounds after smoothing, including forward/reverse transitions."""
import math
import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node


class Guard(Node):
    def __init__(self):
        super().__init__('ackermann_command_guard')
        for key, default in [('minimum_turning_radius', .8), ('max_forward_speed', .2),
                             ('max_reverse_speed', .15), ('max_angular_speed', .25)]:
            self.declare_parameter(key, default, ParameterDescriptor(read_only=True))
            value = self.get_parameter(key).value
            if not math.isfinite(value) or value < 0 or (value == 0 and key != 'max_reverse_speed'):
                raise ValueError(key + ' must be positive (reverse speed may be zero)')
            setattr(self, key, value)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Twist, 'cmd_vel_ackermann_smoothed', self.on_command, 10)

    def on_command(self, msg):
        output = Twist()
        if all(math.isfinite(v) for v in (msg.linear.x, msg.linear.y, msg.angular.z)):
            output.linear.x = max(-self.max_reverse_speed, min(self.max_forward_speed, msg.linear.x))
            bound = min(self.max_angular_speed, abs(output.linear.x) / self.minimum_turning_radius)
            output.angular.z = max(-bound, min(bound, msg.angular.z))
        # Missing input is covered by the base command watchdog. Never repeat
        # an old nonzero command with a timer; an invalid message emits a stop.
        self.publisher.publish(output)


def main():
    rclpy.init()
    node = Guard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
