#!/usr/bin/python3
"""Simulation-only four-wheel steering controller. Input: body-frame Twist."""
import math
import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from swerve_kinematics import inverse_kinematics


class Commander(Node):
    def __init__(self):
        super().__init__('commander')
        defaults = {
            'wheelbase': 0.46, 'steering_track': 0.412, 'wheel_radius': 0.08,
            'max_wheel_speed': 10.0, 'max_linear_speed': 0.3,
            'max_angular_speed': 0.5, 'steering_rate': 1.0,
            'steering_tolerance': 0.08, 'command_timeout': 0.5,
            'joint_state_timeout': 0.5,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value, ParameterDescriptor(read_only=True))
            setattr(self, name, self.get_parameter(name).value)
            if not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f'{name} must be finite and positive')
        self.joints = [f'{p}_steering_wheel' for p in ('fl', 'fr', 'rl', 'rr')]
        self.positions = None
        self.targets = [0.0] * 4
        self.command = (0.0, 0.0, 0.0)
        self.command_received = -math.inf
        self.state_received = -math.inf
        self.previous_sim_time = None
        self.pub_pos = self.create_publisher(
            Float64MultiArray, 'forward_position_controller/commands', 10)
        self.pub_vel = self.create_publisher(
            Float64MultiArray, 'forward_velocity_controller/commands', 10)
        self.create_subscription(Twist, 'cmd_vel', self.on_command, 10)
        self.create_subscription(JointState, 'joint_states', self.on_state, 10)
        self.create_timer(0.02, self.update)
        self.get_logger().info(
            f'SIMULATION swerve: wheelbase={self.wheelbase}, '
            f'track={self.steering_track}, radius={self.wheel_radius}; '
            'wheel motion waits for steering feedback')

    def on_command(self, msg):
        command = (msg.linear.x, msg.linear.y, msg.angular.z)
        if not all(math.isfinite(v) for v in command):
            self.command = (0.0, 0.0, 0.0)
            self.command_received = -math.inf
            return
        vx, vy, wz = command
        # Uniform scaling preserves the requested direction and curvature.
        scale = max(1.0, math.hypot(vx, vy) / self.max_linear_speed,
                    abs(wz) / self.max_angular_speed)
        self.command = tuple(v / scale for v in command)
        self.command_received = time.monotonic()

    def on_state(self, msg):
        values = dict(zip(msg.name, msg.position))
        if not all(j in values and math.isfinite(values[j]) for j in self.joints):
            return
        if self.positions is None:
            self.targets = [values[j] for j in self.joints]
        self.positions = [values[j] for j in self.joints]
        self.state_received = time.monotonic()

    def update(self):
        now = time.monotonic()
        sim_time = self.get_clock().now().nanoseconds / 1e9
        dt = 0.0 if self.previous_sim_time is None else sim_time - self.previous_sim_time
        self.previous_sim_time = sim_time
        if dt < 0:
            self.command_received = -math.inf
        dt = min(max(dt, 0.0), 0.1)
        wheels = [0.0] * 4
        fresh = (now - self.command_received <= self.command_timeout and
                 now - self.state_received <= self.joint_state_timeout)
        if fresh and self.positions is not None and any(abs(v) > 1e-6 for v in self.command):
            angles, wheels = inverse_kinematics(
                *self.command, self.wheelbase, self.steering_track,
                self.wheel_radius, self.max_wheel_speed, self.positions)
            step = self.steering_rate * dt
            self.targets = [p + max(-step, min(step, a - p))
                            for p, a in zip(self.targets, angles)]
            if any(abs(a - p) > self.steering_tolerance
                   for a, p in zip(angles, self.positions)):
                wheels = [0.0] * 4
        # Stop commands retain steering angles.
        self.pub_pos.publish(Float64MultiArray(data=self.targets))
        self.pub_vel.publish(Float64MultiArray(data=wheels))


def main():
    rclpy.init()
    node = Commander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.pub_vel.publish(Float64MultiArray(data=[0.0] * 4))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
