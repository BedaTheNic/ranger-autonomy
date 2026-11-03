"""Controller timing and stop behavior, without a running ROS graph."""
import math
import sys
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from robot_control import Commander
from geometry_msgs.msg import Twist


class Publisher:
    def publish(self, msg):
        self.message = msg


def controller():
    node = SimpleNamespace(
        previous_sim_time=0.0, positions=[0.0]*4, targets=[0.0]*4,
        command=(0.0, .2, 0.0), command_received=time.monotonic(),
        state_received=time.monotonic(), command_timeout=.5, joint_state_timeout=.5,
        wheelbase=.46, steering_track=.412, wheel_radius=.08, max_wheel_speed=10.0,
        max_linear_speed=.3, max_angular_speed=.5, steering_rate=1.0,
        steering_tolerance=.08, pub_pos=Publisher(), pub_vel=Publisher())
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=100000000))
    return node


class ControllerTest(unittest.TestCase):
    def test_recorded_lateral_commands_keep_rolling_on_each_measured_branch(self):
        node = controller()
        node.positions = [-math.pi/2, math.pi/2] * 2
        node.targets = node.positions[:]
        for vy_sign in (1, -1):
            for vx, vy in ((.0002135, -.0602226), (-.0003847, -.1199994)):
                node.command = vx, vy_sign * vy, 0.
                Commander.update(node)
                for a, speed, measured in zip(node.targets, node.pub_vel.message.data, node.positions):
                    self.assertLess(abs(a - measured), .01)
                    self.assertAlmostEqual(.08 * speed * math.sin(measured), vy_sign * vy, places=4)

    def test_alignment_gate_uses_measured_branch_while_target_catches_up(self):
        node = controller()
        node.positions = [-1.2]*4
        node.targets = [.01]*4
        node.command = (-.0004, -.12, 0.)
        Commander.update(node)
        self.assertLess(node.targets[0], 0.)
        self.assertEqual(list(node.pub_vel.message.data), [0.]*4)

    def test_sparse_clock_does_not_undo_steering_command(self):
        node = controller()
        Commander.update(node)
        targets = node.targets[:]
        Commander.update(node)  # same clock tick, feedback has not caught up
        self.assertEqual(node.targets, targets)
        self.assertAlmostEqual(targets[0], .1)
        self.assertEqual(list(node.pub_vel.message.data), [0.0]*4)

    def test_feedback_required_and_timeouts_stop_wheels(self):
        for expired in ('command_received', 'state_received'):
            node = controller()
            node.positions = [math.pi/2]*4
            node.targets = node.positions[:]
            Commander.update(node)
            self.assertEqual(list(node.pub_vel.message.data), [2.5]*4)
            setattr(node, expired, -math.inf)
            Commander.update(node)
            self.assertEqual(list(node.pub_vel.message.data), [0.0]*4)

    def test_nonfinite_command_cancels_previous_motion(self):
        node = controller()
        msg = Twist()
        msg.linear.x = math.nan
        Commander.on_command(node, msg)
        self.assertEqual(node.command, (0.0, 0.0, 0.0))
        self.assertEqual(node.command_received, -math.inf)


if __name__ == '__main__':
    unittest.main()
