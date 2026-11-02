"""Final command feasibility, including a reversal that crosses the two cones."""
import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from ackermann_command_guard import Guard
from geometry_msgs.msg import Twist


class GuardTest(unittest.TestCase):
    def output(self, x, y, w):
        messages = []
        node = SimpleNamespace(max_reverse_speed=.15, max_forward_speed=.2,
                               max_angular_speed=.25, minimum_turning_radius=.8,
                               publisher=SimpleNamespace(publish=messages.append))
        msg = Twist(); msg.linear.x=x; msg.linear.y=y; msg.angular.z=w
        Guard.on_command(node, msg)
        return messages[-1]

    def test_forward_and_reverse_arcs_unchanged(self):
        for x in [.15,-.15]:
            m=self.output(x,0.,.1)
            self.assertEqual(m.linear.x,x)
            self.assertEqual(m.angular.z,.1)

    def test_direction_transition_cannot_pivot(self):
        for step in range(21):
            x=.15-.015*step
            m=self.output(x,.03,.18)
            self.assertEqual(m.linear.y,0.)
            self.assertLessEqual(abs(m.angular.z)*.8,abs(m.linear.x)+1e-12)
        self.assertEqual(self.output(0.,0.,.18),Twist())

    def test_invalid_command_stops(self):
        for v in [math.nan,math.inf,-math.inf]:
            self.assertEqual(self.output(v,0.,.1),Twist())
            self.assertEqual(self.output(.1,0.,v),Twist())
