import math
import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from swerve_kinematics import inverse_kinematics, STEERING_LIMIT


class KinematicsTest(unittest.TestCase):
    def test_body_twist_reconstructed_at_every_wheel(self):
        for vx, vy, wz in [(0.2, 0, 0), (-0.2, 0, 0), (0, 0.2, 0),
                           (0, -0.2, 0), (0, 0, 0.3), (0.2, 0.1, 0.25),
                           (-0.2, -0.1, -0.25), (0, 0, 0)]:
            with self.subTest(twist=(vx, vy, wz)):
                angles, speeds = inverse_kinematics(vx, vy, wz, .46, .412, .08, 10)
                for a, speed, (x, y) in zip(angles, speeds,
                        ((.23, .206), (.23, -.206), (-.23, .206), (-.23, -.206))):
                    self.assertLessEqual(abs(a), math.pi/2)
                    self.assertAlmostEqual(.08*speed*math.cos(a), vx-wz*y)
                    self.assertAlmostEqual(.08*speed*math.sin(a), vy+wz*x)

    def test_limits_preserve_relative_wheel_speeds(self):
        _, reference = inverse_kinematics(2, 1, 3, .46, .412, .08, 100)
        _, limited = inverse_kinematics(2, 1, 3, .46, .412, .08, 10)
        factor = 10/max(abs(s) for s in reference)
        self.assertAlmostEqual(max(abs(s) for s in limited), 10)
        for a, b in zip(reference, limited):
            self.assertAlmostEqual(a*factor, b)

    def test_lateral_reverse_keeps_steering(self):
        angles, speeds = inverse_kinematics(0, -.2, 0, .46, .412, .08, 10,
                                             [math.pi/2]*4)
        self.assertEqual(angles, [math.pi/2]*4)
        self.assertEqual(speeds, [-2.5]*4)

    def test_invalid_values(self):
        for radius in (0, -1, math.nan):
            with self.assertRaises(ValueError):
                inverse_kinematics(.2, 0, 0, .46, .412, radius, 10)

    def test_equivalent_wheel_vectors_at_joint_stops(self):
        model = Path(__file__).resolve().parents[2] / 'ranger_mini/urdf/ranger_mini_gazebo.xacro'
        robot = ET.parse(model)
        previous = [-STEERING_LIMIT, STEERING_LIMIT, -1.56, 1.56]
        for name in ('fl', 'fr', 'rl', 'rr'):
            limit = robot.find(f"./joint[@name='{name}_steering_wheel']/limit")
            self.assertEqual((float(limit.get('lower')), float(limit.get('upper'))),
                             (-STEERING_LIMIT, STEERING_LIMIT))
        for degrees in range(-180, 181):
            vx = .2 * math.cos(math.radians(degrees))
            vy = .2 * math.sin(math.radians(degrees))
            angles, speeds = inverse_kinematics(vx, vy, 0, .46, .412, .08, 10, previous)
            for a, speed, measured in zip(angles, speeds, previous):
                self.assertLessEqual(abs(a), STEERING_LIMIT)
                self.assertAlmostEqual(.08 * speed * math.cos(a), vx)
                self.assertAlmostEqual(.08 * speed * math.sin(a), vy)
                for other in (a - math.pi, a + math.pi):
                    if abs(other) <= STEERING_LIMIT:
                        self.assertLessEqual(abs(a - measured), abs(other - measured) + 1e-12)


if __name__ == '__main__':
    unittest.main()
