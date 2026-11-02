import math
import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from odometry_core import EncoderOdometry


class OdometryTest(unittest.TestCase):
    def estimator(self):
        return EncoderOdometry(.46, .412, .08)

    def test_forward_then_reverse_distance_is_not_displacement(self):
        estimator = self.estimator()
        for i in range(201):
            rotations = [min(i, 200-i)*.025]*4
            estimator.update(i*.01, rotations, [0]*4)
        np.testing.assert_allclose(estimator.pose, [0, 0, 0], atol=1e-12)
        self.assertAlmostEqual(estimator.distance, .4)

    def test_lateral_encoder_motion(self):
        estimator = self.estimator()
        for i in range(101):
            estimator.update(i*.01, [i*.025]*4, [math.pi/2]*4)
        np.testing.assert_allclose(estimator.pose, [0, .2, 0], atol=1e-12)

    def test_known_rotation_from_wheel_geometry(self):
        estimator = self.estimator()
        locations = ((.23, .206), (.23, -.206), (-.23, .206), (-.23, -.206))
        angles = [math.atan2(x, -y) for x, y in locations]
        speeds = [.3*math.hypot(x, y)/.08 for x, y in locations]
        for i in range(101):
            estimator.update(i*.01, [v*i*.01 for v in speeds], angles)
        np.testing.assert_allclose(estimator.pose, [0, 0, .3], atol=1e-12)
        self.assertLess(estimator.distance, 1e-12)

    def test_curved_motion_analytic_arc(self):
        estimator = self.estimator()
        locations = ((.23, .206), (.23, -.206), (-.23, .206), (-.23, -.206))
        angles = [math.atan2(.25*x, .2-.25*y) for x, y in locations]
        speeds = [math.hypot(.2-.25*y, .25*x)/.08 for x, y in locations]
        for i in range(201):
            estimator.update(i*.01, [v*i*.01 for v in speeds], angles)
        np.testing.assert_allclose(estimator.pose,
                                    [.8*math.sin(.5), .8*(1-math.cos(.5)), .5], atol=1e-12)
        self.assertAlmostEqual(estimator.distance, .4)
        self.assertTrue(np.all(np.linalg.eigvalsh(estimator.covariance) > 0))

    def test_wrapped_encoder_and_duplicate_stamp(self):
        estimator = self.estimator()
        estimator.update(0, [math.pi-.01]*4, [0]*4)
        estimator.update(.01, [-math.pi+.01]*4, [0]*4)
        self.assertAlmostEqual(estimator.pose[0], .0016)
        before = estimator.pose.copy()
        self.assertIsNone(estimator.update(.01, [0]*4, [0]*4))
        np.testing.assert_array_equal(before, estimator.pose)

    def test_gap_reset_and_bad_samples(self):
        estimator = self.estimator()
        estimator.update(1, [0]*4, [0]*4)
        self.assertIsNone(estimator.update(2, [10]*4, [0]*4))
        self.assertEqual(estimator.last_event, 'gap')
        self.assertEqual(estimator.distance, 0)
        self.assertIsNone(estimator.update(2.01, [math.nan]*4, [0]*4))
        self.assertEqual(estimator.last_event, 'invalid')
        self.assertIsNone(estimator.update(0, [0]*4, [0]*4))
        self.assertEqual(estimator.last_event, 'time_reset')


if __name__ == '__main__':
    unittest.main()
