#!/usr/bin/env python3

import math
import unittest

import numpy as np

from grasp_orientation import AxialAngleFilter, axial_difference, estimate_planar_axis


class GraspOrientationTest(unittest.TestCase):
    def test_rotated_rectangle(self):
        rng = np.random.default_rng(7)
        local = np.column_stack((rng.uniform(-0.06, 0.06, 10000), rng.uniform(-0.02, 0.02, 10000)))
        yaw = math.radians(37.0)
        rotation = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
        points = local @ rotation.T + np.array([-0.32, 0.43])
        points = np.vstack((points, [[4.0, -3.0]]))
        estimate = estimate_planar_axis(points, min_points=100, min_aspect_ratio=1.5)
        self.assertIsNotNone(estimate)
        self.assertLess(math.degrees(axial_difference(estimate.yaw, yaw)), 0.5)
        self.assertGreater(estimate.aspect_ratio, 2.5)

    def test_square_is_rejected(self):
        rng = np.random.default_rng(8)
        points = rng.uniform(-0.03, 0.03, size=(1000, 2))
        self.assertIsNone(estimate_planar_axis(points, 100, 1.3))

    def test_axial_wrap_and_filter(self):
        filt = AxialAngleFilter(8, math.radians(25.0), 0.03)
        values = [math.radians(x) for x in (89.0, -89.5, 88.5, -89.0, 89.5)]
        for value in values:
            mean, deviation, count = filt.update(value, np.array([0.0, 0.0]))
        self.assertEqual(count, 5)
        self.assertLess(math.degrees(axial_difference(mean, math.pi / 2.0)), 1.0)
        self.assertLess(math.degrees(deviation), 2.0)


if __name__ == "__main__":
    unittest.main()
