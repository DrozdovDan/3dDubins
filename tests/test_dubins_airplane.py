"""Run with: python -m unittest discover -s tests -v"""

import math
import random
import unittest

from dubins_airplane import Pose, plan_dubins_airplane


def yaw_difference(a, b):
    return (a - b + math.pi) % (2 * math.pi) - math.pi


class DubinsAirplaneTests(unittest.TestCase):
    def check_path(self, start, goal, radius=30.0, angle=math.radians(15)):
        path = plan_dubins_airplane(start, goal, radius, angle)
        points = path.sample(7.0)
        self.assertAlmostEqual(points[0].x, start.x, places=7)
        self.assertAlmostEqual(points[0].y, start.y, places=7)
        self.assertAlmostEqual(points[0].z, start.z, places=7)
        self.assertAlmostEqual(points[-1].x, goal.x, delta=1e-6)
        self.assertAlmostEqual(points[-1].y, goal.y, delta=1e-6)
        self.assertAlmostEqual(points[-1].z, goal.z, delta=1e-6)
        self.assertAlmostEqual(yaw_difference(points[-1].yaw, goal.yaw), 0, delta=1e-8)
        self.assertLessEqual(abs(path.flight_path_angle), angle + 1e-10)
        self.assertTrue(all(s.kind in "LRS" for s in path.segments))
        self.assertTrue(all(s.horizontal_length >= 0 for s in path.segments))
        self.assertAlmostEqual(path.length, math.hypot(path.horizontal_length, goal.z - start.z))
        return path

    def test_straight_and_level(self):
        path = self.check_path(Pose(0, 0, 100, 0), Pose(100, 0, 100, 0))
        self.assertEqual(path.extra_turns, 0)
        self.assertAlmostEqual(path.length, 100)

    def test_climb_and_descent(self):
        start = Pose(0, 0, 100, math.radians(30))
        for z in (140, -500, 1000):
            with self.subTest(z=z):
                path = self.check_path(start, Pose(150, 75, z, math.radians(-45)))
                self.assertEqual(math.copysign(1, path.flight_path_angle),
                                 math.copysign(1, z - start.z))
                if z == 1000:
                    self.assertGreater(path.extra_turns, 0)

    def test_same_horizontal_pose(self):
        start = Pose(0, 0, 100, math.pi)
        path = self.check_path(start, start)
        self.assertEqual(path.length, 0)
        self.assertEqual(len(path.sample()), 1)
        climb = self.check_path(start, Pose(0, 0, 300, math.pi))
        self.assertGreater(climb.extra_turns, 0)

    def test_random_endpoint_and_climb_constraints(self):
        rng = random.Random(237)
        words = set()
        for _ in range(400):
            start = Pose(rng.uniform(-500, 500), rng.uniform(-500, 500),
                         rng.uniform(0, 800), rng.uniform(-math.pi, math.pi))
            goal = Pose(rng.uniform(-500, 500), rng.uniform(-500, 500),
                        rng.uniform(0, 800), rng.uniform(-math.pi, math.pi))
            path = self.check_path(start, goal, radius=rng.uniform(10, 80),
                                   angle=math.radians(rng.uniform(5, 35)))
            words.add(path.word)
        self.assertEqual(words, {"LSL", "RSR", "LSR", "RSL", "RLR", "LRL"})

    def test_distance_bounds_and_segment_sampling(self):
        path = self.check_path(Pose(0, 0, 100, 0),
                               Pose(220, 120, 180, math.radians(90)))
        with self.assertRaises(ValueError):
            path.pose_at(-1)
        with self.assertRaises(ValueError):
            path.pose_at(path.horizontal_length + 1)
        self.assertEqual(path.sample(10)[0], path.pose_at(0))
        self.assertEqual(path.sample(10)[-1], path.pose_at(path.horizontal_length))

    def test_invalid_inputs(self):
        a = Pose(0, 0, 0, 0)
        b = Pose(100, 0, 0, 0)
        for radius, angle in ((0, 0.2), (10, 0), (10, math.pi / 2)):
            with self.subTest(radius=radius, angle=angle):
                with self.assertRaises(ValueError):
                    plan_dubins_airplane(a, b, radius, angle)
        with self.assertRaises(ValueError):
            plan_dubins_airplane(Pose(float("nan"), 0, 0, 0), b, 10, 0.2)
        with self.assertRaises(ValueError):
            plan_dubins_airplane(a, b, 10, 0.2).sample(0)


if __name__ == "__main__":
    unittest.main()
