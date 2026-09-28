"""Run with: python -m unittest discover -s tests -v"""

import math
import random
import unittest

from dubins_airplane import Pose, minimum_turn_radius, plan_dubins_airplane


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
        self.assertLessEqual(abs(path.flight_path_angle), angle + 1e-9)
        self.assertGreaterEqual(path.radius, radius)
        self.assertTrue(all(s.kind in "LRS" for s in path.segments))
        self.assertTrue(all(s.horizontal_length >= 0 for s in path.segments))
        self.assertAlmostEqual(path.length, math.hypot(path.horizontal_length, goal.z - start.z))
        return path

    def test_straight_and_level(self):
        path = self.check_path(Pose(0, 0, 100, 0), Pose(100, 0, 100, 0))
        self.assertEqual(path.extra_turns, 0)
        self.assertAlmostEqual(path.length, 100)
        self.assertEqual(path.altitude_case, "low")

    def test_low_medium_high_and_exact_length_bound(self):
        # For these separated poses, gamma_max is the active constraint in
        # medium/high. Their 3D length must reach |delta_z| / sin(gamma_max).
        start = Pose(0, 0, 100, 0)
        level_goal = Pose(500, 200, 100, math.pi / 2)
        radius, angle = 30, math.radians(15)
        level = self.check_path(start, level_goal, radius, angle)
        base = level.horizontal_length
        for required, case in ((0.6 * base, "low"),
                               (base + 0.45 * math.tau * radius, "medium"),
                               (base + 2.4 * math.tau * radius, "high")):
            for sign in (-1, 1):
                with self.subTest(case=case, sign=sign):
                    goal = Pose(level_goal.x, level_goal.y,
                                start.z + sign * required * math.tan(angle),
                                level_goal.yaw)
                    path = self.check_path(start, goal, radius, angle)
                    self.assertEqual(path.altitude_case, case)
                    if case == "low":
                        self.assertAlmostEqual(path.horizontal_length, base)
                    else:
                        self.assertAlmostEqual(path.horizontal_length, required, delta=1e-6)
                        self.assertAlmostEqual(abs(path.flight_path_angle), angle, delta=1e-9)
                        self.assertAlmostEqual(path.length,
                                               abs(goal.z - start.z) / math.sin(angle),
                                               delta=1e-6)
                    if case == "medium":
                        self.assertEqual(path.extra_turns, 0)
                        self.assertEqual(path.radius, radius)
                        self.assertEqual(len(path.word), 4)
                        self.assertGreater(path.extension_angle, 0)
                    elif case == "high":
                        self.assertEqual(path.extra_turns, 2)  # floor, not ceil
                        self.assertGreater(path.radius, radius)

    def test_all_csc_branches_and_medium_maneuvers(self):
        # Geometry from the reference's examples, converted from NED to ENU.
        fixtures = (
            # Re-planning L_car at the intermediate pose can change the final
            # turn family; unlike the NTNU code we do not freeze that family.
            (0, 200, 0, 270, "RSR", "RLSL", "RSLR"),
            (100, 100, -70, -70, "RSL", "RLSL", "RSRL"),
            (100, -100, 70, 70, "LSR", "LRSR", "LSLR"),
            (100, -100, 70, -135, "LSL", "LRSL", "LSRL"),
        )
        radius = minimum_turn_radius(15, math.pi / 4, gravity=9.8065)
        angle = math.pi / 6
        for north, east, yaw_start, yaw_goal, base_word, up_word, down_word in fixtures:
            start = Pose(0, 0, 100, math.pi / 2 - math.radians(yaw_start))
            level_goal = Pose(east, north, 100, math.pi / 2 - math.radians(yaw_goal))
            level = self.check_path(start, level_goal, radius, angle)
            self.assertEqual(level.word, base_word)
            required = level.horizontal_length + 0.4 * math.tau * radius
            for sign, expected in ((1, up_word), (-1, down_word)):
                with self.subTest(word=expected):
                    goal = Pose(east, north, 100 + sign * required * math.tan(angle),
                                level_goal.yaw)
                    path = self.check_path(start, goal, radius, angle)
                    self.assertEqual(path.word, expected)
                    self.assertAlmostEqual(path.horizontal_length, required, delta=1e-6)

    def test_case_boundaries(self):
        start, level_goal = Pose(0, 0, 100, 0), Pose(500, 200, 100, 1)
        radius, angle = 30, math.radians(15)
        base = self.check_path(start, level_goal, radius, angle).horizontal_length
        for required in (base, base + 1e-4, base + math.tau * radius - 1e-4,
                         base + math.tau * radius, base + math.tau * radius + 1e-4):
            path = self.check_path(start,
                                   Pose(500, 200, 100 + required * math.tan(angle), 1),
                                   radius, angle)
            self.assertAlmostEqual(path.horizontal_length, required, delta=1e-6)

    def test_ntnu_reference_examples(self):
        # Offline values from ntnu-arl/DubinsAirplane (Python reference).
        # Reference solvers have about 0.2 m length / 0.02 m radius accuracy.
        # Coordinates/headings below are NED, converted to ENU for our API.
        fixtures = (
            # north, east, start altitude, end altitude, chi_s, chi_e, L, R
            (0, 200, 100, 125, 0, 270, 287.745430, 22.943966),
            (100, 100, 100, 125, -70, -70, 214.607470, 22.943966),
            (100, -100, 100, 125, 70, 70, 214.607470, 22.943966),
            (100, -100, 100, 125, 70, -135, 181.980366, 22.943966),
            (0, 200, 100, 250, 0, 270, 323.531193, 22.943966),
            (100, 100, 100, 350, -70, -70, 500.148789, 29.654628),
            (100, -100, 350, 100, 70, 70, 500.148789, 29.654628),
            (100, -100, 350, 100, 70, -135, 500.036723, 36.454914),
            (0, 200, 100, 200, 0, 270, 303.599132, 22.943966),
            (100, 100, 100, 200, 0, -90, 215.476552, 22.943966),
            (100, -100, 100, 200, 0, 90, 215.476552, 22.943966),
            (100, -100, 100, 200, 0, -90, 200.117600, 22.943966),
            (100, 100, 200, 100, 0, 90, 200.117600, 22.943966),
            (100, 100, 200, 100, 0, -90, 215.476552, 22.943966),
            (100, -100, 200, 100, 70, 90, 249.695199, 22.943966),
            (100, -100, 150, 100, 0, -90, 153.391838, 22.943966),
        )
        radius = minimum_turn_radius(15, math.pi / 4, 9.8065)
        for index, (north, east, z0, z1, chi0, chi1, length, fitted_radius) in enumerate(fixtures):
            with self.subTest(example=index + 1):
                path = self.check_path(
                    Pose(0, 0, z0, math.pi / 2 - math.radians(chi0)),
                    Pose(east, north, z1, math.pi / 2 - math.radians(chi1)),
                    radius, math.pi / 6,
                )
                self.assertAlmostEqual(path.length, length, delta=0.2)
                self.assertAlmostEqual(path.radius, fitted_radius, delta=0.02)

    def test_radius_search_reselects_csc_across_wrapped_arc(self):
        # Freezing the initial LSL word makes the radius solver converge to
        # a 2*pi wrap jump, not a root; recomputing L_car avoids that failure.
        start = Pose(22.349094971, 145.992916363, 504.984550639, 0.514836966)
        goal = Pose(124.767288105, 260.929637175, 130.870993692, -2.482809910)
        path = self.check_path(start, goal, 21.975929104, 0.351072113)
        self.assertEqual(path.altitude_case, "high")
        self.assertEqual(path.base_word, "LSL")
        self.assertAlmostEqual(abs(path.flight_path_angle), 0.351072113, delta=1e-9)

    def test_high_turns_extend_correct_helix(self):
        # RSL has opposite start/end directions. A descent must extend the
        # left end helix, not insert a right loop inherited from the start.
        heading = math.pi / 2 - math.radians(-70)
        start = Pose(0, 0, 100, heading)
        radius, angle = minimum_turn_radius(15, math.pi / 4, 9.8065), math.pi / 6
        for sign in (-1, 1):
            with self.subTest(sign=sign):
                path = self.check_path(start, Pose(100, 100, 100 + sign * 250, heading),
                                       radius, angle)
                planar = self.check_path(start, Pose(100, 100, 100, heading),
                                         path.radius, angle)
                self.assertEqual(path.word, "RSL")
                self.assertEqual(path.altitude_case, "high")
                extended_index = 0 if sign > 0 else 2
                for index, (actual, base) in enumerate(zip(path.segments, planar.segments)):
                    addition = math.tau * path.extra_turns * path.radius if index == extended_index else 0
                    self.assertEqual(actual.kind, base.kind)
                    self.assertAlmostEqual(actual.horizontal_length,
                                           base.horizontal_length + addition, delta=1e-6)

    def test_join_tangents_are_continuous(self):
        start = Pose(0, 0, 100, 0)
        goal = Pose(500, 200, 270, math.pi / 2)
        path = self.check_path(start, goal)
        self.assertEqual(path.altitude_case, "medium")
        distance = 0
        step = 1e-4
        for segment in path.segments[:-1]:
            distance += segment.horizontal_length
            left, center, right = (path.pose_at(distance + delta)
                                    for delta in (-step, 0, step))
            for coordinate in ("x", "y", "z"):
                before = (getattr(center, coordinate) - getattr(left, coordinate)) / step
                after = (getattr(right, coordinate) - getattr(center, coordinate)) / step
                self.assertAlmostEqual(before, after, delta=1e-5)

    def test_radius_from_bank_angle(self):
        self.assertAlmostEqual(minimum_turn_radius(15, math.pi / 4), 225 / 9.81)
        for speed, bank, gravity in ((0, 0.5, 9.81), (15, 0, 9.81),
                                     (15, math.pi / 2, 9.81), (15, 0.5, -1)):
            with self.assertRaises(ValueError):
                minimum_turn_radius(speed, bank, gravity)

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
        cases = set()
        for _ in range(200):
            start = Pose(rng.uniform(-500, 500), rng.uniform(-500, 500),
                         rng.uniform(0, 800), rng.uniform(-math.pi, math.pi))
            # The reference uses a conservative >= 6*R_min separation.
            radius = rng.uniform(10, 80)
            heading = rng.uniform(-math.pi, math.pi)
            separation = rng.uniform(6, 15) * radius
            goal = Pose(start.x + separation * math.cos(heading),
                        start.y + separation * math.sin(heading),
                        rng.uniform(0, 800), rng.uniform(-math.pi, math.pi))
            path = self.check_path(start, goal, radius=radius,
                                   angle=math.radians(rng.uniform(5, 35)))
            cases.add(path.altitude_case)
            self.assertIn(path.base_word, {"LSL", "RSR", "LSR", "RSL"})
        self.assertEqual(cases, {"low", "medium", "high"})

    def test_unsupported_close_medium_case_is_not_old_loop_fallback(self):
        with self.assertRaisesRegex(ValueError, "CSC branch"):
            plan_dubins_airplane(Pose(0, 0, 100, 0), Pose(0, 0, 110, 0),
                                30, math.radians(15))

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
