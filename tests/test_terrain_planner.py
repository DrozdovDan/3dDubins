"""Terrain, continuous collision checks, safe loiters and directed RRT*."""

import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from dubins_airplane import (BoxObstacle, ElevationMap, FlightWorld, LoiterCircle,
                            NoPathError, Pose, plan_dubins_airplane, plan_safe_rrt_star)
from dubins_airplane.rrt_star import _Node, _rewire
from examples.example_obstacles import make_scenario, make_world
from dubins_airplane.visualization import RouteLeg, _route_samples, visualize_routes


def flat_world(obstacles=()):
    return FlightWorld(ElevationMap.from_function(-100, -100, 20, 40, 20,
                                                  lambda x, y: 0), 20, 100, obstacles)


class TerrainPlannerTests(unittest.TestCase):
    def test_flat_offsets_and_unknown_space(self):
        world = flat_world()
        self.assertEqual(world.altitude_band(50, 50), (20, 100))
        self.assertTrue(world.is_free(Pose(0, 0, 60, 0)))
        for pose in (Pose(0, 0, 20, 0), Pose(0, 0, 100, 0),
                     Pose(1000, 0, 60, 0), Pose(0, 0, float("nan"), 0)):
            self.assertFalse(world.is_free(pose))
        with self.assertRaises(ValueError):
            world.terrain.height_at(1000, 0)

    def test_euclidean_offset_sees_neighbouring_cliff(self):
        grid = ElevationMap(0, 0, 10, ((0, 0, 80, 80),) * 4)
        world = FlightWorld(grid, 15, 60)
        # Own terrain is zero. H(x,y)+15 would miss the nearby 80 m cliff.
        self.assertGreater(world.lower[1][0], 80)
        self.assertFalse(world.is_free(Pose(5, 15, 50, 0)))

    def test_safe_loiter_altitude_and_tangent_headings(self):
        world = flat_world()
        circle = world.choose_loiter(0, 0, 30)
        self.assertEqual(circle.z, 60)
        self.assertTrue(world.validate_loiter(circle))
        for direction, sign in (("CCW", 1), ("CW", -1)):
            for pose in circle.states(16, direction):
                dx, dy = pose.x - circle.x, pose.y - circle.y
                self.assertAlmostEqual(math.hypot(dx, dy), circle.radius)
                self.assertAlmostEqual(dx * math.cos(pose.yaw) + dy * math.sin(pose.yaw), 0)
                self.assertAlmostEqual(dx * math.sin(pose.yaw) - dy * math.cos(pose.yaw), sign * circle.radius)
                self.assertTrue(world.is_free(pose))
        with self.assertRaises(ValueError):
            world.choose_loiter(-95, 0, 30)  # outside known map

    def test_free_point_is_not_necessarily_safe_loiter(self):
        world = flat_world((BoxObstacle(25, -10, 0, 35, 10, 200),))
        self.assertTrue(world.is_free(Pose(0, 0, 60, 0)))
        self.assertFalse(world.validate_loiter(LoiterCircle(0, 0, 60, 30)))
        with self.assertRaises(ValueError):
            world.choose_loiter(0, 0, 30)

    def test_thin_obstacle_between_samples_cannot_be_skipped(self):
        world = flat_world((BoxObstacle(249.99, -10, 0, 250.01, 10, 200),))
        path = plan_dubins_airplane(Pose(0, 0, 60, 0), Pose(500, 0, 60, 0), 20, 0.2)
        self.assertFalse(world.path_is_free(path, max_step=120))
        self.assertFalse(world.path_is_free(path, max_step=500))
        safe = plan_dubins_airplane(Pose(0, 30, 60, 0), Pose(500, 30, 60, 0), 20, 0.2)
        self.assertTrue(world.path_is_free(safe, max_step=120))

    def test_rewire_updates_descendants_and_prevents_cycles(self):
        a, b, c = Pose(0, 0, 60, 0), Pose(100, 0, 60, 0), Pose(200, 0, 60, 0)
        ab = plan_dubins_airplane(a, b, 20, 0.2)
        bc = plan_dubins_airplane(b, c, 20, 0.2)
        nodes = [_Node(a, children={1}), _Node(b, 300, 0, ab, {2}), _Node(c, 400, 1, bc),
                 _Node(a)]
        _rewire(nodes, 1, 3, ab)
        self.assertEqual(nodes[1].cost, 100)
        self.assertEqual(nodes[2].cost, 200)
        self.assertEqual(nodes[0].children, set())
        self.assertEqual(nodes[3].children, {1})
        with self.assertRaises(ValueError):
            _rewire(nodes, 1, 2, ab)

    def test_blocked_search_reports_budget_not_impossibility(self):
        world = flat_world((BoxObstacle(250, -100, 0, 270, 300, 300),))
        start, goal = world.choose_loiter(0, 0, 20), world.choose_loiter(500, 0, 20)
        first_iterations = []
        with self.assertRaisesRegex(NoPathError, "iterations"):
            plan_safe_rrt_star(world, start, goal, 20, 0.2, iterations=20, seed=9,
                               circle_samples=4, on_first_solution=first_iterations.append)
        self.assertEqual(first_iterations, [])

    def test_direct_open_route_and_goal_tangency(self):
        world = flat_world()
        start, goal = world.choose_loiter(0, 0, 20), world.choose_loiter(500, 0, 20)
        first_iterations = []
        result = plan_safe_rrt_star(world, start, goal, 20, 0.2,
                                    iterations=10, seed=5, circle_samples=4,
                                    on_first_solution=first_iterations.append)
        self.assertEqual(first_iterations, [result.cost_history[0][0]])
        self.assertTrue(all(world.path_is_free(path) for path in result.paths))
        end = result.paths[-1].goal
        self.assertAlmostEqual(math.hypot(end.x - goal.x, end.y - goal.y), goal.radius)
        self.assertAlmostEqual((end.x - goal.x) * math.cos(end.yaw)
                               + (end.y - goal.y) * math.sin(end.yaw), 0, delta=1e-8)

    def test_invalid_inputs(self):
        with self.assertRaises(ValueError):
            ElevationMap(0, 0, -1, ((0,),))
        with self.assertRaises(ValueError):
            ElevationMap(0, 0, 1, ((0,), (0, 1)))
        with self.assertRaises(ValueError):
            FlightWorld(ElevationMap(0, 0, 1, ((0,),)), 100, 50)
        with self.assertRaises(ValueError):
            BoxObstacle(1, 0, 0, 0, 1, 1)
        world = flat_world()
        start, goal = world.choose_loiter(0, 0, 20), world.choose_loiter(500, 0, 20)
        for kwargs in ({"iterations": 0}, {"goal_bias": 2}, {"time_limit": -1}):
            with self.assertRaises(ValueError):
                plan_safe_rrt_star(world, start, goal, 20, 0.2, **kwargs)
        with self.assertRaises(ValueError):
            plan_safe_rrt_star(world, start, goal, 30, 0.2)

    def test_rrt_finds_and_improves_detour_without_manual_waypoints(self):
        world = make_world()
        start, goal = world.choose_loiter(0, 0, 40), world.choose_loiter(800, 0, 40)
        direct = plan_dubins_airplane(start.states()[0], goal.states()[0], 40, math.radians(15))
        self.assertFalse(world.path_is_free(direct))
        results = [plan_safe_rrt_star(world, start, goal, 40, math.radians(15),
                                      iterations=120, seed=7) for _ in range(2)]
        result = results[0]
        self.assertEqual(result, results[1])  # local seeded RNG is reproducible
        self.assertGreaterEqual(len(result.paths), 2)
        self.assertGreater(result.rewires, 0)
        self.assertGreater(len(result.cost_history), 1)
        self.assertAlmostEqual(result.cost_history[-1][1], result.length, delta=1e-6)
        self.assertTrue(all(a[1] > b[1] for a, b in zip(result.cost_history, result.cost_history[1:])))
        for path in result.paths:
            self.assertGreater(path.length, 0)
            self.assertTrue(world.path_is_free(path, max_step=3))
            self.assertLessEqual(abs(path.flight_path_angle), math.radians(15) + 1e-9)
            self.assertGreaterEqual(path.radius, 40)
        for a, b in zip(result.paths, result.paths[1:]):
            self.assertEqual(a.goal, b.start)
        self.assertEqual(result.sample()[0], result.paths[0].pose_at(0))
        self.assertEqual(result.sample()[-1], result.paths[-1].pose_at(result.paths[-1].horizontal_length))

    def test_first_solution_callback_reports_iteration_once(self):
        world = make_world()
        start, goal = world.choose_loiter(0, 0, 40), world.choose_loiter(800, 0, 40)
        observed = []
        result = plan_safe_rrt_star(world, start, goal, 40, math.radians(15),
                                    iterations=120, seed=7, on_first_solution=observed.append)
        self.assertEqual(observed, [result.cost_history[0][0]])
        self.assertGreater(observed[0], 0)

    def test_new_scenarios_find_safe_routes_and_classify_direct_path(self):
        for name in ("buildings", "ridge", "climb"):
            with self.subTest(scenario=name):
                scene = make_scenario(name)
                self.assertTrue(scene.world.validate_loiter(scene.start))
                self.assertTrue(scene.world.validate_loiter(scene.goal))
                result = plan_safe_rrt_star(scene.world, scene.start, scene.goal,
                                            scene.radius, scene.max_angle, iterations=200, seed=7)
                direct = plan_dubins_airplane(result.paths[0].start, result.paths[-1].goal,
                                              scene.radius, scene.max_angle)
                self.assertEqual(scene.world.path_is_free(direct), name == "climb")
                for leg in result.paths:
                    self.assertTrue(scene.world.path_is_free(leg, max_step=3))
                    self.assertLessEqual(abs(leg.flight_path_angle), scene.max_angle + 1e-9)
                if name == "climb":
                    self.assertGreater(scene.goal.z - scene.start.z, 100)
                elif name == "ridge":
                    self.assertEqual(scene.world.obstacles, ())

    def test_unknown_scenario(self):
        with self.assertRaisesRegex(ValueError, "unknown scenario"):
            make_scenario("missing")

    def test_visual_samples_have_exact_cumulative_distance(self):
        a, b, c = Pose(0, 0, 60, 0), Pose(100, 0, 70, 0), Pose(300, 50, 60, 0)
        paths = [plan_dubins_airplane(a, b, 20, 0.2), plan_dubins_airplane(b, c, 20, 0.2)]
        poses, distances, offsets = _route_samples([RouteLeg("leg", p) for p in paths])
        self.assertEqual(offsets, [0, paths[0].length])
        self.assertEqual(distances[0], 0)
        self.assertAlmostEqual(distances[-1], sum(p.length for p in paths))
        self.assertTrue(all(a < b for a, b in zip(distances, distances[1:])))
        self.assertEqual(poses[0], paths[0].pose_at(0))
        self.assertEqual(poses[-1], paths[-1].pose_at(paths[-1].horizontal_length))


class VisualizationTests(unittest.TestCase):
    def setUp(self):
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except ImportError:
            self.skipTest("optional plotting dependencies are not installed")
        self.plt = plt
        self.world = flat_world()
        self.path = plan_dubins_airplane(Pose(0, 0, 60, 0), Pose(500, 0, 60, 0), 20, 0.2)
        self.legs = [RouteLeg("test", self.path)]

    def test_map_labels_classify_both_free_and_blocked_reference(self):
        from PIL import Image

        for blocked in (False, True):
            world = flat_world((BoxObstacle(249, -10, 0, 251, 10, 200),)) if blocked else self.world
            reference = self.path
            safe = plan_dubins_airplane(Pose(0, 30, 60, 0), Pose(500, 30, 60, 0), 20, 0.2)
            with self.subTest(blocked=blocked), tempfile.TemporaryDirectory() as directory:
                output = Path(directory) / "route.png"
                with patch.object(self.plt, "close", wraps=self.plt.close) as close:
                    visualize_routes([RouteLeg("safe", safe)], [], "Проверка", png=output,
                                     world=world, reference_path=reference)
                figure = close.call_args.args[0]
                labels = [text.get_text() for text in figure.legends[0].get_texts()]
                self.assertIn("Прямой путь: заблокирован" if blocked else "Прямой путь: свободен", labels)
                self.assertEqual(len(figure.axes), 3)
                self.assertFalse(figure.axes[1].computed_zorder)
                with Image.open(output) as image:
                    image.verify()

    def test_animation_markers_and_trace_reach_exact_endpoints(self):
        # Exercise all callbacks without encoding 56 frames in the test suite.
        middle = Pose(200, 30, 75, 0)
        paths = [plan_dubins_airplane(self.path.start, middle, 20, 0.2),
                 plan_dubins_airplane(middle, self.path.goal, 20, 0.2)]
        with patch("matplotlib.animation.FuncAnimation") as animation:
            visualize_routes([RouteLeg("leg", p) for p in paths], [], "GIF",
                             gif=Path("unused-test.gif"), world=self.world)
        update = animation.call_args.args[1]
        for frame in range(56):
            trace_map, trace_3d, trace_alt, marker_map, marker_3d, marker_alt, _ = update(frame)
            self.assertEqual(trace_map.get_xdata()[-1], marker_map.get_xdata()[0])
            self.assertEqual(trace_map.get_ydata()[-1], marker_map.get_ydata()[0])
            self.assertEqual(trace_alt.get_ydata()[-1], marker_alt.get_ydata()[0])
            self.assertEqual(trace_3d.get_data_3d()[2][-1], marker_3d.get_data_3d()[2][0])
        self.assertAlmostEqual(marker_map.get_xdata()[0], self.path.goal.x)
        self.assertAlmostEqual(marker_alt.get_xdata()[0], sum(p.length for p in paths))

    def test_legacy_non_world_gif_still_constructs_animation(self):
        with patch("matplotlib.animation.FuncAnimation") as animation:
            visualize_routes(self.legs, [], "Legacy", gif=Path("unused-test.gif"))
        self.assertTrue(animation.called)
        animation.call_args.args[1](55)


if __name__ == "__main__":
    unittest.main()
