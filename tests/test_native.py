"""Differential checks against the readable Python reference, not just samples.

Build the library first; these tests skip cleanly in a Python-only checkout.
Floating-point ties can change redundant rewires, so compare route geometry,
cost history and safety rather than requiring bit-identical tree counters.
"""

from concurrent.futures import ThreadPoolExecutor
from contextlib import redirect_stderr, redirect_stdout
import ctypes as ct
import io
import importlib.util
import math
import os
from random import Random
import unittest
from unittest.mock import patch

from dubins_airplane import (BoxObstacle, ElevationMap, FlightWorld, NoPathError,
                            Pose, backend_name, plan_dubins_airplane, plan_safe_rrt_star)
from dubins_airplane.core import _plan_dubins_airplane_python
from dubins_airplane.native import library_path
from dubins_airplane import native as cpp_native
from examples.example_obstacles import SCENARIOS, make_scenario


@unittest.skipUnless(library_path().is_file(), "build the C++ backend to run differential tests")
class NativeParityTests(unittest.TestCase):
    def native_tree_size(self, world, roots, sampled, neighbor_radius):
        """One-sample C ABI search, for precise nearest-fallback fixtures."""
        with patch.dict(os.environ, DUBINS_BACKEND="cpp"):
            lib = cpp_native.get_library()
        data, keepalive = cpp_native._world(world)
        root_data = (cpp_native._Pose * len(roots))(*map(cpp_native._pose, roots))
        goal_data = (cpp_native._Pose * 1)(cpp_native._pose(Pose(400, 0, 65, 0)))
        sample_data = (cpp_native._Pose * 1)(cpp_native._pose(sampled))
        options = cpp_native._Options(20, math.radians(15), neighbor_radius, 10, -1)
        paths, history = (cpp_native._Path * 2)(), (cpp_native._History * 2)()
        counts = [ct.c_int() for _ in range(5)]
        cpp_native._check(lib, lib.dubins_rrt(
            ct.byref(data), root_data, len(roots), goal_data, 1, sample_data, 1,
            ct.byref(options), paths, ct.byref(counts[0]), history, ct.byref(counts[1]),
            ct.byref(counts[2]), ct.byref(counts[3]), ct.byref(counts[4]), cpp_native._Callback()))
        self.assertEqual(counts[2].value, 1)  # Full budget, not early stopping.
        return counts[3].value

    def test_blocked_nearest_does_not_become_nearest_free_fallback(self):
        grid = ElevationMap.from_function(-100, -200, 10, 70, 40, lambda x, y: 0)
        world = FlightWorld(grid, 20, 110, [BoxObstacle(145, -5, 0, 155, 5, 200)])
        roots = [Pose(100, 0, 65, 0), Pose(0, 120, 65, 0)]
        sample = Pose(200, 0, 65, 0)
        near = _plan_dubins_airplane_python(roots[0], sample, 20, math.radians(15))
        far = _plan_dubins_airplane_python(roots[1], sample, 20, math.radians(15))
        self.assertLess(near.length, 150)
        self.assertGreater(far.length, 150)
        self.assertFalse(world._path_is_free_python(near))
        self.assertTrue(world._path_is_free_python(far))
        # The original rule rejects this sample: the exact nearest is blocked
        # inside the neighbourhood. A farther free node is NOT a substitute.
        self.assertEqual(self.native_tree_size(world, roots, sample, 150), 2)

    def test_exact_nearest_outside_neighborhood_is_still_added(self):
        grid = ElevationMap.from_function(-100, -200, 10, 70, 40, lambda x, y: 0)
        world = FlightWorld(grid, 20, 110)
        self.assertEqual(self.native_tree_size(world, [Pose(0, 0, 65, 0)],
                                               Pose(200, 0, 65, 0), 50), 2)

    def test_steep_rrt_bounds_preserve_python_paths_in_both_directions(self):
        grid = ElevationMap.from_function(-200, -400, 20, 60, 40, lambda x, y: 0)
        with patch.dict(os.environ, DUBINS_BACKEND='cpp'):
            world = FlightWorld(grid, 20, 1200)
        for start_z, goal_z in ((70, 1070), (1070, 70)):
            with self.subTest(start_z=start_z):
                start = world.choose_loiter(0, 0, 30, altitude=start_z)
                goal = world.choose_loiter(600, 0, 30, altitude=goal_z)
                results = []
                for backend in ('python', 'cpp'):
                    with patch.dict(os.environ, DUBINS_BACKEND=backend):
                        results.append(plan_safe_rrt_star(world, start, goal, 30, math.radians(15),
                                                         iterations=35, seed=7))
                reference, native = results
                self.assertEqual(native.iterations, 35)
                self.assertEqual(reference.nodes, native.nodes)
                self.assertEqual([i for i, _ in reference.cost_history],
                                 [i for i, _ in native.cost_history])
                self.assertAlmostEqual(reference.length, native.length, delta=1e-7)
                self.assertEqual(len(reference.paths), len(native.paths))
                for a, b in zip(reference.paths, native.paths):
                    self.assert_path_equal(a, b)
                    self.assertTrue(world._path_is_free_python(b))

    def test_deferred_terminals_preserve_python_solution_after_static_rewiring(self):
        scene = make_scenario('hill_box')
        results = []
        for backend in ('python', 'cpp'):
            with patch.dict(os.environ, DUBINS_BACKEND=backend):
                results.append(plan_safe_rrt_star(scene.world, scene.start, scene.goal,
                    scene.radius, scene.max_angle, iterations=100, seed=7))
        reference, current = results
        self.assertGreater(current.rewires, 0)
        self.assertEqual([i for i, _ in reference.cost_history], [i for i, _ in current.cost_history])
        self.assertAlmostEqual(reference.length, current.length, delta=1e-7)
        self.assertEqual(len(reference.paths), len(current.paths))
        for a, b in zip(reference.paths, current.paths):
            self.assert_path_equal(a, b)

    def assert_path_equal(self, a, b):
        self.assertEqual((a.word, a.base_word, a.altitude_case, a.extra_turns),
                         (b.word, b.base_word, b.altitude_case, b.extra_turns))
        for name in ("radius", "horizontal_length", "length", "extension_angle", "flight_path_angle"):
            self.assertAlmostEqual(getattr(a, name), getattr(b, name),
                                   delta=1e-9 * max(1, abs(getattr(a, name))))
        for fraction in (0, 0.1, 0.5, 0.9, 1):
            p, q = a.pose_at(a.horizontal_length * fraction), b.pose_at(b.horizontal_length * fraction)
            self.assertLess(math.hypot(p.x-q.x, p.y-q.y, p.z-q.z),
                            1e-8 * max(1, a.horizontal_length))
            self.assertLess(abs((p.yaw-q.yaw+math.pi) % math.tau-math.pi), 1e-8)

    def test_2000_random_low_medium_high_climb_and_descent(self):
        rng = Random(827)
        cases, words, supported = set(), set(), 0
        with patch.dict(os.environ, DUBINS_BACKEND="cpp"):
            self.assertEqual(backend_name(), "cpp")
            for i in range(2000):
                radius, angle = rng.uniform(5, 100), math.radians(rng.uniform(5, 35))
                a = Pose(rng.uniform(-500, 500), rng.uniform(-500, 500),
                         rng.uniform(-100, 500), rng.uniform(-math.tau, math.tau))
                b = Pose(a.x+rng.uniform(-18*radius, 18*radius),
                         a.y+rng.uniform(-18*radius, 18*radius),
                         a.z+rng.uniform(-30*radius, 30*radius)*math.tan(angle),
                         rng.uniform(-math.tau, math.tau))
                with self.subTest(pair=i):
                    try:
                        reference = _plan_dubins_airplane_python(a, b, radius, angle)
                    except ValueError:
                        with self.assertRaises(ValueError):
                            plan_dubins_airplane(a, b, radius, angle)
                        continue
                    native = plan_dubins_airplane(a, b, radius, angle)
                    self.assert_path_equal(reference, native)
                    cases.add((native.altitude_case, b.z > a.z))
                    words.add(native.base_word)
                    supported += 1
        self.assertGreater(supported, 1900)
        self.assertEqual(len(cases), 6)
        self.assertEqual(words, {"RSR", "RSL", "LSR", "LSL"})

    def test_euclidean_offsets_match_at_cliffs_and_map_edges(self):
        rng = Random(37)
        heights = tuple(tuple(rng.uniform(-30, 150) for _ in range(12)) for _ in range(11))
        grid = ElevationMap(-17.5, 13.25, 7.5, heights)
        with patch.dict(os.environ, DUBINS_BACKEND="cpp"):
            world = FlightWorld(grid, 15, 60)
            for distance in (0, 7.5, 15, 23.1, 60):
                for upper in (False, True):
                    a, b = world._offset_python(distance, upper), world._offset(distance, upper)
                    for py_row, cpp_row in zip(a, b):
                        for py, cpp in zip(py_row, cpp_row):
                            self.assertAlmostEqual(py, cpp, delta=1e-12)

    def test_continuous_checker_matches_reference_including_thin_box(self):
        rng = Random(59)
        grid = ElevationMap.from_function(-150, -150, 10, 80, 40, lambda x, y: 0)
        obstacle = BoxObstacle(251.1, -1, 0, 251.2, 1, 150)
        with patch.dict(os.environ, DUBINS_BACKEND="cpp"):
            world = FlightWorld(grid, 20, 110, [obstacle], obstacle_margin=0.1)
            straight = plan_dubins_airplane(Pose(0, 0, 60, 0), Pose(500, 0, 60, 0), 30, 0.25)
            self.assertFalse(world.path_is_free(straight, max_step=200))
            decisions = set()
            for _ in range(150):
                start = Pose(rng.uniform(-100, 100), rng.uniform(-110, 110), rng.uniform(15, 115),
                             rng.uniform(-math.pi, math.pi))
                goal = Pose(rng.uniform(300, 550), rng.uniform(-110, 110), rng.uniform(15, 115),
                            rng.uniform(-math.pi, math.pi))
                try:
                    path = plan_dubins_airplane(start, goal, 30, 0.25)
                except ValueError:
                    continue
                for step in (10, 200):
                    reference = world._path_is_free_python(path, max_step=step)
                    self.assertEqual(reference, world.path_is_free(path, max_step=step))
                    decisions.add(reference)
            self.assertEqual(decisions, {True, False})

    def test_cached_curve_matches_checker_for_medium_and_full_helix(self):
        grid = ElevationMap.from_function(-150, -150, 10, 80, 40, lambda x, y: 0)
        with patch.dict(os.environ, DUBINS_BACKEND="cpp"):
            paths = [plan_dubins_airplane(Pose(0, 0, 25, 0), Pose(x, 0, 105, 0), 30, 0.25)
                     for x in (0, 80, 200)]
            self.assertEqual([p.altitude_case for p in paths], ["high", "high", "medium"])
            # Exercise cached prefixes, zero-length joins and recursive
            # intervals, both over a full helix and the four-segment case.
            for obstacles in ([], [BoxObstacle(-25, -105, 40, 25, 105, 80)]):
                world = FlightWorld(grid, 20, 110, obstacles)
                for path in paths:
                    for step in (3, 10, 200):
                        self.assertEqual(world._path_is_free_python(path, max_step=step),
                                         world.path_is_free(path, max_step=step))

    def test_rrt_same_seed_cost_history_and_independent_python_safety(self):
        for name in SCENARIOS:
            with patch.dict(os.environ, DUBINS_BACKEND="python"):
                scene = make_scenario(name)
            for seed in (7, 8, 9):
                with self.subTest(scenario=name, seed=seed):
                    results, notifications = [], []
                    for backend in ("python", "cpp"):
                        calls = []
                        with patch.dict(os.environ, DUBINS_BACKEND=backend):
                            result = plan_safe_rrt_star(
                                scene.world, scene.start, scene.goal, scene.radius, scene.max_angle,
                                iterations=200, seed=seed, on_first_solution=calls.append)
                        results.append(result)
                        notifications.append(calls)
                    a, b = results
                    self.assertEqual(a.iterations, b.iterations)
                    self.assertEqual(a.nodes, b.nodes)
                    self.assertEqual(notifications[0], notifications[1])
                    self.assertEqual([i for i, _ in a.cost_history], [i for i, _ in b.cost_history])
                    for (_, py), (_, cpp) in zip(a.cost_history, b.cost_history):
                        self.assertAlmostEqual(py, cpp, delta=1e-8)
                    self.assertAlmostEqual(a.length, b.length, delta=1e-8)
                    self.assertEqual(len(a.paths), len(b.paths))
                    for reference, native in zip(a.paths, b.paths):
                        self.assert_path_equal(reference, native)
                        # Independent checker, with the original subdivision tolerance.
                        self.assertTrue(scene.world._path_is_free_python(native))
                        self.assertGreaterEqual(native.radius, scene.radius)
                        self.assertLessEqual(abs(native.flight_path_angle), scene.max_angle+1e-9)

    def test_callback_exception_is_propagated_and_stops_native_search(self):
        def fail(_):
            raise LookupError("callback failed")
        with patch.dict(os.environ, DUBINS_BACKEND="cpp"):
            scene = make_scenario("climb")
            with self.assertRaisesRegex(LookupError, "callback failed"):
                plan_safe_rrt_star(scene.world, scene.start, scene.goal, scene.radius, scene.max_angle,
                                   iterations=200, on_first_solution=fail)

    def test_soft_time_limit_retains_initial_solution(self):
        with patch.dict(os.environ, DUBINS_BACKEND="cpp"):
            scene = make_scenario("climb")
            calls = []
            result = plan_safe_rrt_star(scene.world, scene.start, scene.goal, scene.radius,
                                       scene.max_angle, time_limit=1e-9, on_first_solution=calls.append)
            self.assertEqual(result.iterations, 0)
            self.assertEqual(calls, [0])
            self.assertTrue(all(scene.world._path_is_free_python(p) for p in result.paths))

    def test_world_subclass_collision_override_is_preserved(self):
        class NeverFreeWorld(FlightWorld):
            def path_is_free(self, path, max_step=10, min_step=0.05):
                return False
        with patch.dict(os.environ, DUBINS_BACKEND="cpp"):
            grid = ElevationMap.from_function(-150, -150, 20, 50, 20, lambda x, y: 0)
            world = NeverFreeWorld(grid, 20, 110)
            start, goal = world.choose_loiter(0, 0, 30), world.choose_loiter(600, 0, 30)
            with self.assertRaises(NoPathError):
                plan_safe_rrt_star(world, start, goal, 30, 0.25, iterations=10)

    def test_parallel_calls_and_thread_local_errors(self):
        with patch.dict(os.environ, DUBINS_BACKEND="cpp"):
            def compute(i):
                if i % 3 == 0:
                    with self.assertRaises(ValueError):
                        plan_dubins_airplane(Pose(0, 0, 0, 0), Pose(500, 0, 0, 0), -1, 0.25)
                return plan_dubins_airplane(Pose(0, 0, 0, 0), Pose(500, 0, 0, 0), 30, 0.25).length
            with ThreadPoolExecutor(max_workers=4) as pool:
                for length in pool.map(compute, range(120)):
                    self.assertAlmostEqual(length, 500, delta=1e-10)


class BenchmarkDeadlineTests(unittest.TestCase):
    def run_benchmark(self, elapsed, limit="100"):
        from examples import benchmark_planner
        scene = object()
        output = io.StringIO()
        with patch("sys.argv", ["benchmark", "--backend", "python", "--scenario", "hill_box",
                                "--iterations", "600", "--repeats", "1", "--max-ms", limit]), \
                patch.object(benchmark_planner, "make_scenario", return_value=scene), \
                patch.object(benchmark_planner, "measure",
                             return_value=(0.001, elapsed, 1, 900)) as measure, \
                redirect_stdout(output), redirect_stderr(io.StringIO()):
            try:
                benchmark_planner.main()
                status = 0
            except SystemExit as exc:
                status = exc.code
        return status, output.getvalue(), measure.call_args_list

    def test_deadline_passes_without_shortening_iteration_budget(self):
        status, output, calls = self.run_benchmark(0.08)
        self.assertEqual(status, 0)
        self.assertIn("<100 ms: 1/1", output)
        self.assertEqual(calls[-1].args[1:], (600, 7))

    def test_equal_deadline_fails_but_full_search_is_still_measured(self):
        status, output, calls = self.run_benchmark(0.1)
        self.assertEqual(status, 1)
        self.assertIn("Timing target missed", output)
        self.assertEqual(calls[-1].args[1:], (600, 7))

    def test_invalid_deadline_is_rejected_before_planning(self):
        status, _, calls = self.run_benchmark(0.08, limit="nan")
        self.assertEqual(status, 2)
        self.assertEqual(calls, [])


class NativeOptimizationBenchmarkTests(unittest.TestCase):
    def test_optimization_metric_pairs_normalize_time_but_not_mean_length_twice(self):
        from examples.plot_native_speed import mission_pairs
        row = dict(agents=5, baseline=dict(wall_ms=10000, mean_length_m=4000, complete=True, safety_valid=True),
                   optimized=dict(wall_ms=2000, mean_length_m=4000, complete=True, safety_valid=True))
        incomplete = dict(row, optimized=dict(wall_ms=3000, mean_length_m=0, complete=False, safety_valid=True))
        report = dict(runs=[row, incomplete])
        self.assertEqual(mission_pairs(report, 'wall_ms'), [(2, .4), (2, .6)])
        self.assertEqual(mission_pairs(report, 'mean_length_m'), [(4, 4), (4, None)])

    @unittest.skipUnless(importlib.util.find_spec('matplotlib'), 'optional Matplotlib')
    def test_optimization_plot_cli_only_reads_saved_measurements_and_exports_slides(self):
        import json
        from pathlib import Path
        import tempfile
        from PIL import Image
        from examples.plot_native_speed import main as plot_main

        libraries = dict(baseline=dict(sha256='old'), optimized=dict(sha256='new'))
        mission = dict(agents=5, topology='open', baseline=dict(wall_ms=10000, mean_length_m=4000,
                       complete=True, safety_valid=True), optimized=dict(wall_ms=2000, mean_length_m=4000,
                       complete=True, safety_valid=True))
        call = dict(budget=600, baseline_ms=100, optimized_ms=50, succeeded=True, exact_match=True)
        data = [dict(libraries=libraries, runs=[mission]), dict(libraries=libraries, runs=[call]),
                dict(libraries=libraries, runs=[dict(call, budget=3000)])]
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            sources = []
            for name, report in zip(('missions.json', 'local_searches.json', 'local_heavy.json'), data):
                path = root/name
                path.write_text(json.dumps(report), encoding='utf-8')
                sources.append((path, path.read_bytes()))
            with patch('dubins_airplane.plan_safe_rrt_star') as planner, redirect_stdout(io.StringIO()):
                self.assertEqual(plot_main([str(root)]), 0)
                planner.assert_not_called()
            for name in ('metrics', 'local_time'):
                with Image.open(root/'figures'/f'{name}.png') as picture:
                    self.assertEqual(picture.size, (3840, 2160))
                self.assertTrue((root/'figures'/f'{name}.svg').is_file())
            self.assertTrue(all(path.read_bytes() == original for path, original in sources))
            data[2]['libraries'] = dict(baseline=dict(sha256='old'), optimized=dict(sha256='wrong'))
            (root/'local_heavy.json').write_text(json.dumps(data[2]), encoding='utf-8')
            with self.assertRaisesRegex(ValueError, 'different library versions'):
                plot_main([str(root)])

    def test_paired_no_path_comparison_keeps_budget_and_restores_library(self):
        from types import SimpleNamespace
        from examples.benchmark_native_speed import compare_search
        previous, old, new = object(), object(), object()
        request = SimpleNamespace(start=object(), goal=object(), radius=40, max_angle=.25)
        libraries = dict(baseline=old, optimized=new)
        seen = []

        def no_path(*args, **kwargs):
            seen.append((cpp_native._library, kwargs['iterations']))
            raise NoPathError('same full-budget failure')

        with patch.object(cpp_native, '_library', previous), \
                patch('examples.benchmark_native_speed.plan_safe_rrt_star', side_effect=no_path):
            row, result = compare_search(object(), request, dict(iterations=3000), libraries, reverse=True)
            self.assertIs(cpp_native._library, previous)
        self.assertEqual(seen, [(new, 3000), (old, 3000)])
        self.assertTrue(row['exact_match'])
        self.assertFalse(row['succeeded'])
        self.assertEqual(result, 'same full-budget failure')

    def test_native_exception_does_not_leave_wrong_library_active(self):
        from types import SimpleNamespace
        from examples.benchmark_native_speed import compare_search
        previous = object()
        request = SimpleNamespace(start=object(), goal=object(), radius=40, max_angle=.25)
        with patch.object(cpp_native, '_library', previous), \
                patch('examples.benchmark_native_speed.plan_safe_rrt_star', side_effect=RuntimeError('broken')):
            with self.assertRaisesRegex(RuntimeError, 'broken'):
                compare_search(object(), request, dict(iterations=600),
                               dict(baseline=object(), optimized=object()))
            self.assertIs(cpp_native._library, previous)


@unittest.skipUnless(library_path().is_file(), "requires built C++ backend")
class AdaptiveSearchTests(unittest.TestCase):
    def fixture(self):
        with patch.dict(os.environ, DUBINS_BACKEND='cpp'):
            grid = ElevationMap.from_function(-200, -400, 20, 60, 40, lambda x, y: 0)
            world = FlightWorld(grid, 20, 1200)
            start = world.choose_loiter(0, 0, 30, altitude=70)
            goal = world.choose_loiter(600, 0, 30, altitude=1070)
        return world, start, goal

    def test_quality_stop_keeps_a_checked_route_and_matches_both_backends(self):
        from dubins_airplane.rrt_star import _route_lower_bound
        world, start, goal = self.fixture()
        routes = []
        for backend in ('python', 'cpp'):
            with self.subTest(backend=backend), patch.dict(os.environ, DUBINS_BACKEND=backend):
                route = plan_safe_rrt_star(world, start, goal, 30, math.radians(15),
                                          iterations=35, relative_gap=.03)
                bound = _route_lower_bound(start.states(8, 'CCW'),
                    goal.states(8, 'CW')+goal.states(8, 'CCW'), 30, math.radians(15))
                self.assertEqual(route.iterations, 0)
                self.assertLessEqual(route.length, 1.03*bound)
                self.assertTrue(all(world._path_is_free_python(p) for p in route.paths))
                routes.append(route)
        self.assertAlmostEqual(routes[0].length, routes[1].length, places=7)

    def test_unreached_quality_target_uses_full_cap_and_zero_preserves_results(self):
        world, start, goal = self.fixture()
        for backend in ('python', 'cpp'):
            with self.subTest(backend=backend), patch.dict(os.environ, DUBINS_BACKEND=backend):
                fixed = plan_safe_rrt_star(world, start, goal, 30, math.radians(15), iterations=12)
                tight = plan_safe_rrt_star(world, start, goal, 30, math.radians(15),
                                          iterations=12, relative_gap=1e-12)
                self.assertEqual(tight, fixed)
                self.assertEqual(tight.iterations, 12)

    def test_no_path_never_becomes_early_success(self):
        world, start, goal = self.fixture()
        world = FlightWorld(world.terrain, 20, 1200, [BoxObstacle(200, -400, 0, 250, 400, 2000)])
        for backend in ('python', 'cpp'):
            with self.subTest(backend=backend), patch.dict(os.environ, DUBINS_BACKEND=backend):
                with self.assertRaisesRegex(NoPathError, 'no route in 5 iterations'):
                    plan_safe_rrt_star(world, start, goal, 30, math.radians(15),
                                       iterations=5, relative_gap=.03)

    def test_invalid_quality_targets_are_rejected_before_search(self):
        world, start, goal = self.fixture()
        for backend in ('python', 'cpp'):
            for gap in (-.01, 1.01, float('inf'), float('nan')):
                with self.subTest(backend=backend, gap=gap), patch.dict(os.environ, DUBINS_BACKEND=backend):
                    with self.assertRaisesRegex(ValueError, 'relative_gap'):
                        plan_safe_rrt_star(world, start, goal, 30, math.radians(15),
                                           iterations=1, relative_gap=gap)


if __name__ == "__main__":
    unittest.main()
