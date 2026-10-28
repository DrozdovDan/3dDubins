"""Airborne waiting/arrival, continuous separation, schedule and native parity."""

from dataclasses import replace
import importlib.util
from math import hypot, pi, radians, tau
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from dubins_airplane import (AgentRequest, FleetPlan, LoiterCircle, NoScheduleError, PlanningResult,
                            Pose, ScheduledFlight, check_separation, plan_dubins_airplane, plan_fleet)
from dubins_airplane.fleet import _separation_python, loiter_disk_distance, sample_flight
from dubins_airplane.native import library_path
from examples.example_fleet import make_fleet_scenario


def straight(name, start, goal, speed=20, departure=0):
    radius = 10
    def circle(p):
        from math import sin, cos
        return LoiterCircle(p.x - radius * sin(p.yaw), p.y + radius * cos(p.yaw), p.z, radius)
    request = AgentRequest(name, circle(start), circle(goal), speed=speed, radius=radius)
    with patch.dict(os.environ, DUBINS_BACKEND="python"):
        path = plan_dubins_airplane(start, goal, radius, radians(15))
    route = PlanningResult((path,), request.start, request.goal, 0, 0, 0, ())
    return ScheduledFlight(request, route, departure)


class FleetGeometryTests(unittest.TestCase):
    def setUp(self):
        self.backend = patch.dict(os.environ, DUBINS_BACKEND="python")
        self.backend.start()
        self.addCleanup(self.backend.stop)

    def test_full_lap_waiting_is_motion_and_joins_route(self):
        f = straight("A", Pose(-100, 0, 75, 0), Pose(100, 0, 75, 0), departure=pi)
        self.assertEqual(f.waiting_laps, 1)
        a, b = f.pose_at(0), f.pose_at(f.lap_period / 4)
        self.assertGreater(hypot(a.x - b.x, a.y - b.y), 10)
        for at, endpoint in ((f.departure, f.route.paths[0].start), (f.arrival, f.route.paths[-1].goal)):
            exact = f.pose_at(at)
            self.assertLess(hypot(exact.x - endpoint.x, exact.y - endpoint.y, exact.z - endpoint.z), 1e-8)
            self.assertLess(abs((exact.yaw - endpoint.yaw + pi) % tau - pi), 1e-8)
            left, right = f.pose_at(at - 1e-7), f.pose_at(at + 1e-7)
            self.assertLess(hypot(left.x - right.x, left.y - right.y, left.z - right.z), 1e-4)
        after = f.pose_at(f.arrival + f.lap_period / 4)
        self.assertAlmostEqual(hypot(after.x - f.request.goal.x, after.y - f.request.goal.y), 10)

    def test_conflict_between_coarse_samples_is_not_missed(self):
        a = straight("A", Pose(-86.2, 0, 75, 0), Pose(113.8, 0, 75, 0))
        b = straight("B", Pose(0, -86.2, 75, pi / 2), Pose(0, 113.8, 75, pi / 2))
        for at in (0, 5, 10):
            p, q = a.pose_at(at), b.pose_at(at)
            self.assertGreater(hypot(p.x-q.x, p.y-q.y, p.z-q.z), 1)
        result = check_separation(a, b, 1, 10, step=10)
        self.assertFalse(result.safe)
        self.assertFalse(result.uncertain)
        self.assertLess(abs(result.time - 4.31), 0.05)

    def test_start_wait_and_post_arrival_are_not_ignored(self):
        a = straight("A", Pose(-100, 0, 75, 0), Pose(100, 0, 75, 0), departure=pi)
        b = straight("B", Pose(-100, 0, 75, 0), Pose(100, 0, 75, 0))
        self.assertFalse(check_separation(a, b, 5, 2).safe)  # Waiting aircraft at t=0.
        a = replace(a, departure=0)
        b = straight("B", Pose(100, -200, 75, pi / 2), Pose(100, 200, 75, pi / 2))
        self.assertFalse(check_separation(a, b, 5, 15, begin=a.arrival).safe)

    def test_vertical_separation_and_terminal_disk_bound(self):
        a = straight("A", Pose(-100, 0, 75, 0), Pose(100, 0, 75, 0))
        b = straight("B", Pose(-100, 0, 125, 0), Pose(100, 0, 125, 0))
        self.assertTrue(check_separation(a, b, 40, 30).safe)
        self.assertEqual(loiter_disk_distance(a.request.goal, b.request.goal), 50)
        self.assertFalse(check_separation(a, b, 50, 30).safe)  # Strict limit plus numeric margin.

    def test_tight_unproven_interval_is_rejected(self):
        a = straight("A", Pose(-100, 0, 75, 0), Pose(100, 0, 75, 0))
        b = straight("B", Pose(-100, 0, 115.001, 0), Pose(100, 0, 115.001, 0))
        result = check_separation(a, b, 40, 1, step=1, tolerance=1)
        self.assertFalse(result.safe)
        self.assertTrue(result.uncertain)
        self.assertGreater(result.distance, 40)

    def test_sampling_includes_wait_flight_and_arrival_on_shared_clock(self):
        f = straight("A", Pose(-100, 0, 75, 0), Pose(100, 0, 75, 0), departure=pi)
        end = f.arrival + 2
        rows = sample_flight(f, end, fps=3)
        self.assertEqual(rows[0][0], 0)
        self.assertEqual(rows[-1][0], end)
        self.assertTrue(all(a[0] < b[0] for a, b in zip(rows, rows[1:])))
        for at in (f.departure, f.arrival):
            self.assertTrue(any(abs(r[0] - at) < 1e-9 for r in rows))
        for row in rows:
            p = f.pose_at(row[0])
            self.assertLess(hypot(row[1]-p.x, row[2]-p.y, row[3]-p.z), 1e-9)

    def test_invalid_flight_requests_times_and_checks(self):
        circle = LoiterCircle(0, 0, 75, 10)
        for speed in (0, -1, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                AgentRequest("A", circle, circle, speed=speed)
        f = straight("A", Pose(-100, 0, 75, 0), Pose(100, 0, 75, 0))
        with self.assertRaises(ValueError):
            replace(f, departure=f.lap_period / 2)
        for time in (-1, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                f.pose_at(time)
        for kw in ({"separation": 0}, {"end": -1}, {"step": 0}, {"tolerance": 0}):
            args = {"separation": 40, "end": 10, **kw}
            with self.assertRaises(ValueError):
                check_separation(f, f, **args)
        fleet = FleetPlan((f,), 40, 30, 0, 0)
        for changes in ({"flights": ()}, {"flights": (f, f)}, {"horizon": 1},
                        {"horizon": float("nan")}, {"separation": 0}, {"routing_ms": -1}):
            with self.assertRaises(ValueError):
                replace(fleet, **changes)


@unittest.skipUnless(library_path().is_file(), "requires compiled fleet C++ backend")
class FleetPlanningTests(unittest.TestCase):
    def setUp(self):
        self.backend = patch.dict(os.environ, DUBINS_BACKEND="cpp")
        self.backend.start()
        self.addCleanup(self.backend.stop)

    def test_all_scenarios_are_safe_with_real_waits_and_full_search_budget(self):
        for name in ("crossing", "three_way", "bottleneck"):
            with self.subTest(name=name):
                world, requests = make_fleet_scenario(name)
                plan = plan_fleet(world, requests)
                self.assertTrue(plan.validate(world))
                self.assertTrue(any(f.waiting_laps > 0 for f in plan.flights))
                self.assertTrue(all(f.route.iterations == 600 for f in plan.flights))
                self.assertGreater(plan.horizon, plan.makespan)
                for i, a in enumerate(plan.flights):
                    for path in a.route.paths:
                        self.assertTrue(world._path_is_free_python(path))
                    for b in plan.flights[:i]:
                        result = _separation_python(a, b, plan.separation, 0, plan.horizon, 2, 0.01)
                        self.assertTrue(result.safe)
                # Same geometric paths launched immediately do have a conflict.
                immediate = [replace(f, departure=0) for f in plan.flights]
                self.assertTrue(any(not check_separation(a, b, 40, plan.horizon).safe
                                    for i, a in enumerate(immediate) for b in immediate[:i]))

    def test_zero_budget_reports_no_schedule_and_does_not_fake_safe_result(self):
        world, requests = make_fleet_scenario("crossing")
        with self.assertRaises(NoScheduleError):
            plan_fleet(world, requests, max_delay=0)

    def test_terrain_scenarios_change_altitude_and_are_independently_safe(self):
        from rerun_scripts.fleet import export_fleet
        for name in ("hills", "mountain_pass"):
            with self.subTest(name=name):
                world, requests = make_fleet_scenario(name)
                grid = world.terrain
                self.assertEqual((grid.resolution, grid.columns, grid.rows), (10, 118, 80))
                low, high = min(map(min, grid.heights)), max(map(max, grid.heights))
                self.assertGreater(high - low, 100)
                self.assertEqual(bool(world.obstacles), name == "mountain_pass")
                plan = plan_fleet(world, requests)
                self.assertTrue(plan.validate(world))
                for i, a in enumerate(plan.flights):
                    self.assertEqual(a.route.iterations, 600)
                    self.assertGreater(abs(a.request.goal.z - a.request.start.z), 60)
                    for path in a.route.paths:
                        self.assertTrue(world._path_is_free_python(path))
                    for b in plan.flights[:i]:
                        self.assertTrue(_separation_python(a, b, 40, 0, plan.horizon, 2, 0.01).safe)
                doc = export_fleet(name, world, plan, fps=2)
                self.assertEqual(doc["fleet"]["terrain_height_range_m"], [low, high])
                self.assertEqual(doc["fleet"]["terrain_size_m"], [1180, 800])
                mesh = next(o for o in doc["scene"]["objects"] if o.get("name") == "terrain")
                self.assertGreater(max(p[2] for p in mesh["vertices"]), 100)
                self.assertGreater(len({tuple(c) for c in mesh["vertex_colors"]}), 10)

    def test_default_fleet_scenario_has_nonflat_terrain(self):
        world, requests = make_fleet_scenario()
        self.assertEqual(len(requests), 3)
        self.assertGreater(max(map(max, world.terrain.heights)) - min(map(min, world.terrain.heights)), 100)

    def test_earliest_release_rounds_up_to_whole_lap(self):
        world, requests = make_fleet_scenario("crossing")
        request = replace(requests[0], earliest_departure=1)
        plan = plan_fleet(world, [request], max_delay=30)
        self.assertEqual(plan.flights[0].waiting_laps, 1)
        self.assertGreater(plan.flights[0].departure, 1)

    def test_duplicate_names_and_overlapping_terminal_circles_fail_early(self):
        world, requests = make_fleet_scenario("crossing")
        for invalid in ([], [requests[0], requests[0]],
                        [requests[0], replace(requests[1], goal=requests[0].goal)]):
            with self.assertRaises(ValueError):
                plan_fleet(world, invalid)

    def test_native_and_reference_continuous_results_agree(self):
        world, requests = make_fleet_scenario("three_way")
        plan = plan_fleet(world, requests)
        candidates = [*plan.flights, *[replace(f, departure=0) for f in plan.flights]]
        outcomes = set()
        for i, a in enumerate(candidates):
            for b in candidates[:i]:
                cpp = check_separation(a, b, 40, plan.horizon)
                py = _separation_python(a, b, 40, 0, plan.horizon, 2, 0.01)
                self.assertEqual((cpp.safe, cpp.uncertain), (py.safe, py.uncertain))
                outcomes.add(cpp.safe)
                if not cpp.safe:
                    self.assertAlmostEqual(cpp.time, py.time, places=8)
                    self.assertAlmostEqual(cpp.distance, py.distance, places=8)
        self.assertEqual(outcomes, {True, False})

    def test_export_keeps_all_agents_and_common_timeline(self):
        from rerun_scripts.fleet import export_fleet
        world, requests = make_fleet_scenario("crossing")
        plan = plan_fleet(world, requests)
        doc = export_fleet("crossing", world, plan, fps=2)
        self.assertEqual([a["name"] for a in doc["agents"]], ["A", "B"])
        end = doc["fleet"]["display_end_s"]
        self.assertGreater(end, plan.makespan)
        for a, f in zip(doc["agents"], plan.flights):
            rows = a["trajectory"]
            self.assertEqual(rows[0][0], 0)
            self.assertEqual(rows[-1][0], end)
            self.assertEqual(rows[-1][3], f.request.goal.z)

    @unittest.skipUnless(importlib.util.find_spec("rerun"), "optional Rerun SDK required")
    def test_real_fleet_recording_has_no_sdk_warnings(self):
        from rerun_scripts.fleet import export_fleet, render_fleet
        import rerun as rr
        import warnings
        world, requests = make_fleet_scenario("hills")
        plan = plan_fleet(world, requests)
        doc = export_fleet("hills", world, plan, fps=2)
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "fleet.rrd"
            try:
                with warnings.catch_warnings():
                    warnings.simplefilter("error")
                    render_fleet("hills", world, plan, doc, save=target)
                self.assertGreater(target.stat().st_size, 1000)
            finally:
                rr.disconnect()


if __name__ == "__main__":
    unittest.main()
