"""Fixed physical starts, unreliable P2P negotiation, safety and version checks."""

from dataclasses import replace
import importlib.util
from math import cos, hypot, pi, sin
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from dubins_airplane import (AgentRequest, DecentralizedPlanningError, ElevationMap, FlightWorld,
                            LoiterCircle, NetworkSettings, NoPathError, PlanningResult, Pose,
                            ScheduledFlight, plan_dubins_airplane, simulate_decentralized)
from dubins_airplane.decentralized import LocalPlanner, Message, SimulatedTransport, holding_flight
from dubins_airplane.fleet import _separation_python
from dubins_airplane.native import library_path
from examples.example_fleet import make_fleet_scenario


class PythonReferenceTests(unittest.TestCase):
    def test_route_visualization_never_shows_future_geometry_and_replaces_replanned_paths(self):
        from rerun_scripts.fleet import NEW_ROUTE_SECONDS, route_frame, route_strips
        original = [[0, 0, 150], [100, 0, 160]]
        replacement = [[0, 0, 150], [50, 50, 155], [100, 0, 160]]
        fleet = {"flights": [{"name": n} for n in "AB"],
                 "commit_times_s": {"A": 1.7},
                 "route_history": [
                     {"time_s": .2, "agent": "A", "attempt": 1, "points": original},
                     {"time_s": .3, "agent": "B", "attempt": 1, "points": original},
                     {"time_s": 1.5, "agent": "A", "attempt": 2, "points": replacement}],
                 "events": [{"time": t, "agent": n, "kind": k} for t, n, k in (
                     (.2, "A", "ROUTE_READY"), (.3, "B", "ROUTE_READY"),
                     (1.1, "A", "REPLAN"), (1.5, "A", "ROUTE_READY"),
                     (2.0, "A", "TIMEOUT"), (2.0, "B", "TIMEOUT"))]}
        self.assertTrue(all(f["route"] is None for f in route_frame(fleet, 0).values()))
        first = route_frame(fleet, .2)["A"]
        self.assertEqual(first["state"], "candidate")
        self.assertIs(first["route"]["points"], original)
        self.assertFalse(first["new"])
        self.assertEqual(route_frame(fleet, 1.2)["A"]["state"], "replanning")
        self.assertIs(route_frame(fleet, 1.2)["A"]["route"]["points"], original)
        changed = route_frame(fleet, 1.5)["A"]
        self.assertIs(changed["route"]["points"], replacement)
        self.assertTrue(changed["new"])
        self.assertEqual(route_frame(fleet, 1.7)["A"]["state"], "committed")
        self.assertTrue(route_frame(fleet, 1.7)["A"]["new"])
        self.assertFalse(route_frame(fleet, 1.5+NEW_ROUTE_SECONDS)["A"]["new"])
        final = route_frame(fleet, 2)
        self.assertEqual(final["A"]["state"], "committed")  # Missing receipts do not undo COMMIT.
        self.assertEqual(final["B"]["state"], "hold")
        self.assertIsNone(final["B"]["route"])
        points = [[i, 0, 0] for i in range(25)]
        self.assertEqual(route_strips(points, True), [points])
        dashed = route_strips(points, False)
        self.assertTrue(all(len(part) >= 2 for part in dashed))
        self.assertNotIn(points[8], [p for part in dashed for p in part])

    def test_partial_circle_departure_from_fixed_physical_start(self):
        with patch.dict(os.environ, DUBINS_BACKEND="python"):
            start, goal = Pose(-100, 0, 75, 0), Pose(100, 0, 75, 0)
            request = AgentRequest("A", LoiterCircle(-100, 10, 75, 10),
                                   LoiterCircle(100, 10, 75, 10), radius=10, initial_phase=0)
            path = plan_dubins_airplane(start, goal, 10, pi/12)
            route = PlanningResult((path,), request.start, request.goal, 0, 0, 0, ())
            flight = ScheduledFlight(request, route, 1.5*pi*10/20 + 2*pi*10/20)
            self.assertEqual(flight.waiting_laps, 1)
            self.assertAlmostEqual(flight.pose_at(0).x, -90)
            self.assertAlmostEqual(flight.pose_at(0).y, 10)
            self.assertAlmostEqual(flight.pose_at(flight.departure).x, -100)
            self.assertAlmostEqual(flight.pose_at(flight.departure).y, 0)
            with self.assertRaises(ValueError):
                replace(request, initial_phase=float("nan"))

    def test_simulator_without_native_backend(self):
        with patch.dict(os.environ, DUBINS_BACKEND="python"):
            grid = ElevationMap.from_function(-100, -200, 20, 30, 20, lambda x, y: 0)
            world = FlightWorld(grid, 40, 110)
            request = AgentRequest("A", world.choose_loiter(0, 0, 10),
                                   world.choose_loiter(300, 0, 10), radius=10)
            result = simulate_decentralized(world, [request], iterations=1, planning_window=3,
                                            max_delay=20, horizon=60)
            self.assertTrue(result.plan.validate(world))
            self.assertEqual(result.plan.flights[0].request.initial_phase, 0)
            self.assertEqual(result.plan.flights[0].route.iterations, 1)


@unittest.skipUnless(library_path().is_file(), "requires C++ library")
class DecentralizedTests(unittest.TestCase):
    def setUp(self):
        self.backend = patch.dict(os.environ, DUBINS_BACKEND="cpp")
        self.backend.start()
        self.addCleanup(self.backend.stop)

    def test_budget_grows_after_no_path_and_all_failed_work_is_recorded(self):
        world, requests = make_fleet_scenario('hills')
        requests = tuple(replace(r, initial_phase=0) for r in requests)
        roster = {r.name: r for r in requests}
        node = LocalPlanner(world, requests[0], roster, SimulatedTransport(NetworkSettings()),
                            separation=40, max_delay=120, planning_window=100, horizon=600,
                            iterations=10, retry_iterations=30, seed=7, max_route_attempts=8)
        with patch('dubins_airplane.decentralized.plan_safe_rrt_star',
                   side_effect=NoPathError('test search exhausted')) as search:
            node.build_route()
            node.build_route()
        self.assertEqual([c.kwargs['iterations'] for c in search.call_args_list], [10, 30])
        self.assertEqual([c.iteration_budget for c in node.planning_calls], [10, 30])
        self.assertTrue(all(not c.succeeded for c in node.planning_calls))
        self.assertEqual(node.attempts, 2)
        self.assertIsNone(node.fixed_departure)

    def test_retry_budget_cannot_reduce_search_quality(self):
        world, requests = make_fleet_scenario('hills')
        with self.assertRaises(ValueError):
            simulate_decentralized(world, requests, iterations=600, retry_iterations=100)

    def test_all_scenarios_preserve_starts_and_continuous_safety(self):
        for name in ("hills", "mountain_pass", "crossing", "three_way", "bottleneck"):
            with self.subTest(name=name):
                world, requests = make_fleet_scenario(name)
                requests = [replace(r, initial_phase=i * pi / 3) for i, r in enumerate(requests)]
                result = simulate_decentralized(world, requests)
                self.assertTrue(result.plan.validate(world))
                self.assertLess(result.negotiation_time, 5)
                self.assertGreater(result.messages_sent, 0)
                for i, f in enumerate(result.plan.flights):
                    self.assertEqual(f.route.iterations, 600)
                    self.assertGreaterEqual(f.departure, 5)
                    r, p = f.request, f.pose_at(0)
                    self.assertLess(hypot(p.x - r.start.x - r.start.radius * cos(r.initial_phase),
                                          p.y - r.start.y - r.start.radius * sin(r.initial_phase)), 1e-8)
                    for path in f.route.paths:
                        self.assertTrue(world._path_is_free_python(path))
                    for other in result.plan.flights[:i]:
                        self.assertTrue(_separation_python(f, other, 40, 0, result.plan.horizon, 2, .01).safe)

    def test_loss_duplication_and_out_of_order_delivery(self):
        world, requests = make_fleet_scenario("hills")
        network = NetworkSettings(loss=.3, duplicates=1, maximum_delay=.25, seed=7)
        result = simulate_decentralized(world, requests, network=network, planning_window=10)
        self.assertGreater(result.messages_lost, 0)
        sent = [e for e in result.events if e.kind == "SEND"]
        self.assertGreater(len(sent), len({(e.agent, e.peer, e.detail, e.version) for e in sent}))
        self.assertTrue(result.plan.validate(world))
        for f in result.plan.flights:
            acked = {e.peer for e in result.events if e.kind == "RECEIVE" and e.agent == f.request.name
                     and e.detail == "ACK" and e.version == 1}
            self.assertEqual(acked, {r.name for r in requests} - {f.request.name})

    def test_partition_never_releases_unconfirmed_agents(self):
        world, requests = make_fleet_scenario("crossing")
        with self.assertRaises(DecentralizedPlanningError) as caught:
            simulate_decentralized(world, requests, network=NetworkSettings(loss=1))
        failure = caught.exception
        self.assertTrue(failure.reservations.validate(world))
        self.assertFalse(any(e.kind == "COMMIT" for e in failure.events))
        self.assertIsNotNone(failure.result)
        metadata = failure.result.metadata()
        self.assertFalse(metadata["agreement_complete"])
        self.assertEqual(metadata["commit_times_s"], {})
        self.assertTrue(any(e.kind == "TIMEOUT" for e in failure.events))
        self.assertTrue(all(p["lost"] and p["delivery_s"] is None for p in metadata["packets"]))
        for f in failure.reservations.flights:
            self.assertEqual(f.request.goal, f.request.start)
            self.assertEqual(f.duration, 0)
            self.assertGreater(hypot(f.pose_at(0).x - f.pose_at(2).x,
                                     f.pose_at(0).y - f.pose_at(2).y), 1)

    def test_timeout_retains_already_committed_reservations(self):
        world, requests = make_fleet_scenario("hills")
        # A can receive all votes/commit, but B's proposal cannot reach C.
        network = NetworkSettings(loss=0, blocked_links=frozenset({("B", "C")}))
        with self.assertRaises(DecentralizedPlanningError) as caught:
            simulate_decentralized(world, requests, network=network)
        failure = caught.exception
        self.assertTrue(any(e.kind == "COMMIT" and e.agent == "A" for e in failure.events))
        flights = {f.request.name: f for f in failure.reservations.flights}
        self.assertGreater(flights["A"].duration, 0)
        self.assertEqual(flights["B"].duration, 0)
        self.assertEqual(flights["C"].duration, 0)
        self.assertTrue(failure.reservations.validate(world))
        from rerun_scripts.fleet import export_fleet, network_frame
        doc = export_fleet("hills", world, failure.reservations, fps=2, protocol=failure.result)
        states = network_frame(doc["fleet"], 5)["states"]
        self.assertEqual(states["A"], "согласован · круг")
        self.assertEqual(states["B"], "нет согласования · круг")
        self.assertEqual(states["C"], "нет согласования · круг")
        self.assertFalse(network_frame(doc["fleet"], 6)["packets"])

    def test_packet_traces_use_real_delivery_times_and_unique_retry_ids(self):
        world, requests = make_fleet_scenario("hills")
        network = NetworkSettings(minimum_delay=.15, maximum_delay=.15, loss=0, duplicates=1)
        result = simulate_decentralized(world, requests, network=network)
        data = result.metadata()
        sent = [e for e in result.events if e.kind == "SEND"]
        self.assertEqual(len(sent), len({e.packet_id for e in sent}))
        self.assertEqual(len(data["packets"]), 2 * len(sent))
        self.assertEqual(data["network"]["minimum_delay_s"], .15)
        for packet in data["packets"]:
            self.assertAlmostEqual(packet["delivery_s"] - packet["sent_s"], .15)
        for event in result.events:
            if event.kind == "RECEIVE":
                self.assertTrue(any(p["id"] == event.packet_id and p["receiver"] == event.agent
                                    and abs(p["delivery_s"] - event.time) < 1e-12
                                    for p in data["packets"]))
        from rerun_scripts.fleet import export_fleet, network_frame
        doc = export_fleet("hills", world, result.plan, protocol=result)
        first = data["packets"][0]
        frame = network_frame(doc["fleet"], first["sent_s"] + .075)
        self.assertTrue(any(p["id"] == first["id"] and abs(p["progress"] - .5) < 1e-10
                            for p in frame["packets"]))
        for flight in result.plan.flights:
            self.assertLess(data["commit_times_s"][flight.request.name], flight.departure)

    def test_large_network_delay_preserves_committed_and_holding_motion(self):
        world, requests = make_fleet_scenario("hills")
        network = NetworkSettings(minimum_delay=1.5, maximum_delay=1.5, loss=0, duplicates=0)
        with self.assertRaises(DecentralizedPlanningError) as caught:
            simulate_decentralized(world, requests, network=network)
        result = caught.exception.result
        data = result.metadata()
        self.assertFalse(result.agreement_complete)
        self.assertEqual(set(data["commit_times_s"]), {"A"})
        self.assertEqual(result.negotiation_time, 5)
        for flight in result.plan.flights:
            if flight.request.name == "A":
                self.assertGreater(flight.duration, 0)
            else:
                self.assertEqual(flight.duration, 0)
                for at in (0, 5, 20, 100):
                    pose = flight.pose_at(at)
                    self.assertAlmostEqual(hypot(pose.x - flight.request.start.x,
                                                pose.y - flight.request.start.y), flight.request.start.radius)
        self.assertTrue(result.plan.validate(world))

    def test_roster_order_does_not_assign_priority(self):
        world, requests = make_fleet_scenario("crossing")
        a = simulate_decentralized(world, requests, network=NetworkSettings(loss=0))
        b = simulate_decentralized(world, reversed(requests), network=NetworkSettings(loss=0))
        self.assertEqual([f.request.name for f in b.plan.flights], ["A", "B"])
        self.assertEqual([(f.route, f.departure) for f in a.plan.flights],
                         [(f.route, f.departure) for f in b.plan.flights])

    def test_one_km_altitude_conflict_rebuilds_geometry_using_received_paths_not_waiting(self):
        from examples.example_fleet import compare_initial_plan
        world, requests = make_fleet_scenario("altitude_conflict_1km")
        grid = world.terrain
        self.assertEqual(grid.resolution, 10)
        self.assertEqual((grid.x_max-grid.x_min, grid.y_max-grid.y_min), (6400, 1600))
        self.assertEqual((min(map(min, grid.heights)), max(map(max, grid.heights))), (0, 1000))
        self.assertEqual([r.goal.z - r.start.z for r in requests], [1000, -1000])
        result = simulate_decentralized(world, requests)
        self.assertTrue(result.agreement_complete)
        a, b = result.initial_flights
        from dubins_airplane.fleet import check_separation
        conflict = check_separation(a, b, 40, 600)
        self.assertFalse(conflict.safe)
        self.assertFalse(conflict.uncertain)
        self.assertLess(conflict.distance, 40)
        comparison = compare_initial_plan(result)
        self.assertLess(comparison["pairs"][0]["initial_closest"]["distance_m"], 5)
        self.assertGreater(comparison["pairs"][0]["final_closest"]["distance_m"], 40)
        self.assertEqual(dict(result.route_attempts), {"A": 1, "B": 2})
        replans = [e for e in result.events if e.kind == "REPLAN" and e.agent == "B"]
        self.assertTrue(replans)
        self.assertIn("received=A:v1", replans[0].detail)
        received_at = next(e.time for e in result.events if e.kind == "RECEIVE" and e.agent == "B" and e.detail == "COMMIT")
        self.assertGreaterEqual(replans[0].time, received_at)
        self.assertTrue(any(e.kind == "CONFLICT" and e.agent == "B" for e in result.events))
        self.assertFalse(any(e.kind == "DELAY" for e in result.events))
        for initial, final in zip(result.initial_flights, result.plan.flights):
            self.assertEqual(initial.route.paths[0].start, final.route.paths[0].start)
            self.assertEqual(initial.departure, final.departure)
            self.assertEqual(final.route.iterations, 600)
            for path in final.route.paths:
                self.assertTrue(world._path_is_free_python(path))
        self.assertEqual(comparison["extra_wait_s"], {"A": 0, "B": 0})
        self.assertEqual(comparison["geometry_changed"], {"A": False, "B": True})
        self.assertGreater(len(result.plan.flights[1].route.paths), 1)
        self.assertTrue(_separation_python(*result.plan.flights, 40, 0, 600, 2, .01).safe)
        self.assertTrue(result.plan.validate(world))

    def test_failed_peer_announces_permanent_holding_reservation(self):
        from dubins_airplane.decentralized import plan_safe_rrt_star
        world, requests = make_fleet_scenario("hills")
        def missing_route(world, start, *args, **kwargs):
            if start == requests[0].start:
                raise NoPathError("test route budget exhausted")
            return plan_safe_rrt_star(world, start, *args, **kwargs)
        with patch("dubins_airplane.decentralized.plan_safe_rrt_star", side_effect=missing_route):
            with self.assertRaises(DecentralizedPlanningError) as caught:
                simulate_decentralized(world, requests, max_route_attempts=1, network=NetworkSettings(loss=0))
        failure = caught.exception
        self.assertTrue(any(e.kind == "RECEIVE" and e.detail == "FAILED" for e in failure.events))
        flights = {f.request.name: f for f in failure.reservations.flights}
        self.assertEqual(flights["A"].duration, 0)
        self.assertGreater(flights["B"].duration, 0)
        self.assertGreater(flights["C"].duration, 0)
        self.assertTrue(failure.reservations.validate(world))

    def test_five_agent_mountain_exchange_has_terrain_obstacles_and_three_initial_conflicts(self):
        from examples.example_fleet import compare_initial_plan
        from rerun_scripts.fleet import export_fleet, large_fleet_summary, encounter_windows
        world, requests = make_fleet_scenario("mountain_exchange_5")
        grid = world.terrain
        self.assertEqual(len(requests), 5)
        self.assertEqual(len(world.obstacles), 3)
        self.assertEqual((grid.x_max-grid.x_min, grid.y_max-grid.y_min), (6400, 2400))
        self.assertEqual(grid.resolution, 10)
        self.assertEqual((min(map(min, grid.heights)), max(map(max, grid.heights))), (0, 1000))
        self.assertEqual([r.goal.z-r.start.z for r in requests], [1000, -1000, 1000, -1000, 1000])
        row, col = grid.cell(2400, 550)
        self.assertGreater(grid.heights[row][col], 1000*(2400-250)/5500 + 150)
        row, col = grid.cell(3000, 0)
        self.assertLess(grid.heights[row][col], 1000*(3000-250)/5500 - 40)
        result = simulate_decentralized(world, requests, planning_window=15)
        self.assertTrue(result.agreement_complete)
        comparison = compare_initial_plan(result)
        conflicts = [p for p in comparison["pairs"] if not p["initial_safe"]]
        self.assertEqual({tuple(p["agents"]) for p in conflicts}, {("B", "D"), ("C", "D"), ("B", "E")})
        self.assertLess(min(p["initial_closest"]["distance_m"] for p in conflicts), 5)
        self.assertGreater(min(p["final_closest"]["distance_m"] for p in comparison["pairs"]), 40)
        self.assertEqual({n for n, flag in comparison["geometry_changed"].items() if flag}, {"D", "E"})
        history = result.route_history
        self.assertEqual([(r.agent, r.attempt) for r in history if r.attempt > 1], [("D", 2), ("E", 2)])
        self.assertEqual(len(history), 7)
        self.assertEqual(len(result.planning_calls), 7)
        self.assertTrue(all(c.succeeded and c.elapsed_ms > 0 for c in result.planning_calls))
        self.assertEqual({c.agent for c in result.planning_calls if c.mode == "traffic"}, {"D", "E"})
        for name, elapsed in result.local_routing_ms:
            self.assertAlmostEqual(elapsed, sum(c.elapsed_ms for c in result.planning_calls if c.agent == name))
        self.assertTrue(all(any(e.agent == r.agent and e.kind == "ROUTE_READY" and e.time == r.time
                                for e in result.events) for r in history))
        for flight in result.plan.flights:
            routes = [r for r in history if r.agent == flight.request.name]
            self.assertEqual(routes[-1].flight, flight)
            self.assertEqual(routes[0].flight, next(f for f in result.initial_flights
                                                  if f.request.name == flight.request.name))
        self.assertTrue(all(delay == 0 for delay in comparison["extra_wait_s"].values()))
        self.assertFalse(any(e.kind == "DELAY" for e in result.events))
        blocked_direct = 0
        for initial, final in zip(result.initial_flights, result.plan.flights):
            self.assertEqual(final.route.iterations, 600)
            self.assertEqual(initial.departure, final.departure)
            direct = plan_dubins_airplane(initial.route.paths[0].start, initial.route.paths[-1].goal,
                                          initial.request.radius, initial.request.max_angle)
            points = direct.sample(10)
            blocked_direct += any(box.intersects(p.x, p.y, p.z, p.x, p.y, p.z, world.obstacle_margin)
                                  for box in world.obstacles for p in points)
            for path in final.route.paths:
                self.assertTrue(world._path_is_free_python(path))
            for other in result.plan.flights[:result.plan.flights.index(final)]:
                self.assertTrue(_separation_python(final, other, 40, 0, 600, 2, .01).safe)
        self.assertGreaterEqual(blocked_direct, 3)
        self.assertTrue(result.plan.validate(world))
        doc = export_fleet("mountain_exchange_5", world, result.plan, fps=1, protocol=result)
        lines = large_fleet_summary("mountain_exchange_5", world, result.plan, doc)
        self.assertLessEqual(len(lines), 12)
        self.assertIn("41.", lines[4])
        self.assertGreaterEqual(len(encounter_windows(doc["fleet"]["initial_comparison"],
                                                     doc["fleet"]["display_end_s"])), 3)
        from rerun_scripts.fleet import network_summary
        with patch("rerun_scripts.fleet.attach_initial_comparison") as attach:
            network_doc = export_fleet("mountain_exchange_5", world, result.plan, fps=1,
                                       protocol=result, network_only=True)
        attach.assert_not_called()
        self.assertTrue(network_doc["fleet"]["network_only"])
        self.assertAlmostEqual(network_doc["fleet"]["display_end_s"], result.negotiation_time+.4)
        self.assertTrue(all(agent["trajectory"][-1][0] == network_doc["fleet"]["display_end_s"]
                            for agent in network_doc["agents"]))
        self.assertNotIn("initial_comparison", network_doc["fleet"])
        self.assertTrue(all(agent["plan_color"][-1] == 0 for agent in network_doc["agents"]))
        from rerun_scripts.fleet import route_frame
        exported_history = network_doc["fleet"]["route_history"]
        self.assertEqual(len(exported_history), 7)
        for r in exported_history:
            state = route_frame(network_doc["fleet"], r["time_s"])[r["agent"]]
            self.assertEqual(state["route"]["points"], r["points"])
        for n in ("D", "E"):
            routes = [r for r in exported_history if r["agent"] == n]
            self.assertNotEqual(routes[0]["points"], routes[1]["points"])
        lines = network_summary("mountain_exchange_5", result.plan, network_doc["fleet"])
        self.assertLessEqual(len(lines), 12)
        self.assertIn("NORMAL", lines[0])
        self.assertEqual(sum(": COMMIT" in line for line in lines), 5)

    def test_five_agent_slow_network_timeout_retains_only_a_and_holds_other_four(self):
        from rerun_scripts.fleet import export_fleet, network_frame, network_summary
        world, requests = make_fleet_scenario("mountain_exchange_5")
        network = NetworkSettings(minimum_delay=3, maximum_delay=3, loss=0)
        with self.assertRaises(DecentralizedPlanningError) as caught:
            simulate_decentralized(world, requests, planning_window=15, network=network)
        result = caught.exception.result
        self.assertFalse(result.agreement_complete)
        self.assertEqual(result.negotiation_time, 15)
        self.assertEqual(set(result.metadata()["commit_times_s"]), {"A"})
        self.assertTrue(result.plan.validate(world))
        for flight, request in zip(result.plan.flights, requests):
            self.assertEqual(flight.request.initial_phase, request.initial_phase)
            if request.name == "A":
                self.assertGreater(flight.duration, 0)
            else:
                self.assertEqual(flight.duration, 0)
                for at in (0, 15, 100, 600):
                    p = flight.pose_at(at)
                    self.assertAlmostEqual(hypot(p.x-request.start.x, p.y-request.start.y), 40)
        doc = export_fleet("mountain_exchange_5", world, result.plan, fps=1,
                           protocol=result, network_only=True)
        states = network_frame(doc["fleet"], 15)["states"]
        self.assertEqual(states["A"], "согласован · круг")
        self.assertTrue(all(states[n] == "нет согласования · круг" for n in "BCDE"))
        self.assertFalse(network_frame(doc["fleet"], 15.4)["packets"])
        from rerun_scripts.fleet import route_frame
        self.assertEqual(len(doc["fleet"]["route_history"]), 5)
        route_states = route_frame(doc["fleet"], 15.4)
        self.assertEqual(route_states["A"]["state"], "committed")
        self.assertTrue(all(route_states[n]["route"] is None for n in "BCDE"))
        lines = network_summary("mountain_exchange_5", result.plan, doc["fleet"])
        self.assertLessEqual(len(lines), 12)
        self.assertIn("TIMEOUT", lines[0])
        self.assertEqual(sum("НЕТ ВЫЛЕТА" in line for line in lines), 4)
        with self.assertRaisesRegex(ValueError, "protocol"):
            export_fleet("mountain_exchange_5", world, result.plan, network_only=True)

    def test_spatial_retry_keeps_full_budget_and_initial_pose(self):
        world, requests = make_fleet_scenario("hills")
        original = LocalPlanner._safe
        def first_route_rejected(node, a, b):
            return False if node.name == "A" and node.attempts == 1 else original(node, a, b)
        with patch.object(LocalPlanner, "_safe", first_route_rejected):
            result = simulate_decentralized(world, requests)
        self.assertEqual(dict(result.route_attempts)["A"], 2)
        self.assertTrue(any(e.kind == "REPLAN" for e in result.events))
        f = result.plan.flights[0]
        self.assertEqual(f.route.iterations, 600)
        self.assertAlmostEqual(f.pose_at(0).x, f.request.start.x + f.request.start.radius)
        self.assertAlmostEqual(f.pose_at(0).y, f.request.start.y)

    def test_phase_delay_then_complete_laps_no_teleport(self):
        world, requests = make_fleet_scenario("crossing")
        f = simulate_decentralized(world, requests).plan.flights[1]
        self.assertGreater(f.phase_delay, 0)
        self.assertAlmostEqual(f.departure, f.phase_delay + f.waiting_laps * f.lap_period)
        start = f.route.paths[0].start
        at = f.pose_at(f.departure)
        self.assertLess(hypot(at.x-start.x, at.y-start.y, at.z-start.z), 1e-8)
        a, b = f.pose_at(f.departure-1e-7), f.pose_at(f.departure+1e-7)
        self.assertLess(hypot(a.x-b.x, a.y-b.y, a.z-b.z), 1e-4)
        with self.assertRaises(ValueError):
            replace(f, departure=f.departure+.1)
        held = holding_flight(f.request)
        for t in (0, 1, 3, f.departure-1e-6):
            a, b = held.pose_at(t), f.pose_at(t)
            self.assertLess(hypot(a.x-b.x, a.y-b.y, a.z-b.z), 1e-8)

    def test_stale_ack_and_unvoted_commit_are_ignored(self):
        world, requests = make_fleet_scenario("hills")
        result = simulate_decentralized(world, requests)
        roster = {f.request.name: f.request for f in result.plan.flights}
        transport = SimulatedTransport(NetworkSettings(loss=0))
        node = LocalPlanner(world, roster["A"], roster, transport, separation=40, max_delay=120,
                            planning_window=5, horizon=600, iterations=600, seed=7, max_route_attempts=3)
        node.pending, node.version = result.plan.flights[0], 2
        node.receive(Message("ACK", "B", "A", 1))
        node.receive(Message("ACK", "unknown", "A", 2))
        self.assertFalse(node.acks)
        node.receive(Message("ACK", "B", "A", 2))
        self.assertEqual(node.acks, {"B"})
        flight = result.plan.flights[1]
        node.receive(Message("COMMIT", "B", "A", 1, flight))
        self.assertFalse(node.received)
        node.votes["B"] = (1, flight)
        altered = replace(flight, departure=flight.departure+flight.lap_period)
        node.receive(Message("COMMIT", "B", "A", 1, altered))
        self.assertFalse(node.received)
        node.receive(Message("COMMIT", "B", "A", 1, flight))
        self.assertEqual(node.received["B"].flight, flight)

    def test_single_agent_and_invalid_inputs(self):
        world, requests = make_fleet_scenario("hills")
        result = simulate_decentralized(world, requests[:1], network=NetworkSettings(loss=1))
        self.assertEqual(result.messages_sent, 0)
        self.assertTrue(result.plan.validate(world))
        for changes in ({"planning_window": 0}, {"horizon": 120}, {"max_route_attempts": 0}):
            with self.assertRaises(ValueError):
                simulate_decentralized(world, requests, **changes)
        for changes in ({"loss": 2}, {"maximum_delay": -1}, {"retry_interval": 0}):
            with self.assertRaises(ValueError):
                NetworkSettings(**changes)

    @unittest.skipUnless(importlib.util.find_spec("rerun"), "optional Rerun SDK")
    def test_comparison_recordings_keep_unsafe_mode_explicit_and_emit_no_warnings(self):
        from dubins_airplane.fleet import FleetPlan
        from rerun_scripts.fleet import attach_initial_comparison, export_fleet, render_fleet
        import rerun as rr
        import warnings
        world, requests = make_fleet_scenario("crossing")
        result = simulate_decentralized(world, requests)
        coordinated = export_fleet("crossing", world, result.plan, fps=2, protocol=result)
        attach_initial_comparison(coordinated, result)
        initial_plan = FleetPlan(result.initial_flights, 40, 600, result.plan.routing_ms, 0)
        initial = export_fleet("crossing", world, initial_plan, fps=2)
        initial["fleet"].update(coordination="uncoordinated_demo", safety_validated=False,
                               initial_comparison=coordinated["fleet"]["initial_comparison"])
        self.assertTrue(initial["fleet"]["initial_comparison"]["initial_distance_series"])
        self.assertFalse(initial["fleet"]["safety_validated"])
        with tempfile.TemporaryDirectory() as directory:
            try:
                for name, plan, doc in (("coordinated", result.plan, coordinated), ("initial_UNSAFE", initial_plan, initial)):
                    with self.subTest(name=name), warnings.catch_warnings():
                        warnings.simplefilter("error")
                        target = Path(directory) / (name + ".rrd")
                        render_fleet("crossing", world, plan, doc, save=target)
                        self.assertGreater(target.stat().st_size, 1000)
                        self.assertEqual(doc["fleet"]["recording_completion_timeline"], "render_complete")
            finally:
                rr.disconnect()

    @unittest.skipUnless(importlib.util.find_spec("rerun"), "optional Rerun SDK")
    def test_protocol_export_and_actual_recording(self):
        from rerun_scripts.fleet import export_fleet, render_fleet
        import rerun as rr
        import warnings
        world, requests = make_fleet_scenario("mountain_pass")
        result = simulate_decentralized(world, requests)
        doc = export_fleet("mountain_pass", world, result.plan, fps=2, protocol=result)
        self.assertEqual(doc["fleet"]["coordination"], "decentralized")
        self.assertTrue(doc["fleet"]["events"])
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "fleet.rrd"
            try:
                with warnings.catch_warnings():
                    warnings.simplefilter("error")
                    render_fleet("mountain_pass", world, result.plan, doc, save=target)
                self.assertGreater(target.stat().st_size, 1000)
            finally:
                rr.disconnect()


if __name__ == "__main__":
    unittest.main()
