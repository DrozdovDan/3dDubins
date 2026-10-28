"""Time-aware geometry, exact edge endpoints, descendant retiming and safe failure."""

from dataclasses import replace
from math import pi
import os
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from dubins_airplane import (AgentRequest, DecentralizedPlanningError, LoiterCircle, NoPathError,
                            Pose, ScheduledFlight, NetworkSettings, plan_dubins_airplane,
                            plan_safe_rrt_star_traffic, simulate_decentralized)
from dubins_airplane.decentralized import holding_flight
from dubins_airplane.fleet import _separation_python
from dubins_airplane.native import library_path
from dubins_airplane.rrt_star import _Node, _traffic_rewire_subtree
from dubins_airplane.traffic import _TrafficConstraints
from examples.example_fleet import make_fleet_scenario


class TrafficReferenceTests(unittest.TestCase):
    def test_internal_edge_endpoint_is_not_replaced_by_a_fake_goal_circle(self):
        with patch.dict(os.environ, DUBINS_BACKEND="python"):
            path = plan_dubins_airplane(Pose(0, 0, 75, 0), Pose(100, 0, 75, 0), 10, pi/12)
            circle = LoiterCircle(100, 10, 75, 10)
            # Peer is exactly at the internal edge's endpoint at t=5 seconds.
            peer = holding_flight(AgentRequest("peer", circle, circle, radius=10,
                                              initial_phase=1.5*pi - 10))
            traffic = _TrafficConstraints(LoiterCircle(0, 10, 75, 10),
                LoiterCircle(300, 10, 75, 10), 20, 0, (peer,), 5, 60)
            self.assertTrue(traffic.hold_free(path.start))
            self.assertFalse(traffic.edge_free(path, 0))

    def test_rewire_rejects_unsafe_descendant_arrival_without_mutating_tree(self):
        p = Pose(0, 0, 75, 0)
        nodes = [_Node(p), _Node(p, 100, 0, SimpleNamespace(length=100)),
                 _Node(p, 200, 1, SimpleNamespace(length=100)), _Node(p, 50, 0, SimpleNamespace(length=50))]
        nodes[0].children = {1, 3}
        nodes[1].children = {2}
        seen = []
        def free(edge, cost):
            seen.append(cost)
            return cost >= 100
        traffic = SimpleNamespace(edge_free=free)
        self.assertIsNone(_traffic_rewire_subtree(nodes, 1, 3, SimpleNamespace(length=25), traffic))
        self.assertEqual(seen, [75])  # Descendant previously began at cost 100, now 75.
        self.assertEqual([(n.cost, n.parent) for n in nodes], [(0, None), (100, 0), (200, 1), (50, 0)])
        self.assertEqual(nodes[1].children, {2})
        self.assertIsNone(_traffic_rewire_subtree(nodes, 1, 2, SimpleNamespace(length=25), traffic))


@unittest.skipUnless(library_path().is_file(), "requires C++ library")
class TrafficNativeTests(unittest.TestCase):
    def setUp(self):
        self.backend = patch.dict(os.environ, DUBINS_BACKEND="cpp")
        self.backend.start()
        self.addCleanup(self.backend.stop)
        self.world, requests = make_fleet_scenario("crossing")
        self.result = simulate_decentralized(self.world, requests, network=NetworkSettings(loss=0))
        self.peer, self.initial = self.result.initial_flights

    def search(self, peers, *, backend="cpp", **changes):
        f = self.initial
        parameters = dict(start_pose=f.route.paths[0].start, departure=f.departure,
                          speed=f.request.speed, reservations=peers, iterations=60, seed=1017,
                          separation=40, horizon=600)
        parameters.update(changes)
        with patch.dict(os.environ, DUBINS_BACKEND=backend):
            route = plan_safe_rrt_star_traffic(self.world, f.request.start, f.request.goal,
                                              f.request.radius, f.request.max_angle, **parameters)
        return ScheduledFlight(f.request, route, f.departure)

    def test_same_peer_geometry_at_different_times_changes_the_planned_geometry(self):
        early = self.search([self.peer])
        late_peer = replace(self.peer, departure=self.peer.departure + 10*self.peer.lap_period)
        late = self.search([late_peer])
        self.assertNotEqual(early.route.paths, late.route.paths)
        self.assertGreater(early.route.length, late.route.length)
        for own, peer in ((early, self.peer), (late, late_peer)):
            self.assertEqual(own.departure, self.initial.departure)
            self.assertEqual(own.route.paths[0].start, self.initial.route.paths[0].start)
            self.assertTrue(_separation_python(own, peer, 40, 0, 600, 2, .01).safe)
            self.assertTrue(all(self.world._path_is_free_python(p) for p in own.route.paths))

    def test_python_and_cpp_time_aware_searches_both_produce_safe_fixed_departure_routes(self):
        cpp, python = self.search([self.peer]), self.search([self.peer], backend="python")
        self.assertEqual(cpp.route.iterations, 60)
        self.assertEqual(python.route.iterations, 60)
        self.assertAlmostEqual(cpp.route.length, python.route.length, places=5)
        self.assertTrue(_separation_python(python, self.peer, 40, 0, 600, 2, .01).safe)
        self.assertTrue(all(self.world._path_is_free_python(p) for p in python.route.paths))

    def test_invalid_times_and_fixed_root_are_rejected(self):
        for changes in ({"speed": 0}, {"departure": -1}, {"separation": 0},
                        {"horizon": 1}, {"start_pose": Pose(400, 0, 75, 0)}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                self.search([self.peer], **changes)
        with self.assertRaises(NoPathError):
            self.search([], horizon=self.initial.departure + 1)
        with self.assertRaises(ValueError):
            self.search([self.peer], horizon=self.peer.arrival - 1)

    def test_adaptive_search_checks_traffic_before_accepting_a_short_route(self):
        from dubins_airplane.rrt_star import _route_lower_bound
        results = []
        for backend in ('python', 'cpp'):
            with self.subTest(backend=backend):
                own = self.search([self.peer], backend=backend, relative_gap=.03)
                self.assertNotEqual(own.route.paths, self.initial.route.paths)
                self.assertTrue(_separation_python(own, self.peer, 40, 0, 600, 2, .01).safe)
                self.assertTrue(all(self.world._path_is_free_python(p) for p in own.route.paths))
                bound = _route_lower_bound([self.initial.route.paths[0].start],
                    own.request.goal.states(8, 'CW')+own.request.goal.states(8, 'CCW'),
                    own.request.radius, own.request.max_angle)
                if own.route.iterations < 60:
                    self.assertLessEqual(own.route.length, 1.03*bound)
                results.append(own.route)
        self.assertEqual(results[0].iterations, results[1].iterations)
        self.assertAlmostEqual(results[0].length, results[1].length, places=5)

    def test_no_replanning_budget_never_falls_back_to_delaying_an_unsafe_route(self):
        world, requests = make_fleet_scenario("altitude_conflict_1km")
        with self.assertRaises(DecentralizedPlanningError) as caught:
            simulate_decentralized(world, requests, max_route_attempts=1)
        result = caught.exception.result
        self.assertFalse(result.agreement_complete)
        self.assertFalse(any(e.kind == "DELAY" for e in result.events))
        flights = {f.request.name: f for f in result.plan.flights}
        self.assertGreater(flights["A"].duration, 0)
        self.assertEqual(flights["B"].duration, 0)  # Safe failure: permanent initial hold.
        self.assertTrue(result.plan.validate(world))


if __name__ == "__main__":
    unittest.main()
