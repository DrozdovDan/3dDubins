"""Task diversity, nested fleets and the actual time-aware spatial replanner."""

from contextlib import redirect_stdout
import io
import importlib.util
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from dubins_airplane import Pose, plan_dubins_airplane
from dubins_airplane.native import library_path
from dubins_airplane.traffic import plan_safe_rrt_star_traffic
from examples.benchmark_scenarios import (COUNTS, DIRECTIONS, FLIGHT_BOUNDS, GOAL_REGION, SCENARIOS, TOPOLOGIES, describe_scene, goal_region,
                                          main, make_requests, make_scene, measure_task, summary_rows)


class ScenarioSummaryTests(unittest.TestCase):
    def test_per_planning_time_keeps_wall_and_local_timers_distinct_and_includes_failures(self):
        from examples.benchmark_scenarios import planning_time_metrics
        run = dict(agents=2, wall_ms=70, rrt_calls=2,
                   calls=[dict(elapsed_ms=40, succeeded=True), dict(elapsed_ms=60, succeeded=False)])
        self.assertEqual(planning_time_metrics(run), (35, 50))
        self.assertEqual(planning_time_metrics(dict(wall_ms=0, rrt_calls=0, calls=[])), (None, None))
        self.assertEqual(planning_time_metrics(dict(wall_ms=70, rrt_calls=2)), (35, None))

    def test_per_planning_summary_takes_median_of_mission_ratios(self):
        base = dict(agents=2, complete=True, safety_valid=True, total_length_m=8000,
                    replan_calls=0, geometry_changed_agents=[])
        runs = [dict(base, wall_ms=10, rrt_calls=1, calls=[dict(elapsed_ms=10)]),
                dict(base, wall_ms=300, rrt_calls=3, calls=[dict(elapsed_ms=t) for t in (30, 90, 180)])]
        row, = summary_rows(runs)
        self.assertEqual(row['wall_per_planning_median_ms'], 55)
        self.assertEqual(row['local_per_planning_median_ms'], 55)
        self.assertEqual(row['wall_per_planning_tasks'], 2)
        self.assertEqual(row['local_per_planning_tasks'], 2)

    def test_call_summary_separates_budgets_roles_and_includes_unsuccessful_calls(self):
        from examples.benchmark_scenarios import planning_call_summary
        runs = [dict(calls=[dict(attempt=1, mode='static', iteration_budget=600, elapsed_ms=10, succeeded=False),
                            dict(attempt=2, mode='static', iteration_budget=3000, elapsed_ms=40, succeeded=True),
                            dict(attempt=3, mode='traffic', iteration_budget=3000, elapsed_ms=50, succeeded=True)])]
        rows = planning_call_summary(runs)
        self.assertEqual([(r['iteration_budget'], r['kind'], r['count'], r['failed']) for r in rows],
                         [(600, 'initial', 1, 1), (3000, 'static_retry', 1, 0), (3000, 'traffic', 1, 0)])
        self.assertEqual([r['share_of_call_time'] for r in rows], [.1, .4, .5])

    def test_incomplete_mission_is_not_a_short_success(self):
        good = dict(agents=2, complete=True, safety_valid=True, wall_ms=100,
                    total_length_m=12000, mean_length_m=6000, replan_calls=1,
                    geometry_changed_agents=["B"])
        bad = dict(good, complete=False, wall_ms=200, total_length_m=None, mean_length_m=None,
                   replan_calls=2, geometry_changed_agents=[])
        row = summary_rows([good, bad])[0]
        self.assertEqual(row["tasks"], 2)
        self.assertEqual(row["complete"], 1)
        self.assertEqual(row["wall_median_ms"], 150)
        self.assertEqual(row["wall_per_agent_median_ms"], 75)
        self.assertEqual(row["length_median_m"], 12000)
        self.assertEqual(row["mean_agent_length_m"], 6000)
        self.assertEqual(row["replan_calls"], 3)
        self.assertEqual(row['replans_per_agent_median'], .75)
        self.assertEqual(row['replans_per_agent_mean'], .75)
        self.assertEqual(row["changed_agents"], 1)

    def test_directions_are_pooled_before_taking_the_median(self):
        base = dict(agents=2, complete=True, safety_valid=True,
                    total_length_m=8000, replan_calls=0, geometry_changed_agents=[])
        runs = [dict(base, direction=direction, wall_ms=wall)
                for direction, wall in [('ascent', 100), ('ascent', 200), ('descent', 1000)]]
        row, = summary_rows(runs)
        self.assertEqual(row['tasks'], 3)
        self.assertEqual(row['wall_per_agent_median_ms'], 100)

    def test_replans_normalise_each_mission_and_keep_mean_distinct_from_median(self):
        base = dict(complete=True, safety_valid=True, wall_ms=100, total_length_m=8000,
                    geometry_changed_agents=[])
        runs = [dict(base, agents=2, replan_calls=k) for k in (0, 1, 5)]
        runs += [dict(base, agents=4, replan_calls=k) for k in (0, 4)]
        small, large = summary_rows(runs)
        self.assertEqual(small['replans_per_agent_median'], .5)
        self.assertEqual(small['replans_per_agent_mean'], 1)
        self.assertEqual(large['replans_per_agent_median'], .5)
        self.assertEqual(large['replans_per_agent_mean'], .5)


@unittest.skipUnless(library_path().is_file(), "requires built C++ backend")
class ScenarioBenchmarkTests(unittest.TestCase):
    def setUp(self):
        backend = patch.dict(os.environ, DUBINS_BACKEND="cpp")
        backend.start()
        self.addCleanup(backend.stop)

    def test_all_topologies_and_rosters_have_valid_airborne_endpoints(self):
        obstacle_counts = dict(open=0, pinch=2, fork=3, slalom=6, islands=5, mountains=3)
        for topology in TOPOLOGIES:
            for scenario in SCENARIOS:
                with self.subTest(topology=topology, scenario=scenario):
                    world = make_scene(topology, scenario)
                    full = make_requests(world, scenario, 8)
                    self.assertEqual((world.terrain.columns, world.terrain.rows, world.terrain.resolution),
                                     (100, 100, 10))
                    self.assertEqual((world.terrain.x_min, world.terrain.x_max, world.terrain.y_min, world.terrain.y_max),
                                     (0, 1000, -500, 500))
                    self.assertEqual(len(world.obstacles), obstacle_counts[topology])
                    self.assertEqual((min(map(min, world.terrain.heights)), max(map(max, world.terrain.heights))),
                                     (0, 1000))
                    self.assertTrue(all(abs(r.start.z-r.goal.z) == 1000 for r in full))
                    self.assertTrue(all(r.speed == 20 and r.start.x < 300 and r.goal.x > 700 for r in full))
                    for r in full:
                        self.assertGreaterEqual(r.goal.x-r.goal.radius, GOAL_REGION["x"][0])
                        self.assertLessEqual(r.goal.x+r.goal.radius, GOAL_REGION["x"][1])
                        self.assertGreaterEqual(r.goal.y-r.goal.radius, GOAL_REGION["y"][0])
                        self.assertLessEqual(r.goal.y+r.goal.radius, GOAL_REGION["y"][1])
                    self.assertTrue(all(b.z_min == 150 and b.z_max == 1150 for b in world.obstacles))
                    self.assertTrue(all(z >= 150-1e-6 for row in world.lower for z in row))
                    self.assertTrue(all(z <= 1150+1e-6 for row in world.upper for z in row))
                    for count in COUNTS:
                        self.assertEqual(make_requests(world, scenario, count), full[:count])
                    scene = describe_scene(world, topology, scenario)
                    self.assertEqual(scene["cell_m"], 10)
                    self.assertEqual(scene["overview_cell_m"], 50)
                    self.assertEqual(scene["flight_bounds_m"], FLIGHT_BOUNDS)
                    self.assertEqual(scene["mission"]["objective"], "all_agents_reach_common_region")
                    self.assertFalse(scene["mission"]["formation_required"])

    def test_different_motion_patterns_are_not_different_seeds_of_one_task(self):
        world = make_scene("open", "transfer")
        requests = [make_requests(world, scenario, 8) for scenario in SCENARIOS]
        self.assertEqual(len(set(requests)), 3)
        self.assertTrue(all(r.start.x < 500 for r in requests[2]))
        self.assertTrue(all(r.start.x < 500 and r.goal.x > 500 for group in requests for r in group))
        self.assertTrue(all((r.goal.x-r.start.x, r.goal.y-r.start.y) == (700, 0) for r in requests[0]))
        self.assertTrue(all(r.goal.y == -r.start.y for r in requests[2]))
        self.assertGreater(max(r.start.y for r in requests[1])-min(r.start.y for r in requests[1]),
                           max(r.goal.y for r in requests[1])-min(r.goal.y for r in requests[1]))
        with self.assertRaises(ValueError):
            make_requests(world, "transfer", 9)

    def test_descent_is_a_common_swarm_mission_and_gathering_stays_gathering(self):
        region = goal_region('descent')
        for topology in TOPOLOGIES:
            for scenario in SCENARIOS:
                with self.subTest(topology=topology, scenario=scenario):
                    world = make_scene(topology, scenario)
                    ascent = make_requests(world, scenario, 8)
                    descent = make_requests(world, scenario, 8, 'descent')
                    self.assertTrue(all((r.start.z, r.goal.z) == (1150, 150) for r in descent))
                    self.assertTrue(all(r.start.x > 700 and r.goal.x < 300 for r in descent))
                    self.assertTrue(all(world.validate_loiter(c) for r in descent for c in (r.start, r.goal)))
                    for up, down in zip(ascent, descent):
                        self.assertEqual((down.start.x, down.start.y), (1000-up.start.x, up.start.y))
                        self.assertEqual((down.goal.x, down.goal.y), (1000-up.goal.x, up.goal.y))
                        self.assertLessEqual(region['x'][0], down.goal.x-down.goal.radius)
                        self.assertGreaterEqual(region['x'][1], down.goal.x+down.goal.radius)
                        self.assertLessEqual(region['y'][0], down.goal.y-down.goal.radius)
                        self.assertGreaterEqual(region['y'][1], down.goal.y+down.goal.radius)
                    for n in COUNTS:
                        self.assertEqual(make_requests(world, scenario, n, 'descent'), descent[:n])
                    if scenario == 'gather':
                        self.assertGreater(max(r.start.y for r in descent)-min(r.start.y for r in descent),
                                           max(r.goal.y for r in descent)-min(r.goal.y for r in descent))
                    scene = describe_scene(world, topology, scenario, 'descent')
                    self.assertEqual(scene['mission']['direction'], 'descent')
                    self.assertEqual(scene['mission']['goal_region_m'], region)
        with self.assertRaises(ValueError):
            make_requests(world, 'transfer', 5, 'mixed')

    def test_real_swarm_descent_is_new_planning_with_common_lower_goal(self):
        world = make_scene('open', 'reconfigure')
        requests = make_requests(world, 'reconfigure', 5, 'descent')
        run = measure_task(world, requests, 'open', 'reconfigure', direction='descent')
        self.assertTrue(run['complete'], run['failure'])
        self.assertTrue(run['safety_valid'])
        self.assertEqual(run['direction'], 'descent')
        self.assertEqual(len(run['final_routes']), 5)
        self.assertTrue(all(r['points'][0][2] == 1150 and r['points'][-1][2] == 150 for r in run['final_routes']))
        self.assertTrue(all(abs(delay) < 1e-9 for delay in run['extra_departure_delay_s'].values()))
        with self.assertRaises(ValueError):
            measure_task(world, requests, 'open', 'reconfigure', direction='ascent')

    def test_legacy_example_is_separate_from_the_new_cube(self):
        from examples.example_fleet import make_fleet_scenario
        original_world, original_requests = make_fleet_scenario("mountain_exchange_5")
        world = make_scene("mountains", "transfer")
        self.assertEqual(original_world.terrain.x_max-original_world.terrain.x_min, 6400)
        self.assertEqual(len(original_requests), 5)
        self.assertEqual(world.terrain.x_max-world.terrain.x_min, 1000)
        self.assertNotEqual(make_requests(world, "transfer", 5), original_requests)

    def test_cube_z_limits_are_enforced_by_native_continuous_edge_checker(self):
        world = make_scene("open", "transfer")
        for start, end in ((Pose(80, -300, 140, 0), Pose(240, -300, 140, 0)),
                           (Pose(760, -300, 1160, 0), Pose(920, -300, 1160, 0))):
            path = plan_dubins_airplane(start, end, 40, .26)
            self.assertFalse(world.path_is_free(path))

    def test_real_full_budget_spatial_replanning_changes_geometry_not_departure(self):
        world = make_scene("open", "reconfigure")
        requests = make_requests(world, "reconfigure", 5)
        with patch("dubins_airplane.decentralized.plan_safe_rrt_star_traffic",
                   wraps=plan_safe_rrt_star_traffic) as traffic:
            run = measure_task(world, requests, "open", "reconfigure", retry_iterations=600)
        self.assertTrue(run["complete"])
        self.assertTrue(run["safety_valid"])
        self.assertTrue(run["initial_conflicts"])
        self.assertTrue(run["geometry_changed_agents"])
        self.assertEqual(run["replan_calls"], traffic.call_count)
        self.assertGreaterEqual(traffic.call_count, 2)
        self.assertTrue(all(c.kwargs["iterations"] == 600 for c in traffic.call_args_list))
        self.assertTrue(all(c.kwargs["reservations"] for c in traffic.call_args_list))
        self.assertTrue(all(abs(v) < 1e-9 for v in run["extra_departure_delay_s"].values()))
        self.assertTrue(any(e["kind"] == "REPLAN" for e in run["events"]))
        self.assertGreater(len(run["route_history"]), 5)
        self.assertEqual(len(run["initial_routes"]), 5)
        self.assertEqual(len(run["final_routes"]), 5)
        self.assertEqual(run["wall_per_agent_ms"], run["wall_ms"]/5)
        self.assertEqual(run["mean_length_m"], run["total_length_m"]/5)
        self.assertEqual(run["iterations_per_call"], 600)
        self.assertEqual(run["total_iteration_budget"], 600*run["rrt_calls"])
        self.assertTrue(all(r["rrt_iterations"] == 600 for r in run["final_routes"]))
        for route in run["initial_routes"] + run["final_routes"]:
            self.assertGreater(route["length_m"], 1000)
            for x, y, z in route["points"]:
                self.assertTrue(-1e-5 <= x <= 1000+1e-5)
                self.assertTrue(-500-1e-5 <= y <= 500+1e-5)
                self.assertTrue(150-1e-5 <= z <= 1150+1e-5)

    def test_cli_executes_each_selected_task_once_and_keeps_a_journal(self):
        row = dict(id="open/transfer/n2", agents=2, complete=True, safety_valid=True, wall_ms=1,
                   total_length_m=12000, mean_length_m=6000, replan_calls=1, geometry_changed_agents=["B"])
        with tempfile.TemporaryDirectory() as folder, redirect_stdout(io.StringIO()):
            output = Path(folder)/"results.json"
            with patch("examples.benchmark_scenarios.measure_task", return_value=row) as measurement:
                self.assertEqual(main(["--topology", "open", "--scenario", "transfer", "--agents", "2",
                                       "--direction", "ascent", "--no-plot", "--output", str(output)]), 0)
            self.assertEqual(measurement.call_count, 1)
            report = json.loads(output.read_text(encoding="utf-8"))
            self.assertTrue(report["configuration"]["spatial_replanning"])
            self.assertEqual(report["configuration"]["iterations"], 600)
            self.assertEqual(report["configuration"]["retry_iterations"], 3000)
            self.assertEqual(report["configuration"]["max_route_attempts"], 8)
            self.assertEqual(report['configuration']['directions'], ['ascent'])
            self.assertNotIn("runs", report["configuration"])
            self.assertEqual(len(report["runs"]), 1)
            journal = [json.loads(line) for line in output.with_suffix(".runs.jsonl").read_text(encoding="utf-8").splitlines()]
            self.assertEqual([r["type"] for r in journal], ["header", "scene", "run"])

    @unittest.skipUnless(importlib.util.find_spec("matplotlib"), "optional Matplotlib")
    def test_normalised_plot_pools_both_directions_in_one_box_per_n(self):
        from matplotlib.figure import Figure
        world = make_scene("open", "transfer")
        run = measure_task(world, make_requests(world, "transfer", 2), "open", "transfer")
        self.assertTrue(run["complete"])
        # Synthetic second observation tests presentation only, not a measured mission.
        descent = dict(run, direction='descent', wall_ms=run['wall_ms']*3,
                       total_length_m=run['total_length_m']*1.2, replan_calls=run['replan_calls']+4,
                       rrt_calls=run['rrt_calls']+7,
                       calls=run['calls']+[dict(elapsed_ms=25, succeeded=False, attempt=2, mode='static',
                                               iteration_budget=3000) for _ in range(7)])
        report = {"configuration": {"iterations": 600}, "runs": [run, descent],
                  "scenes": {run['scene_id']: describe_scene(world, "open", "transfer")}}
        original_save = Figure.savefig
        observed = {}

        def inspect_and_save(figure, filename, **kwargs):
            name = Path(filename).name
            if name in ("time.png", "length.png", "plannings.png", "replans.png"):
                offsets = [point for collection in figure.axes[0].collections for point in collection.get_offsets()]
                self.assertEqual(len(offsets), 2)
                labels = [label.get_text() for label in figure.axes[0].get_xticklabels()]
                self.assertEqual(labels, ['2 агента\n2/2 задач'])
                observed[name] = [float(point[1]) for point in offsets]
            elif name == 'metrics.png':
                self.assertEqual(len(figure.axes), 3)
                self.assertEqual(tuple(figure.get_size_inches()), (16, 9))
                for label, axis in zip(('slide_time', 'slide_length', 'slide_plannings'), figure.axes):
                    offsets = [point for collection in axis.collections for point in collection.get_offsets()]
                    self.assertEqual(len(offsets), 2)
                    observed[label] = [float(point[1]) for point in offsets]
            elif name == 'planning_vs_time.png':
                self.assertEqual(len(figure.axes), 1)
                self.assertEqual(tuple(figure.get_size_inches()), (16, 9))
                observed['scatter'] = [tuple(map(float, point)) for collection in figure.axes[0].collections
                                       for point in collection.get_offsets()]
            elif name == 'time_per_planning.png':
                self.assertEqual(len(figure.axes), 2)
                self.assertEqual(tuple(figure.get_size_inches()), (16, 9))
                for label, axis in zip(('wall_per_call', 'local_per_call'), figure.axes):
                    observed[label] = [float(point[1]) for collection in axis.collections
                                       for point in collection.get_offsets()]
            elif name == 'planning_call_times.png':
                self.assertEqual(len(figure.axes), 2)
                self.assertEqual(tuple(figure.get_size_inches()), (16, 9))
                observed['individual_calls'] = [float(point[1]) for axis in figure.axes
                    for collection in axis.collections for point in collection.get_offsets()]
            return original_save(figure, filename, **kwargs)

        with tempfile.TemporaryDirectory() as folder, redirect_stdout(io.StringIO()):
            source = Path(folder)/'results.json'
            source.write_text(json.dumps(report, ensure_ascii=False), encoding='utf-8')
            original = source.read_bytes()
            destination = Path(folder)/'report'
            destination.mkdir()
            (destination/'routes.png').write_bytes(b'previous Rerun capture')
            with patch.object(Figure, "savefig", new=inspect_and_save), \
                    patch('examples.render_benchmark_routes.render_routes') as renderer, \
                    patch('examples.benchmark_scenarios.simulate_decentralized') as planner:
                self.assertEqual(main(['--plot-only', str(source), '--metrics-only']), 0)
                renderer.assert_not_called()
                planner.assert_not_called()
            self.assertEqual(source.read_bytes(), original)
            self.assertEqual((destination/'routes.png').read_bytes(), b'previous Rerun capture')
            summary = json.loads((destination/'summary.json').read_text(encoding='utf-8'))
            self.assertEqual(summary[0]['replans_per_agent_median'],
                             (run['replan_calls']+descent['replan_calls'])/4)
            text = (Path(folder)/"report/README.md").read_text(encoding="utf-8")
            self.assertIn("T/N", text)
            self.assertIn("ΣLᵢ/N", text)
            self.assertIn('R/N (среднее)', text)
            self.assertIn('Подъём и спуск объединены', text)
            self.assertNotIn('### Отдельно подъём и спуск', text)
            from PIL import Image
            with Image.open(Path(folder)/'report/metrics.png') as combined:
                self.assertEqual(combined.size, (3840, 2160))
            self.assertTrue((Path(folder)/'report/metrics.svg').is_file())
            self.assertTrue((destination/'planning_vs_time.svg').is_file())
            self.assertTrue((destination/'time_per_planning.svg').is_file())
            self.assertTrue((destination/'planning_call_times.svg').is_file())
            expected_calls = [c['elapsed_ms'] for r in (run, descent) for c in r['calls']]
            self.assertCountEqual(observed['individual_calls'], expected_calls)
            with Image.open(destination/'planning_vs_time.png') as scatter:
                self.assertEqual(scatter.size, (3840, 2160))
        for index, observation in enumerate((run, descent)):
            self.assertAlmostEqual(observed["time.png"][index], observation["wall_ms"]/2)
            self.assertAlmostEqual(observed["length.png"][index], observation["total_length_m"]/2/1000)
            self.assertAlmostEqual(observed['slide_time'][index], observation['wall_ms']/2)
            self.assertAlmostEqual(observed['slide_length'][index], observation['total_length_m']/2/1000)
            self.assertAlmostEqual(observed['replans.png'][index], observation['replan_calls']/2)
            self.assertAlmostEqual(observed['plannings.png'][index], observation['rrt_calls']/2)
            self.assertAlmostEqual(observed['slide_plannings'][index], observation['rrt_calls']/2)
            self.assertAlmostEqual(observed['scatter'][index][0], observation['rrt_calls']/2)
            self.assertAlmostEqual(observed['scatter'][index][1], observation['wall_ms']/2)
            self.assertAlmostEqual(observed['wall_per_call'][index], observation['wall_ms']/observation['rrt_calls'])
            self.assertAlmostEqual(observed['local_per_call'][index],
                sum(c['elapsed_ms'] for c in observation['calls'])/observation['rrt_calls'])

    def test_incomplete_task_makes_cli_unsuccessful_and_is_kept_in_results(self):
        row = dict(id='open/transfer/n2', agents=2, complete=False, safety_valid=True, wall_ms=1,
                   total_length_m=None, replan_calls=0, geometry_changed_agents=[])
        with tempfile.TemporaryDirectory() as folder, redirect_stdout(io.StringIO()):
            output = Path(folder)/'results.json'
            with patch('examples.benchmark_scenarios.measure_task', return_value=row):
                self.assertEqual(main(['--topology', 'open', '--scenario', 'transfer', '--agents', '2',
                                       '--direction', 'ascent', '--no-plot', '--output', str(output)]), 1)
            self.assertEqual(json.loads(output.read_text(encoding='utf-8'))['runs'], [row])

    def test_representative_selection_does_not_filter_failed_missions(self):
        from examples.render_benchmark_routes import select_runs
        failed = dict(id='open/reconfigure/n5', topology='open', scenario='reconfigure', agents=5, complete=False)
        good = dict(failed, id='open/reconfigure/n2', agents=2, complete=True)
        self.assertEqual(select_runs({'runs': [good, failed]}), [failed])

    def test_cli_defaults_to_paired_whole_swarm_directions(self):
        seen = []

        def fake_measure(world, requests, topology, scenario, *args, direction, **kwargs):
            seen.append((direction, [(r.start.z, r.goal.z) for r in requests]))
            return dict(id=f'{topology}/{scenario}/{direction}/n2', direction=direction, agents=2,
                        complete=True, safety_valid=True, wall_ms=1, total_length_m=8000,
                        replan_calls=1, geometry_changed_agents=['B'])

        with tempfile.TemporaryDirectory() as folder, redirect_stdout(io.StringIO()):
            output = Path(folder)/'results.json'
            with patch('examples.benchmark_scenarios.measure_task', side_effect=fake_measure) as measurement:
                self.assertEqual(main(['--topology', 'open', '--scenario', 'transfer', '--agents', '2',
                                       '--no-plot', '--output', str(output)]), 0)
            self.assertEqual(measurement.call_count, 2)
            self.assertEqual(seen, [('ascent', [(150, 1150)]*2), ('descent', [(1150, 150)]*2)])
            report = json.loads(output.read_text(encoding='utf-8'))
            self.assertEqual(report['configuration']['directions'], list(DIRECTIONS))
            self.assertEqual(len({r['id'] for r in report['runs']}), 2)

    def test_ideal_network_delivers_every_packet_at_send_time_with_full_replanning(self):
        import examples.benchmark_scenarios as benchmark
        from dubins_airplane import NetworkSettings
        actual_simulation = benchmark.simulate_decentralized
        simulations = []

        def capture(*args, **kwargs):
            self.assertEqual(kwargs['network'], NetworkSettings(**benchmark.IDEAL_NETWORK))
            self.assertEqual(kwargs['relative_gap'], .03)
            result = actual_simulation(*args, **kwargs)
            simulations.append(result)
            return result

        with tempfile.TemporaryDirectory() as folder, redirect_stdout(io.StringIO()):
            output = Path(folder)/'results.json'
            with patch.object(benchmark, 'simulate_decentralized', side_effect=capture):
                self.assertEqual(main(['--network', 'ideal', '--topology', 'open',
                                       '--scenario', 'transfer', '--direction', 'ascent',
                                       '--agents', '2', '--no-plot', '--output', str(output)]), 0)
            report = json.loads(output.read_text(encoding='utf-8'))
        self.assertEqual(report['configuration']['network'], benchmark.IDEAL_NETWORK)
        self.assertEqual(report['configuration']['network_profile'], 'ideal')
        self.assertTrue(report['configuration']['spatial_replanning'])
        self.assertEqual(len(simulations), 1)
        events = simulations[0].events
        self.assertFalse(any(e.kind == 'DROP' for e in events))
        sends = [e for e in events if e.kind == 'SEND']
        deliveries = [e for e in events if e.kind == 'IN_FLIGHT']
        self.assertTrue(sends)
        self.assertEqual(len(sends), len(deliveries))
        self.assertTrue(all(e.delivery_time == e.time for e in deliveries))
        self.assertTrue(report['runs'][0]['complete'] and report['runs'][0]['safety_valid'])

    def test_ideal_cli_has_a_separate_default_output_directory(self):
        row = dict(id='open/transfer/ascent/n2', agents=2, complete=True, safety_valid=True,
                   wall_ms=1, total_length_m=8000, replan_calls=0, geometry_changed_agents=[])
        original = Path.cwd()
        with tempfile.TemporaryDirectory() as folder, redirect_stdout(io.StringIO()):
            try:
                os.chdir(folder)
                with patch('examples.benchmark_scenarios.measure_task', return_value=row):
                    self.assertEqual(main(['--network', 'ideal', '--topology', 'open',
                                           '--scenario', 'transfer', '--direction', 'ascent',
                                           '--agents', '2', '--no-plot']), 0)
                self.assertTrue(Path('outputs/benchmark_swarm_1km_instant/results.json').is_file())
                self.assertFalse(Path('outputs/benchmark_swarm_1km_adaptive/results.json').exists())
            finally:
                os.chdir(original)

    def test_rerun_overview_has_ascent_and_descent_without_cherry_picking(self):
        from examples.render_benchmark_routes import select_runs
        runs = [dict(id=f'{top}/reconfigure/{d}/n5', topology=top, scenario='reconfigure',
                     direction=d, agents=5, complete=False) for top in TOPOLOGIES for d in DIRECTIONS]
        overview = select_runs({'runs': runs})
        self.assertEqual([r['direction'] for r in overview], ['descent', 'ascent']*3)
        for direction in DIRECTIONS:
            self.assertTrue(all(r['direction'] == direction for r in select_runs({'runs': runs}, direction)))

    def test_compose_only_uses_existing_viewer_captures_without_rendering(self):
        from examples.render_benchmark_routes import render_routes
        runs = [dict(id=f'{top}/reconfigure/{d}/n5', topology=top, scenario='reconfigure', direction=d,
                     agents=5, complete=True) for top in TOPOLOGIES for d in DIRECTIONS]
        with tempfile.TemporaryDirectory() as folder, redirect_stdout(io.StringIO()):
            destination = Path(folder)
            (destination/'rerun').mkdir()
            sources = {r['id']: f"rerun/{r['id'].replace('/', '_')}.png" for r in runs}
            for relative in sources.values():
                (destination/relative).touch()  # compose is mocked; no synthetic images delivered.
            (destination/'routes.render.json').write_text(json.dumps(dict(
                renderer='Rerun Viewer', camera_elevation_deg=55, camera_azimuth_deg=-135,
                source_images=sources)), encoding='utf-8')
            with patch('examples.render_benchmark_routes.compose') as compose, \
                 patch('examples.render_benchmark_routes.record_scene') as record, \
                 patch('examples.render_benchmark_routes.capture') as capture:
                render_routes({'runs': runs}, destination, compose_only=True)
                record.assert_not_called()
                capture.assert_not_called()
                self.assertEqual(compose.call_count, 3)
                self.assertEqual([r['direction'] for r in compose.call_args_list[0].args[0]],
                                 ['descent', 'ascent']*3)
                (destination/sources[runs[0]['id']]).unlink()
                with self.assertRaisesRegex(ValueError, 'missing recorded Rerun image'):
                    render_routes({'runs': runs}, destination, compose_only=True)


if __name__ == "__main__":
    unittest.main()
