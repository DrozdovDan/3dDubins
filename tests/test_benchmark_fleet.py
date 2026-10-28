"""Benchmark fixtures, measurement boundaries, failures and replay fidelity."""

from contextlib import redirect_stdout
import io
import importlib.util
import json
import os
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

from dubins_airplane.native import library_path
from dubins_airplane import NoPathError
from examples.benchmark_fleet import (AGENTS, FAMILIES, case_parameters, main, make_case,
                                      measure, quantile, replay_local, summarize, timing_summary, value_summary)


class BenchmarkStatisticsTests(unittest.TestCase):
    def test_quantiles_and_fast_failures_are_not_counted_as_success(self):
        self.assertIsNone(quantile([], .95))
        self.assertEqual(quantile([100], .99), 100)
        self.assertEqual(quantile([0, 100], .95), 95)
        stats = timing_summary([{"elapsed_ms": t, "succeeded": ok}
                                for t, ok in ((1, False), (80, True), (100, True))])
        self.assertEqual(stats["successful_under_100ms_fraction"], 1/3)
        self.assertEqual(stats["success_fraction"], 2/3)
        self.assertEqual(stats["max_ms"], 100)

    def test_quality_statistics_use_the_same_quantile_convention(self):
        self.assertEqual(value_summary([]), {"count": 0})
        self.assertEqual(value_summary([20, 10]),
                         {"count": 2, "min": 10, "p50": 15, "p95": 19.5, "max": 20})

    def test_parameters_are_seeded_and_slope_maps_are_not_duplicate_instances(self):
        for family in FAMILIES:
            self.assertEqual(case_parameters(family, 0), case_parameters(family, 0))
        self.assertNotEqual(case_parameters("slope", 0)["ramp"], case_parameters("slope", 9)["ramp"])
        for args in (("missing", 0), ("combined", -1), ("combined", 10), ("reference", 1)):
            with self.assertRaises(ValueError):
                case_parameters(*args)


class BenchmarkPlotDataTests(unittest.TestCase):
    def test_tukey_boxes_use_quartiles_and_do_not_drop_outliers(self):
        from examples.plot_benchmark_fleet import box_statistics
        stats = box_statistics([0, 1, 2, 3, 4, 100])
        self.assertEqual(stats, {"med": 2.5, "q1": 1.25, "q3": 3.75,
                                 "whislo": 0, "whishi": 4, "fliers": [100]})
        self.assertEqual(box_statistics([42])["whislo"], 42)
        self.assertEqual(box_statistics([1, 1, 1, 1, 20])["fliers"], [20])
        self.assertEqual(box_statistics([0, 2, 3, 4, 7])["fliers"], [])  # Fence is inclusive.
        self.assertEqual(box_statistics([0, 100, 100, 100])["whislo"], 75)
        with self.assertRaises(ValueError):
            box_statistics([])

    def test_replanning_is_not_the_same_as_retrying_a_failed_static_search(self):
        from examples.plot_benchmark_fleet import call_kind
        self.assertEqual(call_kind({"attempt": 1, "mode": "static"}), "initial")
        self.assertEqual(call_kind({"attempt": 2, "mode": "static"}), "static_retry")
        self.assertEqual(call_kind({"attempt": 2, "mode": "traffic"}), "traffic")
        self.assertEqual(call_kind({"attempt": 3, "mode": "traffic"}), "traffic")

    @unittest.skipUnless(importlib.util.find_spec("matplotlib"), "optional Matplotlib")
    def test_slow_replan_remains_visible_and_missing_lengths_have_no_fake_box(self):
        from matplotlib.figure import Figure
        from examples.plot_benchmark_fleet import main as plot_main
        original_save = Figure.savefig
        checked = []

        def inspect_and_save(figure, filename, **kwargs):
            if Path(filename).stem.endswith("_search"):
                self.assertEqual(len(figure.axes), 2)
                self.assertEqual(figure.axes[0].get_ylim(), figure.axes[1].get_ylim())
                self.assertGreater(figure.axes[1].get_ylim()[1], 3000)
                checked.append("search")
            if Path(filename).stem.endswith("_quality"):
                self.assertEqual(len(figure.axes[0].patches), 0)
                checked.append("quality")
            return original_save(figure, filename, **kwargs)

        row = {"case_id": "a", "family": "reference", "map_seed": 0, "agents": 1,
               "seed": 7, "network": "normal", "wall_ms": 3500,
               "agreement_complete": False, "safety_valid": True, "committed_agents": [],
               "total_length_m": None, "makespan_s": None,
               "calls": [{"mode": "static", "attempt": 1, "elapsed_ms": 80, "succeeded": True},
                         {"mode": "traffic", "attempt": 2, "elapsed_ms": 3000, "succeeded": False}]}
        with tempfile.TemporaryDirectory() as folder, redirect_stdout(io.StringIO()):
            source = Path(folder)/"slow_search.json"
            source.write_text(json.dumps({"runs": [row]}), encoding="utf-8")
            with patch.object(Figure, "savefig", new=inspect_and_save):
                self.assertEqual(plot_main([str(source), "--output", str(Path(folder)/"figures")]), 0)
        self.assertCountEqual(checked, ["search", "quality"])

    def test_normal_report_keeps_failed_missions_out_of_length_statistics(self):
        from examples.plot_benchmark_fleet import performance_rows
        base = {"case_id": "a", "network": "normal", "family": "reference", "map_seed": 0,
                "agents": 5, "wall_ms": 200, "safety_valid": True,
                "calls": [{"elapsed_ms": 80, "attempt": 1}, {"elapsed_ms": 120, "attempt": 2}]}
        good = dict(base, agreement_complete=True, total_length_m=35000, makespan_s=400)
        partial = dict(base, agreement_complete=False, total_length_m=None, makespan_s=None)
        row = performance_rows({"runs": [good, partial]})[0]
        self.assertEqual(len(row["runs"]), 2)
        self.assertEqual(len(row["good"]), 1)
        self.assertEqual(row["lengths"], [35])
        self.assertEqual(row["initial"], [80, 80])
        self.assertEqual(row["repeat"], [120, 120])
        self.assertEqual(row["repeats"], [1, 1])
        with self.assertRaisesRegex(ValueError, "normal-only"):
            performance_rows({"runs": [good, dict(partial, network="slow")]})

    def test_plot_groups_do_not_mix_cases_or_networks_and_seed_labels_are_explicit(self):
        from examples.plot_benchmark_fleet import grouped_runs, sample_note
        runs = [{"case_id": case, "network": net, "seed": seed, "family": family, "map_seed": 0}
                for case, net, seed, family in (("a", "ideal", 7, "reference"),
                                                ("a", "normal", 7, "reference"),
                                                ("b", "ideal", 8, "combined"))]
        groups, cases = grouped_runs({"runs": runs})
        self.assertEqual(len(groups), 3)
        self.assertEqual(len(cases), 2)
        self.assertIn("RRT-seed: 7, 8", sample_note({"runs": runs}))
        self.assertIn("ПРОБНЫЕ", sample_note({"runs": runs}))

    @unittest.skipUnless(importlib.util.find_spec("matplotlib"), "optional Matplotlib")
    def test_plot_cli_creates_readable_index_and_handles_missing_replay(self):
        from examples.plot_benchmark_fleet import main as plot_main
        row = {"case_id": "test", "family": "reference", "map_seed": 0, "agents": 5,
               "seed": 7, "network": "normal", "wall_ms": 230,
               "agreement_complete": False, "safety_valid": True, "committed_agents": ["A"],
               "calls": [{"agent": "B", "mode": "static", "elapsed_ms": 80, "succeeded": False}],
               "isolated_calls": []}
        with tempfile.TemporaryDirectory() as folder, redirect_stdout(io.StringIO()):
            source, destination = Path(folder)/"test.json", Path(folder)/"figures"
            source.write_text(json.dumps({"runs": [row]}), encoding="utf-8")
            self.assertEqual(plot_main([str(source), "--compare-links", "--output", str(destination)]), 0)
            for name in ("test_results.png", "test_latency.png"):
                self.assertTrue((destination/name).read_bytes().startswith(b"\x89PNG\r\n\x1a\n"))
            index = (destination/"index.md").read_text(encoding="utf-8")
            self.assertIn("../test.json", index)
            self.assertIn("test_latency.png", index)

    @unittest.skipUnless(importlib.util.find_spec("matplotlib"), "optional Matplotlib")
    def test_normal_report_contains_compute_time_lengths_and_no_link_comparison(self):
        from examples.plot_benchmark_fleet import main as plot_main
        row = {"case_id": "test", "family": "reference", "map_seed": 0, "agents": 1,
               "seed": 7, "network": "normal", "wall_ms": 230,
               "agreement_complete": True, "safety_valid": True, "committed_agents": ["A"],
               "calls": [{"agent": "A", "mode": "static", "attempt": 1,
                          "elapsed_ms": 80, "succeeded": True}], "isolated_calls": [],
               "total_length_m": 6500, "makespan_s": 350,
               "per_agent": [{"name": "A", "length_m": 6500}]}
        with tempfile.TemporaryDirectory() as folder, redirect_stdout(io.StringIO()):
            source, destination = Path(folder)/"normal.json", Path(folder)/"figures"
            source.write_text(json.dumps({"runs": [row]}), encoding="utf-8")
            self.assertEqual(plot_main([str(source), "--output", str(destination)]), 0)
            for name in ("normal_timing.png", "normal_quality.png", "normal_search.png"):
                self.assertTrue((destination/name).read_bytes().startswith(b"\x89PNG\r\n\x1a\n"))
            summary = (destination/"normal_summary.md").read_text(encoding="utf-8")
            self.assertIn("6.50", summary)
            self.assertIn("0.23", summary)
            self.assertIn("30 экспериментов — не 30 перепланирований", summary)
            self.assertIn("Время поиска одного пути", summary)
            self.assertIn("80.00", summary)
            self.assertIn("1,5 IQR", summary)
            self.assertIn("Новая попытка после неудачи", summary)
            self.assertIn("![normal_timing]", summary)
            self.assertIn("Начать здесь", (destination/"index.md").read_text(encoding="utf-8"))
            self.assertFalse((destination/"normal_results.png").exists())


@unittest.skipUnless(library_path().is_file(), "requires C++ library for kilometre-scale fixtures")
class BenchmarkFleetTests(unittest.TestCase):
    def setUp(self):
        backend = patch.dict(os.environ, DUBINS_BACKEND="cpp")
        backend.start()
        self.addCleanup(backend.stop)

    def test_all_map_families_preserve_scale_physics_and_safe_endpoints(self):
        for family in FAMILIES:
            for seed in ((0,) if family == "reference" else (0, 9)):
                with self.subTest(family=family, seed=seed):
                    world, requests, descriptor = make_case(family, seed, 5)
                    grid = world.terrain
                    self.assertEqual((grid.columns, grid.rows, grid.resolution), (640, 240, 10))
                    self.assertEqual((min(map(min, grid.heights)), max(map(max, grid.heights))), (0, 1000))
                    self.assertEqual(len(requests), 5)
                    self.assertTrue(all(abs(r.goal.z-r.start.z) == 1000 for r in requests))
                    self.assertTrue(all(world.validate_loiter(c) for r in requests for c in (r.start, r.goal)))
                    self.assertEqual((world.min_clearance, world.max_clearance), (40, 260))
                    self.assertEqual(descriptor["map_size_m"], [6400, 2400])
                    self.assertEqual(len(world.obstacles), 3 if family == "reference" else
                                     6 if family in ("gates", "combined") else 0)
                    if family in ("gates", "combined"):
                        self.assertTrue(all(b.z_max == 1600 for b in world.obstacles))
                        for lower, upper in zip(world.obstacles[::2], world.obstacles[1::2]):
                            self.assertTrue(320 <= upper.y_min-lower.y_max <= 460)

    def test_case_ids_and_agent_subsets_are_reproducible(self):
        _, first, a = make_case("combined", 2, 3)
        _, second, b = make_case("combined", 2, 3)
        self.assertEqual(first, second)
        self.assertEqual(a, b)
        self.assertEqual("".join(r.name for r in first), AGENTS[3])
        _, solo, c = make_case("slope", 0, 1)
        self.assertEqual([r.name for r in solo], ["C"])
        self.assertNotEqual(a["id"], c["id"])

    def test_reference_runs_and_isolated_replays_match_original_inputs(self):
        world, requests, descriptor = make_case()
        normal = measure(world, requests, descriptor, "normal", 7, replay=True)
        self.assertTrue(normal["agreement_complete"])
        self.assertTrue(normal["safety_valid"])
        self.assertEqual(len(normal["calls"]), 7)
        self.assertEqual(len(normal["isolated_calls"]), 7)
        self.assertTrue(all(c["matches_original"] for c in normal["isolated_calls"]))
        self.assertEqual({tuple(pair) for pair in normal["initial_conflict_pairs"]},
                         {("B", "D"), ("C", "D"), ("B", "E")})
        self.assertGreater(normal["length_over_euclidean_lower_bound"], 1)
        self.assertTrue(all(c["roots"] == (1 if c["mode"] == "traffic" else 8)
                            for c in normal["isolated_calls"]))
        slow = measure(world, requests, descriptor, "slow", 7, replay=True)
        self.assertFalse(slow["agreement_complete"])
        self.assertTrue(slow["safety_valid"])
        self.assertEqual(slow["committed_agents"], ["A"])
        self.assertIsNone(slow["total_length_m"])
        self.assertIsNone(slow["makespan_s"])
        self.assertTrue(all(c["matches_original"] for c in slow["isolated_calls"]))
        summaries = summarize([normal, slow])
        self.assertEqual(len(summaries), 2)  # Different networks must not be pooled.
        stats = summaries[descriptor["id"]+"/slow"]
        self.assertEqual(stats["safe_complete_fraction"], 0)
        self.assertEqual(stats["committed_agent_fraction"], .2)
        self.assertEqual(stats["safety_failures"], 0)
        self.assertEqual(stats["system_traffic"]["count"], 0)
        self.assertEqual(stats["total_length_m"], {"count": 0})
        self.assertEqual(stats["makespan_s"], {"count": 0})
        full = summaries[descriptor["id"]+"/normal"]
        self.assertEqual(full["total_length_m"]["p50"], normal["total_length_m"])
        self.assertEqual(full["individual_length_m"]["count"], 5)

    def test_failed_searches_remain_in_report_and_are_not_fast_successes(self):
        world, requests, descriptor = make_case("reference", 0, 1)
        with patch("dubins_airplane.decentralized.plan_safe_rrt_star", side_effect=NoPathError("budget")):
            row = measure(world, requests, descriptor, "ideal", 7)
        self.assertFalse(row["agreement_complete"])
        self.assertTrue(row["safety_valid"])
        self.assertIsNone(row["initial_conflict_pairs"])
        self.assertEqual(len(row["calls"]), 3)
        self.assertTrue(all(not c["succeeded"] for c in row["calls"]))
        self.assertEqual(timing_summary(row["calls"])["successful_under_100ms_fraction"], 0)

    def test_cli_writes_streaming_journal_and_manifest_without_search(self):
        with tempfile.TemporaryDirectory() as folder, redirect_stdout(io.StringIO()):
            report = Path(folder)/"run.json"
            status = main(["--family", "reference", "--agents", "1", "--runs", "1",
                           "--warmup", "0", "--iterations", "1", "--replay-local", "--output", str(report)])
            self.assertEqual(status, 0)
            doc = json.loads(report.read_text(encoding="utf-8"))
            self.assertEqual(len(doc["runs"]), 1)
            lines = [json.loads(line) for line in report.with_suffix(".runs.jsonl").read_text(encoding="utf-8").splitlines()]
            self.assertEqual([line["type"] for line in lines], ["header", "instance", "run"])
            self.assertEqual(lines[-1]["run"], doc["runs"][0])
            manifest = Path(folder)/"manifest.json"
            with patch("examples.benchmark_fleet.measure") as planner:
                self.assertEqual(main(["--family", "reference", "--agents", "1", "3", "5",
                                       "--manifest-only", "--output", str(manifest)]), 0)
            planner.assert_not_called()
            self.assertEqual(len(json.loads(manifest.read_text(encoding="utf-8"))["instances"]), 3)
            self.assertFalse(manifest.with_suffix(".runs.jsonl").exists())

    def test_replay_recovers_fixed_start_after_an_initial_failed_search(self):
        world, requests, _ = make_case("reference", 0, 1)
        route = NS(paths=(NS(start="fixed pose"),), length=6000)
        flight = NS(request=requests[0], duration=300, route=route, departure=31)
        result = NS(plan=NS(flights=(flight,)), initial_flights=(),
                    route_history=(NS(agent="C", attempt=2, flight=flight),),
                    events=(NS(agent="C", kind="REPLAN", time=.1),
                            NS(agent="C", kind="REPLAN", time=.2)),
                    planning_calls=(NS(agent="C", attempt=3, mode="traffic", succeeded=True),))
        with patch("examples.benchmark_fleet.plan_safe_rrt_star_traffic", return_value=route) as planner:
            calls = replay_local(world, requests, result, 7, 600)
        self.assertTrue(calls[0]["matches_original"])
        self.assertEqual(planner.call_args.kwargs["start_pose"], "fixed pose")
        self.assertEqual(planner.call_args.kwargs["departure"], 31)

