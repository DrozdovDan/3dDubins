"""Exporter tests need neither Rerun nor an interactive viewer."""

import json
from contextlib import redirect_stderr, redirect_stdout
import importlib.util
import io
from math import atan2, cos, degrees, hypot, radians, sin
import os
from pathlib import Path
import tempfile
import types
import sys
import unittest
from unittest.mock import patch

from dubins_airplane import ElevationMap, Pose, plan_dubins_airplane
from rerun_scripts.dubins import (export_document, main, plan_scenario, sample_route,
                                  terrain_mesh, terrain_risers, contour_segments, camera_pose,
                                  height_color, terrain_color_limits, terrain_map_colors,
                                  endpoint_distances, distance_dimension, summary_text, Demo)
from examples.example_obstacles import SCENARIOS, make_scenario
from rerun_scripts.validate import SCHEMA_DIR


class RerunExportTests(unittest.TestCase):
    def setUp(self):
        self.backend = patch.dict(os.environ, DUBINS_BACKEND="python")
        self.backend.start()
        self.addCleanup(self.backend.stop)

    def test_three_owen_cases_endpoints_time_and_orientation(self):
        for name in ("low", "medium", "high"):
            with self.subTest(name=name):
                demo = plan_scenario(name)
                doc = export_document(demo, speed=25, fps=10)
                rows = doc["agents"][0]["trajectory"]
                self.assertEqual(demo.paths[0].altitude_case, name)
                self.assertEqual(rows[0][0], 0)
                self.assertTrue(all(b[0] > a[0] for a, b in zip(rows, rows[1:])))
                self.assertLessEqual(max(b[0] - a[0] for a, b in zip(rows, rows[1:])), 0.10000001)
                self.assertAlmostEqual(rows[-1][0], demo.paths[0].length / 25)
                for endpoint, pose in ((rows[0], demo.paths[0].start),
                                       (rows[-1], demo.paths[-1].goal)):
                    for actual, expected in zip(endpoint[1:4], (pose.x, pose.y, pose.z)):
                        self.assertAlmostEqual(actual, expected, places=7)
                # Mesh +x nose under Ry(pitch): z=-sin(pitch), so climb must point up.
                self.assertGreater(-sin(rows[1][5]), 0)
                self.assertAlmostEqual(-rows[1][5], demo.paths[0].flight_path_angle)
                json.dumps(doc, allow_nan=False)

    def test_delivery_joins_are_not_duplicated(self):
        demo = plan_scenario("delivery")
        rows = sample_route(demo.paths)
        self.assertEqual(len(demo.paths), 3)
        self.assertTrue(all(b[0] > a[0] for a, b in zip(rows, rows[1:])))
        self.assertAlmostEqual(rows[-1][0], sum(p.length for p in demo.paths) / 20)
        for path in demo.paths[:-1]:
            self.assertTrue(any(abs(row[1] - path.goal.x) < 1e-7
                                and abs(row[2] - path.goal.y) < 1e-7 for row in rows))

    def test_sampling_follows_exact_curve_not_chord_length(self):
        path = plan_dubins_airplane(Pose(0, 0, 0, 0), Pose(500, 200, 80, 1.5), 50, radians(15))
        for row in sample_route((path,), speed=20, fps=5):
            expected = path.pose_at(min(path.horizontal_length, row[0] * 20 * cos(path.flight_path_angle)))
            self.assertAlmostEqual(row[1], expected.x, places=8)
            self.assertAlmostEqual(row[2], expected.y, places=8)

    def test_exact_raster_top_geometry_and_winding(self):
        grid = ElevationMap(-20, 30, 10, ((1, 5), (7, 9)))
        mesh = terrain_mesh(grid)
        self.assertEqual(len(mesh["vertices"]), 16)
        self.assertEqual(len(mesh["faces"]), 8)
        self.assertEqual(mesh["vertices"][:4], [[-20, 30, 1], [-10, 30, 1],
                                               [-10, 40, 1], [-20, 40, 1]])
        self.assertEqual(mesh["vertices"][-1], [-10, 50, 9])
        self.assertEqual(len(mesh["vertex_colors"]), len(mesh["vertices"]))
        for start in range(0, len(mesh["vertices"]), 4):
            self.assertEqual(mesh["vertex_colors"][start:start + 4],
                             [height_color(mesh["vertices"][start][2], terrain_color_limits(grid))] * 4)

    def test_terrain_color_scale_and_map_orientation(self):
        grid = ElevationMap(0, 0, 10, ((0, 50), (100, 200)))
        limits = terrain_color_limits(grid)
        self.assertEqual(limits, (0, 200))
        colors = terrain_map_colors(grid)
        self.assertEqual(colors[0][0], height_color(100, limits))  # Northern row is top.
        self.assertEqual(colors[1][0], height_color(0, limits))
        self.assertEqual(colors[0][1], height_color(200, limits))
        self.assertEqual(height_color(-100, limits), height_color(0, limits))
        self.assertEqual(height_color(1000, limits), height_color(200, limits))
        self.assertEqual(len({tuple(height_color(z, limits)) for z in (0, 50, 100, 150, 200)}), 5)
        flat = ElevationMap(0, 0, 10, ((0, 0), (0, 0)))
        self.assertEqual(terrain_color_limits(flat), (0, 25))
        self.assertTrue(all(color == height_color(0, (0, 25))
                            for row in terrain_map_colors(flat) for color in row))

    def test_distances_are_for_actual_endpoints_not_path_length(self):
        path = plan_dubins_airplane(Pose(0, 0, 0, 0), Pose(300, 400, 120, 0), 40, radians(15))
        demo = Demo("test", "Test", (path,), [("Start", path.start), ("Goal", path.goal)], 0)
        distances = endpoint_distances(demo)
        self.assertEqual(distances["endpoint_xy_m"], 500)
        self.assertAlmostEqual(distances["endpoint_3d_m"], hypot(500, 120))
        self.assertGreater(path.length, distances["endpoint_3d_m"])
        ruler = distance_dimension(demo)
        p, q = ruler["line"]
        self.assertAlmostEqual(hypot(q[0] - p[0], q[1] - p[1]), 500)
        self.assertIn("500.0", ruler["label"])
        doc = export_document(demo)
        self.assertEqual(doc["dubins"]["endpoint_xy_m"], 500)
        self.assertEqual(doc["dubins"]["length_m"], path.length)
        self.assertIsNone(distance_dimension(plan_scenario("delivery")))

    def test_all_example_maps_have_ten_meter_cells_and_preserve_extent(self):
        # Native offsets only: this test checks fixtures, not backend parity.
        from dubins_airplane.native import library_path
        backend = "cpp" if library_path().is_file() else "python"
        with patch.dict(os.environ, DUBINS_BACKEND=backend):
            for name in SCENARIOS:
                scene = make_scenario(name)
                grid = scene.world.terrain
                self.assertEqual(grid.resolution, 10)
                self.assertEqual(grid.x_max - grid.x_min, 1180)
                self.assertEqual(grid.y_max - grid.y_min, 1000 if name == "ridge" else 800)
                self.assertEqual(grid.columns, 118)
                self.assertEqual(grid.rows, 100 if name == "ridge" else 80)

    def test_overview_text_is_short_and_contains_key_metrics(self):
        from dubins_airplane.native import library_path
        backend = "cpp" if library_path().is_file() else "python"
        with patch.dict(os.environ, DUBINS_BACKEND=backend):
            for name in ("low", "delivery", "slalom", "mountain_pass"):
                demo = plan_scenario(name)
                doc = export_document(demo)
                text = summary_text(demo, doc)
                self.assertLessEqual(len(text.splitlines()), 8)
                self.assertTrue(all(len(line) <= 85 for line in text.splitlines()))
                self.assertIn(f"{doc['dubins']['length_m']:.1f} м", text)
                self.assertIn(f"{doc['dubins']['endpoint_xy_m']:.1f} м в плане", text)
                if demo.world is not None:
                    self.assertIn("клетка 10 × 10 м", text)
                    self.assertIn("800.0 м", text)

    def test_invalid_animation_settings(self):
        demo = plan_scenario("low")
        for value in (0, -1, float("inf"), float("nan")):
            with self.assertRaises(ValueError):
                sample_route(demo.paths, speed=value)
            with self.assertRaises(ValueError):
                sample_route(demo.paths, fps=value)
            with self.assertRaises(ValueError):
                export_document(demo, plane_radius=value)

    def test_camera_is_above_scene_at_requested_elevation(self):
        doc = export_document(plan_scenario("high"))
        for elevation in (45, 60, 75):
            eye, target = camera_pose(doc, elevation)
            delta = [a - b for a, b in zip(eye, target)]
            self.assertAlmostEqual(degrees(atan2(delta[2], hypot(*delta[:2]))), elevation)
            self.assertLess(eye[1], target[1])  # From south; east remains right.
        for invalid in (0, 90, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                camera_pose(doc, invalid)

    def test_terrain_steps_close_gaps_without_changing_cell_tops(self):
        grid = ElevationMap(-20, 30, 10, ((1, 5), (7, 9)))
        walls = terrain_risers(grid)
        self.assertEqual(len(walls["vertices"]), 16)  # Four shared boundaries.
        self.assertEqual(len(walls["faces"]), 8)
        self.assertEqual(walls["vertices"][:4],
                         [[-10, 30, 1], [-10, 40, 1], [-10, 40, 5], [-10, 30, 5]])
        flat = terrain_risers(ElevationMap(0, 0, 10, ((2, 2), (2, 2))))
        self.assertEqual(flat["vertices"], [])

    def test_schema_directory_matches_imported_scripts(self):
        self.assertTrue((Path(SCHEMA_DIR) / "combined.schema.json").is_file())

    def test_contours_follow_cell_edges_in_north_up_coordinates(self):
        grid = ElevationMap(-20, 30, 10, ((1, 5), (7, 9)))
        self.assertEqual(contour_segments(grid, 6), [
            [[-20, -40], [-10, -40]], [[-10, -40], [0, -40]],
        ])

    def test_cli_json_only_and_cpp_no_silent_fallback(self):
        with tempfile.TemporaryDirectory() as directory:
            with patch("rerun_scripts.dubins.render_demo") as renderer, redirect_stdout(io.StringIO()):
                main(["--scenario", "low", "--backend", "python", "--json-only",
                      "--output", directory])
                renderer.assert_not_called()
            doc = json.loads((Path(directory) / "low.json").read_text(encoding="utf-8"))
            self.assertEqual(doc["dubins"]["backend"], "python")
            with patch("rerun_scripts.dubins.backend_name", side_effect=RuntimeError("C++ missing")):
                with self.assertRaises(SystemExit) as exit_result, redirect_stderr(io.StringIO()):
                    main(["--json-only", "--output", directory])
                self.assertEqual(exit_result.exception.code, 2)

    def test_all_requires_noninteractive_output(self):
        with self.assertRaises(SystemExit) as result, redirect_stderr(io.StringIO()):
            main(["--scenario", "all"])
        self.assertEqual(result.exception.code, 2)

    def test_trail_chunks_do_not_include_future_points_or_skip_joins(self):
        # No SDK dependency: capture only the renderer's trail payloads.
        import numpy as np
        fake = types.ModuleType("rerun")
        fake.LineStrips3D = lambda strips, **kwargs: strips[0]
        fake.log = lambda path, value: captured.append((path, value))
        captured = []
        from importlib import util
        spec = util.spec_from_file_location("test_rerunlib",
                                           Path(__file__).resolve().parents[1] / "rerun_scripts/rerunlib.py")
        module = util.module_from_spec(spec)
        with patch.dict(sys.modules, rerun=fake):
            spec.loader.exec_module(module)
        points = np.arange(150 * 3).reshape(150, 3)
        visible = {}
        for i in range(len(points)):
            module._log_trail("plane", points, i, [1, 2, 3], 1.2)
            path, payload = captured[-1]
            self.assertLessEqual(len(payload), 65)
            visible[path] = payload.tolist()
            # Last-known values of earlier chunks form exactly the flown prefix.
            flattened = {tuple(p) for chunk in visible.values() for p in chunk}
            self.assertEqual(flattened, {tuple(p) for p in points[:i + 1]})

    @unittest.skipUnless(importlib.util.find_spec("rerun"), "optional rerun-sdk is not installed")
    def test_real_sdk_recording_without_opening_viewer(self):
        from rerun_scripts.dubins import render_demo
        import rerun as rr
        import warnings
        with tempfile.TemporaryDirectory() as directory:
            for name in ("low", "slalom", "mountain_pass"):
                with self.subTest(name=name):
                    from dubins_airplane.native import library_path
                    with patch.dict(os.environ, DUBINS_BACKEND="cpp" if library_path().is_file() else "python"):
                        demo = plan_scenario(name)
                    doc = export_document(demo, speed=200, fps=3)
                    target = Path(directory) / f"{name}.rrd"
                    try:
                        with warnings.catch_warnings():
                            warnings.simplefilter("error")
                            render_demo(demo, doc, save=target)
                        self.assertGreater(target.stat().st_size, 1000)
                    finally:
                        rr.disconnect()


if __name__ == "__main__":
    unittest.main()
