"""The harder scenarios have the old map scale, not hand-picked route poses."""

from math import hypot
import os
import unittest
from unittest.mock import patch

from dubins_airplane.native import library_path
from rerun_scripts.dubins import plan_scenario
from rerun_scripts.record_gif import frame_times, wait_for_recording, gif_palette, GIF_ACCENTS


@unittest.skipUnless(library_path().is_file(), "requires the compiled C++ backend")
class ComplexScenarioTests(unittest.TestCase):
    def test_safe_detours_in_same_scale_for_three_seeds(self):
        with patch.dict(os.environ, DUBINS_BACKEND="cpp"):
            for name in ("slalom", "mountain_pass"):
                for seed in (7, 8, 9):
                    with self.subTest(name=name, seed=seed):
                        demo = plan_scenario(name, iterations=600, seed=seed)
                        grid = demo.world.terrain
                        self.assertEqual(grid.x_max - grid.x_min, 1180)
                        self.assertEqual(grid.y_max - grid.y_min, 800)
                        self.assertEqual(grid.resolution, 10)
                        self.assertEqual((grid.columns, grid.rows), (118, 80))
                        self.assertEqual(demo.statistics["iterations"], 600)
                        self.assertFalse(demo.reference_free)
                        self.assertGreater(len(demo.paths), 1)
                        for path in demo.paths:
                            self.assertTrue(demo.world._path_is_free_python(path))
                            self.assertGreaterEqual(path.radius, 40)
                        for a, b in zip(demo.paths, demo.paths[1:]):
                            self.assertEqual(a.goal, b.start)
                        self.assertAlmostEqual(hypot(demo.loiters[1].x - demo.loiters[0].x,
                                                     demo.loiters[1].y - demo.loiters[0].y), 800)
                        if name == "mountain_pass":
                            self.assertGreater(demo.loiters[-1].z - demo.loiters[0].z, 90)

    def test_seed_seven_uses_the_gap_between_peaks(self):
        with patch.dict(os.environ, DUBINS_BACKEND="cpp"):
            demo = plan_scenario("mountain_pass")
        points = [p for path in demo.paths for p in path.sample(2) if 420 < p.x < 440]
        self.assertTrue(points)
        self.assertTrue(all(abs(p.y) < 120 for p in points))


class GifTimingTests(unittest.TestCase):
    def test_small_semantic_colors_survive_a_fixed_palette(self):
        try:
            from PIL import Image
        except ImportError:
            self.skipTest("optional Pillow is not installed")
        image = Image.new("RGB", (200, 200), (70, 120, 80))
        for i, color in enumerate(GIF_ACCENTS):
            image.putpixel((i, 0), color)
        result = image.quantize(palette=gif_palette(image), dither=Image.Dither.NONE).convert("RGB")
        for i, color in enumerate(GIF_ACCENTS):
            self.assertEqual(result.getpixel((i, 0)), color)

    def test_even_times_include_exact_endpoints(self):
        times = frame_times(55.315, duration=8, fps=10)
        self.assertEqual(len(times), 80)
        self.assertEqual(times[0], 0)
        self.assertEqual(times[-1], 55.315)
        self.assertTrue(all(a < b for a, b in zip(times, times[1:])))

    def test_invalid_duration_is_rejected(self):
        for value in (0, -1, float("inf"), float("nan")):
            with self.assertRaises(ValueError):
                frame_times(value)
            with self.assertRaises(ValueError):
                frame_times(30, duration=value)
        with self.assertRaises(ValueError):
            frame_times(30, fps=0)

    def test_recording_loader_requires_expected_timeline_range(self):
        from types import SimpleNamespace as NS
        loading = NS(active_recording="recording", views=[None] * 4,
                     recordings=[NS(store_id="recording", timelines=[NS(name="sim_time", end=3e9)])])
        loaded = NS(active_recording="recording", views=[None] * 4,
                    recordings=[NS(store_id="recording", timelines=[NS(name="sim_time", end=55e9)])])
        client = unittest.mock.Mock()
        client.viewer_state.side_effect = [loading, loaded]
        with patch("rerun_scripts.record_gif.time.sleep"):
            self.assertEqual(wait_for_recording(client, expected_end=55), 55)
        self.assertEqual(client.viewer_state.call_count, 2)


if __name__ == "__main__":
    unittest.main()
