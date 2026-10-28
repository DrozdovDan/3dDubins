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

    def test_network_focus_slows_playback_without_changing_simulation_clock(self):
        times = frame_times(75, duration=10, fps=20, focus_until=2)
        self.assertEqual(len(times), 200)
        self.assertEqual((times[0], times[-1]), (0, 75))
        self.assertEqual(times[80], 2)
        self.assertTrue(all(a < b for a, b in zip(times, times[1:])))
        self.assertLess(times[1] - times[0], times[-1] - times[-2])
        for focus in (0, -1, 76, float("nan")):
            with self.assertRaises(ValueError):
                frame_times(75, focus_until=focus)

    def test_invalid_duration_is_rejected(self):
        for value in (0, -1, float("inf"), float("nan")):
            with self.assertRaises(ValueError):
                frame_times(value)
            with self.assertRaises(ValueError):
                frame_times(30, duration=value)
        with self.assertRaises(ValueError):
            frame_times(30, fps=0)

    def test_middle_focus_keeps_order_and_collision_window_visible(self):
        times = frame_times(360, duration=10, fps=20, focus_window=(155, 190), focus_fraction=.6)
        self.assertEqual(len(times), 200)
        self.assertEqual((times[0], times[-1]), (0, 360))
        self.assertTrue(all(a < b for a, b in zip(times, times[1:])))
        self.assertGreater(sum(155 <= t <= 190 for t in times), 115)
        for window in ((-1, 5), (10, 10), (30, 20), (10, 361), (float("nan"), 20)):
            with self.assertRaises(ValueError):
                frame_times(360, focus_window=window)
        with self.assertRaises(ValueError):
            frame_times(360, focus_window=(155, 190), focus_until=1)
        for window in ((0, 100), (200, 360), (0, 360)):
            focused = frame_times(360, focus_window=window)
            self.assertEqual((focused[0], focused[-1]), (0, 360))
            self.assertTrue(all(a < b for a, b in zip(focused, focused[1:])))

    def test_multiple_encounters_get_distinct_slow_playback_windows(self):
        windows = [(70, 90), (180, 205), (260, 275)]
        times = frame_times(360, duration=15, fps=20, focus_windows=windows, focus_fraction=.6)
        self.assertEqual(len(times), 300)
        self.assertEqual((times[0], times[-1]), (0, 360))
        self.assertTrue(all(a < b for a, b in zip(times, times[1:])))
        self.assertGreater(sum(any(a <= t <= b for a, b in windows) for t in times), 175)
        self.assertTrue(all(bound in times for window in windows for bound in window))
        merged = frame_times(360, focus_windows=[(10, 20), (15, 25)])
        self.assertTrue(all(a < b for a, b in zip(merged, merged[1:])))
        for invalid in ([], [(5, 4)], [(-1, 10)], [(10, 361)], [(float("nan"), 20)]):
            with self.assertRaises(ValueError):
                frame_times(360, focus_windows=invalid)
        with self.assertRaises(ValueError):
            frame_times(360, focus_windows=windows, focus_window=(100, 110))

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

    def test_fleet_loader_waits_for_last_scene_marker_not_just_trajectory_end(self):
        from types import SimpleNamespace as NS
        loading = NS(active_recording="recording", views=[None] * 4,
            recordings=[NS(store_id="recording", timelines=[NS(name="sim_time", end=360e9)])])
        complete = NS(active_recording="recording", views=[None] * 4,
            recordings=[NS(store_id="recording", timelines=[NS(name="sim_time", end=360e9),
                                                          NS(name="render_complete", end=1)])])
        client = unittest.mock.Mock()
        client.viewer_state.side_effect = [loading, complete]
        with patch("rerun_scripts.record_gif.time.sleep"):
            self.assertEqual(wait_for_recording(client, expected_end=360,
                                               completion_timeline="render_complete"), 360)
        self.assertEqual(client.viewer_state.call_count, 2)


if __name__ == "__main__":
    unittest.main()
