"""Saved-run comparisons align missions and normalize each fleet once."""
from copy import deepcopy
import unittest

from examples.compare_benchmarks import compare_reports, settings_label


def report():
    return dict(configuration=dict(iterations=600, retry_iterations=3000,
                                   network=dict(minimum_delay=0, maximum_delay=.15,
                                                loss=.1, duplicates=.1)),
                runs=[dict(id=str(i), agents=2, topology='open', scenario='transfer',
                           direction='ascent', requests=[dict(start=[i, 0, 0])],
                           wall_ms=t, total_length_m=length,
                           complete=True, safety_valid=True)
                      for i, (t, length) in enumerate([(100, 8000), (300, 10000)])])


class CompareReportsTests(unittest.TestCase):
    def test_matches_ids_not_order_and_normalizes_each_mission(self):
        before = report()
        original = deepcopy(before)
        after = deepcopy(before)
        after['runs'].reverse()
        for run in after['runs']:
            run['wall_ms'] /= 4
        after['configuration']['relative_gap'] = .03
        after['configuration']['network']['maximum_delay'] = 0
        pairs, rows = compare_reports(before, after)
        self.assertEqual(len(pairs), 2)
        self.assertEqual(rows, [dict(agents=2, missions=2,
                                    time_median_ms=[100, 25], median_time_ratio=4,
                                    length_median_km=[4.5, 4.5], safe_complete=[2, 2])])
        self.assertEqual(before, original)
        self.assertIn('3%', settings_label(after))
        self.assertIn('150', settings_label(before))

    def test_incomplete_mission_keeps_time_but_not_partial_length(self):
        before = report()
        after = deepcopy(before)
        after['runs'][1].update(complete=False, total_length_m=1000, wall_ms=1000)
        _, rows = compare_reports(before, after)
        self.assertEqual(rows[0]['time_median_ms'], [100, 275])
        self.assertEqual(rows[0]['length_median_km'], [4.5, 4])
        self.assertEqual(rows[0]['safe_complete'], [2, 1])

    def test_rejects_changed_workload_or_mismatched_endpoints(self):
        for change in ('ids', 'endpoints', 'budget', 'duplicate'):
            with self.subTest(change=change):
                before = report()
                after = deepcopy(before)
                if change == 'ids':
                    after['runs'].pop()
                elif change == 'endpoints':
                    after['runs'][0]['requests'][0]['start'][0] = 999
                elif change == 'budget':
                    after['configuration']['iterations'] = 300
                else:
                    after['runs'].append(deepcopy(after['runs'][0]))
                with self.assertRaises(ValueError):
                    compare_reports(before, after)


if __name__ == '__main__':
    unittest.main()
