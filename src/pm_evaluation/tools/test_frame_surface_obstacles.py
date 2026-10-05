"""分位点の補間、元画素index、空cellの扱いを検査する。"""
import unittest
import numpy as np
from diagnose_frame_surface_obstacles import grouped_statistics


class GroupedStatisticsTest(unittest.TestCase):
    def test_quantile_and_source_index(self):
        slots = np.array([1, 0, 1, 0, 1]); z = np.array([8., 4., 1., 2., 9.])
        result = grouped_statistics(slots, z, 3)
        for cell in (0, 1):
            values = z[slots == cell]
            self.assertAlmostEqual(result['q10'][cell], np.quantile(values, .1))
            self.assertAlmostEqual(result['q90'][cell], np.quantile(values, .9))
            self.assertEqual(z[result['min_index'][cell]], values.min())
            self.assertEqual(z[result['max_index'][cell]], values.max())
        self.assertTrue(np.isnan(result['q10'][2]))

    def test_single_and_empty(self):
        result = grouped_statistics(np.array([0]), np.array([.5]), 2)
        self.assertEqual(result['q10'][0], .5)
        self.assertEqual(result['q90'][0], .5)
        empty = grouped_statistics(np.array([], dtype=int), np.array([]), 2)
        self.assertTrue(np.isnan(empty['q90']).all())

    def test_sloping_plane_detrending(self):
        x = np.array([.01, .09]); z = .7*x+.2
        residual = z-(.7*x+.2)
        self.assertGreater(np.ptp(z), .05)
        self.assertLess(np.ptp(residual), 1e-9)


if __name__ == '__main__':
    unittest.main()
