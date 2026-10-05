"""近距離優先の境界・点数・未観測を検査する。"""
import unittest
from compare_near_observation_retention import reject_update


class RetentionTest(unittest.TestCase):
    def test_far_with_equal_points(self):
        self.assertTrue(reject_update(1, 10, 2, 10))

    def test_far_with_more_old_points(self):
        self.assertTrue(reject_update(1, 11, 2, 10))

    def test_far_but_new_has_more_points(self):
        self.assertFalse(reject_update(1, 9, 2, 10))

    def test_nearer_and_same_distance(self):
        self.assertFalse(reject_update(2, 10, 1, 5))
        self.assertFalse(reject_update(2, 10, 2, 5))

    def test_margin(self):
        self.assertFalse(reject_update(1, 10, 1.04, 5))
        self.assertTrue(reject_update(1, 10, 1.06, 5))

    def test_unknown_has_no_priority(self):
        self.assertFalse(reject_update(float('nan'), 10, 2, 5))


if __name__ == '__main__':
    unittest.main()
