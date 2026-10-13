import unittest
from unittest.mock import Mock

from obstacle_detection import Confirmation, largest_black_blob, STOP_LINE


def blob(pixels, x=320, y=240):
    return Mock(pixels=lambda: pixels, cx=lambda: x, cy=lambda: y)


class ObstacleDetectionTest(unittest.TestCase):
    def test_largest_and_near_large_object_is_not_discarded(self):
        near = blob(200000)
        self.assertIs(near, largest_black_blob([blob(4999), blob(6000), near], 5000))
        self.assertIsNone(largest_black_blob([blob(4999)], 5000))
        self.assertEqual(STOP_LINE, b"OB:STOP\n")

    def test_requires_consecutive_frames_and_resets_on_missing_or_jump(self):
        stable = Confirmation()
        self.assertFalse(stable.update(blob(6000)))
        self.assertFalse(stable.update(blob(6000, 330)))
        self.assertTrue(stable.update(blob(6000, 340)))
        self.assertFalse(stable.update(None))
        self.assertFalse(stable.update(blob(6000)))
        self.assertFalse(stable.update(blob(6000, 400)))
        self.assertFalse(stable.update(blob(6000, 410)))
        self.assertTrue(stable.update(blob(6000, 420)))


if __name__ == "__main__":
    unittest.main()
