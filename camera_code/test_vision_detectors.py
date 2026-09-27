import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).parent))

from vision_detectors import _cluster_rings, _unique_near


class VisionDetectorTest(unittest.TestCase):
    def test_unique_target_rejects_ambiguous_and_outside_candidates(self):
        self.assertEqual((101, 99, 80, 20, 20),
                         _unique_near([(101, 99, 80, 20, 20)], (100, 100), 5))
        self.assertIsNone(_unique_near([(101, 99, 80, 20, 20),
                                        (99, 101, 75, 18, 18)],
                                       (100, 100), 5))
        self.assertIsNone(_unique_near([(120, 120, 80, 20, 20)],
                                       (100, 100), 5))

    def test_concentric_contours_form_one_ring_target(self):
        targets = _cluster_rings([(100, 80, 10), (101, 80, 15),
                                  (99, 81, 20), (200, 200, 12)])
        self.assertEqual(1, len(targets))
        self.assertEqual((100, 80), targets[0][:2])
        self.assertEqual((40, 40), targets[0][3:])


if __name__ == "__main__":
    unittest.main()
