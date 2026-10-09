import pathlib
import sys
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(pathlib.Path(__file__).parent))

from vision_detectors import _cluster_rings, _unique_near, find_material
import vision_settings as settings


class VisionDetectorTest(unittest.TestCase):
    def test_unique_target_rejects_ambiguous_and_outside_candidates(self):
        self.assertEqual((101, 99, 80, 20, 20),
                         _unique_near([(101, 99, 80, 20, 20)], (100, 100), 5))
        self.assertIsNone(_unique_near([(101, 99, 80, 20, 20),
                                        (99, 101, 75, 18, 18)],
                                       (100, 100), 5))
        self.assertIsNone(_unique_near([(120, 120, 80, 20, 20)],
                                       (100, 100), 5))

    def test_material_alignment_accepts_all_colors_but_rejects_multiple_targets(self):
        blob = Mock()
        for name, value in [("x", 100), ("y", 80), ("w", 50), ("h", 50),
                            ("cx", 125), ("cy", 105), ("pixels", 2000)]:
            getattr(blob, name).return_value = value
        frame = Mock()
        frame.find_blobs.side_effect = lambda thresholds, **kwargs: (
            [blob] if thresholds == [settings.COLOR_THRESHOLDS[4]] else [])
        request = {"selector": 0, "values": [0, 0, 320, 240, 160, 120, 0, 0]}
        self.assertEqual((125, 105, 80, 50, 50, 4), find_material(frame, request))
        self.assertEqual(6, frame.find_blobs.call_count)
        request["values"][6] = 63
        for color in range(1, 7):
            frame.find_blobs.side_effect = lambda thresholds, **kwargs: (
                [blob] if thresholds == [settings.COLOR_THRESHOLDS[color]] else [])
            self.assertEqual((125, 105, 80, 50, 50, color), find_material(frame, request))
        frame.find_blobs.side_effect = lambda thresholds, **kwargs: (
            [blob, blob] if thresholds == [settings.COLOR_THRESHOLDS[4]] else [])
        self.assertIsNone(find_material(frame, request))
        request["selector"] = 4
        frame.find_blobs.side_effect = None
        frame.find_blobs.return_value = [blob]
        self.assertIsNotNone(find_material(frame, request))
        self.assertEqual([settings.COLOR_THRESHOLDS[4]],
                         frame.find_blobs.call_args.args[0])

    def test_material_mask_excludes_collected_and_other_colors(self):
        frame = Mock()
        frame.find_blobs.return_value = []
        request = {"selector": 0, "values": [0, 0, 320, 240, 160, 120, 34, 0]}
        self.assertIsNone(find_material(frame, request))
        self.assertEqual([[settings.COLOR_THRESHOLDS[2]], [settings.COLOR_THRESHOLDS[6]]],
                         [call.args[0] for call in frame.find_blobs.call_args_list])

    def test_overlapping_color_detections_are_ambiguous(self):
        blob = Mock()
        for name, value in [("x", 100), ("y", 80), ("w", 50), ("h", 50),
                            ("cx", 125), ("cy", 105), ("pixels", 2000)]:
            getattr(blob, name).return_value = value
        frame = Mock()
        frame.find_blobs.return_value = [blob]
        request = {"selector": 0, "values": [0, 0, 320, 240, 160, 120, 42, 0]}
        self.assertIsNone(find_material(frame, request))

    def test_concentric_contours_form_one_ring_target(self):
        targets = _cluster_rings([(100, 80, 10), (101, 80, 15),
                                  (99, 81, 20), (200, 200, 12)])
        self.assertEqual(1, len(targets))
        self.assertEqual((100, 80), targets[0][:2])
        self.assertEqual((40, 40), targets[0][3:])


if __name__ == "__main__":
    unittest.main()
