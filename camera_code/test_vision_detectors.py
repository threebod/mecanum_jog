import pathlib
import sys
import types
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(pathlib.Path(__file__).parent))

from vision_detectors import _cluster_rings, _unique_near, find_material, find_ring
import vision_settings as settings
from vision_protocol import FLAG_RING_NO_DIGIT


class VisionDetectorTest(unittest.TestCase):
    def test_material_aspect_ratio_accepts_square_and_inclusive_boundary(self):
        for scale in (1, 2):
            for width, height, accepted in [(50, 50, True), (60, 50, True),
                                             (56, 40, True), (40, 56, True),
                                             (63, 45, True), (45, 63, True),
                                             (57, 40, False), (40, 57, False),
                                             (100, 40, False)]:
                with self.subTest(scale=scale, width=width, height=height):
                    blob = Mock()
                    for name, value in [("area", width * height * scale ** 2),
                                        ("pixels", width * height * scale ** 2),
                                        ("w", width * scale), ("h", height * scale),
                                        ("cx", 100 * scale), ("cy", 80 * scale)]:
                        getattr(blob, name).return_value = value
                    frame = Mock()
                    frame.find_blobs.return_value = [blob]
                    request = {"selector": 4, "target": 2,
                               "values": [0, 0, 320, 240, 160, 120, 0, 0]}
                    result = find_material(frame, request, scale=scale)
                    self.assertEqual(accepted, result is not None)
                    if accepted:
                        self.assertEqual((100, 80), result[:2])

    def test_material_aspect_filter_preserves_largest_blob_selection(self):
        square, rectangle = Mock(), Mock()
        for blob, width, height in [(square, 50, 50), (rectangle, 100, 40)]:
            blob.area.return_value = width * height
            blob.w.return_value = width
            blob.h.return_value = height
        frame = Mock()
        frame.find_blobs.return_value = [square, rectangle]
        request = {"selector": 4, "target": 0,
                   "values": [0, 0, 320, 240, 160, 120, 0, 0]}
        self.assertIsNone(find_material(frame, request))

    def test_pickup_limits_detection_to_top_three_quarters(self):
        frame = Mock()
        frame.find_blobs.return_value = []
        request = {"selector": 5, "target": 0,
                   "values": [0, 0, 320, 240, 160, 120, 0, 0]}
        find_material(frame, request)
        self.assertEqual((0, 0, 320, 180), frame.find_blobs.call_args.kwargs["roi"])
        find_material(frame, request, scale=2)
        self.assertEqual((0, 0, 640, 360), frame.find_blobs.call_args.kwargs["roi"])
        request["values"][:4] = [20, 100, 200, 100]
        find_material(frame, request)
        self.assertEqual((20, 100, 200, 80), frame.find_blobs.call_args.kwargs["roi"])
        frame.reset_mock()
        request["values"][:4] = [0, 180, 320, 60]
        self.assertIsNone(find_material(frame, request))
        frame.find_blobs.assert_not_called()

    def test_storage_material_detection_keeps_requested_roi(self):
        frame = Mock()
        frame.find_blobs.return_value = []
        request = {"selector": 5, "target": 2,
                   "values": [0, 0, 320, 240, 160, 120, 0, 0]}
        find_material(frame, request)
        self.assertEqual((0, 0, 320, 240), frame.find_blobs.call_args.kwargs["roi"])

    def test_unique_target_rejects_ambiguous_and_outside_candidates(self):
        self.assertEqual((101, 99, 80, 20, 20),
                         _unique_near([(101, 99, 80, 20, 20)], (100, 100), 5))
        self.assertIsNone(_unique_near([(101, 99, 80, 20, 20),
                                        (99, 101, 75, 18, 18)],
                                       (100, 100), 5))
        self.assertIsNone(_unique_near([(120, 120, 80, 20, 20)],
                                       (100, 100), 5))

    def test_material_alignment_accepts_all_colors_and_one_blob_per_color(self):
        blob = Mock()
        for name, value in [("x", 100), ("y", 80), ("w", 50), ("h", 50),
                            ("cx", 125), ("cy", 105), ("pixels", 2000), ("area", 2500)]:
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
        self.assertEqual((125, 105, 80, 50, 50, 4), find_material(frame, request))
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

    def test_equal_area_colors_use_color_number_order(self):
        blob = Mock()
        for name, value in [("x", 100), ("y", 80), ("w", 50), ("h", 50),
                            ("cx", 125), ("cy", 105), ("pixels", 2000), ("area", 2500)]:
            getattr(blob, name).return_value = value
        frame = Mock()
        frame.find_blobs.return_value = [blob]
        request = {"selector": 0, "values": [0, 0, 320, 240, 160, 120, 42, 0]}
        self.assertEqual(2, find_material(frame, request)[5])

    def test_material_uses_current_thresholds(self):
        self.assertEqual({
            1: [30, 80, 55, 105, 25, 80],
            2: [50, 105, -30, 25, 35, 100],
            3: [17, 37, 8, 32, -59, -35],
            4: [50, 100, -100, -50, 20, 80],
            5: [8, 25, -14, 10, -8, 16],
            6: [50, 84, -27, -3, -36, -12],
        }, settings.COLOR_THRESHOLDS)

    def test_any_color_selects_largest_area_instead_of_quality(self):
        small, large = Mock(), Mock()
        for blob, area, pixels, cx in [(small, 2000, 2000, 90), (large, 3000, 0, 190)]:
            for name, value in [("area", area), ("pixels", pixels), ("cx", cx),
                                ("cy", 120), ("w", 50), ("h", 60)]:
                getattr(blob, name).return_value = value
        frame = Mock()
        frame.find_blobs.side_effect = lambda thresholds, **kwargs: (
            [small] if thresholds == [settings.COLOR_THRESHOLDS[1]] else
            [large] if thresholds == [settings.COLOR_THRESHOLDS[6]] else [])
        request = {"selector": 0, "values": [0, 0, 320, 240, 160, 120, 63, 0]}
        self.assertEqual((190, 120, 0, 50, 60, 6), find_material(frame, request))
        self.assertEqual({"roi": (0, 0, 320, 180), "area_threshold": 50,
                          "pixels_threshold": 50}, frame.find_blobs.call_args.kwargs)

    def test_material_selects_largest_blob_and_uses_its_centroid(self):
        small, large = Mock(), Mock()
        for blob, values in ((small, (70, 60, 40, 40, 90, 80, 1200)),
                             (large, (110, 80, 60, 50, 133, 99, 2400))):
            for name, value in zip(("x", "y", "w", "h", "cx", "cy", "pixels"), values):
                getattr(blob, name).return_value = value
            blob.area.return_value = values[2] * values[3]
        frame = Mock()
        frame.find_blobs.return_value = [small, large]
        request = {"selector": 3, "values": [0, 0, 320, 240, 160, 120, 0, 0]}
        self.assertEqual((133, 99, 80, 60, 50, 3), find_material(frame, request))
        large.area.return_value = 31000
        self.assertIsNone(find_material(frame, request))

    def test_material_accepts_roi_edge_and_exact_area_limits(self):
        blob = Mock()
        for name, value in [("x", 0), ("y", 0), ("cx", 10), ("cy", 10),
                            ("w", 50), ("h", 50), ("pixels", 50)]:
            getattr(blob, name).return_value = value
        frame = Mock()
        frame.find_blobs.return_value = [blob]
        request = {"selector": 3, "values": [0, 0, 320, 240, 160, 120, 0, 0]}
        for area, accepted in [(1499, False), (1500, True), (30000, True), (30001, False)]:
            blob.area.return_value = area
            with self.subTest(area=area):
                self.assertEqual(accepted, find_material(frame, request) is not None)

    def test_concentric_contours_form_one_ring_target(self):
        targets = _cluster_rings([(100, 80, 10), (101, 80, 15),
                                  (99, 81, 20), (200, 200, 12)])
        self.assertEqual(1, len(targets))
        self.assertEqual((100, 80), targets[0][:2])
        self.assertEqual((40, 40), targets[0][3:])

    def test_ring_resolution_preserves_center_and_diameter_with_offset_roi(self):
        import cv2
        import numpy as np

        maix = types.ModuleType("maix")
        maix.image = Mock()
        request = {"target": 2, "values": [50, 30, 140, 130, 120, 90, 0, 0]}
        results = []
        for scale in (1, 2):
            gray = np.full((240 * scale, 320 * scale), 255, dtype=np.uint8)
            for radius in (24, 40):
                cv2.circle(gray, (120 * scale, 90 * scale), radius * scale,
                           0, 3 * scale)
            maix.image.image2cv.return_value = gray
            with patch.dict(sys.modules, {"maix": maix}), \
                    patch("ring_digits.read_ring_digit", return_value=(2, 1.0)):
                result = find_ring(Mock(), request)
            self.assertIsNotNone(result)
            self.assertLessEqual(abs(result[0] - 120), 1)
            self.assertLessEqual(abs(result[1] - 90), 1)
            results.append(result)
        self.assertLessEqual(abs(results[0][3] - results[1][3]), 2)

    def test_unnumbered_ring_uses_material_center_only_for_multiple_rings(self):
        import cv2
        import numpy as np

        maix = types.ModuleType("maix")
        maix.image = Mock()
        request = {"target": 2, "selector": 5, "flags": FLAG_RING_NO_DIGIT,
                   "values": [0, 0, 320, 240, 160, 120, 0, 0]}
        for centers, material_present in [([80], False), ([80, 220], True),
                                          ([80, 220], False)]:
            with self.subTest(centers=centers, material_present=material_present):
                gray = np.full((480, 640), 255, dtype=np.uint8)
                for center in centers:
                    for radius in (24, 40):
                        cv2.circle(gray, (center * 2, 240), radius * 2, 0, 6)
                maix.image.image2cv.return_value = gray
                frame = Mock()
                blob = Mock()
                for name, value in [("area", 6400), ("pixels", 100), ("cx", 170),
                                    ("cy", 240), ("w", 80), ("h", 80)]:
                    getattr(blob, name).return_value = value
                frame.find_blobs.return_value = [blob] if material_present else []
                with patch.dict(sys.modules, {"maix": maix}), \
                        patch("ring_digits.read_ring_digit", side_effect=AssertionError(
                            "stack alignment must not read digits")) as digit:
                    result = find_ring(frame, request)
                digit.assert_not_called()
                if len(centers) == 1:
                    frame.find_blobs.assert_not_called()
                else:
                    frame.find_blobs.assert_called_once_with([settings.COLOR_THRESHOLDS[5]],
                        roi=(0, 0, 640, 480), area_threshold=50, pixels_threshold=50)
                if len(centers) > 1 and not material_present:
                    self.assertIsNone(result)
                else:
                    self.assertIsNotNone(result)
                    self.assertLessEqual(abs(result[0] - 80), 1)
                    self.assertLessEqual(abs(result[1] - 120), 1)

    def test_material_area_and_center_scale_for_ring_resolution(self):
        blob = Mock()
        for name, value in [("pixels", 600), ("cx", 330), ("cy", 220),
                            ("w", 120), ("h", 100)]:
            getattr(blob, name).return_value = value
        frame = Mock()
        frame.find_blobs.return_value = [blob]
        request = {"selector": 5, "target": 2,
                   "values": [10, 20, 200, 180, 160, 120, 0, 0]}
        for area, accepted in [(5999, False), (6000, True), (120000, True), (120001, False)]:
            blob.area.return_value = area
            result = find_material(frame, request, scale=2)
            self.assertEqual(accepted, result is not None)
            if accepted:
                self.assertEqual((165, 110), result[:2])
                self.assertEqual((60, 50, 5), result[3:])
        self.assertEqual((20, 40, 400, 360), frame.find_blobs.call_args.kwargs["roi"])


if __name__ == "__main__":
    unittest.main()
