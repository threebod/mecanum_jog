import pathlib
import sys
import types
import unittest
from unittest.mock import Mock, patch

import cv2
import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))

from ring_digits import classify_digit
from vision_detectors import find_ring


class RingDigitsTest(unittest.TestCase):
    def scene(self, digits):
        frame = np.full((480, 640), 255, dtype=np.uint8)
        for center_x, digit in zip((120, 320, 520), digits):
            for radius in (55, 75):
                cv2.circle(frame, (center_x, 240), radius, 0, 3)
            if digit is not None:
                text = str(digit)
                (width, height), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 1.4, 3)
                cv2.putText(frame, text, (center_x - width // 2, 240 + height // 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.4, 0, 3)
        return frame

    def detect(self, frame, target=2):
        maix = types.ModuleType("maix")
        maix.image = Mock()
        maix.image.image2cv.return_value = frame
        request = {"target": target, "values": [0, 0, 320, 240, 160, 120, 0, 0]}
        with patch.dict(sys.modules, {"maix": maix}):
            return find_ring(Mock(), request)

    def test_only_number_two_is_selected(self):
        found = self.detect(self.scene((1, 2, 3)))
        self.assertIsNotNone(found)
        self.assertLessEqual(abs(found[0] - 160), 1)
        self.assertLessEqual(abs(found[1] - 120), 1)
        # The requested center is the ring's center, not the glyph centroid.
        self.assertGreater(found[3], 60)

    def test_number_two_can_be_anywhere_in_the_row(self):
        for digits, expected_x in (((2, 1, 3), 60), ((3, 1, 2), 260)):
            with self.subTest(digits=digits):
                found = self.detect(self.scene(digits))
                self.assertIsNotNone(found)
                self.assertLessEqual(abs(found[0] - expected_x), 1)

    def test_missing_blank_and_multiple_twos_are_rejected(self):
        for digits in ((1, 3), (None, None, None), (2, 2)):
            with self.subTest(digits=digits):
                self.assertIsNone(self.detect(self.scene(digits)))

    def test_requests_for_other_ring_numbers_are_rejected(self):
        for target in (1, 3):
            self.assertIsNone(self.detect(self.scene((1, 2, 3)), target))

    def test_blank_and_non_numeric_shapes_are_not_digits(self):
        blank = np.zeros((64, 64), dtype=np.uint8)
        self.assertIsNone(classify_digit(blank)[0])
        cv2.rectangle(blank, (12, 12), (50, 50), 255, -1)
        self.assertIsNone(classify_digit(blank)[0])

    def test_rotated_digit_two_keeps_its_identity(self):
        frame = self.scene((1, 2, 3))
        for angle in (90, 180, 270, -15, 15):
            factor = 0.75 if angle in (90, 270) else 1
            rotation = cv2.getRotationMatrix2D((320, 240), angle, factor)
            rotated = cv2.warpAffine(frame, rotation, (640, 480), borderValue=255)
            with self.subTest(angle=angle):
                self.assertIsNotNone(self.detect(rotated))


if __name__ == "__main__":
    unittest.main()
