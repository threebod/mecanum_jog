import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).parent))

from vision_protocol import (FLAG_RING_NO_DIGIT, FLAG_STABLE, FLAG_VALID, KIND_REQUEST,
                             MODE_MATERIAL, MODE_RING,
                             Parser, StableWindow, encode, validate_request)


class VisionProtocolTest(unittest.TestCase):
    def test_unnumbered_ring_request_preserves_fallback_color_and_flag(self):
        for selector, target, accepted in [(5, 2, True), (0, 2, False), (7, 2, False),
                                            (5, 1, False)]:
            packet = Parser().feed(encode(KIND_REQUEST, 17, MODE_RING, selector, target,
                FLAG_RING_NO_DIGIT, [0, 0, 320, 240, 160, 120, 0, 0]))[0]
            self.assertEqual(accepted, validate_request(packet))
            self.assertEqual(FLAG_RING_NO_DIGIT, packet["flags"])
            self.assertEqual(selector, packet["selector"])

    def test_round_trip_preserves_request(self):
        frame = encode(KIND_REQUEST, 17, MODE_RING, 0, 2, 0,
                       [10, 20, 100, 80, 60, 55, 0, 0])
        packets = Parser().feed(frame)
        self.assertEqual(1, len(packets))
        self.assertEqual(17, packets[0]["token"])
        self.assertTrue(validate_request(packets[0]))

    def test_any_color_alignment_selector(self):
        for selector, accepted in [(0, True), (4, True), (7, False)]:
            packet = Parser().feed(encode(KIND_REQUEST, 1, MODE_MATERIAL,
                                         selector, 0, 0,
                                         [0, 0, 320, 240, 160, 120, 0, 0]))[0]
            self.assertEqual(accepted, validate_request(packet))

    def test_crc_corruption_is_rejected(self):
        frame = bytearray(encode(KIND_REQUEST, 1, MODE_RING, 0, 1, 0,
                                 [0, 0, 320, 240, 160, 120, 0, 0]))
        frame[12] ^= 1
        self.assertEqual([], Parser().feed(frame))

    def test_stability_is_request_local(self):
        window = StableWindow()
        for index in range(4):
            self.assertFalse(window.update((100, 80), index * 50))
        self.assertTrue(window.update((101, 80), 200))
        window.reset()
        self.assertFalse(window.update((100, 80), 250))

    def test_stability_uses_full_window_spread(self):
        window = StableWindow()
        self.assertFalse(window.update((100, 80), 0))
        self.assertFalse(window.update((98, 80), 50))
        self.assertFalse(window.update((102, 80), 100))
        self.assertFalse(window.update((100, 80), 150))
        self.assertFalse(window.update((100, 80), 200))

    def test_storage_material_waits_for_half_second_without_motion(self):
        window = StableWindow(minimum_ms=500)
        for now, point in [(0, (100, 80)), (100, (100, 80)),
                           (200, (101, 80)), (300, (100, 80)),
                           (400, (100, 80))]:
            self.assertFalse(window.update(point, now))
        self.assertTrue(window.update((100, 80), 500))
        self.assertFalse(window.update((104, 80), 600))

    def test_pickup_allows_eight_pixel_spread_and_returns_median(self):
        window = StableWindow(count=10, tolerance=8, minimum_ms=0, trend_tolerance=2)
        points = (100, 108, 104, 103, 101) * 2
        for index, u in enumerate(points[:9]):
            self.assertFalse(window.update((u, 80, 90, 50, 50, 4), index * 10))
        self.assertTrue(window.update((points[9], 80, 90, 50, 50, 4), 90))
        self.assertEqual([103, 80], window.center())
        self.assertFalse(window.update((109, 80, 90, 50, 50, 4), 100))
        self.assertEqual(100, window.started_ms)

    def test_pickup_color_change_and_loss_reset_stability(self):
        window = StableWindow(count=10, tolerance=8, minimum_ms=0, trend_tolerance=2)
        for now in range(0, 90, 10):
            window.update((100, 80, 90, 50, 50, 4), now)
        self.assertFalse(window.update((100, 80, 90, 50, 50, 2), 90))
        self.assertEqual(90, window.started_ms)
        for now in range(100, 180, 10):
            self.assertFalse(window.update((100, 80, 90, 50, 50, 2), now))
        self.assertTrue(window.update((100, 80, 90, 50, 50, 2), 180))
        self.assertFalse(window.update(None, 190))
        self.assertEqual([], window.samples)

    def test_pickup_rejects_nine_pixel_spread_on_either_axis(self):
        for axis in (0, 1):
            window = StableWindow(count=10, tolerance=8, minimum_ms=0, trend_tolerance=2)
            for now in range(9):
                self.assertFalse(window.update((100, 80), now))
            shifted = [100, 80]
            shifted[axis] += 9
            self.assertFalse(window.update(tuple(shifted), 9))
            self.assertEqual([tuple(shifted)], window.samples)

    def test_pickup_trend_two_pixel_boundary_on_either_axis(self):
        for axis in (0, 1):
            for shift in (-3, -2, 2, 3):
                with self.subTest(axis=axis, shift=shift):
                    window = StableWindow(count=10, tolerance=8, minimum_ms=0,
                                          trend_tolerance=2)
                    for index in range(10):
                        point = [100, 80]
                        if index >= 5:
                            point[axis] += shift
                        confirmed = window.update(tuple(point), index * 40)
                        if index < 9:
                            self.assertFalse(confirmed)
                    self.assertEqual(abs(shift) <= 2, confirmed)
                    if not confirmed:
                        self.assertEqual([tuple(point)], window.samples)
                        self.assertEqual(360, window.started_ms)

    def test_pickup_rejects_gradual_drift_within_eight_pixel_spread(self):
        for axis in (0, 1):
            for direction in (-1, 1):
                with self.subTest(axis=axis, direction=direction):
                    window = StableWindow(count=10, tolerance=8, minimum_ms=0,
                                          trend_tolerance=2)
                    for index, shift in enumerate((0, 1, 2, 3, 4, 4, 5, 6, 7, 8)):
                        point = [100, 80]
                        point[axis] += direction * shift
                        self.assertFalse(window.update(tuple(point), index * 40))
                    self.assertEqual([tuple(point)], window.samples)

    def test_pickup_trend_uses_medians_with_isolated_jitter(self):
        window = StableWindow(count=10, tolerance=8, minimum_ms=0, trend_tolerance=2)
        for index, (u, v) in enumerate([(100, 80)] * 4 + [(108, 88)] +
                                      [(100, 80)] * 5):
            confirmed = window.update((u, v), index * 40)
            self.assertEqual(index == 9, confirmed)
        self.assertEqual([100, 80], window.center())

    def test_material_mask_validation_and_round_trip(self):
        for mask, accepted in [(0, True), (42, True), (63, True), (64, False), (-1, False)]:
            packet = Parser().feed(encode(KIND_REQUEST, 1, MODE_MATERIAL, 0, 0, 0,
                                         [0, 0, 320, 240, 160, 120, mask, 0]))[0]
            self.assertEqual(accepted, validate_request(packet))
            self.assertEqual(mask, packet["values"][6])


if __name__ == "__main__":
    unittest.main()
