import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).parent))

from vision_protocol import (FLAG_STABLE, FLAG_VALID, KIND_REQUEST, MODE_MATERIAL, MODE_RING,
                             Parser, StableWindow, encode, validate_request)


class VisionProtocolTest(unittest.TestCase):
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

    def test_material_waits_for_half_second_without_motion(self):
        window = StableWindow(minimum_ms=500)
        for now, point in [(0, (100, 80)), (100, (100, 80)),
                           (200, (101, 80)), (300, (100, 80)),
                           (400, (100, 80))]:
            self.assertFalse(window.update(point, now))
        self.assertTrue(window.update((100, 80), 500))
        self.assertFalse(window.update((104, 80), 600))

    def test_pickup_allows_eight_pixel_spread_and_returns_median(self):
        window = StableWindow(tolerance=8, minimum_ms=500)
        for now, u in [(0, 100), (100, 108), (200, 104), (300, 103), (400, 101)]:
            self.assertFalse(window.update((u, 80, 90, 50, 50, 4), now))
        self.assertTrue(window.update((102, 80, 90, 50, 50, 4), 500))
        self.assertEqual([102, 80], window.center())
        self.assertFalse(window.update((109, 80, 90, 50, 50, 4), 600))
        self.assertEqual(600, window.started_ms)

    def test_pickup_color_change_and_loss_reset_stability(self):
        window = StableWindow(tolerance=8, minimum_ms=500)
        for now in (0, 100, 200, 300, 400):
            window.update((100, 80, 90, 50, 50, 4), now)
        self.assertFalse(window.update((100, 80, 90, 50, 50, 2), 500))
        self.assertEqual(500, window.started_ms)
        for now in (600, 700, 800, 900):
            self.assertFalse(window.update((100, 80, 90, 50, 50, 2), now))
        self.assertTrue(window.update((100, 80, 90, 50, 50, 2), 1000))
        self.assertFalse(window.update(None, 1100))
        self.assertEqual([], window.samples)

    def test_material_mask_validation_and_round_trip(self):
        for mask, accepted in [(0, True), (42, True), (63, True), (64, False), (-1, False)]:
            packet = Parser().feed(encode(KIND_REQUEST, 1, MODE_MATERIAL, 0, 0, 0,
                                         [0, 0, 320, 240, 160, 120, mask, 0]))[0]
            self.assertEqual(accepted, validate_request(packet))
            self.assertEqual(mask, packet["values"][6])


if __name__ == "__main__":
    unittest.main()
