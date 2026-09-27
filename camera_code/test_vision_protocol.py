import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).parent))

from vision_protocol import (FLAG_STABLE, FLAG_VALID, KIND_REQUEST, MODE_RING,
                             Parser, StableWindow, encode, validate_request)


class VisionProtocolTest(unittest.TestCase):
    def test_round_trip_preserves_request(self):
        frame = encode(KIND_REQUEST, 17, MODE_RING, 0, 2, 0,
                       [10, 20, 100, 80, 60, 55, 0, 0])
        packets = Parser().feed(frame)
        self.assertEqual(1, len(packets))
        self.assertEqual(17, packets[0]["token"])
        self.assertTrue(validate_request(packets[0]))

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


if __name__ == "__main__":
    unittest.main()
