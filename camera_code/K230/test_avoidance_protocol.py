import unittest
from unittest.mock import Mock
from avoidance_protocol import RequestParser, ObservationWindow, crc16, encode_result, front_distance


class AvoidanceProtocolTest(unittest.TestCase):
    def test_crc_and_partial_requests(self):
        self.assertEqual(0x29b1, crc16(b"123456789"))
        payload = b"AVQ:17:3:1"
        line = payload + ":{:04X}\n".format(crc16(payload)).encode()
        parser = RequestParser()
        self.assertEqual([], parser.feed(line[:4]))
        self.assertEqual([(17, 3, 1)], parser.feed(line[4:]))
        self.assertEqual([], parser.feed(b"QR:AVQ:17:3:1\n" + line.replace(b":17:", b":18:")))
        self.assertEqual([(17, 3, 1)], parser.feed(b"X" * 100 + b"\n" + line))
        response = encode_result(17, 3, True, True, 320, 407, 2300)
        data, checksum = response.strip().rsplit(b":", 1)
        self.assertEqual(crc16(data), int(checksum, 16))

    def test_calibration_no_extrapolation(self):
        for row, distance in [(17, 235), (23, 280), (72, 365), (104, 370), (127, 490)]:
            self.assertEqual(distance, front_distance(479 - row))
        self.assertEqual(285, front_distance(453))
        self.assertIsNone(front_distance(463))
        self.assertIsNone(front_distance(351))

    def test_qr_control_crc_partial_and_value_validation(self):
        parser = RequestParser()
        for payload, expected in [(b"QRQ:23:1", ("QR", 23, 1)), (b"QRQ:24:0", ("QR", 24, 0)),
                                  (b"QRQ:0:1", None), (b"QRQ:23:2", None), (b"QRQ:-1:1", None)]:
            line = payload + ":{:04X}\n".format(crc16(payload)).encode()
            self.assertEqual([], parser.feed(line[:5]))
            self.assertEqual([] if expected is None else [expected], parser.feed(line[5:]))
        self.assertEqual([], parser.feed(b"QRQ:23:1:0000\n"))

    def test_moving_and_stationary_confirmation(self):
        blob = Mock(cx=lambda: 320, cy=lambda: 350)
        window = ObservationWindow()
        for _ in range(5):
            self.assertFalse(window.update(blob, False))
        window = ObservationWindow()
        self.assertFalse(window.update(blob, True))
        self.assertFalse(window.update(blob, True))
        self.assertTrue(window.update(blob, True))
        self.assertFalse(window.update(None, True))
        self.assertFalse(window.update(None, True))
        self.assertTrue(window.update(None, True))


if __name__ == "__main__":
    unittest.main()
