import unittest

from qr_protocol import encode_qr_result


class QrProtocolTest(unittest.TestCase):
    def test_ascii_and_utf8(self):
        self.assertEqual(encode_qr_result("426"), b"QR:426\n")
        self.assertEqual(encode_qr_result(" 绿黄蓝 "), "QR:绿黄蓝\n".encode())

    def test_reject_empty_long_and_control(self):
        self.assertIsNone(encode_qr_result("\r\n"))
        self.assertIsNone(encode_qr_result("X" * 49))
        self.assertEqual(encode_qr_result("A\nB"), b"QR:A B\n")


if __name__ == "__main__":
    unittest.main()
