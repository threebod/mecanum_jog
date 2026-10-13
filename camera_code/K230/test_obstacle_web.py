import json
import socket
import unittest
from unittest.mock import Mock, patch

import obstacle_web as web


class ObstacleWebTest(unittest.TestCase):
    def test_real_socket_request_and_response(self):
        client, peer = socket.socketpair()
        try:
            peer.sendall(b"GET /frame.jpg?time=123 HTTP/1.1\r\nHost: k230\r\n\r\n")
            with patch.object(web.time, "ticks_ms", return_value=0, create=True), \
                    patch.object(web.time, "ticks_diff", return_value=0, create=True), \
                    patch.object(web.time, "sleep_ms", create=True), patch.object(web.os, "exitpoint", create=True):
                self.assertEqual(("GET", "/frame.jpg", b""), web.read_request(client))
                web.respond(client, "200 OK", "image/jpeg", b"\xff\xd8JPEG")
            peer.settimeout(1)
            response = peer.recv(1024)
            self.assertTrue(response.startswith(b"HTTP/1.1 200 OK\r\n"))
            self.assertTrue(response.endswith(b"\xff\xd8JPEG"))
        finally:
            client.close()
            peer.close()

    def test_validates_ranges_and_does_not_mutate_default(self):
        config = web.validate_config(web.DEFAULT_CONFIG)
        config["threshold"][0] = 0
        self.assertEqual(8, web.DEFAULT_CONFIG["threshold"][0])
        for invalid in ({}, {"threshold": [8, 25], "min_pixels": 5000},
                        {"threshold": [30, 25, -14, 10, -8, 16], "min_pixels": 5000},
                        {"threshold": [8, 101, -14, 10, -8, 16], "min_pixels": 5000},
                        {"threshold": [8, 25, -129, 10, -8, 16], "min_pixels": 5000},
                        {"threshold": [8, 25, -14, 10, -8, 16], "min_pixels": True},
                        {"threshold": [8, 25, -14, 10, -8, 16], "min_pixels": 0}):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                web.validate_config(invalid)

    def test_fragmented_post_reads_full_body(self):
        body = json.dumps(web.DEFAULT_CONFIG).encode()
        client = Mock()
        header = b"POST /config HTTP/1.1\r\ncontent-length: %d\r\n\r\n" % len(body)
        client.recv.side_effect = [header[:10], OSError(11), header[10:] + body[:3], body[3:]]
        with patch.object(web.time, "ticks_ms", return_value=0, create=True), \
                patch.object(web.time, "ticks_diff", side_effect=lambda a, b: a - b, create=True), \
                patch.object(web.time, "sleep_ms", create=True), patch.object(web.os, "exitpoint", create=True):
            self.assertEqual(("POST", "/config", body), web.read_request(client))
        client.setblocking.assert_called_once_with(False)

    def test_rejects_truncated_oversized_and_invalid_requests(self):
        for chunks in ([b"GET /", b""], [b"X" * 4097],
                       [b"POST /config HTTP/1.1\r\nContent-Length: 513\r\n\r\n"],
                       [b"BAD\r\n\r\n"]):
            client = Mock()
            client.recv.side_effect = chunks
            with patch.object(web.time, "ticks_ms", return_value=0, create=True), \
                    patch.object(web.time, "ticks_diff", return_value=0, create=True), \
                    patch.object(web.time, "sleep_ms", create=True), patch.object(web.os, "exitpoint", create=True), \
                    self.assertRaises(ValueError):
                web.read_request(client)

    def test_partial_sends_and_would_block_preserve_entire_response(self):
        client = Mock()
        output = bytearray()
        blocked = [True]

        def send(data):
            if blocked.pop() if blocked else False:
                raise OSError(11)
            output.extend(data[:3])
            return min(3, len(data))

        client.send.side_effect = send
        with patch.object(web.time, "ticks_ms", return_value=0, create=True), \
                patch.object(web.time, "ticks_diff", return_value=0, create=True), \
                patch.object(web.time, "sleep_ms", create=True), patch.object(web.os, "exitpoint", create=True):
            web.respond(client, "200 OK", "image/jpeg", b"\xff\xd8JPEG", {"found": False})
        header, body = bytes(output).split(b"\r\n\r\n")
        self.assertIn(b"Content-Length: 6", header)
        self.assertIn(b'X-Detection: {"found": false}', header)
        self.assertEqual(b"\xff\xd8JPEG", body)

    def test_stalled_receive_and_send_timeout(self):
        client = Mock()
        with patch.object(web.time, "ticks_ms", side_effect=[0, 2000], create=True), \
                patch.object(web.time, "ticks_diff", side_effect=lambda a, b: a - b, create=True), \
                self.assertRaises(OSError):
            web.read_request(client)
        with patch.object(web.time, "ticks_ms", side_effect=[0, 2000], create=True), \
                patch.object(web.time, "ticks_diff", side_effect=lambda a, b: a - b, create=True), \
                self.assertRaises(OSError):
            web.send_all(client, b"JPEG")


if __name__ == "__main__":
    unittest.main()
