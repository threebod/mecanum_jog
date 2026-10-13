import importlib.util
import json
import pathlib
import socket
import sys
import types
import unittest
from unittest.mock import Mock, patch


class CameraPreviewTest(unittest.TestCase):
    def setUp(self):
        self.now = 0
        self.wifi = Mock()
        self.wifi.isconnected.return_value = True
        self.wifi.ifconfig.return_value = ("192.168.1.20", "", "", "")
        network = types.ModuleType("network")
        network.WLAN = Mock(return_value=self.wifi)
        network.STA_IF = 0
        config = types.ModuleType("wifi_config")
        config.WIFI_SSID, config.WIFI_PASSWORD = "test", "test"
        clock = types.ModuleType("time")
        clock.ticks_ms = lambda: self.now
        clock.ticks_diff = lambda now, before: now - before
        spec = importlib.util.spec_from_file_location(
            "preview_module_test", pathlib.Path(__file__).with_name("camera_preview.py"))
        self.entry = importlib.util.module_from_spec(spec)
        self.modules = patch.dict(sys.modules, {"network": network, "wifi_config": config, "time": clock})
        self.modules.start()
        self.addCleanup(self.modules.stop)
        spec.loader.exec_module(self.entry)
        self.server = Mock()
        self.server.accept.side_effect = OSError(11)
        self.real_socket = socket.socket
        self.socket = patch.object(self.entry.socket, "socket", return_value=self.server)
        self.socket.start()
        self.addCleanup(self.socket.stop)
        with patch("builtins.print"):
            self.preview = self.entry.Preview({"threshold": [8, 25, -14, 10, -8, 16], "min_pixels": 2200})
        self.frame = Mock()
        self.frame.compressed.return_value = b"\xff\xd8JPEG"
        self.detection = {"found": True, "bottom": 479, "gap": 0, "front_mm": None,
                          "region": 4, "qr_scanning": False, "qr_payload": None}

    def request(self, path="/frame.jpg", method="GET", body=b"", recv=None, send=None, running=False):
        self.client = Mock()
        header = "{} {} HTTP/1.1\r\nContent-Length: {}\r\n\r\n".format(method, path, len(body)).encode()
        self.client.recv.side_effect = recv or [header + body]
        output = bytearray()

        def collect(data):
            count = min(len(data), send if send is not None else len(data))
            output.extend(data[:count])
            return count

        self.client.send.side_effect = collect
        self.server.accept.side_effect = [(self.client, None)] + [OSError(11)] * 100
        changed = False
        for _ in range(30):
            with patch("builtins.print"):
                changed |= self.preview.poll(self.frame, self.detection, running)
            if self.client.close.called:
                break
            self.now += 5
        return bytes(output), changed

    def test_jpeg_contains_diagnostics_and_partial_sends_complete(self):
        response, _ = self.request(send=37)
        self.assertTrue(response.startswith(b"HTTP/1.1 200 OK"))
        self.assertTrue(response.endswith(b"\xff\xd8JPEG"))
        self.assertIn(b'"bottom": 479', response)
        self.assertIn(b'"front_mm": null', response)
        self.frame.compressed.assert_called_once_with(quality=50)
        self.client.setblocking.assert_called_once_with(False)

    def test_real_local_socket_frame_response(self):
        server = self.real_socket(socket.AF_INET, socket.SOCK_STREAM)
        peer = self.real_socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            server.bind(("127.0.0.1", 0))
            server.listen(1)
            server.setblocking(False)
            peer.connect(server.getsockname())
            peer.sendall(b"GET /frame.jpg HTTP/1.1\r\n\r\n")
            peer.setblocking(False)
            self.preview.server = server
            response = bytearray()
            for _ in range(30):
                with patch.object(self.entry.socket, "socket", self.real_socket):
                    self.preview.poll(self.frame, self.detection, True)
                try:
                    chunk = peer.recv(8192)
                    if not chunk:
                        break
                    response.extend(chunk)
                except OSError:
                    pass
                self.now += 5
            self.assertTrue(response.startswith(b"HTTP/1.1 200 OK"))
            self.assertTrue(response.endswith(b"\xff\xd8JPEG"))
            self.assertIn(b'"front_mm": null', response)
        finally:
            self.preview.close()
            peer.close()

    def test_fragmented_request_does_not_wait_inside_poll(self):
        response, _ = self.request(recv=[b"GET /frame", OSError(11), b".jpg HTTP/1.1\r\n\r\n"])
        self.assertTrue(response.endswith(b"\xff\xd8JPEG"))

    def test_stalled_client_is_dropped_without_compression(self):
        self.client = Mock()
        self.client.recv.side_effect = OSError(11)
        self.server.accept.side_effect = [(self.client, None)]
        with patch("builtins.print"):
            self.preview.poll(self.frame, self.detection, False)
        self.assertEqual(1, self.client.recv.call_count)
        self.now = 301
        self.preview.poll(self.frame, self.detection, False)
        self.client.close.assert_called_once()
        self.frame.compressed.assert_not_called()

    def test_stalled_send_does_not_repeat_inside_poll(self):
        client = Mock()
        client.recv.return_value = b"GET /frame.jpg HTTP/1.1\r\n\r\n"
        client.send.side_effect = OSError(11)
        self.server.accept.side_effect = [(client, None)]
        with patch("builtins.print"):
            self.preview.poll(self.frame, self.detection, True)
        self.assertEqual(1, client.send.call_count)
        self.now = 301
        self.preview.poll(self.frame, self.detection, True)
        client.close.assert_called_once()

    def test_config_changes_only_while_stopped(self):
        config = {"threshold": [8, 30, -14, 10, -8, 16], "min_pixels": 2300}
        body = json.dumps(config).encode()
        response, changed = self.request("/config", "POST", body, running=True)
        self.assertIn(b"409 Conflict", response)
        self.assertFalse(changed)
        self.assertEqual(2200, self.preview.config["min_pixels"])
        response, changed = self.request("/config", "POST", body)
        self.assertIn(b"200 OK", response)
        self.assertTrue(changed)
        self.assertEqual(config, self.preview.config)

    def test_invalid_config_and_pixel_protocol_floor(self):
        for config in ({}, {"threshold": [8, 25, -14, 10, -8, 16], "min_pixels": 2199}):
            response, changed = self.request("/config", "POST", json.dumps(config).encode())
            self.assertIn(b"400 Bad Request", response)
            self.assertFalse(changed)
        self.assertEqual(2200, self.preview.config["min_pixels"])

    def test_wifi_connects_in_background_and_loss_preserves_camera(self):
        self.wifi.isconnected.return_value = False
        self.now = 5000
        self.preview.poll(self.frame, self.detection, True)
        self.assertEqual(2, self.wifi.connect.call_count)
        self.frame.compressed.assert_not_called()
        self.server.accept.assert_not_called()

    def test_page_and_resource_cleanup(self):
        response, _ = self.request("/")
        self.assertIn("底部行需352～462".encode(), response)
        self.assertIn(b"/frame.jpg", response)
        self.preview.close()
        self.server.close.assert_called_once()


if __name__ == "__main__":
    unittest.main()
