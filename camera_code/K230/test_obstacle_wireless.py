import importlib.util
import json
import pathlib
import sys
import types
import unittest
from unittest.mock import Mock, patch

from obstacle_detection import Confirmation
from obstacle_web import DEFAULT_CONFIG


class ObstacleWirelessTest(unittest.TestCase):
    def load_entry(self):
        modules = {name: types.ModuleType(name) for name in
                   ("media", "media.sensor", "media.display", "media.media", "network")}
        sensor = Mock()
        modules["media.sensor"].Sensor = Mock(return_value=sensor)
        modules["media.display"].Display = Mock()
        modules["media.media"].MediaManager = Mock()
        wifi = Mock()
        wifi.ifconfig.return_value = ("192.168.1.20", "", "", "")
        modules["network"].WLAN = Mock(return_value=wifi)
        modules["network"].STA_IF = 0
        modules["wifi_config"] = types.ModuleType("wifi_config")
        modules["wifi_config"].WIFI_SSID = "test"
        modules["wifi_config"].WIFI_PASSWORD = "test"
        spec = importlib.util.spec_from_file_location(
            "obstacle_wireless_entry_test", pathlib.Path(__file__).with_name("避障无线预览.py"))
        entry = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, modules):
            spec.loader.exec_module(entry)
        return entry, sensor, modules

    def test_capture_uses_live_threshold_draws_overlay_and_confirms(self):
        entry, sensor, _ = self.load_entry()
        target = Mock(pixels=lambda: 200000, cx=lambda: 300, cy=lambda: 200,
                      y=lambda: 160, h=lambda: 90)
        frame = sensor.snapshot.return_value
        frame.find_blobs.return_value = [target]
        frame.compressed.return_value = b"\xff\xd8JPEG"
        config = {"threshold": [0, 30, -20, 20, -20, 20], "min_pixels": 1500}
        stable = Confirmation()
        for index in range(3):
            jpeg, result = entry.capture(sensor, config, stable)
            self.assertEqual(index == 2, result["confirmed"])
        self.assertEqual(b"\xff\xd8JPEG", jpeg)
        self.assertEqual(249, result["bottom"])
        frame.find_blobs.assert_called_with([(0, 30, -20, 20, -20, 20)], roi=(0, 160, 640, 320),
                                           pixels_threshold=1500, area_threshold=1500,
                                           merge=True, margin=10)
        names = [call[0] for call in frame.mock_calls]
        self.assertLess(names.index("draw_rectangle"), names.index("compressed"))

    def test_main_invalid_config_is_rejected_and_valid_config_resets_confirmation(self):
        entry, sensor, modules = self.load_entry()
        server = Mock()
        clients = [Mock() for _ in range(4)]
        server.accept.side_effect = [(client, None) for client in clients] + [KeyboardInterrupt()]
        new_config = {"threshold": [0, 30, -20, 20, -20, 20], "min_pixels": 1500}
        requests = [("POST", "/config", b"{}"), ("GET", "/frame.jpg", b""),
                    ("POST", "/config", json.dumps(new_config).encode()),
                    ("GET", "/frame.jpg", b"")]
        wifi = Mock()
        wifi.ifconfig.return_value = ("192.168.1.20", "", "", "")
        with patch.object(entry, "connect_wifi", return_value=wifi), \
                patch.object(entry.socket, "socket", return_value=server), \
                patch.object(entry, "read_request", side_effect=requests), \
                patch.object(entry, "respond") as response, \
                patch.object(entry, "capture", return_value=(b"\xff\xd8JPEG", {})) as capture, \
                patch.object(entry.time, "sleep_ms", create=True), \
                patch.object(entry.os, "exitpoint", create=True), patch("builtins.print"):
            entry.main()
        self.assertEqual("400 Bad Request", response.call_args_list[0].args[1])
        self.assertEqual(DEFAULT_CONFIG, capture.call_args_list[1].args[1])
        self.assertEqual(new_config, capture.call_args_list[2].args[1])
        self.assertIsNot(capture.call_args_list[1].args[2], capture.call_args_list[2].args[2])
        for client in clients:
            client.close.assert_called_once()
        sensor.stop.assert_called_once()
        server.close.assert_called_once()
        modules["media.display"].Display.deinit.assert_called_once()
        modules["media.media"].MediaManager.deinit.assert_called_once()


if __name__ == "__main__":
    unittest.main()
