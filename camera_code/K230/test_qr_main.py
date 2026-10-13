import importlib.util
import pathlib
import sys
import types
import unittest
from unittest.mock import Mock, patch


class QrMainTest(unittest.TestCase):
    def run_camera(self, payloads, write_count=None):
        sensor = Mock()
        frames = [Mock() for _ in payloads]
        sensor.snapshot.side_effect = frames
        serial = Mock()
        serial.write.side_effect = (lambda line: len(line)) if write_count is None else (
            lambda line: write_count)
        modules = {name: types.ModuleType(name) for name in
                   ("media", "media.sensor", "media.display", "media.media", "machine", "time")}
        modules["media.sensor"].Sensor = Mock(return_value=sensor)
        display = modules["media.display"].Display = Mock()
        media = modules["media.media"].MediaManager = Mock()
        modules["machine"].FPIOA = Mock()
        modules["machine"].UART = Mock(return_value=serial)
        clock = modules["time"].clock = Mock()
        sleep = modules["time"].sleep_ms = Mock()
        events = Mock()
        for name, method in [("write", serial.write), ("stop", sensor.stop),
                             ("display_deinit", display.deinit), ("sleep", sleep),
                             ("media_deinit", media.deinit), ("uart_deinit", serial.deinit)]:
            events.attach_mock(method, name)
        path = pathlib.Path(__file__).with_name("扫码.py")
        spec = importlib.util.spec_from_file_location("qr_entry_test", path)
        entry = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, modules):
            spec.loader.exec_module(entry)
            with patch.object(entry, "process_qrcode", side_effect=payloads), patch("builtins.print"):
                if write_count is None:
                    entry.main()
                else:
                    with self.assertRaisesRegex(RuntimeError, "QR UART short write"):
                        entry.main()
        sensor.stop.assert_called_once()
        display.deinit.assert_called_once()
        media.deinit.assert_called_once()
        serial.deinit.assert_called_once()
        return sensor, serial, sleep, events, clock

    def test_sends_first_result_once_then_stops_camera_and_uart(self):
        sensor, serial, sleep, events, _ = self.run_camera(["156+123+516+231", "another"])
        serial.write.assert_called_once_with(b"QR:156+123+516+231\n")
        self.assertEqual(1, sensor.snapshot.call_count)
        sleep.assert_any_call(2)
        self.assertEqual(["write", "sleep", "stop", "display_deinit", "sleep",
                          "media_deinit", "uart_deinit"],
                         [call[0] for call in events.mock_calls])

    def test_missing_and_invalid_payloads_keep_scanning_until_valid_send(self):
        sensor, serial, _, _, clock = self.run_camera([None, "X" * 49, "426", "another"])
        self.assertEqual(3, sensor.snapshot.call_count)
        serial.write.assert_called_once_with(b"QR:426\n")
        self.assertEqual(3, clock.return_value.tick.call_count)

    def test_short_write_releases_resources(self):
        self.run_camera(["426"], write_count=1)


if __name__ == "__main__":
    unittest.main()
