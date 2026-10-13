import importlib.util
import pathlib
import sys
import types
import unittest
from unittest.mock import Mock, patch

from obstacle_detection import STOP_LINE


class ObstacleMainTest(unittest.TestCase):
    def run_camera(self, short_write=False, uart_enabled=True):
        sensor = Mock()
        target = Mock(pixels=lambda: 6000, cx=lambda: 320, cy=lambda: 240,
                      y=lambda: 200, h=lambda: 80)
        frame = Mock()
        frame.find_blobs.return_value = [target]
        sensor.snapshot.side_effect = [frame] * 7 + [KeyboardInterrupt()]
        serial = Mock()
        serial.write.side_effect = lambda data: 1 if short_write else len(data)
        modules = {name: types.ModuleType(name) for name in
                   ("media", "media.sensor", "media.display", "media.media", "machine", "time")}
        modules["media.sensor"].Sensor = Mock(return_value=sensor)
        display = modules["media.display"].Display = Mock()
        media = modules["media.media"].MediaManager = Mock()
        modules["machine"].FPIOA = Mock()
        modules["machine"].UART = Mock(return_value=serial)
        modules["time"].sleep_ms = Mock()
        modules["time"].ticks_ms = Mock(side_effect=[1000, 1050, 1200, 1250, 1400])
        modules["time"].ticks_diff = lambda now, before: now - before
        spec = importlib.util.spec_from_file_location(
            "obstacle_entry_test", pathlib.Path(__file__).with_name("避障停车.py"))
        entry = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, modules), patch("os.exitpoint", create=True) as exitpoint, \
                patch("os.EXITPOINT_ENABLE", 1, create=True), \
                patch("os.EXITPOINT_ENABLE_SLEEP", 2, create=True), patch("builtins.print"):
            spec.loader.exec_module(entry)
            self.assertFalse(entry.UART_STOP_ENABLED)
            entry.UART_STOP_ENABLED = uart_enabled
            if short_write:
                with self.assertRaisesRegex(RuntimeError, "Obstacle UART short write"):
                    entry.main()
            else:
                entry.main()
        self.assertEqual(1, exitpoint.call_args_list[0].args[0])
        self.assertEqual(2, exitpoint.call_args_list[-1].args[0])
        self.assertEqual(2 if short_write else 7,
                         sum(call.args == (20,) for call in modules["time"].sleep_ms.call_args_list))
        sensor.stop.assert_called_once()
        self.assertEqual(2 if short_write else 7, frame.compress_for_ide.call_count)
        frame.compress_for_ide.assert_called_with(quality=70)
        frame.find_blobs.assert_called_with(entry.BLACK_THRESHOLD, roi=(0, 160, 640, 320),
                                           pixels_threshold=entry.MIN_PIXELS, area_threshold=entry.MIN_PIXELS,
                                           merge=True, margin=10)
        display.init.assert_not_called()
        display.show_image.assert_not_called()
        display.deinit.assert_not_called()
        media.deinit.assert_called_once()
        if uart_enabled:
            serial.deinit.assert_called_once()
        else:
            modules["machine"].UART.assert_not_called()
            modules["machine"].FPIOA.assert_not_called()
            serial.deinit.assert_not_called()
        return sensor, serial

    def test_confirms_before_sending_and_repeats_every_200ms(self):
        sensor, serial = self.run_camera()
        self.assertEqual(8, sensor.snapshot.call_count)
        self.assertEqual(3, serial.write.call_count)
        serial.write.assert_called_with(STOP_LINE)

    def test_short_write_releases_resources(self):
        sensor, _ = self.run_camera(short_write=True)
        self.assertEqual(3, sensor.snapshot.call_count)

    def test_standalone_preview_never_initializes_or_writes_uart(self):
        _, serial = self.run_camera(uart_enabled=False)
        serial.write.assert_not_called()


if __name__ == "__main__":
    unittest.main()
