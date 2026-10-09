import importlib.util
import pathlib
import sys
import types
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(pathlib.Path(__file__).parent))

from vision_protocol import KIND_REQUEST, MODE_MATERIAL, MODE_RING, Parser, encode


class VisionMainTest(unittest.TestCase):
    def run_camera(self, incoming, times, confirmations, write_error=None, found=None):
        maix = types.ModuleType("maix")
        for name in ("app", "camera", "display", "err", "pwm", "image", "pinmap", "uart"):
            setattr(maix, name, Mock())
        maix.app.need_exit.side_effect = [False] * len(incoming) + [True]
        serial = maix.uart.UART.return_value
        serial.read.side_effect = incoming
        serial.write.side_effect = write_error or (lambda response: len(response))
        path = pathlib.Path(__file__).with_name("main.py")
        spec = importlib.util.spec_from_file_location("vision_entry_test", path)
        entry = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, {"maix": maix}):
            spec.loader.exec_module(entry)
        stable = Mock()
        stable.update.side_effect = confirmations
        stable.center.return_value = [164, 121]
        with patch.object(entry.time, "monotonic", side_effect=times), \
                patch.object(entry, "StableWindow", return_value=stable) as constructor, \
                patch.object(entry, "find_material", return_value=found):
            if write_error:
                with self.assertRaises(RuntimeError):
                    entry.main()
            else:
                entry.main()
        self.window_calls = constructor.call_args_list
        return maix

    def request(self, mode, token, target=1, selector=1, mask=0):
        return encode(KIND_REQUEST, token, mode, selector, target, 0,
                      [0, 0, 320, 240, 160, 120, mask, 0])

    def test_pickup_result_preserves_frame_size_and_color_with_median_center(self):
        maix = self.run_camera([self.request(MODE_MATERIAL, 9, 0, 0, 42), b"", b""],
                               [0, 0.1, 0.6], [False, False, True],
                               found=(168, 124, 90, 50, 50, 4))
        response = maix.uart.UART.return_value.write.call_args.args[0]
        self.assertEqual(30, len(response))
        packet = Parser().feed(response)[0]
        self.assertEqual([164, 121, 90, 50, 50, 4, 0, 0], packet["values"])
        self.assertEqual(0, packet["selector"])
        self.assertEqual({"tolerance": 8, "minimum_ms": 500}, self.window_calls[-1].kwargs)

    def test_storage_material_keeps_original_stability_and_center(self):
        maix = self.run_camera([self.request(MODE_MATERIAL, 10, 2, 4), b"", b""],
                               [0, 0.1, 0.6], [False, False, True],
                               found=(168, 124, 90, 50, 50, 4))
        response = maix.uart.UART.return_value.write.call_args.args[0]
        self.assertEqual([168, 124], Parser().feed(response)[0]["values"][:2])
        self.assertEqual({"tolerance": 2, "minimum_ms": 500}, self.window_calls[-1].kwargs)

    def test_ring_stays_on_after_result_and_timeout_until_material_request(self):
        maix = self.run_camera(
            [self.request(MODE_RING, 1), b"", self.request(MODE_RING, 2),
             b"", self.request(MODE_MATERIAL, 3), b""],
            [0, 0.1, 0.2, 1.3, 1.4, 1.5],
            [True, False, False, True])
        self.assertEqual([20, 0, 0],
                         [call.args[0] for call in maix.pwm.PWM.return_value.duty.call_args_list])
        self.assertEqual(3, maix.uart.UART.return_value.write.call_count)
        maix.uart.UART.return_value.close.assert_called_once()

    def test_exception_turns_off_ring_light(self):
        maix = self.run_camera([self.request(MODE_RING, 1)], [0], [True],
                               RuntimeError("UART failure"))
        self.assertEqual([20, 0],
                         [call.args[0] for call in maix.pwm.PWM.return_value.duty.call_args_list])
        maix.uart.UART.return_value.close.assert_called_once()


if __name__ == "__main__":
    unittest.main()
