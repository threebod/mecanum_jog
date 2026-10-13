import importlib.util
import pathlib
import sys
import types
import unittest
from unittest.mock import Mock, patch

from avoidance_protocol import crc16


class AvoidanceMainTest(unittest.TestCase):
    def run_camera(self, short_write=False, region=3, qr_payload=None, frames=7,
                   requests=True, qr_short_write=False, qr_requests=False, qr_cancel=False, qr_expire=False):
        blob = Mock(pixels=lambda: 2300, cx=lambda: 320, cy=lambda: 350,
                    y=lambda: 320, h=lambda: 88)
        frame = Mock()
        frame.find_blobs.return_value = [blob]
        frame.find_qrcodes.return_value = [] if qr_payload is None else [
            Mock(payload=lambda: qr_payload, rect=lambda: (100, 20, 80, 80))]
        sensor = Mock()
        sensor.snapshot.side_effect = [frame] * frames + [KeyboardInterrupt()]
        serial = Mock()
        payload = "AVQ:17:{}:1".format(region).encode()
        request = payload + ":{:04X}\n".format(crc16(payload)).encode()
        incoming = ([request[:5], request[5:]] if requests else [b"", b""]) + [b""] * frames
        if qr_requests:
            payload = b"QRQ:23:1"
            qr_line = payload + ":{:04X}\n".format(crc16(payload)).encode()
            for index in range(1, frames, 2):
                incoming[index] += qr_line
            if qr_cancel:
                payload = b"QRQ:24:0"
                incoming[3:] = [payload + ":{:04X}\n".format(crc16(payload)).encode()] + [b""] * (len(incoming) - 4)
            if qr_expire:
                incoming[2:] = [b""] * (len(incoming) - 2)
        serial.read.side_effect = incoming
        serial.write.side_effect = lambda line: 1 if short_write or (qr_short_write and line.startswith(b"QR:")) else len(line)
        modules = {name: types.ModuleType(name) for name in
                   ("media", "media.sensor", "media.media", "machine", "time", "camera_preview")}
        modules["media.sensor"].Sensor = Mock(return_value=sensor)
        media = modules["media.media"].MediaManager = Mock()
        modules["machine"].UART = Mock(return_value=serial)
        modules["machine"].FPIOA = Mock()
        modules["time"].ticks_ms = Mock(side_effect=range(0, (frames + 3) * 100, 100))
        modules["time"].ticks_diff = lambda now, before: now - before
        modules["time"].ticks_add = lambda now, delta: now + delta
        modules["time"].sleep_ms = Mock()
        preview = Mock()
        preview.config = {"threshold": [8, 25, -14, 10, -8, 16], "min_pixels": 2200}
        preview.poll.return_value = False
        modules["camera_preview"].Preview = Mock(return_value=preview)
        spec = importlib.util.spec_from_file_location(
            "avoidance_entry_test", pathlib.Path(__file__).with_name("避障跑图.py"))
        entry = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, modules), patch("os.exitpoint", create=True), \
                patch("os.EXITPOINT_ENABLE", 1, create=True), \
                patch("os.EXITPOINT_ENABLE_SLEEP", 2, create=True), patch("builtins.print"):
            spec.loader.exec_module(entry)
            if short_write or qr_short_write:
                with self.assertRaisesRegex(RuntimeError, "QR UART short write" if qr_short_write else "AV UART short write"):
                    entry.main()
            else:
                entry.main()
        frame.find_blobs.assert_called_with([(8, 25, -14, 10, -8, 16)], roi=(0, 160, 640, 320),
                                           pixels_threshold=2200, area_threshold=2200, merge=True, margin=10)
        if not qr_short_write:
            frame.draw_line.assert_called_with(0, 160, 639, 160, color=(255, 0, 0), thickness=2)
            frame.compress_for_ide.assert_called_with(quality=70)
        if frame.find_qrcodes.called:
            frame.find_qrcodes.assert_called_with()
        if not qr_requests or (requests and region != 0):
            frame.find_qrcodes.assert_not_called()
        if qr_expire:
            self.assertEqual(3, frame.find_qrcodes.call_count)
        sensor.stop.assert_called_once()
        serial.deinit.assert_called_once()
        media.deinit.assert_called_once()
        preview.close.assert_called_once()
        if preview.poll.called:
            detection = preview.poll.call_args.args[1]
            self.assertEqual(407, detection["bottom"])
            self.assertEqual(72, detection["gap"])
            self.assertEqual(365, detection["front_mm"])
        return serial

    def test_real_loop_bounds_preview_crc_and_confirmation(self):
        serial = self.run_camera()
        packets = [call.args[0] for call in serial.write.call_args_list]
        self.assertEqual(6, len(packets))
        self.assertIn(b"AVR:17:3:1:0:320:407:2300:", packets[0])
        self.assertIn(b"AVR:17:3:1:1:320:407:2300:", packets[2])
        for line in packets:
            payload, check = line.strip().rsplit(b":", 1)
            self.assertEqual(crc16(payload), int(check, 16))

    def test_short_write_releases_camera_and_uart(self):
        self.run_camera(short_write=True)

    def test_qr_in_work_area_keeps_avoidance_and_preview_running(self):
        serial = self.run_camera(region=0, qr_payload="156+123+516+231", qr_requests=True)
        packets = [call.args[0] for call in serial.write.call_args_list]
        self.assertEqual(1, packets.count(b"QR:156+123+516+231\n"))
        self.assertEqual(6, sum(line.startswith(b"AVR:17:0:") for line in packets))

    def test_qr_request_repeats_result_without_exiting(self):
        serial = self.run_camera(requests=False, qr_payload="156+123+516+231", frames=15, qr_requests=True)
        packets = [call.args[0] for call in serial.write.call_args_list]
        self.assertEqual([b"QR:156+123+516+231\n"] * 2, packets)

    def test_invalid_qr_is_not_sent(self):
        serial = self.run_camera(requests=False, qr_payload="X" * 49, qr_requests=True)
        serial.write.assert_not_called()

    def test_qr_short_write_releases_resources(self):
        self.run_camera(requests=False, qr_payload="156+123+516+231", qr_short_write=True, qr_requests=True)

    def test_no_arrival_request_means_no_qr_scan(self):
        serial = self.run_camera(requests=False, qr_payload="156+123+516+231")
        serial.write.assert_not_called()

    def test_cancel_qr_leaves_avoidance_alive(self):
        serial = self.run_camera(region=0, qr_payload="156+123+516+231", qr_requests=True, qr_cancel=True)
        packets = [call.args[0] for call in serial.write.call_args_list]
        self.assertEqual(1, sum(line.startswith(b"QR:") for line in packets))
        self.assertEqual(6, sum(line.startswith(b"AVR:") for line in packets))

    def test_expired_arrival_request_stops_scanning_without_exiting(self):
        serial = self.run_camera(requests=False, qr_payload="156+123+516+231", qr_requests=True, qr_expire=True, frames=15)
        serial.write.assert_called_once_with(b"QR:156+123+516+231\n")


if __name__ == "__main__":
    unittest.main()
