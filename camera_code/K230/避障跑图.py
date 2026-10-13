"""K230 avoidance and full-frame QR camera; use one sensor and one UART."""
import os
import time
import gc
from media.sensor import Sensor
from media.media import MediaManager
from machine import FPIOA, UART
from obstacle_detection import largest_black_blob
from avoidance_protocol import RequestParser, ObservationWindow, encode_result, front_distance
from qr_protocol import encode_qr_result
from camera_preview import Preview

BLACK_THRESHOLD = [(8, 25, -14, 10, -8, 16)]
MIN_PIXELS = 2200
ROI = (0, 160, 640, 320)


def main():
    sensor = serial = None
    preview = None
    media_ready = False
    try:
        os.exitpoint(os.EXITPOINT_ENABLE)
        sensor = Sensor()
        sensor.reset()
        sensor.set_framesize(width=640, height=480)
        sensor.set_pixformat(Sensor.RGB565)
        pins = FPIOA()
        pins.set_function(9, FPIOA.UART1_TXD)
        pins.set_function(10, FPIOA.UART1_RXD)
        serial = UART(UART.UART1, baudrate=115200, bits=UART.EIGHTBITS,
                      parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE)
        MediaManager.init()
        media_ready = True
        sensor.run()
        time.sleep_ms(1000)
        preview = Preview({"threshold": list(BLACK_THRESHOLD[0]), "min_pixels": MIN_PIXELS})
        parser = RequestParser()
        window = ObservationWindow()
        request = None
        qr_request = None
        requested_at = sent_at = time.ticks_ms()
        qr_requested_at = requested_at
        qr_checked_at = time.ticks_add(requested_at, -300)
        qr_sent_at = requested_at
        last_qr_line = None
        print("AV + QR camera ready; UART1 IO9 TX / IO10 RX, 115200")
        while True:
            os.exitpoint()
            now = time.ticks_ms()
            for packet in parser.feed(serial.read() or b""):
                if packet[0] == "QR":
                    if packet[1:] != qr_request:
                        last_qr_line = None
                        qr_checked_at = time.ticks_add(now, -300)
                    qr_request = packet[1:]
                    qr_requested_at = now
                    continue
                if packet != request:
                    window = ObservationWindow()
                    sent_at = time.ticks_add(now, -100)
                request = packet
                requested_at = now
            if request is not None and time.ticks_diff(now, requested_at) > 1500:
                request = None
            if qr_request is not None and time.ticks_diff(now, qr_requested_at) > 700:
                qr_request = None
            img = sensor.snapshot()
            config = preview.config
            blobs = img.find_blobs([tuple(config["threshold"])], roi=ROI,
                                   pixels_threshold=config["min_pixels"], area_threshold=config["min_pixels"],
                                   merge=True, margin=10)
            blob = largest_black_blob(blobs, config["min_pixels"])
            distance = front_distance(blob.y() + blob.h() - 1) if blob is not None else None
            result = {"found": blob is not None, "confirmed": False,
                      "region": request[1] if request is not None else 0,
                      "qr_scanning": bool(qr_request is not None and qr_request[1]), "qr_payload": None,
                      "front_mm": distance}
            if blob is not None:
                result.update(u=blob.cx(), v=blob.cy(), bottom=blob.y() + blob.h() - 1,
                              gap=479 - (blob.y() + blob.h() - 1), pixels=blob.pixels())
            label = "AV WAIT REQUEST"
            if request is not None:
                token, region, confirm = request
                stable = window.update(blob, confirm)
                result["confirmed"] = stable
                u = blob.cx() if blob is not None else 0
                bottom = blob.y() + blob.h() - 1 if blob is not None else 0
                pixels = blob.pixels() if blob is not None else 0
                if time.ticks_diff(now, sent_at) >= 100:
                    line = encode_result(token, region, blob is not None, stable, u, bottom, pixels)
                    if serial.write(line) != len(line):
                        raise RuntimeError("AV UART short write")
                    sent_at = now
                label = "AV R{} {} {} front={}".format(region,
                    "CONFIRM" if confirm else "WATCH", "FOUND" if blob is not None else "CLEAR", distance)
            # Only the MCU's arrival request enables QR; the camera stays alive after the result.
            label += " QR:SCAN" if qr_request is not None and qr_request[1] else " QR:WAIT"
            if qr_request is not None and qr_request[1] and (request is None or request[1] == 0) and time.ticks_diff(now, qr_checked_at) >= 300:
                qr_checked_at = now
                for code in img.find_qrcodes():
                    line = encode_qr_result(code.payload())
                    if line is None:
                        continue
                    if line != last_qr_line or time.ticks_diff(now, qr_sent_at) >= 1000:
                        if serial.write(line) != len(line):
                            raise RuntimeError("QR UART short write")
                        last_qr_line = line
                        qr_sent_at = now
                        print("QR SENT:", code.payload())
                    img.draw_rectangle(code.rect(), color=(0, 255, 255), thickness=2)
                    result["qr_payload"] = code.payload()
                    label += " QR=" + code.payload()
                    break
            if blob is not None:
                img.draw_rectangle(blob.rect(), color=(255, 0, 0), thickness=2)
                img.draw_cross(blob.cx(), blob.cy(), color=(0, 255, 0))
            img.draw_line(0, 160, 639, 160, color=(255, 0, 0), thickness=2)
            img.draw_string_advanced(0, 0, 20, label, color=(0, 255, 0))
            if preview.poll(img, result, request is not None):
                window = ObservationWindow()
            img.compress_for_ide(quality=70)
            time.sleep_ms(20)
    except KeyboardInterrupt:
        print("AV camera stopped")
    finally:
        try:
            if preview is not None:
                preview.close()
        finally:
            try:
                if sensor is not None:
                    sensor.stop()
            finally:
                try:
                    if media_ready:
                        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
                        time.sleep_ms(100)
                        MediaManager.deinit()
                finally:
                    if serial is not None:
                        serial.deinit()
                    gc.collect()


if __name__ == "__main__":
    main()
