"""K230 standalone black blob preview; optionally send stop to STM32."""
import time
import os
import gc
from media.sensor import Sensor
from media.media import MediaManager

from obstacle_detection import Confirmation, largest_black_blob, STOP_LINE

WIDTH, HEIGHT = 640, 480
BLACK_THRESHOLD = [(8, 25, -14, 10, -8, 16)]
ROI = (0, HEIGHT // 3, WIDTH, HEIGHT - HEIGHT // 3)
MIN_PIXELS = 2200
CONFIRM_FRAMES = 3
CENTER_TOLERANCE = 40
SEND_INTERVAL_MS = 200
UART_STOP_ENABLED = False  # Standalone preview; enable only for STM32 stop testing.


def main():
    sensor = None
    serial = None
    media_ready = False
    try:
        os.exitpoint(os.EXITPOINT_ENABLE)
        sensor = Sensor()
        sensor.reset()
        sensor.set_framesize(width=WIDTH, height=HEIGHT)
        sensor.set_pixformat(Sensor.RGB565)
        if UART_STOP_ENABLED:
            from machine import FPIOA, UART
            fpioa = FPIOA()
            fpioa.set_function(9, FPIOA.UART1_TXD)
            fpioa.set_function(10, FPIOA.UART1_RXD)
            serial = UART(UART.UART1, baudrate=115200, bits=UART.EIGHTBITS,
                          parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE)
        MediaManager.init()
        media_ready = True
        sensor.run()
        time.sleep_ms(1000)
        confirmation = Confirmation(CONFIRM_FRAMES, CENTER_TOLERANCE)
        last_sent = None
        while True:
            os.exitpoint()
            img = sensor.snapshot()
            blobs = img.find_blobs(BLACK_THRESHOLD, roi=ROI,
                                   pixels_threshold=MIN_PIXELS, area_threshold=MIN_PIXELS,
                                   merge=True, margin=10)
            blob = largest_black_blob(blobs, MIN_PIXELS)
            confirmed = confirmation.update(blob)
            status = "SEARCH"
            if blob is not None:
                img.draw_rectangle(blob.rect(), color=(255, 0, 0), thickness=2)
                img.draw_cross(blob.cx(), blob.cy(), color=(0, 255, 0))
                # Bounding-box bottom is a preview estimate, not a calibrated ground point.
                bottom = blob.y() + blob.h() - 1
                status = "{} u={} v={} bottom={} pixels={}".format(
                    ("STOP TX" if serial is not None else "CONFIRMED") if confirmed else "CANDIDATE",
                    blob.cx(), blob.cy(), bottom, blob.pixels())
            if confirmed:
                now = time.ticks_ms()
                if last_sent is None or time.ticks_diff(now, last_sent) >= SEND_INTERVAL_MS:
                    if serial is not None and serial.write(STOP_LINE) != len(STOP_LINE):
                        raise RuntimeError("Obstacle UART short write")
                    last_sent = now
                    print(status)
            else:
                last_sent = None
            img.draw_rectangle(ROI, color=(0, 255, 255))
            img.draw_line(0, ROI[1], WIDTH - 1, ROI[1], color=(255, 0, 0), thickness=2)
            img.draw_string_advanced(0, 0, 20, status, color=(0, 255, 0))
            img.compress_for_ide(quality=70)
            time.sleep_ms(20)
    except KeyboardInterrupt:
        print("Obstacle test stopped")
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
