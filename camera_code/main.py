"""MaixCAM vehicle alignment entry point. Deploy this directory together."""
import time

from maix import app, camera, display, err, pwm, image, pinmap, uart

from vision_detectors import find_material, find_ring
from vision_protocol import (FLAG_STABLE, FLAG_VALID, KIND_RESULT, MODE_MATERIAL, MODE_RING,
                             Parser, StableWindow, encode, validate_request)
import vision_settings as settings


def main():
    err.check_raise(pinmap.set_pin_function("B25", "PWM6"), "illumination PWM mapping")
    illuminator = pwm.PWM(6, freq=100000, duty=0, enable=True)
    for pin, function in settings.UART_PINS.items():
        err.check_raise(pinmap.set_pin_function(pin, function), "UART pin mapping")
    serial = uart.UART(settings.UART_DEVICE, settings.BAUD)
    cam = camera.Camera(320, 240)
    screen = display.Display()
    parser = Parser()
    stable = StableWindow()
    request = None
    light_mode = MODE_MATERIAL
    request_started = 0
    discard_frames = 0
    try:
        while not app.need_exit():
            now_ms = int(time.monotonic() * 1000)
            incoming = serial.read(-1, 0) or b""
            if incoming:
                print("VISION UART RX bytes={}".format(len(incoming)))
            for packet in parser.feed(incoming):
                if validate_request(packet):
                    print("VISION REQUEST token={} mode={} target={}".format(
                        packet["token"], packet["mode"], packet["target"]))
                    request = packet
                    if packet["mode"] != light_mode:
                        illuminator.duty(20 if packet["mode"] == MODE_RING else 0)
                        light_mode = packet["mode"]
                    request_started = now_ms
                    discard_frames = 2
                    pickup = packet["mode"] == MODE_MATERIAL and packet["target"] == 0
                    stable = StableWindow(tolerance=8 if pickup else 2,
                                          minimum_ms=500 if packet["mode"] == MODE_MATERIAL
                                          else 180)
            frame = cam.read()
            found = None
            if request is not None and frame is not None:
                if discard_frames:
                    discard_frames -= 1
                else:
                    found = (find_ring(frame, request) if request["mode"] == MODE_RING
                             else find_material(frame, request))
                confirmed = stable.update(found, now_ms)
                timeout_ms = 1200 if request["mode"] == MODE_MATERIAL else 1000
                if confirmed or now_ms - request_started >= timeout_ms:
                    values = [0] * 8
                    if found is not None:
                        values[:len(found)] = found
                        if confirmed and request["mode"] == MODE_MATERIAL and request["target"] == 0:
                            values[:2] = stable.center()
                    flags = FLAG_VALID | FLAG_STABLE if confirmed else 0
                    response = encode(KIND_RESULT, request["token"], request["mode"],
                                      request["selector"], request["target"],
                                      flags, values)
                    if serial.write(response) != len(response):
                        raise RuntimeError("UART short write")
                    print("VISION RESULT token={} valid={}".format(
                        request["token"], bool(flags)))
                    request = None
            if frame is not None:
                if request is not None:
                    x, y, width, height, u, v, _, _ = request["values"]
                    frame.draw_rect(x, y, width, height, image.COLOR_YELLOW, 1)
                    frame.draw_cross(u, v, image.COLOR_RED, 8, 2)
                if found is not None:
                    frame.draw_cross(int(found[0]), int(found[1]), image.COLOR_GREEN, 8, 2)
                screen.show(frame)
    finally:
        illuminator.duty(0)
        serial.close()


if __name__ == "__main__":
    main()
