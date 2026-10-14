"""MaixCAM vehicle alignment entry point. Deploy this directory together."""
import math
import time

from maix import app, camera, display, err, gpio, image, pinmap, uart

from vision_detectors import find_material, find_ring
from vision_protocol import (FLAG_RING_NO_DIGIT, FLAG_STABLE, FLAG_VALID, KIND_RESULT,
                             MODE_MATERIAL, MODE_RING,
                             Parser, StableWindow, encode, validate_request)
import vision_settings as settings


def main():
    err.check_raise(pinmap.set_pin_function("B25", "GPIOB25"), "illumination GPIO mapping")
    illuminator = gpio.GPIO("GPIOB25", gpio.Mode.OUT)
    illuminator.value(0)
    for pin, function in settings.UART_PINS.items():
        err.check_raise(pinmap.set_pin_function(pin, function), "UART pin mapping")
    serial = uart.UART(settings.UART_DEVICE, settings.BAUD)
    cam = camera.Camera(320, 240)
    screen = display.Display()
    parser = Parser()
    stable = StableWindow()
    request = None
    light_mode = MODE_MATERIAL
    ring_no_digit = False
    material_contrast = None
    request_started = 0
    discard_frames = 0
    detection_frames = 0
    detection_started = None
    ring_hits = 0
    ring_detect_ms = 0
    material_status = "material waiting"
    material_detail = ""
    material_metrics = ""
    ring_label = "ring 2"
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
                    if packet["mode"] == MODE_RING:
                        ring_label = "ring" if packet["flags"] & FLAG_RING_NO_DIGIT else "ring 2"
                    no_digit = packet["mode"] == MODE_RING and bool(packet["flags"] & FLAG_RING_NO_DIGIT)
                    if packet["mode"] != light_mode or no_digit != ring_no_digit:
                        if packet["mode"] != light_mode:
                            if packet["mode"] == MODE_RING and material_contrast is None:
                                material_contrast = cam.constrast()
                            resolution = (640, 480) if packet["mode"] == MODE_RING else (320, 240)
                            err.check_raise(cam.set_resolution(*resolution), "camera resolution")
                        # Unnumbered rings use material colors to select the target.
                        contrast = (settings.RING_CONTRAST
                                    if packet["mode"] == MODE_RING and not no_digit else material_contrast)
                        contrast_readback = cam.constrast(contrast)
                        print("VISION CAMERA mode={} no_digit={} contrast_set={} contrast_readback={}".format(
                            packet["mode"], int(no_digit), contrast, contrast_readback))
                        light_mode = packet["mode"]
                        ring_no_digit = no_digit
                    request_started = now_ms
                    discard_frames = 2
                    detection_frames = 0
                    detection_started = None
                    ring_hits = 0
                    ring_detect_ms = 0
                    pickup = packet["mode"] == MODE_MATERIAL and packet["target"] == 0
                    stable = StableWindow(count=10 if pickup else
                                          3 if packet["mode"] == MODE_RING else 5,
                                          tolerance=8 if pickup else 2,
                                          minimum_ms=0 if pickup else
                                          500 if packet["mode"] == MODE_MATERIAL else 180,
                                          trend_tolerance=2 if pickup else None)
                    if packet["mode"] == MODE_MATERIAL:
                        material_status = "material waiting"
                        material_detail = ""
                        material_metrics = ""
            frame = cam.read()
            found = None
            if request is not None and (frame is not None or
                    (request["mode"] == MODE_MATERIAL and request["target"] == 0)):
                detected = False
                if frame is None:
                    pass  # A missing pickup frame resets the consecutive-frame window.
                elif discard_frames:
                    discard_frames -= 1
                else:
                    if request["mode"] == MODE_RING:
                        detect_started = time.monotonic()
                        found = find_ring(frame, request)
                        ring_detect_ms += int((time.monotonic() - detect_started) * 1000)
                        ring_hits += int(found is not None)
                    else:
                        found = find_material(frame, request)
                    detected = True
                if request["mode"] == MODE_MATERIAL:
                    now_ms = int(time.monotonic() * 1000)
                if detected:
                    detection_frames += 1
                    if detection_started is None:
                        detection_started = now_ms
                confirmed = stable.update(found, now_ms)
                timeout_ms = 1200 if request["mode"] == MODE_MATERIAL else 1000
                values = [0] * 8
                if found is not None:
                    values[:len(found)] = found
                    if confirmed and request["mode"] == MODE_MATERIAL and request["target"] == 0:
                        values[:2] = stable.center()
                if request["mode"] == MODE_MATERIAL:
                    elapsed = now_ms - detection_started if detection_started is not None else 0
                    fps = (detection_frames - 1) * 1000 / elapsed if elapsed > 0 else 0
                    material_metrics = "fps={:.1f} elapsed={}ms".format(
                        fps, now_ms - request_started)
                    if found is None:
                        material_status = "material missing still=0"
                        material_detail = ""
                    else:
                        color = ("unknown", "red", "yellow", "blue", "green", "black",
                                 "light_blue")[found[5]]
                        dx, dy = values[0] - 160, 120 - values[1]
                        material_status = "{} center=({},{}) still={}".format(
                            color, values[0], values[1], int(confirmed))
                        material_detail = "dx={} dy={} d={:.1f}px".format(
                            dx, dy, math.hypot(dx, dy))
                if confirmed or now_ms - request_started >= timeout_ms:
                    accepted = confirmed and found is not None
                    flags = FLAG_VALID | FLAG_STABLE if accepted else 0
                    response = encode(KIND_RESULT, request["token"], request["mode"],
                                      request["selector"], request["target"],
                                      flags, values)
                    if serial.write(response) != len(response):
                        raise RuntimeError("UART short write")
                    print("VISION RESULT token={} valid={}".format(
                        request["token"], bool(flags)))
                    if request["mode"] == MODE_MATERIAL:
                        print("VISION MATERIAL {} {} {}".format(
                            material_status, material_detail, material_metrics))
                    else:
                        samples = stable.samples
                        span = [max(point[axis] for point in samples) -
                                min(point[axis] for point in samples) if samples else 0
                                for axis in (0, 1)]
                        window_ms = now_ms - stable.started_ms if stable.started_ms is not None else 0
                        print("VISION RING token={} valid={} found={} center=({},{}) "
                              "frames={} hits={} window={} window_ms={} span={}/{}px "
                              "elapsed_ms={} detect_ms={}".format(
                                  request["token"], accepted, found is not None, values[0], values[1],
                                  detection_frames, ring_hits, len(samples), window_ms, span[0], span[1],
                                  int(time.monotonic() * 1000) - request_started, ring_detect_ms))
                    request = None
            if frame is not None:
                scale = 2 if light_mode == MODE_RING else 1
                if light_mode == MODE_MATERIAL:
                    frame.draw_cross(160, 120, image.COLOR_WHITE, 10, 1)
                    for index, line in enumerate((material_status, material_detail,
                                                   material_metrics)):
                        frame.draw_string(2, 2 + index * 18, line, image.COLOR_WHITE)
                if request is not None:
                    x, y, width, height, u, v, _, _ = request["values"]
                    frame.draw_rect(x * scale, y * scale, width * scale, height * scale,
                                    image.COLOR_YELLOW, 1)
                    frame.draw_cross(u * scale, v * scale, image.COLOR_RED, 8, 2)
                if found is not None:
                    frame.draw_cross(int(found[0] * scale), int(found[1] * scale),
                                     image.COLOR_GREEN, 8, 2)
                    if light_mode == MODE_RING:
                        frame.draw_string(2, 2, ring_label, image.COLOR_GREEN)
                screen.show(frame)
    finally:
        illuminator.value(0)
        serial.close()


if __name__ == "__main__":
    main()
