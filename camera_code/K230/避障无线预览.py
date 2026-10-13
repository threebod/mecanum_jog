"""Yahboom K230 Wi-Fi browser preview. No UART or vehicle control."""
import gc
import json
import os
import socket
import time
import network
from media.sensor import Sensor
from media.display import Display
from media.media import MediaManager

from obstacle_detection import Confirmation, largest_black_blob
from obstacle_web import DEFAULT_CONFIG, PAGE, read_request, respond, validate_config

from wifi_config import WIFI_SSID, WIFI_PASSWORD
PORT = 8080
JPEG_QUALITY = 50
ROI = (0, 160, 640, 320)  # Exclude the upper third; keep full-frame coordinates.


def connect_wifi():
    if not WIFI_SSID:
        raise ValueError("Set WIFI_SSID and WIFI_PASSWORD first")
    wifi = network.WLAN(network.STA_IF)
    wifi.active(True)
    print("[WIFI] Connecting; waiting for IP (20s timeout)")
    wifi.connect(WIFI_SSID, WIFI_PASSWORD)
    started = time.ticks_ms()
    while not wifi.isconnected() or wifi.ifconfig()[0] == "0.0.0.0":
        if time.ticks_diff(time.ticks_ms(), started) >= 20000:
            raise RuntimeError("Wi-Fi connection timed out; check SSID/password and network")
        os.exitpoint()
        time.sleep_ms(100)
    print("[WIFI] Connected; IP={}".format(wifi.ifconfig()[0]))
    return wifi


def capture(sensor, config, confirmation):
    img = sensor.snapshot()
    blobs = img.find_blobs([tuple(config["threshold"])], roi=ROI,
                           pixels_threshold=config["min_pixels"],
                           area_threshold=config["min_pixels"], merge=True, margin=10)
    blob = largest_black_blob(blobs, config["min_pixels"])
    result = {"found": blob is not None, "confirmed": confirmation.update(blob)}
    status = "SEARCH"
    if blob is not None:
        result.update(u=blob.cx(), v=blob.cy(), bottom=blob.y() + blob.h() - 1,
                      pixels=blob.pixels())
        img.draw_rectangle(blob.rect(), color=(255, 0, 0), thickness=2)
        img.draw_cross(blob.cx(), blob.cy(), color=(0, 255, 0))
        status = "{} u={} v={} pixels={}".format(
            "CONFIRMED" if result["confirmed"] else "CANDIDATE",
            result["u"], result["v"], result["pixels"])
    img.draw_line(0, ROI[1], 639, ROI[1], color=(255, 0, 0), thickness=2)
    img.draw_string_advanced(0, 0, 20, status, color=(0, 255, 0))
    Display.show_image(img)
    # Standard CanMV image API; compress a copy after drawing the detection overlay.
    jpeg = bytes(img.compressed(quality=JPEG_QUALITY))
    if jpeg[:2] != b"\xff\xd8":
        raise RuntimeError("Firmware did not produce JPEG; run k230_debug_info.py")
    return jpeg, result


def main():
    wifi = sensor = server = client = None
    display_ready = media_ready = False
    config = validate_config(DEFAULT_CONFIG)
    confirmation = Confirmation()
    try:
        print("[BOOT] K230 wireless preview starting")
        wifi = connect_wifi()
        print("[CAMERA] Initializing sensor and display")
        sensor = Sensor()
        sensor.reset()
        sensor.set_framesize(width=640, height=480)
        sensor.set_pixformat(Sensor.RGB565)
        Display.init(Display.VIRT, width=640, height=480, fps=30)
        display_ready = True
        MediaManager.init()
        media_ready = True
        sensor.run()
        time.sleep_ms(1000)
        print("[CAMERA] Started; checking first frame and JPEG compression")
        # Check JPEG support before advertising a usable preview URL.
        capture(sensor, config, Confirmation())
        print("[HTTP] JPEG check passed; starting server")
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(socket.getaddrinfo("0.0.0.0", PORT)[0][-1])
        server.listen(2)
        server.setblocking(False)
        print("K230 preview: http://{}:{}/".format(wifi.ifconfig()[0], PORT))
        while True:
            os.exitpoint()
            try:
                client, _ = server.accept()
            except OSError as error:
                if error.args[0] not in (11, 35, 10035):
                    raise
                time.sleep_ms(10)
                continue
            try:
                method, path, body = read_request(client)
                if method == "GET" and path == "/":
                    respond(client, "200 OK", "text/html; charset=utf-8", PAGE)
                elif method == "GET" and path == "/frame.jpg":
                    jpeg, result = capture(sensor, config, confirmation)
                    respond(client, "200 OK", "image/jpeg", jpeg, result)
                elif path == "/config" and method in ("GET", "POST"):
                    if method == "POST":
                        config = validate_config(json.loads(body.decode()))
                        confirmation = Confirmation()
                        print("CONFIG:", config)
                    respond(client, "200 OK", "application/json", json.dumps(config).encode())
                else:
                    respond(client, "404 Not Found", "text/plain", b"Not found")
            except (ValueError, UnicodeError) as error:
                try:
                    respond(client, "400 Bad Request", "text/plain; charset=utf-8", str(error).encode())
                except OSError:
                    pass  # The client may have closed a truncated or invalid request.
            except OSError as error:
                print("Preview request failed:", error)
            finally:
                client.close()
                client = None
                gc.collect()
    except KeyboardInterrupt:
        print("Wireless preview stopped")
    finally:
        try:
            if client is not None:
                client.close()
            if server is not None:
                server.close()
        finally:
            try:
                if sensor is not None:
                    sensor.stop()
            finally:
                try:
                    if display_ready:
                        Display.deinit()
                    if media_ready:
                        time.sleep_ms(100)
                        MediaManager.deinit()
                finally:
                    wifi = None
                    gc.collect()


if __name__ == "__main__":
    main()
