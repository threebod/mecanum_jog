"""Non-blocking Wi-Fi preview for the continuously running QR/avoidance camera."""
import json
import socket
import time
from obstacle_web import PAGE, validate_config
from wifi_config import WIFI_SSID, WIFI_PASSWORD

PORT = 8080
PAGE = PAGE.decode().replace("K230 黑色物块调试", "K230 扫码＋避障实时预览").replace(
    "仅相机检测，不控制车辆。", "显示当前扫码与避障画面；扫码仅在主控到达扫码点后开启。").replace(
    'id="pixels" type="number" min="1"', 'id="pixels" type="number" min="2200"').replace(
    "参数仅在本次运行有效，重启后恢复默认。", "车辆停止后可调参数，运行中拒绝修改；重启恢复默认。").replace(
    "el('rate').textContent=", """if(result.found)el('status').textContent+='；距图底 '+result.gap+' px；'+
    (result.front_mm===null?'超出标定范围（底部行需352～462）':'距车体前沿 '+result.front_mm+' mm');
    el('status').textContent+='；观察区 R'+(result.region||0)+'；扫码'+(result.qr_scanning?'已开启':'等待到点');
    if(result.qr_payload)el('status').textContent+='：'+result.qr_payload;
    el('rate').textContent=""").encode()


class Preview:
    def __init__(self, config):
        self.config = validate_config(config)
        self.wifi = self.server = self.client = None
        self.data = bytearray()
        self.output = b""
        self.offset = 0
        self.started = self.connected_at = time.ticks_ms()
        try:
            import network
            if not WIFI_SSID:
                print("[WIFI] Set WIFI_SSID and WIFI_PASSWORD in wifi_config.py")
                return
            self.wifi = network.WLAN(network.STA_IF)
            self.wifi.active(True)
            self.wifi.connect(WIFI_SSID, WIFI_PASSWORD)
            print("[WIFI] Connecting in background; UART and camera remain active")
        except (ImportError, OSError) as error:
            self.wifi = None
            print("[WIFI] Preview unavailable:", error)

    def drop_client(self):
        if self.client is not None:
            try:
                self.client.close()
            except OSError:
                pass
        self.client = None
        self.data = bytearray()
        self.output = b""
        self.offset = 0

    def respond(self, status, kind, body, detection=None):
        header = "HTTP/1.1 {}\r\nContent-Type: {}\r\nContent-Length: {}\r\n".format(status, kind, len(body))
        if detection is not None:
            header += "X-Detection: {}\r\n".format(json.dumps(detection))
        self.output = (header + "Cache-Control: no-store\r\nConnection: close\r\n\r\n").encode() + body

    def request(self):
        if len(self.data) > 4096:
            raise ValueError("Request too large")
        end = self.data.find(b"\r\n\r\n")
        if end < 0:
            return None
        headers = bytes(self.data[:end]).decode().split("\r\n")
        first = headers[0].split()
        if len(first) != 3:
            raise ValueError("Invalid request")
        length = 0
        for header in headers[1:]:
            name, _, value = header.partition(":")
            if name.lower() == "content-length":
                length = int(value.strip())
        if not 0 <= length <= 512:
            raise ValueError("Body too large")
        if len(self.data) < end + 4 + length:
            return None
        return first[0], first[1].split("?", 1)[0], bytes(self.data[end + 4:end + 4 + length])

    def poll(self, image, detection, running):
        """One non-blocking receive/send per frame; a slow browser cannot hold UART."""
        changed = False
        now = time.ticks_ms()
        if self.wifi is None:
            return changed
        try:
            if not self.wifi.isconnected() or self.wifi.ifconfig()[0] == "0.0.0.0":
                if self.server is not None:
                    self.server.close()
                    self.server = None
                    self.drop_client()
                if time.ticks_diff(now, self.connected_at) >= 5000:
                    self.wifi.connect(WIFI_SSID, WIFI_PASSWORD)
                    self.connected_at = now
                return changed
            if self.server is None:
                self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.server.bind(socket.getaddrinfo("0.0.0.0", PORT)[0][-1])
                self.server.listen(2)
                self.server.setblocking(False)
                print("K230 preview: http://{}:{}/".format(self.wifi.ifconfig()[0], PORT))
            if self.client is None:
                self.client, _ = self.server.accept()
                self.client.setblocking(False)
                self.started = now
            if time.ticks_diff(now, self.started) >= 300:
                self.drop_client()
                return changed
            if not self.output:
                chunk = self.client.recv(1024)
                if not chunk:
                    self.drop_client()
                    return changed
                self.data.extend(chunk)
                request = self.request()
                if request is None:
                    return changed
                method, path, body = request
                if method == "GET" and path == "/":
                    self.respond("200 OK", "text/html; charset=utf-8", PAGE)
                elif method == "GET" and path == "/frame.jpg":
                    jpeg = bytes(image.compressed(quality=50))
                    if jpeg[:2] != b"\xff\xd8":
                        raise ValueError("Firmware did not produce JPEG")
                    self.respond("200 OK", "image/jpeg", jpeg, detection)
                elif path == "/config" and method in ("GET", "POST"):
                    if method == "POST" and running:
                        self.respond("409 Conflict", "text/plain; charset=utf-8", "请先停止避障跑图再调整参数".encode())
                    else:
                        if method == "POST":
                            config = validate_config(json.loads(body.decode()))
                            if config["min_pixels"] < 2200:
                                raise ValueError("min_pixels must be at least 2200 for STM32")
                            self.config = config
                            changed = True
                        self.respond("200 OK", "application/json", json.dumps(self.config).encode())
                else:
                    self.respond("404 Not Found", "text/plain", b"Not found")
            if self.output:
                sent = self.client.send(memoryview(self.output)[self.offset:self.offset + 8192])
                if sent == 0:
                    self.drop_client()
                elif sent:
                    self.offset += sent
                    if self.offset == len(self.output):
                        self.drop_client()
        except (ValueError, UnicodeError) as error:
            self.respond("400 Bad Request", "text/plain; charset=utf-8", str(error).encode())
        except OSError as error:
            if not error.args or error.args[0] not in (11, 35, 10035, 110):
                self.drop_client()
        return changed

    def close(self):
        self.drop_client()
        if self.server is not None:
            self.server.close()
            self.server = None
