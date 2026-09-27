"""Vehicle vision protocol: fixed 30-byte little-endian CRC frames."""
import struct

FRAME_SIZE = 30
FORMAT = "<2sBBIBBBB8hH"
KIND_REQUEST = 1
KIND_RESULT = 2
MODE_MATERIAL = 1
MODE_RING = 2
FLAG_VALID = 1
FLAG_STABLE = 2


def crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xFFFF
    return crc


def encode(kind, token, mode, selector, target, flags, values):
    if len(values) != 8:
        raise ValueError("values must contain eight int16 fields")
    frame = struct.pack(FORMAT, b"\xA5\x5A", 1, kind, token, mode,
                        selector, target, flags, *values, 0)
    return frame[:28] + struct.pack("<H", crc16(frame[:28]))


class Parser:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        packets = []
        while len(self.buffer) >= 2:
            if self.buffer[:2] != b"\xA5\x5A":
                del self.buffer[0]
                continue
            if len(self.buffer) < FRAME_SIZE:
                break
            frame = bytes(self.buffer[:FRAME_SIZE])
            expected = struct.unpack_from("<H", frame, 28)[0]
            if frame[2] != 1 or crc16(frame[:28]) != expected:
                del self.buffer[0]
                continue
            row = struct.unpack(FORMAT, frame)
            packets.append({"kind": row[2], "token": row[3], "mode": row[4],
                            "selector": row[5], "target": row[6],
                            "flags": row[7], "values": list(row[8:16])})
            del self.buffer[:FRAME_SIZE]
        return packets


class StableWindow:
    def __init__(self, count=5, tolerance=2, minimum_ms=180):
        self.count = count
        self.tolerance = tolerance
        self.minimum_ms = minimum_ms
        self.reset()

    def reset(self):
        self.samples = []
        self.started_ms = None

    def update(self, point, now_ms):
        if point is None:
            self.reset()
            return False
        next_samples = self.samples + [point]
        if self.samples and any(
                max(sample[i] for sample in next_samples) -
                min(sample[i] for sample in next_samples) > self.tolerance
                for i in (0, 1)):
            self.reset()
        if not self.samples:
            self.started_ms = now_ms
        self.samples.append(point)
        return len(self.samples) >= self.count and now_ms - self.started_ms >= self.minimum_ms


def validate_request(packet):
    if packet["kind"] != KIND_REQUEST or packet["mode"] not in (MODE_MATERIAL, MODE_RING):
        return False
    if packet["mode"] == MODE_MATERIAL and not 1 <= packet["selector"] <= 6:
        return False
    if packet["mode"] == MODE_RING and not 1 <= packet["target"] <= 3:
        return False
    x, y, width, height, u, v, _, _ = packet["values"]
    return (0 <= x < 320 and 0 <= y < 240 and width > 0 and height > 0 and
            x + width <= 320 and y + height <= 240 and
            x <= u < x + width and y <= v < y + height)
