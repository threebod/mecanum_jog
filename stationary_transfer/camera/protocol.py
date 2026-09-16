"""30 byte fixed frames, little endian; CRC16/CCITT-FALSE over bytes 0..27."""
import struct

SIZE = 30
FORMAT = '<2sBBIBBBB8hH'


def crc16(data):
    crc = 0xffff
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xffff
    return crc


def encode(token, kind=1, mode=1, color=1, target=1, flags=0, values=None):
    values = values if values is not None else [0] * 8
    data = struct.pack(FORMAT, b'\xa5\x5a', 1, kind, token, mode, color, target, flags, *values, 0)
    return data[:28] + struct.pack('<H', crc16(data[:28]))


class Parser:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        result = []
        while len(self.buffer) >= 2:
            if self.buffer[:2] != b'\xa5\x5a':
                del self.buffer[0]
                continue
            if len(self.buffer) < SIZE:
                break
            packet = bytes(self.buffer[:SIZE])
            if packet[2] != 1 or crc16(packet[:28]) != struct.unpack('<H', packet[28:])[0]:
                del self.buffer[0]
                continue
            row = struct.unpack(FORMAT, packet)
            result.append(dict(kind=row[2], token=row[3], mode=row[4], color=row[5],
                               target=row[6], flags=row[7], values=list(row[8:16])))
            del self.buffer[:SIZE]
        return result


class Stable:
    """A request-local window. Compare every sample to the first, not just neighbors."""
    def __init__(self, count=5, tolerance=2, minimum_ms=180):
        self.required, self.tolerance, self.minimum_ms = count, tolerance, minimum_ms
        self.reset()

    def reset(self):
        self.samples = []
        self.started = None

    def update(self, point, now_ms):
        if point is None:
            self.reset()
            return False
        if self.samples and max(abs(point[i] - self.samples[0][i]) for i in (0, 1)) > self.tolerance:
            self.reset()
        if not self.samples:
            self.started = now_ms
        self.samples.append(point)
        return len(self.samples) >= self.required and now_ms - self.started >= self.minimum_ms


def validate_request(p):
    if p['kind'] != 1 or p['mode'] not in (1, 2, 3, 4) or not 1 <= p['color'] <= 6 or not 1 <= p['target'] <= 3:
        return False
    x, y, w, h, u, v, _, _ = p['values']
    return 0 <= x < 320 and 0 <= y < 240 and w > 0 and h > 0 and x+w <= 320 and y+h <= 240 and x <= u < x+w and y <= v < y+h
