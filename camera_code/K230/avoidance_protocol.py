"""CRC-protected ASCII requests/results on K230 UART1 / STM32 USART3."""


def crc16(data):
    crc = 0xffff
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xffff
    return crc


def encode_result(token, region, seen, stable, u=0, bottom=0, pixels=0):
    payload = "AVR:{}:{}:{}:{}:{}:{}:{}".format(
        token, region, int(seen), int(stable), u, bottom, pixels).encode()
    return payload + ":{:04X}\n".format(crc16(payload)).encode()


class RequestParser:
    def __init__(self):
        self.line = bytearray()
        self.invalid = False

    def feed(self, data):
        packets = []
        for byte in data:
            if byte == 13:
                continue
            if byte == 10:
                if not self.invalid:
                    packet = self.decode(bytes(self.line))
                    if packet is not None:
                        packets.append(packet)
                self.line = bytearray()
                self.invalid = False
            elif byte < 32 or byte > 126 or len(self.line) >= 95:
                self.invalid = True
            elif not self.invalid:
                self.line.append(byte)
        return packets

    @staticmethod
    def decode(line):
        try:
            payload, check = line.rsplit(b":", 1)
            if len(check) != 4 or any(byte not in b"0123456789ABCDEF" for byte in check):
                return None
            if crc16(payload) != int(check, 16):
                return None
            fields = payload.split(b":")
            if len(fields) == 3 and fields[0] == b"QRQ":
                if any(not field or any(byte < 48 or byte > 57 for byte in field) for field in fields[1:]):
                    return None
                token, enabled = [int(field) for field in fields[1:]]
                if 1 <= token <= 65535 and enabled in (0, 1):
                    return "QR", token, enabled
                return None
            if len(fields) != 4 or fields[0] != b"AVQ":
                return None
            if any(not field or any(byte < 48 or byte > 57 for byte in field) for field in fields[1:]):
                return None
            token, region, confirm = [int(field) for field in fields[1:]]
            if 1 <= token <= 65535 and 0 <= region <= 9 and confirm in (0, 1):
                return token, region, confirm
        except ValueError:
            pass
        return None


def front_distance(bottom):
    rows = (17, 23, 72, 104, 127)
    distances = (235, 280, 365, 370, 490)
    value = 479 - bottom
    if not rows[0] <= value <= rows[-1]:
        return None
    for index in range(1, len(rows)):
        if value <= rows[index]:
            return distances[index - 1] + (value - rows[index - 1]) * (
                distances[index] - distances[index - 1]) // (rows[index] - rows[index - 1])


class ObservationWindow:
    """Confirmation applies to stationary requests; moving detections are immediate."""
    def __init__(self):
        from obstacle_detection import Confirmation
        self.found = Confirmation()
        self.missing = 0

    def update(self, blob, confirm):
        if blob is None:
            self.found.update(None)
            self.missing = min(3, self.missing + 1)
            return bool(confirm and self.missing >= 3)
        self.missing = 0
        stable = self.found.update(blob)
        return bool(confirm and stable)
