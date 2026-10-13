"""ASCII line sent from K230 UART1 to STM32 USART3."""

MAX_PAYLOAD_BYTES = 48


def encode_qr_result(payload):
    if not isinstance(payload, str):
        return None
    clean = "".join(" " if ord(char) < 32 or ord(char) == 127 else char
                    for char in payload).strip()
    data = clean.encode("utf-8")
    if not data or len(data) > MAX_PAYLOAD_BYTES:
        return None
    return b"QR:" + data + b"\n"
