#ifndef MECANUM_JOG_QR_SCAN_H
#define MECANUM_JOG_QR_SCAN_H

#include <stdint.h>

#define QR_SCAN_PAYLOAD_MAX 48U

typedef struct {
    char line[QR_SCAN_PAYLOAD_MAX + 4U];
    uint8_t length;
    uint8_t invalid;
} QrScanParser;

static void qr_scan_init(QrScanParser *parser)
{
    parser->length = 0U;
    parser->invalid = 0U;
}

/* K230 sends QR:<UTF-8 payload>\n. Only display data reaches this port. */
static uint8_t qr_scan_feed(QrScanParser *parser, uint8_t byte,
                            char result[QR_SCAN_PAYLOAD_MAX + 1U])
{
    uint8_t index;
    if (byte == '\r') return 0U;
    if (byte == '\n') {
        uint8_t valid = !parser->invalid && parser->length > 3U &&
                        parser->line[0] == 'Q' && parser->line[1] == 'R' &&
                        parser->line[2] == ':';
        if (valid) {
            for (index = 3U; index < parser->length; ++index)
                result[index - 3U] = parser->line[index];
            result[parser->length - 3U] = '\0';
        }
        qr_scan_init(parser);
        return valid;
    }
    if (byte < 0x20U || byte == 0x7FU ||
        parser->length >= sizeof(parser->line) || parser->invalid) {
        parser->invalid = 1U;
        return 0U;
    }
    parser->line[parser->length++] = (char)byte;
    return 0U;
}

#endif
