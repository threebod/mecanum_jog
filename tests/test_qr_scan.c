#include <assert.h>
#include <string.h>

#include "../qr_scan.h"

static unsigned feed(QrScanParser *parser, const char *line, char *result)
{
    unsigned count = 0U;
    while (*line != '\0') count += qr_scan_feed(parser, (uint8_t)*line++, result);
    return count;
}

int main(void)
{
    QrScanParser parser;
    char result[QR_SCAN_PAYLOAD_MAX + 1U] = {0};
    char longLine[QR_SCAN_PAYLOAD_MAX + 10U];
    qr_scan_init(&parser);
    assert(feed(&parser, "QR:green-yellow-blue\r", result) == 0U);
    assert(feed(&parser, "\n", result) == 1U);
    assert(strcmp(result, "green-yellow-blue") == 0);
    assert(feed(&parser, "noise\nQR:\nQR:426\n", result) == 1U);
    assert(strcmp(result, "426") == 0);
    assert(feed(&parser, "QR:bad\001data\n", result) == 0U);
    assert(feed(&parser, "QR:recovered\n", result) == 1U);
    assert(strcmp(result, "recovered") == 0);
    memset(longLine, 'X', sizeof(longLine));
    memcpy(longLine, "QR:", 3U);
    longLine[sizeof(longLine) - 2U] = '\n';
    longLine[sizeof(longLine) - 1U] = '\0';
    assert(feed(&parser, longLine, result) == 0U);
    assert(feed(&parser, "QR:ok\n", result) == 1U);
    assert(strcmp(result, "ok") == 0);
    return 0;
}
