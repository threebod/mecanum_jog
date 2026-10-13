#include <assert.h>
#include "../obstacle_stop.h"
#include "../qr_scan.h"

static unsigned feed(ObstacleStopParser *parser, const char *line)
{
    unsigned count = 0U;
    while (*line) count += obstacle_stop_feed(parser, (uint8_t)*line++);
    return count;
}

int main(void)
{
    ObstacleStopParser parser = {0};
    QrScanParser qr = {0};
    char result[QR_SCAN_PAYLOAD_MAX + 1U];
    const char *line = "OB:STOP\nQR:156+123+516+231\n";
    unsigned stops = 0U, codes = 0U;
    assert(feed(&parser, "OB:ST") == 0U);
    assert(feed(&parser, "OP\r\n") == 1U);
    assert(feed(&parser, "QR:OB:STOP\nOB:STOPX\nnoiseOB:STOP\n") == 0U);
    assert(feed(&parser, "OB:ST\001OP\nOB:STOP\n") == 1U);
    while (*line) {
        stops += obstacle_stop_feed(&parser, (uint8_t)*line);
        codes += qr_scan_feed(&qr, (uint8_t)*line++, result);
    }
    assert(stops == 1U && codes == 1U);
    return 0;
}
