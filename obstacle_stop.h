#ifndef MECANUM_JOG_OBSTACLE_STOP_H
#define MECANUM_JOG_OBSTACLE_STOP_H

#include <stdint.h>

typedef struct {
    uint8_t length;
    uint8_t invalid;
} ObstacleStopParser;

/* Exact ASCII line only; QR payloads containing OB:STOP must not stop the car. */
static uint8_t obstacle_stop_feed(ObstacleStopParser *parser, uint8_t byte)
{
    static const char message[] = "OB:STOP";
    if (byte == '\r') return 0U;
    if (byte == '\n') {
        uint8_t valid = !parser->invalid && parser->length == sizeof(message) - 1U;
        parser->length = parser->invalid = 0U;
        return valid;
    }
    if (parser->invalid) return 0U;
    if (parser->length >= sizeof(message) - 1U || byte != (uint8_t)message[parser->length]) {
        parser->invalid = 1U;
    } else {
        ++parser->length;
    }
    return 0U;
}

#endif
