#ifndef MECANUM_AVOIDANCE_PROTOCOL_H
#define MECANUM_AVOIDANCE_PROTOCOL_H
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define AV_LINE_MAX 96U
typedef struct { uint16_t token; uint8_t region, seen, stable; uint16_t u, bottom; uint32_t pixels; } AvObservation;
typedef struct { char line[AV_LINE_MAX]; uint8_t length, invalid; } AvParser;

static uint16_t av_crc16(const char *data, unsigned length)
{
    uint16_t crc = 0xFFFFU; unsigned i, bit;
    for (i = 0; i < length; ++i) {
        crc ^= (uint16_t)(uint8_t)data[i] << 8;
        for (bit = 0; bit < 8; ++bit) crc = (uint16_t)((crc & 0x8000U) ? ((uint32_t)crc << 1) ^ 0x1021U : (uint32_t)crc << 1);
    }
    return crc;
}
static uint8_t av_number(const char **cursor, uint32_t limit, uint32_t *value)
{
    const char *p = *cursor; uint32_t n = 0;
    if (*p < '0' || *p > '9') return 0;
    while (*p >= '0' && *p <= '9') {
        unsigned digit = (unsigned)(*p++ - '0');
        if (n > limit / 10U || (n == limit / 10U && digit > limit % 10U)) return 0;
        n = n * 10U + digit;
    }
    if (*p != ':') return 0;
    *cursor = p + 1; *value = n; return 1;
}
static uint8_t av_decode(const char *line, AvObservation *packet)
{
    uint32_t values[7]; static const uint32_t limits[7] = {65535,9,1,1,639,479,307200};
    const char *p = line + 4; unsigned i; uint16_t crc = 0;
    if (strncmp(line, "AVR:", 4) != 0) return 0;
    for (i = 0; i < 7; ++i) if (!av_number(&p, limits[i], &values[i])) return 0;
    if (strlen(p) != 4) return 0;
    for (i = 0; i < 4; ++i) {
        unsigned n = p[i] >= '0' && p[i] <= '9' ? (unsigned)(p[i]-'0') :
                     p[i] >= 'A' && p[i] <= 'F' ? (unsigned)(p[i]-'A'+10) : 16;
        if (n > 15) return 0;
        crc = (uint16_t)((crc << 4) | n);
    }
    if (!values[0] || crc != av_crc16(line, (unsigned)(p - line - 1))) return 0;
    if ((!values[2] && (values[4] || values[5] || values[6])) ||
        (values[2] && (values[5] < 160 || values[6] < 2200))) return 0;
    packet->token = (uint16_t)values[0]; packet->region = (uint8_t)values[1];
    packet->seen = (uint8_t)values[2]; packet->stable = (uint8_t)values[3];
    packet->u = (uint16_t)values[4]; packet->bottom = (uint16_t)values[5]; packet->pixels = values[6];
    return 1;
}
static uint8_t av_parser_feed(AvParser *parser, uint8_t byte, AvObservation *packet)
{
    if (byte == '\r') return 0;
    if (byte == '\n') {
        uint8_t valid;
        parser->line[parser->length] = 0;
        valid = (uint8_t)(!parser->invalid && av_decode(parser->line, packet));
        parser->length = parser->invalid = 0; return valid;
    }
    if (byte < 32 || byte > 126 || parser->length >= AV_LINE_MAX - 1 || parser->invalid) parser->invalid = 1;
    else parser->line[parser->length++] = (char)byte;
    return 0;
}
static unsigned av_encode_request(char *line, uint16_t token, uint8_t region, uint8_t confirm)
{
    unsigned size = (unsigned)sprintf(line, "AVQ:%u:%u:%u", (unsigned)token, (unsigned)region, (unsigned)confirm);
    return size + (unsigned)sprintf(line + size, ":%04X\n", (unsigned)av_crc16(line, size));
}
#endif
