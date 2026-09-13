#ifndef STRAIGHT_CONTROL_H
#define STRAIGHT_CONTROL_H
#include <stdint.h>

/* Pure control math, also compiled by the host regression test. */
static float headingError(float target, float actual)
{
    float error = target - actual;
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;
    return error;
}

static int16_t headingCorrection(float error, int8_t sign)
{
    float output = error * 2.0f * sign;
    if (output > 12.0f) output = 12.0f;
    if (output < -12.0f) output = -12.0f;
    return (int16_t)output;
}

static uint16_t rampSpeed(uint32_t elapsed, uint32_t duration, uint16_t rpm)
{
    uint32_t edge = elapsed < duration ? duration - elapsed : 0U;
    if (elapsed < edge) edge = elapsed;
    if (edge > 400U) edge = 400U;
    return (uint16_t)(rpm * edge / 400U);
}

static int16_t wheelSpeed(uint8_t id, int16_t forward, int16_t turn,
                          uint16_t trim)
{
    int32_t speed = forward + ((id == 1U || id == 4U) ? turn : -turn);
    return (int16_t)(speed * trim / 1000);
}

static uint8_t decodeYaw(const uint8_t *frame, float *yaw)
{
    uint8_t sum = 0U, i;
    int16_t raw;
    if (frame[0] != 0x55U || frame[1] != 0x53U) return 0U;
    for (i = 0U; i < 10U; ++i) sum = (uint8_t)(sum + frame[i]);
    if (sum != frame[10]) return 0U;
    raw = (int16_t)((uint16_t)frame[6] | ((uint16_t)frame[7] << 8));
    *yaw = raw * (180.0f / 32768.0f);
    return 1U;
}
#endif
