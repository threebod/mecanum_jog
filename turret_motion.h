#ifndef MECANUM_JOG_TURRET_MOTION_H
#define MECANUM_JOG_TURRET_MOTION_H

#include <stdint.h>

/* Quintic peak slope is 15/8. Round up to whole 20 ms PWM updates.
 * speedDps10 is the peak command speed in 0.1 degrees/second. */
static inline uint32_t turretMotionDurationMs(uint32_t deltaMdeg, uint16_t speedDps10)
{
    uint32_t denominator = (uint32_t)speedDps10 * 16U;
    return ((deltaMdeg * 15U + denominator - 1U) / denominator) * 20U;
}

static inline uint32_t turretMotionAngleMdeg(uint32_t start, uint32_t target,
                                      uint32_t elapsedMs, uint32_t durationMs)
{
    float t, s;
    uint32_t delta, offset;
    if (elapsedMs >= durationMs) return target;
    t = (float)elapsedMs / (float)durationMs;
    /* Evaluate near the closest endpoint to avoid cancellation at t=1. */
    if (t <= 0.5f) {
        s = t * t * t * (10.0f + t * (-15.0f + 6.0f * t));
    } else {
        t = 1.0f - t;
        s = 1.0f - t * t * t * (10.0f + t * (-15.0f + 6.0f * t));
    }
    delta = target > start ? target - start : start - target;
    offset = (uint32_t)((float)delta * s + 0.5f);
    if (offset > delta) offset = delta;
    return target >= start ? start + offset : start - offset;
}

#endif
