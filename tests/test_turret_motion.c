#include <assert.h>
#include <stdio.h>
#include "../turret_motion.h"

static void checkMove(uint32_t start, uint32_t target, uint16_t speed)
{
    uint32_t delta = target > start ? target - start : start - target;
    uint32_t duration = turretMotionDurationMs(delta, speed);
    uint32_t previous = start, firstStep = 0U, lastStep = 0U, peakStep = 0U;
    uint32_t elapsed;
    assert(turretMotionAngleMdeg(start, target, 0U, duration) == start);
    for (elapsed = 20U; elapsed <= duration; elapsed += 20U) {
        uint32_t angle = turretMotionAngleMdeg(start, target, elapsed, duration);
        uint32_t step = angle > previous ? angle - previous : previous - angle;
        assert(target > start ? angle >= previous && angle <= target :
                               angle <= previous && angle >= target);
        /* One millidegree allowance for rounding to integer angles. */
        assert(step <= (uint32_t)speed * 2U + 1U);
        if (elapsed == 20U) firstStep = step;
        lastStep = step;
        if (step > peakStep) peakStep = step;
        previous = angle;
    }
    assert(previous == target);
    if (duration > 100U && delta > 1000U) {
        assert(firstStep < peakStep / 4U);
        assert(lastStep < peakStep / 4U);
    }
}

int main(void)
{
    assert(turretMotionDurationMs(0U, 1200U) == 0U);
    assert(turretMotionDurationMs(90000U, 1200U) == 1420U);
    assert(turretMotionDurationMs(136000U, 1200U) == 2140U);
    assert(turretMotionAngleMdeg(123U, 123U, 0U, 0U) == 123U);
    checkMove(0U, 90000U, 1200U);
    checkMove(268000U, 132000U, 1200U);
    checkMove(0U, 360000U, 10U);
    checkMove(360000U, 0U, 1800U);
    checkMove(50000U, 50001U, 1800U);
    puts("PASS: turret ramp, peak speed, reverse motion and endpoints");
    return 0;
}
