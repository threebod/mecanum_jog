#include <assert.h>
#include <stdio.h>
#include "../straight_control.h"

int main(void)
{
    uint8_t frame[11] = {0x55, 0x53, 0, 0, 0, 0, 0, 0x40, 0, 0, 0};
    uint8_t id, i;
    float yaw = 0;
    assert(headingError(179, -179) == -2);
    assert(headingError(-179, 179) == 2);
    assert(headingCorrection(30, 1) == 12);
    assert(headingCorrection(30, -1) == -12);
    assert(rampSpeed(0, 2000, 30) == 0);
    assert(rampSpeed(200, 2000, 30) == 15);
    assert(rampSpeed(400, 2000, 30) == 30);
    assert(rampSpeed(1800, 2000, 30) == 15);
    assert(rampSpeed(2000, 2000, 30) == 0);
    assert(rampSpeed(2001, 2000, 30) == 0);
    for (id = 1; id <= 4; ++id) {
        assert(wheelSpeed(id, 30, 0, 1000) == 30);
        assert(wheelSpeed(id, -30, 0, 1000) == -30);
    }
    assert(wheelSpeed(1, 30, 10, 1000) == 40);
    assert(wheelSpeed(2, 30, 10, 1000) == 20);
    assert(wheelSpeed(3, 30, 10, 1000) == 20);
    assert(wheelSpeed(4, 30, 10, 1000) == 40);
    assert(wheelSpeed(1, -30, 0, 1100) == -33);
    for (i = 0; i < 10; ++i) frame[10] += frame[i];
    assert(decodeYaw(frame, &yaw) && yaw == 90);
    frame[10] ^= 1;
    assert(!decodeYaw(frame, &yaw));
    frame[7] = 0xC0;
    frame[10] = 0;
    for (i = 0; i < 10; ++i) frame[10] += frame[i];
    assert(decodeYaw(frame, &yaw) && yaw == -90);
    frame[1] = 0x52;
    assert(!decodeYaw(frame, &yaw));
    puts("PASS: heading wrap/sign, four-wheel mixing, ramps, IMU checksum/yaw");
    return 0;
}
