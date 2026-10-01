#ifndef ROUTE_PLAN_H
#define ROUTE_PLAN_H
#include <stdint.h>
#include <math.h>

/* Map coordinates in mm: origin bottom left, +X right, +Y up.
 * Headings: 0 up, 1 left, 2 down, -1 right, 4 keep previous heading.
 * No wheel/ground odometry: progress is an estimate from commanded RPM.
 * Replace scales with measured actual_mm / nominal_mm on the test surface. */
#define ROUTE_FORWARD_SCALE 1.0f
/* Six 100 RPM center tests averaged 103.9 cm vs 105 cm nominal at 90%. */
#define ROUTE_LATERAL_SCALE 0.9f
#define ROUTE_MM_PER_REV    314.159265f /* pi * 100 mm, direct drive G=1 */
#define ROUTE_RPM           200
#define ROUTE_LATERAL_RPM_MAX 150U
#define ROUTE_ACCEL_RPM_S   120.0f
#define ROUTE_TURN_RPM      230.0f
#define ROUTE_TURN_ACCEL    90.0f
/* Initial navigation heading PID gains; tune against measured IMU response. */
#define ROUTE_HEADING_KP    2.0f
#define ROUTE_HEADING_KI    0.25f
#define ROUTE_HEADING_KD    0.12f
#define ROUTE_HEADING_MAX_RPM 12.0f
#define ROUTE_HEADING_INTEGRAL_LIMIT 12.0f
#define ROUTE_COUNT         16U
typedef struct { int16_t x, y; const char *event; int8_t heading; } RoutePoint;
static const RoutePoint routeTemplate[ROUTE_COUNT] = {
    {2100,2250,0,4}, {2100,1200,"QR",-1}, {2100,2080,0,4},
    {1200,2080,"RAW_1",1}, {1200,400,"COARSE_1_DROP_PICK",-1},
    {1200,1200,0,4}, {400,1200,"TEMP_1_DROP",2}, {1200,1200,0,4},
    {1200,2080,"RAW_2",1}, {1200,400,"COARSE_2_DROP_PICK",-1},
    {1200,1200,0,4}, {400,1200,"TEMP_2_STACK",2}, {1200,1200,0,4},
    {2100,1200,0,0}, {2100,2250,0,4}, {2250,2250,"HOME",4}
};
static RoutePoint routePoint(uint8_t index, uint8_t start)
{
    RoutePoint p = routeTemplate[index];
    if (start == 2U && (index == 0U || index >= 14U)) p.y = 150;
    return p;
}
static float routeAbs(float value) { return value < 0.0f ? -value : value; }
typedef struct {
    float integral;
    float filteredRate;
    float previousYaw;
    uint32_t previousSampleMs;
    uint8_t initialized;
} RouteHeadingPid;
typedef struct {
    float kp;
    float ki;
    float kd;
} RouteHeadingPidGains;
static void routeHeadingPidReset(RouteHeadingPid *pid)
{
    pid->integral = pid->filteredRate = pid->previousYaw = 0.0f;
    pid->previousSampleMs = 0U;
    pid->initialized = 0U;
}
static float routeHeadingPidStep(RouteHeadingPid *pid,
                                 const RouteHeadingPidGains *gains,
                                 float error, float yaw, uint32_t sampleMs,
                                 uint32_t dt, int8_t sign)
{
    float yawChange, sampleDt, candidate, proportional, output;
    if (!pid->initialized) {
        pid->previousYaw = yaw;
        pid->previousSampleMs = sampleMs;
        pid->initialized = 1U;
    } else if (sampleMs != pid->previousSampleMs) {
        sampleDt = (float)(sampleMs - pid->previousSampleMs);
        yawChange = yaw - pid->previousYaw;
        while (yawChange > 180.0f) yawChange -= 360.0f;
        while (yawChange < -180.0f) yawChange += 360.0f;
        pid->filteredRate += sampleDt / (100.0f + sampleDt) *
            (yawChange * 1000.0f / sampleDt - pid->filteredRate);
        pid->previousYaw = yaw;
        pid->previousSampleMs = sampleMs;
    }
    proportional = routeAbs(error) < 0.6f ? 0.0f : error;
    if (proportional != 0.0f && routeAbs(error) <= 8.0f) {
        candidate = pid->integral + error * dt / 1000.0f;
        if (candidate > ROUTE_HEADING_INTEGRAL_LIMIT)
            candidate = ROUTE_HEADING_INTEGRAL_LIMIT;
        if (candidate < -ROUTE_HEADING_INTEGRAL_LIMIT)
            candidate = -ROUTE_HEADING_INTEGRAL_LIMIT;
        output = gains->kp * proportional + gains->ki * candidate -
                 gains->kd * pid->filteredRate;
        if ((output <= ROUTE_HEADING_MAX_RPM && output >= -ROUTE_HEADING_MAX_RPM) ||
            (output > ROUTE_HEADING_MAX_RPM && error < 0.0f) ||
            (output < -ROUTE_HEADING_MAX_RPM && error > 0.0f))
            pid->integral = candidate;
    }
    output = gains->kp * proportional + gains->ki * pid->integral -
             gains->kd * pid->filteredRate;
    if (output > ROUTE_HEADING_MAX_RPM) output = ROUTE_HEADING_MAX_RPM;
    if (output < -ROUTE_HEADING_MAX_RPM) output = -ROUTE_HEADING_MAX_RPM;
    return output * sign;
}
/* Convert map-axis RPM to body-axis RPM at a cardinal heading. */
static void routeBody(int16_t mapUp, int16_t mapRight, int8_t heading,
                      int16_t *forward, int16_t *right)
{
    switch (heading) {
    case 1: *forward = -mapRight; *right = mapUp; break;
    case 2: *forward = -mapUp; *right = -mapRight; break;
    case -1: *forward = mapRight; *right = -mapUp; break;
    default: *forward = mapUp; *right = mapRight; break;
    }
}
typedef struct {
    float previousError;
    float integral;
    float output;
    uint32_t sampleMs;
    uint8_t initialized;
} RouteTurnPid;

static void routeTurnPidReset(RouteTurnPid *pid)
{
    pid->previousError = pid->integral = pid->output = 0.0f;
    pid->sampleMs = 0U;
    pid->initialized = 0U;
}

/* yyb_stm32 profiles 0/3 use 50 ms error differences and a 7 RPM integral limit. */
static float routeTurnPidStep(RouteTurnPid *pid, float error,
                              uint32_t sampleMs, uint16_t maximumRpm, int8_t sign)
{
    float sampleScale = 1.0f;
    float change = 0.0f;
    float kd = routeAbs(error) > 15.0f ? 0.8f : 0.4f;
    if (pid->initialized) {
        if (sampleMs == pid->sampleMs) return pid->output * sign;
        sampleScale = (sampleMs - pid->sampleMs) / 50.0f;
        change = error - pid->previousError;
        while (change > 180.0f) change -= 360.0f;
        while (change < -180.0f) change += 360.0f;
    }
    if (routeAbs(error) <= 15.0f) {
        pid->integral += error * 0.05f * sampleScale;
        if (pid->integral > 7.0f) pid->integral = 7.0f;
        if (pid->integral < -7.0f) pid->integral = -7.0f;
    } else {
        pid->integral = 0.0f;
    }
    pid->output = error * 2.0f + kd * change / sampleScale + pid->integral;
    if (pid->output > maximumRpm) pid->output = maximumRpm;
    if (pid->output < -(float)maximumRpm) pid->output = -(float)maximumRpm;
    pid->previousError = error;
    pid->sampleMs = sampleMs;
    pid->initialized = 1U;
    return pid->output * sign;
}

static float routeTurnRamp(float current, float target, uint32_t dt)
{
    /* Braking and reversal must not keep the previous turn command alive. */
    if (current * target <= 0.0f || routeAbs(target) < routeAbs(current)) {
        if (current * target < 0.0f) current = 0.0f;
        else if (routeAbs(target) < routeAbs(current)) return target;
    }
    {
        float step = ROUTE_TURN_ACCEL * dt / 1000.0f;
        if (target > current + step) return current + step;
        if (target < current - step) return current - step;
    }
    return target;
}
/* Positive lateral means right. IDs: FR=1 FL=2 RL=3 RR=4.
 * X roller layout, matching the verified chassis_position mapping. */
static int16_t routeWheel(uint8_t id, int16_t forward, int16_t right,
                          int16_t turn, uint16_t trim)
{
    int32_t rpm = forward + ((id == 1U || id == 3U) ? -right : right)
                           + ((id == 1U || id == 4U) ? turn : -turn);
    return (int16_t)(rpm * trim / 1000);
}
static float routeEstimate(int16_t rpm, uint32_t dt, uint8_t lateral)
{
    return rpm * (ROUTE_MM_PER_REV / 60000.0f) * dt *
           (lateral ? ROUTE_LATERAL_SCALE : ROUTE_FORWARD_SCALE);
}
static float routeEstimateScaled(int16_t rpm, uint32_t dt, float scale)
{
    return rpm * (ROUTE_MM_PER_REV / 60000.0f) * dt * scale;
}
/* Slew in physical time; fractional state avoids low-speed quantization stalls. */
static float routeSlew(float current, float target, float rate, uint32_t dt)
{
    float step = rate * dt / 1000.0f;
    if (target > current + step) return current + step;
    if (target < current - step) return current - step;
    return target;
}
static int16_t routeRound(float value)
{
    return (int16_t)(value >= 0 ? value + 0.5f : value - 0.5f);
}
/* Cubic smooth-start + constant-deceleration braking envelope.
 * Unlike linear remaining-distance P control, this avoids a long crawl tail. */
static int16_t routeSpeed(float remaining, uint32_t elapsed,
                          uint16_t maximumRpm)
{
    float t = elapsed < 700U ? elapsed / 700.0f : 1.0f;
    float speed = maximumRpm * t * t * (3.0f - 2.0f * t);
    float brake = sqrtf(2.0f * ROUTE_ACCEL_RPM_S * (remaining > 0 ? remaining : 0) /
                        (ROUTE_MM_PER_REV / 60.0f));
    if (speed > brake) speed = brake;
    return routeRound(speed);
}
#endif
