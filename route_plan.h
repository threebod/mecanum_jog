#ifndef ROUTE_PLAN_H
#define ROUTE_PLAN_H
#include <stdint.h>
#include <math.h>

/* Map coordinates in mm: origin bottom left, +X right, +Y up.
 * Headings: 0 up, 1 left, 2 down, -1 right, 4 keep previous heading.
 * No wheel/ground odometry: progress is an estimate from commanded RPM.
 * Replace scales with measured actual_mm / nominal_mm on the test surface. */
#define ROUTE_FORWARD_SCALE 1.0f
#define ROUTE_LATERAL_SCALE 1.0f
#define ROUTE_MM_PER_REV    314.159265f /* pi * 100 mm, direct drive G=1 */
#define ROUTE_RPM           60
#define ROUTE_ACCEL_RPM_S   120.0f
#define ROUTE_TURN_RPM      30.0f
#define ROUTE_TURN_ACCEL    90.0f
#define ROUTE_COUNT         16U
typedef struct { int16_t x, y; const char *event; int8_t heading; } RoutePoint;
static const RoutePoint routeTemplate[ROUTE_COUNT] = {
    {2100,2250,0,4}, {2100,1200,"QR",-1}, {2100,2080,0,4},
    {1200,2080,"RAW_1",0}, {1200,400,"COARSE_1_DROP_PICK",2},
    {1200,1200,0,4}, {400,1200,"TEMP_1_DROP",1}, {1200,1200,0,4},
    {1200,2080,"RAW_2",0}, {1200,400,"COARSE_2_DROP_PICK",2},
    {1200,1200,0,4}, {400,1200,"TEMP_2_STACK",1}, {1200,1200,0,4},
    {2100,1200,0,0}, {2100,2250,0,4}, {2250,2250,"HOME",4}
};
static RoutePoint routePoint(uint8_t index, uint8_t start)
{
    RoutePoint p = routeTemplate[index];
    if (start == 2U && (index == 0U || index >= 14U)) p.y = 150;
    return p;
}
static float routeAbs(float value) { return value < 0.0f ? -value : value; }
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
static int16_t routeTurnSpeed(float error, int8_t sign)
{
    float speed = error * 1.2f;
    if (speed > ROUTE_TURN_RPM) speed = ROUTE_TURN_RPM;
    if (speed < -ROUTE_TURN_RPM) speed = -ROUTE_TURN_RPM;
    return (int16_t)(speed * sign);
}
/* Positive lateral means right. IDs: FR=1 FL=2 RL=3 RR=4.
 * Matches existing W/S/A/D sign table, conventional X roller layout. */
static int16_t routeWheel(uint8_t id, int16_t forward, int16_t right,
                          int16_t turn, uint16_t trim)
{
    int32_t rpm = forward + ((id == 2U || id == 4U) ? right : -right)
                           + ((id == 1U || id == 4U) ? turn : -turn);
    return (int16_t)(rpm * trim / 1000);
}
static float routeEstimate(int16_t rpm, uint32_t dt, uint8_t lateral)
{
    return rpm * (ROUTE_MM_PER_REV / 60000.0f) * dt *
           (lateral ? ROUTE_LATERAL_SCALE : ROUTE_FORWARD_SCALE);
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
static int16_t routeSpeed(float remaining, uint32_t elapsed)
{
    float t = elapsed < 700U ? elapsed / 700.0f : 1.0f;
    float speed = ROUTE_RPM * t * t * (3.0f - 2.0f * t);
    float brake = sqrtf(2.0f * ROUTE_ACCEL_RPM_S * (remaining > 0 ? remaining : 0) /
                        (ROUTE_MM_PER_REV / 60.0f));
    if (speed > brake) speed = brake;
    return routeRound(speed);
}
#endif
