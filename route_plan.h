#ifndef ROUTE_PLAN_H
#define ROUTE_PLAN_H
#include <stdint.h>

/* Map coordinates in mm: origin bottom left, +X right, +Y up.
 * Robot nose stays up throughout. These are staging points, not grasp poses.
 * No wheel/ground odometry: progress is an estimate from commanded RPM.
 * Replace scales with measured actual_mm / nominal_mm on the test surface. */
#define ROUTE_FORWARD_SCALE 1.0f
#define ROUTE_LATERAL_SCALE 1.0f
#define ROUTE_MM_PER_REV    314.159265f /* pi * 100 mm, direct drive G=1 */
#define ROUTE_RPM           20
#define ROUTE_COUNT         16U
typedef struct { int16_t x, y; const char *event; } RoutePoint;
static const RoutePoint routeTemplate[ROUTE_COUNT] = {
    {2100,2250,0}, {2100,1200,"QR"}, {2100,2100,0},
    {1200,2100,"RAW_1"}, {1200,350,"COARSE_1_DROP_PICK"},
    {1200,1200,0}, {350,1200,"TEMP_1_DROP"}, {1200,1200,0},
    {1200,2100,"RAW_2"}, {1200,350,"COARSE_2_DROP_PICK"},
    {1200,1200,0}, {350,1200,"TEMP_2_STACK"}, {1200,1200,0},
    {2100,1200,0}, {2100,2250,0}, {2250,2250,"HOME"}
};
static RoutePoint routePoint(uint8_t index, uint8_t start)
{
    RoutePoint p = routeTemplate[index];
    if (start == 2U && (index == 0U || index >= 14U)) p.y = 150;
    return p;
}
static float routeAbs(float value) { return value < 0.0f ? -value : value; }
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
/* Low speed at both ends. Hardware commands are integer RPM. */
static int16_t routeSpeed(float remaining, uint32_t elapsed)
{
    int16_t speed = ROUTE_RPM;
    int16_t ramp = (int16_t)(elapsed < 600U ? elapsed * ROUTE_RPM / 600U : ROUTE_RPM);
    if (remaining < 60.0f) speed = (int16_t)(remaining * ROUTE_RPM / 60.0f);
    if (speed > ramp) speed = ramp;
    if (speed < 3) speed = 3;
    return speed;
}
#endif
