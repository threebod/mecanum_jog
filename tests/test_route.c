#include <assert.h>
#include <stdio.h>
#include "../route_plan.h"

/* Sample the swept chassis footprint against the map's four forbidden squares
 * and the two projecting work tables. The selected route never rotates. */
static void clearAt(float x, float y)
{
    unsigned i, j;
    const float low[2] = {550,1400};
    assert(x >= 150 && x <= 2250 && y >= 150 && y <= 2250);
    for (i = 0; i < 2; ++i) for (j = 0; j < 2; ++j)
        assert(x + 150 <= low[i] || x - 150 >= low[i] + 450 ||
               y + 150 <= low[j] || y - 150 >= low[j] + 450);
    assert(x - 150 >= 150 || y + 150 <= 910 || y - 150 >= 1490);
    assert(y - 150 >= 150 || x + 150 <= 910 || x - 150 >= 1490);
}
int main(void)
{
    unsigned start, i, sample, events;
    float x, y;
    RoutePoint p;
    for (start = 1; start <= 2; ++start) {
        x = 2250; y = start == 1 ? 2250 : 150; events = 0;
        for (i = 0; i < ROUTE_COUNT; ++i) {
            p = routePoint((uint8_t)i, (uint8_t)start);
            assert(p.x == x || p.y == y); /* no diagonal corner cutting */
            for (sample = 0; sample <= 100; ++sample)
                clearAt(x + (p.x-x)*sample/100.0f, y + (p.y-y)*sample/100.0f);
            if (p.event) ++events;
            x = p.x; y = p.y;
        }
        assert(events == 8); /* QR + 3 stations twice + home */
        assert(x == 2250 && y == (start == 1 ? 2250 : 150));
    }
    assert(routeWheel(1,20,0,0,1000) == 20);
    assert(routeWheel(2,20,0,0,1000) == 20);
    assert(routeWheel(3,20,0,0,1000) == 20);
    assert(routeWheel(4,20,0,0,1000) == 20);
    assert(routeWheel(1,0,20,0,1000) == -20);
    assert(routeWheel(2,0,20,0,1000) == 20);
    assert(routeWheel(3,0,20,0,1000) == -20);
    assert(routeWheel(4,0,20,0,1000) == 20);
    assert(routeWheel(1,0,0,5,1000) == 5);
    assert(routeWheel(2,0,0,5,1000) == -5);
    assert(routeWheel(3,0,0,5,1000) == -5);
    assert(routeWheel(4,0,0,5,1000) == 5);
    assert(routeAbs(routeEstimate(60,1000,0)-314.159265f) < 0.001f);
    assert(routeAbs(routeEstimate(-60,1000,1)+314.159265f) < 0.001f);
    assert(routeSpeed(1000,1000) == 20);
    assert(routeSpeed(5,1000) < routeSpeed(100,1000));
    puts("PASS: both route geometries, station counts, wheel mixing and distance units");
    return 0;
}
