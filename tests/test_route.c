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
/* Swept circle radius ceil(sqrt(150^2+150^2)) = 213 mm.
 * Includes the full rotation, not only the cardinal endpoint footprint. */
static void circleClear(float x, float y, float x0, float y0, float x1, float y1)
{
    float dx = x < x0 ? x0-x : x > x1 ? x-x1 : 0;
    float dy = y < y0 ? y0-y : y > y1 ? y-y1 : 0;
    assert(dx*dx + dy*dy > 213.0f*213.0f);
}
static void turnClear(float x, float y)
{
    unsigned i,j;
    const float low[2] = {550,1400};
    assert(x > 213 && y > 213 && x < 2187 && y < 2187);
    for (i=0;i<2;++i) for (j=0;j<2;++j)
        circleClear(x,y,low[i],low[j],low[i]+450,low[j]+450);
    circleClear(x,y,0,910,150,1490);
    circleClear(x,y,910,0,1490,150);
    circleClear(x,y,1050,2315,1350,2400); /* conservative raw disk envelope */
}
int main(void)
{
    unsigned start, i, sample, events;
    float x, y;
    RoutePoint p;
    int16_t forward, right;
    int8_t heading;
    for (start = 1; start <= 2; ++start) {
        x = 2250; y = start == 1 ? 2250 : 150; events = 0;
        heading = 0;
        for (i = 0; i < ROUTE_COUNT; ++i) {
            p = routePoint((uint8_t)i, (uint8_t)start);
            assert(p.x == x || p.y == y); /* no diagonal corner cutting */
            for (sample = 0; sample <= 100; ++sample)
                clearAt(x + (p.x-x)*sample/100.0f, y + (p.y-y)*sample/100.0f);
            if (p.event) ++events;
            if (p.heading != 4 && p.heading != heading) {
                turnClear(p.x,p.y); heading = p.heading;
            }
            x = p.x; y = p.y;
        }
        assert(events == 8); /* QR + 3 stations twice + home */
        assert(x == 2250 && y == (start == 1 ? 2250 : 150));
        assert(heading == 0); /* restore nose up BEFORE entering start square */
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
    routeBody(20,0,1,&forward,&right); assert(forward==0 && right==20);
    routeBody(20,0,-1,&forward,&right); assert(forward==0 && right==-20);
    routeBody(0,20,1,&forward,&right); assert(forward==-20 && right==0);
    routeBody(0,20,-1,&forward,&right); assert(forward==20 && right==0);
    routeBody(20,10,2,&forward,&right); assert(forward==-20 && right==-10);
    routeBody(20,10,0,&forward,&right); assert(forward==20 && right==10);
    assert(routeTurnSpeed(90,1)==10 && routeTurnSpeed(-90,1)==-10);
    assert(routeTurnSpeed(90,-1)==-10 && routeTurnSpeed(-90,-1)==10);
    assert(routeTurnSpeed(3,1)==2);
    puts("PASS: both route geometries, station counts, wheel mixing and distance units");
    return 0;
}
