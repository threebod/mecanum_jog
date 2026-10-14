#include <assert.h>
#include <stdio.h>
#include "../route_plan.h"

int main(void)
{
    unsigned start, i, events;
    RouteLeg leg;
    RouteHeadingPid headingPid;
    const RouteHeadingPidGains gains = {
        ROUTE_HEADING_KP, ROUTE_HEADING_KI, ROUTE_HEADING_KD
    };
    int16_t forward, right;
    int8_t heading;
    assert(routeDistanceLeg(0U, 1U).rightMm == -200);
    assert(routeDistanceLeg(3U, 1U).rightMm == -900);
    assert(routeDistanceLeg(0U, 2U).rightMm == -150);
    assert(routeDistanceLeg(3U, 2U).rightMm == -900);
    assert(routeDistanceLeg(0U, 1U).rightMm +
           routeDistanceLeg(3U, 1U).rightMm == -1100);
    assert(routeDistanceLeg(2U, 1U).upMm == 830);
    assert(routeDistanceLeg(2U, 2U).upMm == 825);
    assert(routeDistanceLeg(9U, 1U).upMm == 880);
    assert(routeDistanceLeg(9U, 2U).upMm == 890);
    for (start = 1; start <= 2; ++start) {
        events = 0; heading = 0;
        for (i = 0; i < ROUTE_COUNT; ++i) {
            leg = routeDistanceLeg((uint8_t)i, (uint8_t)start);
            assert((leg.upMm == 0) != (leg.rightMm == 0));
            assert(routeAbs(leg.upMm) <= 2400 && routeAbs(leg.rightMm) <= 2400);
            if (leg.event) ++events;
            if (leg.departureHeading != 4) heading = leg.departureHeading;
            if (i == 2U) assert(heading == 0 && leg.departureHeading == 0 && leg.heading == 1);
            if (i == 4U || i == 11U) assert(heading == 1 && leg.rightMm == -830 && leg.heading == 2 && !leg.event);
            if (i == 5U || i == 12U) assert(heading == 2 && leg.upMm == -1720 && leg.heading == -1 && !leg.event);
            if (i == 6U || i == 13U) assert(heading == -1 && leg.rightMm == 860 && leg.heading == -1 && leg.event);
            if (i == 8U || i == 15U) assert(heading == 2 && leg.upMm == 850);
            if (i == 10U) assert(heading == 1 && leg.rightMm == 850);
            if (i == 16U) assert(heading == 2 && leg.heading == -1 &&
                                leg.upMm == (start == 1 ? 880 : -850));
            if (i >= 17U) assert(heading == -1 && leg.heading == 4);
            if (i == 17U) assert(leg.rightMm == 1700);
            if (i == 18U) assert(leg.upMm == (start == 1 ? 220 : -200));
            if (i == 19U) assert(leg.rightMm == 150);
            if (leg.heading != 4) heading = leg.heading;
        }
        assert(events == 8);
        assert(heading == -1);
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
    /* One revolution is 3200 / 9.9 mm after the verified line calibration. */
    assert(routeAbs(routeEstimate(60,1000,0)-323.232323f) < 0.02f);
    assert(routeAbs(routeEstimate(-60,1000,0)+323.232323f) < 0.02f);
    assert((uint32_t)(500.0f / ROUTE_FORWARD_SCALE *
                     3200.0f / ROUTE_MM_PER_REV + 0.5f) == 4950U);
    assert(routeAbs(routeEstimate(-60,1000,1)+
                    314.159265f*ROUTE_LATERAL_SCALE) < 0.001f);
    assert(routeAbs(routeEstimateScaled(-60,1000,0.925f)+
                    314.159265f*0.925f) < 0.001f);
    assert(routeAbs(ROUTE_LATERAL_SCALE-0.9f) < 0.0001f);
    assert(ROUTE_LATERAL_RPM_MAX == 150U);
    routeHeadingPidReset(&headingPid);
    assert(routeHeadingPidStep(&headingPid, &gains, 0, 179, 100, 20, 1) == 0);
    assert(routeHeadingPidStep(&headingPid, &gains, -2, -179, 200, 20, 1) < -4);
    routeHeadingPidReset(&headingPid);
    for (i = 0; i < 100; ++i)
        assert(routeHeadingPidStep(&headingPid, &gains, 20, 0, i * 20, 20, 1) == 12);
    assert(headingPid.integral == 0); /* no windup while saturated */
    routeHeadingPidReset(&headingPid);
    assert(routeHeadingPidStep(&headingPid, &gains, 5, 0, 100, 100, 1) > 10);
    assert(routeHeadingPidStep(&headingPid, &gains, 4, 1, 200, 100, 1) < 8);
    assert(headingPid.integral > 0);
    assert(routeHeadingPidStep(&headingPid, &gains, 4, 1, 200, 20, -1) < 0);
    routeHeadingPidReset(&headingPid);
    assert(headingPid.integral == 0 && !headingPid.initialized);
    /* Small noise stays quiet; deviations below the old 0.6-degree threshold
     * now produce correction in either direction and accumulate integral. */
    assert(routeHeadingPidStep(&headingPid, &gains, 0.29f, 0, 100, 20, 1) == 0);
    assert(routeHeadingPidStep(&headingPid, &gains, -0.29f, 0, 120, 20, 1) == 0);
    assert(headingPid.integral == 0);
    assert(routeRound(routeHeadingPidStep(&headingPid, &gains,
                                         0.4f, 0, 140, 20, 1)) == 1);
    assert(headingPid.integral > 0);
    routeHeadingPidReset(&headingPid);
    assert(routeRound(routeHeadingPidStep(&headingPid, &gains,
                                         -0.4f, 0, 100, 20, 1)) == -1);
    assert(headingPid.integral < 0);
    assert(routeSpeed(1000,1000,120) == 120);
    assert(routeSpeed(5,1000,120) < routeSpeed(100,1000,120));
    routeBody(20,0,1,&forward,&right); assert(forward==0 && right==20);
    routeBody(20,0,-1,&forward,&right); assert(forward==0 && right==-20);
    routeBody(0,20,1,&forward,&right); assert(forward==-20 && right==0);
    routeBody(0,20,-1,&forward,&right); assert(forward==20 && right==0);
    routeBody(20,10,2,&forward,&right); assert(forward==-20 && right==-10);
    routeBody(20,10,0,&forward,&right); assert(forward==20 && right==10);
    {
        RouteTurnPid turnPid;
        float output;
        routeTurnPidReset(&turnPid);
        assert(routeTurnPidStep(&turnPid, 180, 0, 230, 1) == 230);
        assert(turnPid.integral == 0);
        routeTurnPidReset(&turnPid);
        assert(routeTurnPidStep(&turnPid, 10, 100, 230, 1) > 20);
        output = routeTurnPidStep(&turnPid, 5, 150, 230, 1);
        assert(output < 10); /* decreasing error supplies derivative braking */
        assert(routeTurnPidStep(&turnPid, 5, 150, 230, 1) == output);
        assert(routeTurnPidStep(&turnPid, 5, 150, 230, -1) == -output);
        assert(routeTurnPidStep(&turnPid, -4, 200, 230, 1) < 0);
        routeTurnPidReset(&turnPid);
        routeTurnPidStep(&turnPid, 179, 100, 230, 1);
        assert(routeTurnPidStep(&turnPid, -179, 150, 230, 1) == -230);
        assert(routeTurnRamp(100, 5, 20) == 5);
        assert(routeTurnRamp(100, 0, 20) == 0);
        assert(routeTurnRamp(100, -5, 20) < 0);
        assert(routeTurnRamp(0, 100, 20) < 2);
    }
    assert(routeSpeed(1000,0,60)==0);
    assert(routeSpeed(1000,350,60)==30);
    assert(routeSpeed(5,1000,60)>3); /* no long minimum-speed crawl */
    assert(routeAbs(routeSlew(0,60,120,20)-2.4f)<0.001f);
    assert(routeAbs(routeSlew(30,-30,90,20)-28.2f)<0.001f);
    assert(routeRound(-1.6f)==-2 && routeRound(1.6f)==2);
    puts("PASS: both distance routes, station counts, wheel mixing and distance units");
    return 0;
}
