#include <assert.h>
#include <stdio.h>

#include "../map_navigation.h"

static void assertClear(float x, float y)
{
    unsigned int i;
    unsigned int j;
    const float low[2] = {550.0f, 1400.0f};

    if (x < 150.0f || x > 2250.0f || y < 150.0f || y > 2250.0f) {
        fprintf(stderr, "unsafe boundary center: x=%.1f y=%.1f\n", x, y);
    }
    assert(x >= 150.0f && x <= 2250.0f);
    assert(y >= 150.0f && y <= 2250.0f);
    for (i = 0U; i < 2U; ++i) {
        for (j = 0U; j < 2U; ++j) {
            assert(x + 150.0f <= low[i] || x - 150.0f >= low[i] + 450.0f ||
                   y + 150.0f <= low[j] || y - 150.0f >= low[j] + 450.0f);
        }
    }
    assert(x - 150.0f >= 150.0f || y + 150.0f <= 910.0f ||
           y - 150.0f >= 1490.0f);
    assert(y - 150.0f >= 150.0f || x + 150.0f <= 910.0f ||
           x - 150.0f >= 1490.0f);
}

static void assertEdgeClear(NavPoint from, NavPoint to)
{
    unsigned int sample;

    assert(from.x == to.x || from.y == to.y);
    for (sample = 0U; sample <= 100U; ++sample) {
        assertClear(from.x + (to.x - from.x) * (float)sample / 100.0f,
                    from.y + (to.y - from.y) * (float)sample / 100.0f);
    }
}

int main(void)
{
    uint8_t from;
    uint8_t to;
    uint8_t path[NAV_NODE_COUNT];
    uint8_t count;
    NavPoint route[NAV_PATH_CAPACITY];
    NavPoint repeated[NAV_PATH_CAPACITY];
    NavPoint arbitrary = {300, 300, NAV_HEADING_KEEP};
    NavPoint second = {1800, 300, NAV_HEADING_KEEP};

    assert(navStartNode(1U) == NAV_START_1);
    assert(navStartNode(2U) == NAV_START_2);
    assert(navStartNode(0U) == NAV_INVALID_NODE);
    assert(navNodeAt(2100, 1200) == NAV_QR);
    assert(navNodeAt(1200, 2080) == NAV_RAW);
    assert(navNodeAt(1201, 2080) == NAV_INVALID_NODE);
    assert(navPoint(NAV_QR).arrivalHeading == NAV_HEADING_RIGHT);
    assert(navPoint(NAV_RAW).arrivalHeading == NAV_HEADING_UP);
    assert(navPoint(NAV_COARSE).arrivalHeading == NAV_HEADING_DOWN);
    assert(navPoint(NAV_TEMP).arrivalHeading == NAV_HEADING_LEFT);
    assert(navPointClear(150, 150));
    assert(navPointClear(2250, 2250));
    assert(!navPointClear(149, 150));
    assert(!navPointClear(700, 700));
    assert(navSegmentClear(navPoint(NAV_GATE_1), navPoint(NAV_UPPER_RIGHT)));
    assert(!navSegmentClear((NavPoint){300, 700, NAV_HEADING_KEEP},
                            (NavPoint){1200, 700, NAV_HEADING_KEEP}));

    count = navShortestPath(NAV_START_1, NAV_RAW, path, NAV_NODE_COUNT);
    assert(count == 4U);
    assert(path[0] == NAV_START_1 && path[1] == NAV_GATE_1 &&
           path[2] == NAV_UPPER_RIGHT && path[3] == NAV_RAW);

    count = navShortestPath(NAV_START_2, NAV_TEMP, path, NAV_NODE_COUNT);
    assert(count == 5U);
    assert(path[0] == NAV_START_2 && path[1] == NAV_GATE_2 &&
           path[2] == NAV_QR && path[3] == NAV_CENTER &&
           path[4] == NAV_TEMP);

    assert(navShortestPath(NAV_INVALID_NODE, NAV_TEMP, path,
                           NAV_NODE_COUNT) == 0U);
    assert(navShortestPath(NAV_START_1, NAV_TEMP, path, 2U) == 0U);

    for (from = 0U; from < NAV_NODE_COUNT; ++from) {
        for (to = 0U; to < NAV_NODE_COUNT; ++to) {
            uint8_t index;
            count = navShortestPath(from, to, path, NAV_NODE_COUNT);
            assert(count > 0U);
            assert(path[0] == from && path[count - 1U] == to);
            for (index = 1U; index < count; ++index) {
                assert(navNodesAdjacent(path[index - 1U], path[index]));
                assertEdgeClear(navPoint(path[index - 1U]),
                                navPoint(path[index]));
            }
        }
    }

    count = navPlanPath(navPoint(NAV_START_1), arbitrary, route,
                        NAV_PATH_CAPACITY);
    assert(count > 0U);
    assert(route[count - 1U].x == arbitrary.x &&
           route[count - 1U].y == arbitrary.y &&
           route[count - 1U].arrivalHeading == NAV_HEADING_KEEP);
    for (from = 1U; from < count; ++from) {
        assert(navSegmentClear(route[from - 1U], route[from]));
    }
    assert(navPlanPath(arbitrary, second, route, NAV_PATH_CAPACITY) > 0U);
    count = navPlanPath(navPoint(NAV_START_2), navPoint(NAV_COARSE), route,
                        NAV_PATH_CAPACITY);
    assert(count > 0U &&
           route[count - 1U].arrivalHeading == NAV_HEADING_DOWN);
    assert(navPlanPath(navPoint(NAV_START_1),
                       (NavPoint){700, 700, NAV_HEADING_KEEP}, route,
                       NAV_PATH_CAPACITY) == 0U);
    count = navPlanPath(navPoint(NAV_START_1), arbitrary, route,
                        NAV_PATH_CAPACITY);
    assert(navPlanPath(navPoint(NAV_START_1), arbitrary, repeated,
                       NAV_PATH_CAPACITY) == count);
    for (from = 0U; from < count; ++from) {
        assert(route[from].x == repeated[from].x &&
               route[from].y == repeated[from].y);
    }

    puts("PASS: navigation nodes, deterministic paths and swept clearance");
    return 0;
}
