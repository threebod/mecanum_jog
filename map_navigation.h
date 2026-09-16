#ifndef MAP_NAVIGATION_H
#define MAP_NAVIGATION_H

#include <stdint.h>

#define NAV_NODE_COUNT 10U
#define NAV_INVALID_NODE 0xFFU
#define NAV_HEADING_RIGHT (-1)
#define NAV_HEADING_UP 0
#define NAV_HEADING_LEFT 1
#define NAV_HEADING_DOWN 2
#define NAV_HEADING_KEEP 4
#define NAV_PATH_CAPACITY 16U

typedef enum {
    NAV_START_1 = 0,
    NAV_GATE_1,
    NAV_UPPER_RIGHT,
    NAV_QR,
    NAV_START_2,
    NAV_GATE_2,
    NAV_RAW,
    NAV_CENTER,
    NAV_COARSE,
    NAV_TEMP
} NavNode;

typedef struct {
    int16_t x;
    int16_t y;
    int8_t arrivalHeading;
} NavPoint;

static const NavPoint navPoints[NAV_NODE_COUNT] = {
    {2250, 2250, NAV_HEADING_UP},
    {2100, 2250, NAV_HEADING_KEEP},
    {2100, 2080, NAV_HEADING_KEEP},
    {2100, 1200, NAV_HEADING_RIGHT},
    {2250, 150, NAV_HEADING_UP},
    {2100, 150, NAV_HEADING_KEEP},
    {1200, 2080, NAV_HEADING_UP},
    {1200, 1200, NAV_HEADING_KEEP},
    {1200, 400, NAV_HEADING_DOWN},
    {400, 1200, NAV_HEADING_LEFT}
};

static NavPoint navPoint(uint8_t node)
{
    NavPoint invalid = {0, 0, NAV_HEADING_KEEP};
    return node < NAV_NODE_COUNT ? navPoints[node] : invalid;
}

static uint8_t navStartNode(uint8_t zone)
{
    if (zone == 1U) return NAV_START_1;
    if (zone == 2U) return NAV_START_2;
    return NAV_INVALID_NODE;
}

static uint8_t navNodeAt(int16_t x, int16_t y)
{
    uint8_t node;
    for (node = 0U; node < NAV_NODE_COUNT; ++node) {
        if (navPoints[node].x == x && navPoints[node].y == y) return node;
    }
    return NAV_INVALID_NODE;
}

static uint8_t navNodesAdjacent(uint8_t first, uint8_t second)
{
    static const uint8_t edges[][2] = {
        {NAV_START_1, NAV_GATE_1},
        {NAV_GATE_1, NAV_UPPER_RIGHT},
        {NAV_UPPER_RIGHT, NAV_QR},
        {NAV_UPPER_RIGHT, NAV_RAW},
        {NAV_QR, NAV_GATE_2},
        {NAV_GATE_2, NAV_START_2},
        {NAV_QR, NAV_CENTER},
        {NAV_RAW, NAV_CENTER},
        {NAV_CENTER, NAV_COARSE},
        {NAV_CENTER, NAV_TEMP}
    };
    uint8_t edge;
    for (edge = 0U; edge < (uint8_t)(sizeof(edges) / sizeof(edges[0])); ++edge) {
        if ((edges[edge][0] == first && edges[edge][1] == second) ||
            (edges[edge][0] == second && edges[edge][1] == first)) return 1U;
    }
    return 0U;
}

static uint16_t navEdgeDistance(uint8_t first, uint8_t second)
{
    int16_t dx;
    int16_t dy;
    if (!navNodesAdjacent(first, second)) return 0xFFFFU;
    dx = (int16_t)(navPoints[first].x - navPoints[second].x);
    dy = (int16_t)(navPoints[first].y - navPoints[second].y);
    return (uint16_t)((dx < 0 ? -dx : dx) + (dy < 0 ? -dy : dy));
}

static uint8_t navShortestPath(uint8_t start, uint8_t target,
                               uint8_t *path, uint8_t capacity)
{
    uint16_t distance[NAV_NODE_COUNT];
    uint8_t previous[NAV_NODE_COUNT];
    uint8_t visited[NAV_NODE_COUNT] = {0U};
    uint8_t reverse[NAV_NODE_COUNT];
    uint8_t node;
    uint8_t count = 0U;

    if (start >= NAV_NODE_COUNT || target >= NAV_NODE_COUNT || path == 0) return 0U;
    for (node = 0U; node < NAV_NODE_COUNT; ++node) {
        distance[node] = 0xFFFFU;
        previous[node] = NAV_INVALID_NODE;
    }
    distance[start] = 0U;

    for (;;) {
        uint8_t candidate = NAV_INVALID_NODE;
        uint16_t best = 0xFFFFU;
        uint8_t neighbor;
        for (node = 0U; node < NAV_NODE_COUNT; ++node) {
            if (!visited[node] && distance[node] < best) {
                best = distance[node];
                candidate = node;
            }
        }
        if (candidate == NAV_INVALID_NODE || candidate == target) break;
        visited[candidate] = 1U;
        for (neighbor = 0U; neighbor < NAV_NODE_COUNT; ++neighbor) {
            uint16_t edge = navEdgeDistance(candidate, neighbor);
            uint16_t total;
            if (edge == 0xFFFFU || visited[neighbor]) continue;
            total = (uint16_t)(distance[candidate] + edge);
            if (total < distance[neighbor]) {
                distance[neighbor] = total;
                previous[neighbor] = candidate;
            }
        }
    }
    if (distance[target] == 0xFFFFU) return 0U;

    node = target;
    for (;;) {
        if (count >= NAV_NODE_COUNT) return 0U;
        reverse[count++] = node;
        if (node == start) break;
        node = previous[node];
        if (node == NAV_INVALID_NODE) return 0U;
    }
    if (count > capacity) return 0U;
    for (node = 0U; node < count; ++node) path[node] = reverse[count - node - 1U];
    return count;
}

static uint8_t navPointClear(int16_t x, int16_t y)
{
    static const int16_t obstacleOrigin[2] = {550, 1400};
    uint8_t ix;
    uint8_t iy;
    if (x < 150 || x > 2250 || y < 150 || y > 2250) return 0U;
    for (ix = 0U; ix < 2U; ++ix) {
        for (iy = 0U; iy < 2U; ++iy) {
            if (x > obstacleOrigin[ix] - 150 &&
                x < obstacleOrigin[ix] + 600 &&
                y > obstacleOrigin[iy] - 150 &&
                y < obstacleOrigin[iy] + 600) return 0U;
        }
    }
    if (x < 300 && y > 760 && y < 1640) return 0U;
    if (y < 300 && x > 760 && x < 1640) return 0U;
    return 1U;
}

static uint8_t navSegmentClear(NavPoint from, NavPoint to)
{
    int16_t x = from.x;
    int16_t y = from.y;
    int16_t step;
    if ((from.x != to.x && from.y != to.y) ||
        !navPointClear(from.x, from.y) || !navPointClear(to.x, to.y)) return 0U;
    if (from.x != to.x) {
        step = from.x < to.x ? 1 : -1;
        while (x != to.x) {
            x = (int16_t)(x + step);
            if (!navPointClear(x, y)) return 0U;
        }
    } else {
        step = from.y < to.y ? 1 : -1;
        while (y != to.y) {
            y = (int16_t)(y + step);
            if (!navPointClear(x, y)) return 0U;
        }
    }
    return 1U;
}

typedef struct {
    NavPoint points[2];
    uint8_t count;
} NavConnector;

static uint8_t navConnector(NavPoint from, NavPoint to, uint8_t verticalFirst,
                            NavConnector *connector)
{
    NavPoint bend;
    if (connector == 0 || !navPointClear(from.x, from.y) ||
        !navPointClear(to.x, to.y)) return 0U;
    connector->count = 0U;
    if (from.x == to.x || from.y == to.y) {
        if (!navSegmentClear(from, to)) return 0U;
    } else {
        bend.x = verticalFirst ? from.x : to.x;
        bend.y = verticalFirst ? to.y : from.y;
        bend.arrivalHeading = NAV_HEADING_KEEP;
        if (!navSegmentClear(from, bend) || !navSegmentClear(bend, to)) return 0U;
        connector->points[connector->count++] = bend;
    }
    connector->points[connector->count++] = to;
    return 1U;
}

static uint8_t navAppendPoint(NavPoint *path, uint8_t *count,
                              uint8_t capacity, NavPoint point)
{
    if (*count > 0U && path[*count - 1U].x == point.x &&
        path[*count - 1U].y == point.y) {
        path[*count - 1U].arrivalHeading = point.arrivalHeading;
        return 1U;
    }
    if (*count >= 2U) {
        NavPoint before = path[*count - 2U];
        NavPoint previous = path[*count - 1U];
        if ((before.x == previous.x && previous.x == point.x) ||
            (before.y == previous.y && previous.y == point.y)) {
            path[*count - 1U] = point;
            return 1U;
        }
    }
    if (*count >= capacity) return 0U;
    path[(*count)++] = point;
    return 1U;
}

static uint32_t navManhattan(NavPoint first, NavPoint second)
{
    int32_t dx = (int32_t)first.x - second.x;
    int32_t dy = (int32_t)first.y - second.y;
    return (uint32_t)(dx < 0 ? -dx : dx) +
           (uint32_t)(dy < 0 ? -dy : dy);
}

static uint8_t navPlanPath(NavPoint start, NavPoint target,
                           NavPoint *path, uint8_t capacity)
{
    uint8_t entry;
    uint8_t exitNode;
    uint8_t startOrder;
    uint8_t targetOrder;
    uint8_t bestCount = 0U;
    uint32_t bestDistance = 0xFFFFFFFFUL;
    NavPoint best[NAV_PATH_CAPACITY];
    uint8_t targetNode;
    if (path == 0 || capacity == 0U || capacity > NAV_PATH_CAPACITY ||
        !navPointClear(start.x, start.y) ||
        !navPointClear(target.x, target.y)) return 0U;
    targetNode = navNodeAt(target.x, target.y);
    target.arrivalHeading = targetNode == NAV_INVALID_NODE ?
                            NAV_HEADING_KEEP : navPoint(targetNode).arrivalHeading;
    if (start.x == target.x && start.y == target.y) {
        path[0] = target;
        return 1U;
    }
    for (entry = 0U; entry < NAV_NODE_COUNT; ++entry) {
        for (exitNode = 0U; exitNode < NAV_NODE_COUNT; ++exitNode) {
            uint8_t graph[NAV_NODE_COUNT];
            uint8_t graphCount = navShortestPath(entry, exitNode, graph,
                                                 NAV_NODE_COUNT);
            uint16_t graphDistance = 0U;
            uint8_t graphIndex;
            if (graphCount == 0U) continue;
            for (graphIndex = 1U; graphIndex < graphCount; ++graphIndex) {
                graphDistance = (uint16_t)(graphDistance +
                    navEdgeDistance(graph[graphIndex - 1U], graph[graphIndex]));
            }
            for (startOrder = 0U; startOrder < 2U; ++startOrder) {
                NavConnector startConnector;
                if (!navConnector(start, navPoint(entry), startOrder,
                                  &startConnector)) continue;
                for (targetOrder = 0U; targetOrder < 2U; ++targetOrder) {
                    NavConnector targetConnector;
                    NavPoint candidate[NAV_PATH_CAPACITY];
                    uint8_t candidateCount = 0U;
                    uint8_t index;
                    uint32_t distance;
                    if (!navConnector(navPoint(exitNode), target, targetOrder,
                                      &targetConnector)) continue;
                    distance = navManhattan(start, navPoint(entry)) +
                               graphDistance +
                               navManhattan(navPoint(exitNode), target);
                    if (distance >= bestDistance) continue;
                    for (index = 0U; index < startConnector.count; ++index) {
                        if (!navAppendPoint(candidate, &candidateCount, capacity,
                                            startConnector.points[index])) break;
                    }
                    if (index != startConnector.count) continue;
                    for (index = 1U; index < graphCount; ++index) {
                        NavPoint point = navPoint(graph[index]);
                        point.arrivalHeading = NAV_HEADING_KEEP;
                        if (!navAppendPoint(candidate, &candidateCount, capacity,
                                            point)) break;
                    }
                    if (index != graphCount) continue;
                    for (index = 0U; index < targetConnector.count; ++index) {
                        NavPoint point = targetConnector.points[index];
                        if (index + 1U < targetConnector.count)
                            point.arrivalHeading = NAV_HEADING_KEEP;
                        if (!navAppendPoint(candidate, &candidateCount, capacity,
                                            point)) break;
                    }
                    if (index != targetConnector.count || candidateCount == 0U)
                        continue;
                    for (index = 0U; index < candidateCount; ++index)
                        best[index] = candidate[index];
                    bestCount = candidateCount;
                    bestDistance = distance;
                }
            }
        }
    }
    if (bestCount == 0U) return 0U;
    for (entry = 0U; entry < bestCount; ++entry) path[entry] = best[entry];
    return bestCount;
}

#endif
