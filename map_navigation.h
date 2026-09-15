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

#endif
