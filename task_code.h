#ifndef MECANUM_JOG_TASK_CODE_H
#define MECANUM_JOG_TASK_CODE_H

#include <stdint.h>
#include <string.h>

typedef struct {
    uint8_t colors[2][3];
    uint8_t rings[2][3];
} MissionTask;

/* Parse into a temporary value so a rejected code cannot change the task. */
static inline uint8_t missionTaskParse(const char *code, MissionTask *task)
{
    MissionTask parsed;
    uint8_t group, item, mask;
    if (!code || !task || strlen(code) != 15U) return 0U;
    for (group = 0U; group < 4U; ++group) {
        mask = 0U;
        if (group < 3U && code[group * 4U + 3U] != '+') return 0U;
        for (item = 0U; item < 3U; ++item) {
            char digit = code[group * 4U + item];
            uint8_t value;
            if (digit < '1' || digit > ((group & 1U) ? '3' : '6')) return 0U;
            value = (uint8_t)(digit - '0');
            if (group & 1U) {
                if (mask & (1U << value)) return 0U;
                mask |= (uint8_t)(1U << value);
                parsed.rings[group / 2U][item] = value;
            } else {
                parsed.colors[group / 2U][item] = value;
            }
        }
    }
    *task = parsed;
    return 1U;
}

/* Group four applies only to coarse processing. Storage follows the colors
 * already placed in the first layer, irrespective of second-batch pickup order. */
static inline uint8_t missionTaskStorageRings(const MissionTask *task,
                                             uint8_t batch, uint8_t rings[3])
{
    uint8_t matched[3], used = 0U, item, first;
    if (!batch) {
        memcpy(rings, task->rings[0], sizeof(task->rings[0]));
        return 1U;
    }
    for (item = 0U; item < 3U; ++item) {
        for (first = 0U; first < 3U; ++first)
            if (!(used & (1U << first)) &&
                task->colors[1][item] == task->colors[0][first]) break;
        if (first == 3U) return 0U;
        used |= (uint8_t)(1U << first);
        matched[item] = task->rings[0][first];
    }
    memcpy(rings, matched, sizeof(matched));
    return 1U;
}

#endif
