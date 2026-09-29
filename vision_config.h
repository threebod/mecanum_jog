#ifndef VISION_CONFIG_H
#define VISION_CONFIG_H

#include "vision_control.h"

/* Material scales are rough ring-alignment references. Apply them only after
 * checking direction and pickup anchor; reset keeps auto motion locked. */
static VisionCalibration visionMaterialCalibration = {
    0U, {0, 0, 320, 240}, 160, 120,
    {-0.640f, 0.0f, 0.0f, -0.673f}
};

static VisionCalibration visionRingCalibration[3] = {
    {0U, {0, 0, 320, 240}, 160, 120, {0.0f, 0.0f, 0.0f, 0.0f}},
    {0U, {0, 0, 320, 240}, 160, 120, {0.0f, 0.0f, 0.0f, 0.0f}},
    {0U, {0, 0, 320, 240}, 160, 120, {0.0f, 0.0f, 0.0f, 0.0f}}
};

#endif
