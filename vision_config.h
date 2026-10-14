#ifndef VISION_CONFIG_H
#define VISION_CONFIG_H

#include "vision_control.h"
#include "vision_defaults.h"

/* Material and ring 2 have independent, user-confirmed default scales. */
static VisionCalibration visionMaterialCalibration = {
    0U, {0, 0, 320, 240}, 160, 120,
    {-VISION_MATERIAL_FORWARD_MILLI / 1000.0f, 0.0f, 0.0f,
     -VISION_MATERIAL_RIGHT_MILLI / 1000.0f}
};

static VisionCalibration visionRingCalibration[3] = {
    {0U, {0, 0, 320, 240}, 160, 120, {0.0f, 0.0f, 0.0f, 0.0f}},
    {0U, {0, 0, 320, 240}, 160, 120,
     {-VISION_RING2_FORWARD_MILLI / 1000.0f, 0.0f, 0.0f,
      -VISION_RING2_RIGHT_MILLI / 1000.0f}},
    {0U, {0, 0, 320, 240}, 160, 120, {0.0f, 0.0f, 0.0f, 0.0f}}
};

#endif
