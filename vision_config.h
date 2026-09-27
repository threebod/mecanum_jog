#ifndef VISION_CONFIG_H
#define VISION_CONFIG_H

#include "vision_control.h"

/* Replace the zero matrices with nine-point calibration results, then set
 * calibrated to 1. Keeping them invalid prevents uncalibrated auto motion. */
static const VisionCalibration visionMaterialCalibration = {
    0U, {0, 0, 320, 240}, 160, 120,
    {0.0f, 0.0f, 0.0f, 0.0f}
};

static VisionCalibration visionRingCalibration[3] = {
    {0U, {0, 0, 320, 240}, 160, 120, {0.0f, 0.0f, 0.0f, 0.0f}},
    {0U, {0, 0, 320, 240}, 160, 120, {0.0f, 0.0f, 0.0f, 0.0f}},
    {0U, {0, 0, 320, 240}, 160, 120, {0.0f, 0.0f, 0.0f, 0.0f}}
};

#endif
