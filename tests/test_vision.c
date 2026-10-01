#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "../vision_control.h"

static int require(int condition, const char *message)
{
    if (!condition) {
        fprintf(stderr, "%s\n", message);
        return 0;
    }
    return 1;
}

int main(void)
{
    VisionPacket request;
    VisionPacket decoded;
    VisionParser parser;
    VisionCalibration calibration = {
        1U, {0, 0, 320, 240}, 160, 120,
        {0.5f, 0.0f, 0.0f, -0.25f}
    };
    uint8_t frame[VISION_FRAME_SIZE];
    float forward;
    float right;
    VisionSession session;

    if (!require(vision_crc16((const uint8_t *)"123456789", 9U) == 0x29B1U,
                 "CRC16 vector mismatch")) return 1;

    memset(&request, 0, sizeof(request));
    request.type = VISION_MESSAGE_REQUEST;
    request.token = 0x12345678UL;
    request.mode = VISION_MODE_MATERIAL;
    request.selector = 3U;
    request.value[0] = 10;
    request.value[1] = 20;
    request.value[2] = 200;
    request.value[3] = 100;
    request.value[4] = 160;
    request.value[5] = 120;
    vision_packet_encode(&request, frame);
    vision_parser_init(&parser);
    if (!require(vision_parser_feed(&parser, frame, sizeof(frame), &decoded),
                 "valid packet was not decoded") ||
        !require(decoded.token == request.token && decoded.selector == 3U &&
                     decoded.value[3] == 100,
                 "decoded packet differs from input")) return 1;
    frame[12] ^= 0x01U;
    vision_parser_init(&parser);
    if (!require(!vision_parser_feed(&parser, frame, sizeof(frame), &decoded),
                 "CRC-corrupt packet was accepted")) return 1;

    if (!require(vision_calibration_valid(&calibration),
                 "valid calibration was rejected")) return 1;
    vision_map_error(&calibration, 20, -8, &forward, &right);
    if (!require(fabsf(forward - 10.0f) < 0.001f &&
                     fabsf(right - 2.0f) < 0.001f,
                 "pixel-to-body mapping is incorrect")) return 1;
    vision_limit_step(&forward, &right, 5.0f);
    if (!require(fabsf(forward - 5.0f) < 0.001f &&
                     fabsf(right - 1.0f) < 0.001f,
                 "step limit did not preserve direction")) return 1;
    {
        VisionCalibration colorGate = {
            1U, {0, 0, 320, 240}, 160, 120,
            {0.0f, 0.0f, 0.0f, 0.0f}
        };
        VisionSession pickup;
        uint16_t index;
        vision_session_init(&pickup);
        if (!require(vision_session_start(&pickup, VISION_MODE_MATERIAL,
                                          3U, 0U, &colorGate, 0U),
                     "color-gated pickup session did not start")) return 1;
        for (index = 0U; index < 5U; ++index) {
            vision_session_make_request(&pickup, &colorGate, &request, index);
            decoded = request;
            decoded.type = VISION_MESSAGE_RESULT;
            decoded.flags = 0U;
            decoded.value[2] = 0;
            if (!require(vision_session_wait_for_material(&pickup, &decoded) &&
                         pickup.state == VISION_STATE_REQUEST &&
                         pickup.misses == 0U,
                         "absent material must wait without failing")) return 1;
        }
        for (index = 0U; index < 3U; ++index) {
            vision_session_make_request(&pickup, &colorGate, &request,
                                        (uint32_t)(index + 5U));
            decoded = request;
            decoded.type = VISION_MESSAGE_RESULT;
            decoded.flags = index == 2U ?
                (VISION_FLAG_VALID | VISION_FLAG_STABLE) : 0U;
            decoded.value[2] = index == 2U ? 30 : 80;
            if (!require(vision_session_wait_for_material(&pickup, &decoded) &&
                         pickup.state == VISION_STATE_REQUEST &&
                         pickup.misses == 0U,
                         "unstable or low-quality material must keep waiting"))
                return 1;
        }
        vision_session_make_request(&pickup, &colorGate, &request, 8U);
        decoded = request;
        decoded.type = VISION_MESSAGE_RESULT;
        decoded.token -= 1U;
        if (!require(!vision_session_wait_for_material(&pickup, &decoded) &&
                     pickup.state == VISION_STATE_WAIT,
                     "stale packet must not renew material wait")) return 1;
        decoded = request;
        decoded.type = VISION_MESSAGE_RESULT;
        decoded.flags = 0U;
        decoded.value[2] = 0;
        if (!require(vision_session_wait_for_material(&pickup, &decoded),
                     "current absent packet did not renew material wait")) return 1;
        for (index = 0U; index < 2U; ++index) {
            vision_session_make_request(&pickup, &colorGate, &request, index);
            decoded = request;
            decoded.type = VISION_MESSAGE_RESULT;
            decoded.flags = VISION_FLAG_VALID | VISION_FLAG_STABLE;
            decoded.value[0] = 90;
            decoded.value[1] = 75;
            decoded.value[2] = 80;
            if (!require(!vision_session_wait_for_material(&pickup, &decoded) &&
                         vision_session_observe(&pickup, &colorGate, &decoded,
                                                index) ==
                             (index == 0U ? VISION_EVENT_REQUEST :
                                            VISION_EVENT_ALIGNED) &&
                             pickup.forwardMm == 0.0f &&
                             pickup.rightMm == 0.0f,
                         "color detection requested chassis motion")) return 1;
        }
    }
    calibration.calibrated = 0U;
    if (!require(!vision_calibration_valid(&calibration),
                 "uncalibrated profile was accepted")) return 1;

    calibration.calibrated = 1U;
    vision_session_init(&session);
    if (!require(vision_session_start(&session, VISION_MODE_MATERIAL, 2U, 0U,
                                      &calibration, 100U),
                 "calibrated session did not start")) return 1;
    vision_session_make_request(&session, &calibration, &request, 110U);
    memset(&decoded, 0, sizeof(decoded));
    decoded.type = VISION_MESSAGE_RESULT;
    decoded.token = request.token;
    decoded.mode = request.mode;
    decoded.selector = request.selector;
    decoded.flags = VISION_FLAG_VALID | VISION_FLAG_STABLE;
    decoded.value[0] = 180;
    decoded.value[1] = 112;
    decoded.value[2] = 90;
    if (!require(vision_session_observe(&session, &calibration, &decoded, 200U) ==
                     VISION_EVENT_MOVE &&
                     session.forwardMm == 10.0f && session.rightMm == 2.0f,
                 "valid observation did not request mapped movement")) return 1;
    vision_session_move_complete(&session, 300U);
    if (!require(vision_session_tick(&session, 599U) == VISION_EVENT_NONE &&
                     vision_session_tick(&session, 600U) == VISION_EVENT_REQUEST,
                 "settle interval is incorrect")) return 1;
    vision_session_make_request(&session, &calibration, &request, 600U);
    decoded.token = request.token;
    decoded.value[0] = 162;
    decoded.value[1] = 120;
    if (!require(vision_session_observe(&session, &calibration, &decoded, 700U) ==
                     VISION_EVENT_REQUEST,
                 "first in-tolerance observation completed too early")) return 1;
    vision_session_make_request(&session, &calibration, &request, 710U);
    decoded.token = request.token;
    if (!require(vision_session_observe(&session, &calibration, &decoded, 800U) ==
                     VISION_EVENT_ALIGNED,
                 "two in-tolerance observations did not align")) return 1;

    {
        uint32_t previousToken = request.token;
        if (!require(vision_session_start(&session, VISION_MODE_MATERIAL, 2U, 0U,
                                          &calibration, 900U),
                     "second session did not start")) return 1;
        vision_session_make_request(&session, &calibration, &request, 910U);
        if (!require(request.token > previousToken,
                     "token was reused across sessions")) return 1;
        decoded.token = previousToken;
        if (!require(vision_session_observe(&session, &calibration, &decoded, 920U) ==
                         VISION_EVENT_NONE && session.state == VISION_STATE_WAIT,
                     "stale result affected a new session")) return 1;
    }

    {
        VisionSession limited;
        VisionCalibration identity = {
            1U, {0, 0, 320, 240}, 160, 120,
            {1.0f, 0.0f, 0.0f, 1.0f}
        };
        uint8_t correction;
        vision_session_init(&limited);
        vision_session_start(&limited, VISION_MODE_RING, 0U, 1U,
                             &identity, 0U);
        for (correction = 0U; correction < 3U; ++correction) {
            vision_session_make_request(&limited, &identity, &request,
                                        (uint32_t)correction * 10U);
            decoded = request;
            decoded.type = VISION_MESSAGE_RESULT;
            decoded.flags = VISION_FLAG_VALID | VISION_FLAG_STABLE;
            decoded.value[0] = 260;
            decoded.value[1] = 220;
            decoded.value[2] = 90;
            if (correction < 2U) {
                if (!require(vision_session_observe(&limited, &identity, &decoded,
                                                    10U) == VISION_EVENT_MOVE &&
                                 limited.forwardMm == 20.0f &&
                                 limited.rightMm == 20.0f,
                             "20 mm axis limit was not applied")) return 1;
                vision_session_move_complete(&limited, 10U);
                vision_session_tick(&limited, 310U);
            } else if (!require(
                           vision_session_observe(&limited, &identity, &decoded,
                                                  20U) == VISION_EVENT_FAILED &&
                               strcmp(limited.fault, "CUMULATIVE_LIMIT") == 0,
                           "100 mm cumulative limit was not enforced")) {
                return 1;
            }
        }
    }

    {
        VisionSession timed;
        uint8_t miss;
        vision_session_init(&timed);
        vision_session_start(&timed, VISION_MODE_MATERIAL, 1U, 0U,
                             &calibration, 0U);
        for (miss = 0U; miss < 3U; ++miss) {
            vision_session_make_request(&timed, &calibration, &request,
                                        (uint32_t)miss * 2000U);
            if (miss < 2U) {
                if (!require(vision_session_tick(&timed,
                                                 (uint32_t)miss * 2000U + 1800U) ==
                                 VISION_EVENT_REQUEST,
                             "camera timeout did not retry")) return 1;
            } else if (!require(
                           vision_session_tick(&timed, 5800U) == VISION_EVENT_FAILED &&
                               strcmp(timed.fault, "CAMERA_TIMEOUT") == 0,
                           "three camera timeouts did not fail")) {
                return 1;
            }
        }
        vision_session_pause(&timed);
        if (!require(timed.state == VISION_STATE_PAUSED,
                     "pause did not enter PAUSED")) return 1;
    }

    {
        VisionSession waiting;
        uint16_t attempt;
        vision_session_init(&waiting);
        vision_session_start(&waiting, VISION_MODE_RING, 0U, 2U,
                             &calibration, 0U);
        for (attempt = 0U; attempt < 300U; ++attempt) {
            vision_session_make_request(&waiting, &calibration, &request, attempt);
            decoded = request;
            decoded.type = VISION_MESSAGE_RESULT;
            decoded.flags = 0U;
            if (!require(vision_session_observe(&waiting, &calibration, &decoded,
                                                attempt) == VISION_EVENT_REQUEST &&
                         waiting.misses == 0U && waiting.iteration == 0U &&
                         waiting.cumulativeMm == 0.0f,
                         "missing ring must keep retrying without motion")) return 1;
        }
        for (attempt = 0U; attempt < 3U; ++attempt) {
            vision_session_make_request(&waiting, &calibration, &request, attempt);
            decoded = request;
            decoded.type = VISION_MESSAGE_RESULT;
            decoded.flags = attempt == 1U ? 0U : VISION_FLAG_VALID | VISION_FLAG_STABLE;
            decoded.value[0] = calibration.anchorU;
            decoded.value[1] = calibration.anchorV;
            decoded.value[2] = 90;
            if (!require(vision_session_observe(&waiting, &calibration, &decoded,
                                                attempt) == VISION_EVENT_REQUEST,
                         "alignment confirmations must be consecutive")) return 1;
        }
        for (attempt = 0U; attempt < 3U; ++attempt) {
            vision_session_make_request(&waiting, &calibration, &request,
                                        attempt * 2000U);
            if (!require(vision_session_tick(&waiting, attempt * 2000U + 1800U) ==
                             (attempt < 2U ? VISION_EVENT_REQUEST : VISION_EVENT_FAILED),
                         "ring wait must still fail on communication timeout")) return 1;
        }
        vision_session_start(&waiting, VISION_MODE_MATERIAL, 4U, 2U,
                             &calibration, 0U);
        waiting.waitForTarget = 1U;
        for (attempt = 0U; attempt < 300U; ++attempt) {
            vision_session_make_request(&waiting, &calibration, &request, attempt);
            decoded = request;
            decoded.type = VISION_MESSAGE_RESULT;
            decoded.flags = 0U;
            if (!require(vision_session_observe(&waiting, &calibration, &decoded,
                                                attempt) == VISION_EVENT_REQUEST &&
                         waiting.misses == 0U && waiting.iteration == 0U,
                         "missing storage material must keep waiting")) return 1;
        }
        for (attempt = 0U; attempt < 2U; ++attempt) {
            vision_session_make_request(&waiting, &calibration, &request, attempt);
            decoded = request;
            decoded.type = VISION_MESSAGE_RESULT;
            decoded.flags = VISION_FLAG_VALID | VISION_FLAG_STABLE;
            decoded.value[0] = calibration.anchorU;
            decoded.value[1] = calibration.anchorV;
            decoded.value[2] = 90;
            if (!require(vision_session_observe(&waiting, &calibration, &decoded,
                                                attempt) ==
                             (attempt == 0U ? VISION_EVENT_REQUEST : VISION_EVENT_ALIGNED),
                         "storage material must align after retrying")) return 1;
        }
    }

    {
        VisionSession iterations;
        uint8_t correction;
        vision_session_init(&iterations);
        vision_session_start(&iterations, VISION_MODE_MATERIAL, 1U, 0U,
                             &calibration, 0U);
        for (correction = 0U; correction < 13U; ++correction) {
            vision_session_make_request(&iterations, &calibration, &request,
                                        correction);
            decoded = request;
            decoded.type = VISION_MESSAGE_RESULT;
            decoded.flags = VISION_FLAG_VALID | VISION_FLAG_STABLE;
            decoded.value[0] = 172;
            decoded.value[1] = 120;
            decoded.value[2] = 90;
            if (correction < 12U) {
                if (!require(vision_session_observe(&iterations, &calibration,
                                                    &decoded, correction) ==
                                 VISION_EVENT_MOVE,
                             "correction failed before iteration limit")) return 1;
                vision_session_move_complete(&iterations, correction);
                vision_session_tick(&iterations, correction + 300U);
            } else if (!require(
                           vision_session_observe(&iterations, &calibration,
                                                  &decoded, correction) ==
                                   VISION_EVENT_FAILED &&
                               strcmp(iterations.fault, "NOT_CONVERGED") == 0,
                           "12 correction limit was not enforced")) {
                return 1;
            }
        }
    }
    return 0;
}
