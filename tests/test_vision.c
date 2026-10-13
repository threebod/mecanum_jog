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

    {
        uint8_t color;
        int16_t quality;
        for (color = 1U; color <= 6U; ++color) {
            for (quality = 0; quality <= 100; quality += 50) {
                VisionSession qualitySession;
                vision_session_init(&qualitySession);
                vision_session_start(&qualitySession, VISION_MODE_MATERIAL,
                                     color, 0U, &calibration, 0U);
                vision_session_make_request(&qualitySession, &calibration, &request, 0U);
                decoded = request;
                decoded.type = VISION_MESSAGE_RESULT;
                decoded.flags = VISION_FLAG_VALID | VISION_FLAG_STABLE;
                decoded.value[0] = calibration.anchorU;
                decoded.value[1] = calibration.anchorV;
                decoded.value[2] = quality;
                decoded.value[5] = color;
                if (!require(vision_session_wait_for_material(&qualitySession, &decoded) ==
                                 0U,
                             "material quality must not block a valid target")) return 1;
                qualitySession.state = VISION_STATE_WAIT;
                vision_session_observe(&qualitySession, &calibration, &decoded, 0U);
                if (!require(qualitySession.confirmations == 1U,
                             "material quality must remain diagnostic only")) return 1;
            }
        }
        decoded.mode = VISION_MODE_RING;
        decoded.value[5] = 5;
        decoded.value[2] = 59;
        if (!require(!vision_quality_valid(&decoded),
                     "ring quality below 60 must be rejected")) return 1;
        decoded.value[2] = 60;
        if (!require(vision_quality_valid(&decoded),
                     "ring quality at 60 must be accepted")) return 1;
    }

    if (!require(vision_crc16((const uint8_t *)"123456789", 9U) == 0x29B1U,
                 "CRC16 vector mismatch")) return 1;

    vision_session_init(&session);
    if (!require(vision_session_start(&session, VISION_MODE_MATERIAL,
                                      0U, 0U, &calibration, 0U),
                 "any-color alignment session did not start") ||
        !require(!vision_session_start(&session, VISION_MODE_MATERIAL,
                                       7U, 0U, &calibration, 0U),
                 "invalid material selector was accepted")) return 1;
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
            decoded.value[5] = (int16_t)(request.selector ? request.selector : 1U);
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
            decoded.value[5] = (int16_t)(request.selector ? request.selector : 1U);
            decoded.flags = index == 2U ?
                (VISION_FLAG_VALID | VISION_FLAG_STABLE) : 0U;
            decoded.value[2] = index == 2U ? 30 : 80;
            if (index == 2U) decoded.value[5] = 0;
            if (!require(vision_session_wait_for_material(&pickup, &decoded) &&
                         pickup.state == VISION_STATE_REQUEST &&
                         pickup.misses == 0U,
                         "unstable or unknown-color material must keep waiting"))
                return 1;
        }
        vision_session_make_request(&pickup, &colorGate, &request, 8U);
        decoded = request;
        decoded.type = VISION_MESSAGE_RESULT;
        decoded.value[5] = (int16_t)(request.selector ? request.selector : 1U);
        decoded.token -= 1U;
        if (!require(!vision_session_wait_for_material(&pickup, &decoded) &&
                     pickup.state == VISION_STATE_WAIT,
                     "stale packet must not renew material wait")) return 1;
        decoded = request;
        decoded.type = VISION_MESSAGE_RESULT;
        decoded.value[5] = (int16_t)(request.selector ? request.selector : 1U);
        decoded.flags = 0U;
        decoded.value[2] = 0;
        if (!require(vision_session_wait_for_material(&pickup, &decoded),
                     "current absent packet did not renew material wait")) return 1;
        for (index = 0U; index < 2U; ++index) {
            vision_session_make_request(&pickup, &colorGate, &request, index);
            decoded = request;
            decoded.type = VISION_MESSAGE_RESULT;
            decoded.value[5] = (int16_t)(request.selector ? request.selector : 1U);
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
    decoded.value[5] = (int16_t)(request.selector ? request.selector : 1U);
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
            decoded.value[0] = correction == 0U ? 220 : correction == 1U ? 180 : 166;
            decoded.value[1] = correction == 0U ? 130 : correction == 1U ? 115 : 120;
            decoded.value[2] = 90;
            if (correction < 2U) {
                if (!require(vision_session_observe(&limited, &identity, &decoded,
                                                    10U) == VISION_EVENT_MOVE &&
                                 limited.forwardMm == (correction == 0U ? 60.0f : 20.0f) &&
                                 limited.rightMm == (correction == 0U ? 10.0f : -5.0f),
                             "full correction or necessary supplement was clipped")) return 1;
                vision_session_move_complete(&limited, 10U);
                vision_session_tick(&limited, 310U);
            } else if (!require(
                           vision_session_observe(&limited, &identity, &decoded,
                                                   20U) == VISION_EVENT_MOVE &&
                                strcmp(limited.fault, "NONE") == 0 &&
                                limited.cumulativeMm == 101.0f &&
                                limited.forwardMm == 6.0f,
                            "correction stopped after exceeding 100 mm cumulative travel")) {
                return 1;
            }
        }
    }

    {
        VisionSession excessive;
        vision_session_init(&excessive);
        vision_session_start(&excessive, VISION_MODE_MATERIAL, 0U, 0U,
                             &calibration, 0U);
        vision_session_make_request(&excessive, &calibration, &request, 0U);
        decoded = request;
        decoded.type = VISION_MESSAGE_RESULT;
        decoded.value[5] = (int16_t)(request.selector ? request.selector : 1U);
        decoded.flags = VISION_FLAG_VALID | VISION_FLAG_STABLE;
        decoded.value[0] = 0; /* Full first correction totals 110 mm. */
        decoded.value[1] = 0;
        decoded.value[2] = 90;
        if (!require(vision_session_observe(&excessive, &calibration, &decoded, 1U) ==
                         VISION_EVENT_MOVE && excessive.cumulativeMm == 110.0f &&
                         excessive.forwardMm == -80.0f && excessive.rightMm == 30.0f &&
                         strcmp(excessive.fault, "NONE") == 0,
                      "first correction above 100 mm was rejected or clipped")) return 1;
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
            decoded.value[5] = (int16_t)(request.selector ? request.selector : 1U);
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
            decoded.value[5] = (int16_t)(request.selector ? request.selector : 1U);
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
        VisionSession tolerance;
        uint8_t pickupMode;
        static const int16_t du[8] = {14, -14, 14, -14, 15, -15, 0, 0};
        static const int16_t dv[8] = {14, 14, -14, -14, 0, 0, 15, -15};
        uint8_t direction;
        for (pickupMode = 1U; pickupMode <= 2U; ++pickupMode) {
            for (direction = 0U; direction < 8U; ++direction) {
                vision_session_init(&tolerance);
                vision_session_start(&tolerance, VISION_MODE_MATERIAL,
                                     pickupMode == 1U ? 0U : 4U, 0U, &calibration, 0U);
                tolerance.pickupMode = pickupMode;
                tolerance.materialMask = pickupMode == 1U ? 63U : 8U;
                vision_session_make_request(&tolerance, &calibration, &request, 0U);
                decoded = request;
                decoded.type = VISION_MESSAGE_RESULT;
                decoded.flags = VISION_FLAG_VALID | VISION_FLAG_STABLE;
                decoded.value[0] = (int16_t)(160 + du[direction]);
                decoded.value[1] = (int16_t)(120 + dv[direction]);
                decoded.value[2] = 0;
                decoded.value[5] = 4;
                if (direction < 4U) {
                    if (!require(vision_session_observe(&tolerance, &calibration, &decoded, 0U) ==
                                     VISION_EVENT_ALIGNED && tolerance.iteration == 0U &&
                                     tolerance.confirmations == 1U,
                                 "one stationary result at +/-14 pixels must align without motion")) return 1;
                } else {
                    uint32_t previousToken = request.token;
                    if (!require(vision_session_observe(&tolerance, &calibration, &decoded, 0U) ==
                                     VISION_EVENT_MOVE && tolerance.iteration == 1U,
                                 "either axis outside +/-14 pixels must request correction")) return 1;
                    if (!require(vision_session_observe(&tolerance, &calibration, &decoded, 1U) ==
                                     VISION_EVENT_NONE,
                                 "result during movement must be ignored")) return 1;
                    vision_session_move_complete(&tolerance, 10U);
                    if (!require(vision_session_tick(&tolerance, 309U) == VISION_EVENT_NONE &&
                                 vision_session_tick(&tolerance, 310U) == VISION_EVENT_REQUEST,
                                 "pickup must settle for 300 ms before rechecking")) return 1;
                    vision_session_make_request(&tolerance, &calibration, &request, 310U);
                    decoded.value[0] = 160;
                    decoded.value[1] = 120;
                    if (!require(request.token > previousToken &&
                                 vision_session_observe(&tolerance, &calibration, &decoded, 310U) ==
                                     VISION_EVENT_NONE && tolerance.state == VISION_STATE_WAIT,
                                 "pre-movement result must not complete pickup alignment")) return 1;
                    decoded.token = request.token;
                    decoded.value[0] = (int16_t)(160 + du[direction]);
                    decoded.value[1] = (int16_t)(120 + dv[direction]);
                    if (!require(vision_session_observe(&tolerance, &calibration, &decoded, 311U) ==
                                     VISION_EVENT_MOVE && tolerance.iteration == 2U,
                                 "out-of-range recheck must correct again")) return 1;
                    vision_session_move_complete(&tolerance, 320U);
                    vision_session_tick(&tolerance, 620U);
                    vision_session_make_request(&tolerance, &calibration, &request, 620U);
                    decoded.token = request.token;
                    decoded.value[0] = 160;
                    decoded.value[1] = 120;
                    if (!require(vision_session_observe(&tolerance, &calibration, &decoded, 620U) ==
                                     VISION_EVENT_ALIGNED,
                                 "fresh in-range recheck must complete pickup alignment")) return 1;
                }
            }
        }
        vision_session_start(&tolerance, VISION_MODE_RING, 0U, 2U, &calibration, 0U);
        vision_session_make_request(&tolerance, &calibration, &request, 0U);
        decoded = request; decoded.type = VISION_MESSAGE_RESULT;
        decoded.flags = VISION_FLAG_VALID | VISION_FLAG_STABLE;
        decoded.value[0] = 172; decoded.value[1] = 120; decoded.value[2] = 90;
        if (!require(vision_session_observe(&tolerance, &calibration, &decoded, 0U) ==
                     VISION_EVENT_MOVE, "ring error above 2 pixels must still correct")) return 1;
    }

    {
        const int16_t offsets[6][2] = {{2, 2}, {-2, -2}, {3, 0}, {-3, 0}, {0, 3}, {0, -3}};
        uint8_t index, confirmation, mode;
        for (mode = VISION_MODE_MATERIAL; mode <= VISION_MODE_RING; ++mode) {
        for (index = 0U; index < 6U; ++index) {
            vision_session_init(&session);
            vision_session_start(&session, mode, mode == VISION_MODE_MATERIAL ? 5U : 0U,
                                 2U, &calibration, 0U);
            for (confirmation = 0U; confirmation < (index < 2U ? 2U : 1U); ++confirmation) {
                vision_session_make_request(&session, &calibration, &request, confirmation);
                decoded = request;
                decoded.type = VISION_MESSAGE_RESULT;
                decoded.flags = VISION_FLAG_VALID | VISION_FLAG_STABLE;
                decoded.value[0] = (int16_t)(160 + offsets[index][0]);
                decoded.value[1] = (int16_t)(120 + offsets[index][1]);
                decoded.value[2] = 90;
                decoded.value[5] = 5;
                if (!require(vision_session_observe(&session, &calibration, &decoded, confirmation) ==
                             (index >= 2U ? VISION_EVENT_MOVE : confirmation == 0U ?
                              VISION_EVENT_REQUEST : VISION_EVENT_ALIGNED),
                             "ring/storage alignment must accept +/-2 pixels twice and correct either axis beyond it")) return 1;
            }
        }
        }
    }

    {
        VisionSession stacking;
        uint32_t ringToken;
        uint8_t confirmation;
        vision_session_init(&stacking);
        vision_session_start(&stacking, VISION_MODE_RING, 5U, 2U, &calibration, 0U);
        stacking.requestFlags = VISION_FLAG_RING_NO_DIGIT;
        stacking.iteration = 3U;
        stacking.cumulativeMm = 11.0f;
        vision_session_make_request(&stacking, &calibration, &request, 0U);
        if (!require(request.flags == VISION_FLAG_RING_NO_DIGIT && request.selector == 5U,
                     "stacking request must carry unnumbered-ring flag and fallback color")) return 1;
        ringToken = request.token;
        decoded = request;
        decoded.type = VISION_MESSAGE_RESULT;
        decoded.flags = 0U;
        --decoded.token;
        if (!require(!vision_session_fallback_material(&stacking, &decoded) &&
                     stacking.state == VISION_STATE_WAIT,
                     "stale ring failure must not select material fallback")) return 1;
        decoded.token = ringToken;
        decoded.flags = VISION_FLAG_VALID | VISION_FLAG_STABLE;
        decoded.value[2] = 80;
        if (!require(!vision_session_fallback_material(&stacking, &decoded),
                     "valid stationary ring must remain the alignment target")) return 1;
        decoded.flags = 0U;
        if (!require(vision_session_fallback_material(&stacking, &decoded) &&
                     stacking.state == VISION_STATE_REQUEST &&
                     stacking.mode == VISION_MODE_MATERIAL && stacking.selector == 5U &&
                     stacking.target == 2U && stacking.materialMask == 16U &&
                     stacking.requestFlags == 0U && stacking.iteration == 3U &&
                     stacking.cumulativeMm == 11.0f && stacking.confirmations == 0U,
                     "first ring detection failure must select only the middle material color")) return 1;
        vision_session_make_request(&stacking, &calibration, &request, 1000U);
        if (!require(request.token > ringToken && request.mode == VISION_MODE_MATERIAL &&
                     request.selector == 5U && request.value[6] == 16 && request.flags == 0U,
                     "fallback must send a fresh single-color material request")) return 1;
        if (!require(vision_session_observe(&stacking, &calibration, &decoded, 1000U) ==
                         VISION_EVENT_NONE,
                     "old ring result must not align material fallback")) return 1;
        stacking.waitForTarget = 1U;
        for (confirmation = 0U; confirmation < 2U; ++confirmation) {
            decoded = request;
            decoded.type = VISION_MESSAGE_RESULT;
            decoded.flags = VISION_FLAG_VALID | VISION_FLAG_STABLE;
            decoded.value[0] = calibration.anchorU;
            decoded.value[1] = calibration.anchorV;
            decoded.value[2] = 0;
            decoded.value[5] = 5;
            if (!require(vision_session_observe(&stacking, &calibration, &decoded, 1000U) ==
                             (confirmation == 0U ? VISION_EVENT_REQUEST : VISION_EVENT_ALIGNED),
                         "stack material fallback must retain two confirmations and ignore quality")) return 1;
            if (confirmation == 0U)
                vision_session_make_request(&stacking, &calibration, &request, 1001U);
        }
        vision_session_start(&stacking, VISION_MODE_RING, 0U, 2U, &calibration, 0U);
        vision_session_make_request(&stacking, &calibration, &request, 0U);
        decoded = request; decoded.type = VISION_MESSAGE_RESULT; decoded.flags = 0U;
        if (!require(!vision_session_fallback_material(&stacking, &decoded),
                     "ordinary numbered-ring alignment must not use material fallback")) return 1;
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
            decoded.value[5] = (int16_t)(request.selector ? request.selector : 1U);
            decoded.flags = VISION_FLAG_VALID | VISION_FLAG_STABLE;
            decoded.value[0] = 262;
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
    {
        VisionSession sampling;
        VisionCalibration unknown = {0U, {0,0,320,240}, 160,120, {99,99,99,99}};
        uint32_t previousToken;
        vision_session_init(&sampling);
        if (!require(vision_session_start_sample(&sampling, &unknown, 17U, 0U),
                     "sampling required an existing movement calibration")) return 1;
        vision_session_make_request(&sampling, &unknown, &request, 0U);
        decoded = request;
        decoded.type = VISION_MESSAGE_RESULT;
        decoded.flags = VISION_FLAG_VALID | VISION_FLAG_STABLE;
        decoded.value[0] = 220;
        decoded.value[1] = 180;
        decoded.value[2] = 90;
        if (!require(vision_session_observe(&sampling, &unknown, &decoded, 1U) ==
                         VISION_EVENT_SAMPLED && sampling.state == VISION_STATE_IDLE &&
                     sampling.forwardMm == 0 && sampling.rightMm == 0 && sampling.sampleId == 17U,
                     "read-only sampling requested chassis correction")) return 1;
        previousToken = decoded.token;
        vision_session_start_sample(&sampling, &unknown, 18U, 2U);
        vision_session_make_request(&sampling, &unknown, &request, 2U);
        if (!require(request.token > previousToken &&
                     vision_session_observe(&sampling, &unknown, &decoded, 3U) == VISION_EVENT_NONE,
                     "sampling accepted a stale camera result")) return 1;
        decoded.token = request.token;
        decoded.flags = VISION_FLAG_VALID;
        if (!require(vision_session_observe(&sampling, &unknown, &decoded, 4U) == VISION_EVENT_FAILED,
                     "sampling accepted an unstable result")) return 1;
        vision_session_start_sample(&sampling, &unknown, 19U, 5U);
        vision_session_make_request(&sampling, &unknown, &request, 5U);
        decoded.token = request.token;
        decoded.flags = VISION_FLAG_VALID | VISION_FLAG_STABLE;
        decoded.value[2] = 59;
        if (!require(vision_session_observe(&sampling, &unknown, &decoded, 6U) == VISION_EVENT_FAILED,
                     "sampling accepted a low-quality ring")) return 1;
        unknown.calibrated = 1U;
        unknown.matrix[0] = -0.5f; unknown.matrix[1] = 0;
        unknown.matrix[2] = 0; unknown.matrix[3] = -1.0f;
        vision_session_start(&sampling, VISION_MODE_RING, 0U, 2U, &unknown, 10U);
        vision_session_make_request(&sampling, &unknown, &request, 10U);
        decoded.token = request.token; decoded.value[2] = 90;
        if (!require(!sampling.sampleOnly &&
                     vision_session_observe(&sampling, &unknown, &decoded, 11U) == VISION_EVENT_MOVE &&
                     sampling.forwardMm == -30.0f && sampling.rightMm == -60.0f,
                     "calibrated alignment did not use the full movement vector")) return 1;
    }
    return 0;
}
