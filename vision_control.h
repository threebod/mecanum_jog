#ifndef VISION_CONTROL_H
#define VISION_CONTROL_H

#include <stdint.h>
#include <string.h>

#define VISION_FRAME_SIZE 30U
#define VISION_VALUE_COUNT 8U

#define VISION_MESSAGE_REQUEST 1U
#define VISION_MESSAGE_RESULT 2U
#define VISION_MODE_MATERIAL 1U
#define VISION_MODE_RING 2U
#define VISION_FLAG_VALID 0x01U
#define VISION_FLAG_STABLE 0x02U

typedef struct {
    uint8_t type;
    uint32_t token;
    uint8_t mode;
    uint8_t selector;
    uint8_t target;
    uint8_t flags;
    int16_t value[VISION_VALUE_COUNT];
} VisionPacket;

typedef struct {
    uint8_t bytes[VISION_FRAME_SIZE];
    uint8_t length;
} VisionParser;

typedef struct {
    uint8_t calibrated;
    int16_t roi[4];
    int16_t anchorU;
    int16_t anchorV;
    float matrix[4];
} VisionCalibration;

typedef enum {
    VISION_STATE_IDLE = 0,
    VISION_STATE_REQUEST,
    VISION_STATE_WAIT,
    VISION_STATE_MOVE,
    VISION_STATE_SETTLE,
    VISION_STATE_ALIGNED,
    VISION_STATE_FAILED,
    VISION_STATE_PAUSED,
    VISION_STATE_PREP,
    VISION_STATE_PICK,
    VISION_STATE_PICK_DONE
} VisionState;

typedef enum {
    VISION_EVENT_NONE = 0,
    VISION_EVENT_REQUEST,
    VISION_EVENT_MOVE,
    VISION_EVENT_ALIGNED,
    VISION_EVENT_FAILED
} VisionEvent;

typedef struct {
    VisionState state;
    uint8_t mode;
    uint8_t selector;
    uint8_t target;
    uint8_t misses;
    uint8_t confirmations;
    uint8_t iteration;
    uint8_t waitForTarget;
    uint32_t token;
    uint32_t requestAt;
    uint32_t settleUntil;
    float forwardMm;
    float rightMm;
    float cumulativeMm;
    const char *fault;
} VisionSession;

static uint16_t vision_crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    uint16_t index;
    uint8_t bit;
    for (index = 0U; index < length; ++index) {
        crc ^= (uint16_t)data[index] << 8;
        for (bit = 0U; bit < 8U; ++bit) {
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U)
                                  : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static void vision_write_u32(uint8_t *out, uint32_t value)
{
    out[0] = (uint8_t)value;
    out[1] = (uint8_t)(value >> 8);
    out[2] = (uint8_t)(value >> 16);
    out[3] = (uint8_t)(value >> 24);
}

static uint32_t vision_read_u32(const uint8_t *in)
{
    return (uint32_t)in[0] | ((uint32_t)in[1] << 8) |
           ((uint32_t)in[2] << 16) | ((uint32_t)in[3] << 24);
}

static void vision_packet_encode(const VisionPacket *packet,
                                 uint8_t frame[VISION_FRAME_SIZE])
{
    uint8_t index;
    uint16_t crc;
    memset(frame, 0, VISION_FRAME_SIZE);
    frame[0] = 0xA5U;
    frame[1] = 0x5AU;
    frame[2] = 1U;
    frame[3] = packet->type;
    vision_write_u32(frame + 4, packet->token);
    frame[8] = packet->mode;
    frame[9] = packet->selector;
    frame[10] = packet->target;
    frame[11] = packet->flags;
    for (index = 0U; index < VISION_VALUE_COUNT; ++index) {
        frame[12U + index * 2U] = (uint8_t)packet->value[index];
        frame[13U + index * 2U] = (uint8_t)((uint16_t)packet->value[index] >> 8);
    }
    crc = vision_crc16(frame, 28U);
    frame[28] = (uint8_t)crc;
    frame[29] = (uint8_t)(crc >> 8);
}

static void vision_parser_init(VisionParser *parser)
{
    parser->length = 0U;
}

static uint8_t vision_parser_feed(VisionParser *parser, const uint8_t *data,
                                  uint16_t length, VisionPacket *packet)
{
    uint16_t offset;
    uint8_t index;
    for (offset = 0U; offset < length; ++offset) {
        uint8_t byte = data[offset];
        if (parser->length == 0U && byte != 0xA5U) continue;
        if (parser->length == 1U && byte != 0x5AU) {
            parser->length = byte == 0xA5U ? 1U : 0U;
            continue;
        }
        parser->bytes[parser->length++] = byte;
        if (parser->length != VISION_FRAME_SIZE) continue;
        parser->length = 0U;
        if (parser->bytes[2] != 1U ||
            vision_crc16(parser->bytes, 28U) !=
                ((uint16_t)parser->bytes[28] | ((uint16_t)parser->bytes[29] << 8))) {
            continue;
        }
        packet->type = parser->bytes[3];
        packet->token = vision_read_u32(parser->bytes + 4);
        packet->mode = parser->bytes[8];
        packet->selector = parser->bytes[9];
        packet->target = parser->bytes[10];
        packet->flags = parser->bytes[11];
        for (index = 0U; index < VISION_VALUE_COUNT; ++index) {
            packet->value[index] = (int16_t)((uint16_t)parser->bytes[12U + index * 2U] |
                                  ((uint16_t)parser->bytes[13U + index * 2U] << 8));
        }
        return 1U;
    }
    return 0U;
}

static uint8_t vision_calibration_valid(const VisionCalibration *calibration)
{
    return calibration != (const VisionCalibration *)0 && calibration->calibrated &&
           calibration->roi[2] > 0 && calibration->roi[3] > 0;
}

static void vision_map_error(const VisionCalibration *calibration,
                             int16_t du, int16_t dv,
                             float *forward, float *right)
{
    *forward = calibration->matrix[0] * du + calibration->matrix[1] * dv;
    *right = calibration->matrix[2] * du + calibration->matrix[3] * dv;
}

static void vision_limit_step(float *forward, float *right, float maximum)
{
    float largest = *forward < 0.0f ? -*forward : *forward;
    float lateral = *right < 0.0f ? -*right : *right;
    float scale;
    if (lateral > largest) largest = lateral;
    if (largest <= maximum || largest == 0.0f) return;
    scale = maximum / largest;
    *forward *= scale;
    *right *= scale;
}

static void vision_session_init(VisionSession *session)
{
    memset(session, 0, sizeof(*session));
    session->fault = "NONE";
}

static uint8_t vision_session_start(VisionSession *session, uint8_t mode,
                                    uint8_t selector, uint8_t target,
                                    const VisionCalibration *calibration,
                                    uint32_t now)
{
    uint32_t token = session->token;
    (void)now;
    if (!vision_calibration_valid(calibration) ||
        (mode != VISION_MODE_MATERIAL && mode != VISION_MODE_RING) ||
        (mode == VISION_MODE_MATERIAL && (selector < 1U || selector > 6U)) ||
        (mode == VISION_MODE_RING && (target < 1U || target > 3U))) return 0U;
    vision_session_init(session);
    session->token = token;
    session->mode = mode;
    session->selector = selector;
    session->target = target;
    session->waitForTarget = mode == VISION_MODE_RING;
    session->state = VISION_STATE_REQUEST;
    return 1U;
}

static void vision_session_make_request(VisionSession *session,
                                        const VisionCalibration *calibration,
                                        VisionPacket *packet, uint32_t now)
{
    memset(packet, 0, sizeof(*packet));
    packet->type = VISION_MESSAGE_REQUEST;
    packet->token = ++session->token;
    packet->mode = session->mode;
    packet->selector = session->selector;
    packet->target = session->target;
    packet->value[0] = calibration->roi[0];
    packet->value[1] = calibration->roi[1];
    packet->value[2] = calibration->roi[2];
    packet->value[3] = calibration->roi[3];
    packet->value[4] = calibration->anchorU;
    packet->value[5] = calibration->anchorV;
    session->requestAt = now;
    session->state = VISION_STATE_WAIT;
}

static VisionEvent vision_session_miss(VisionSession *session,
                                       const char *fault)
{
    if (++session->misses >= 3U) {
        session->state = VISION_STATE_FAILED;
        session->fault = fault;
        return VISION_EVENT_FAILED;
    }
    session->state = VISION_STATE_REQUEST;
    return VISION_EVENT_REQUEST;
}

static uint8_t vision_session_wait_for_material(VisionSession *session,
                                                const VisionPacket *packet)
{
    if (session->state != VISION_STATE_WAIT ||
        packet->type != VISION_MESSAGE_RESULT || packet->token != session->token ||
        packet->mode != VISION_MODE_MATERIAL || packet->mode != session->mode ||
        packet->selector != session->selector || packet->target != session->target ||
        ((packet->flags & (VISION_FLAG_VALID | VISION_FLAG_STABLE)) ==
             (VISION_FLAG_VALID | VISION_FLAG_STABLE) && packet->value[2] >= 60))
        return 0U;
    session->state = VISION_STATE_REQUEST;
    session->misses = session->confirmations = 0U;
    return 1U;
}

static VisionEvent vision_session_observe(VisionSession *session,
                                          const VisionCalibration *calibration,
                                          const VisionPacket *packet,
                                          uint32_t now)
{
    float forward;
    float right;
    float absForward;
    float absRight;
    (void)now;
    if (session->state != VISION_STATE_WAIT ||
        packet->type != VISION_MESSAGE_RESULT || packet->token != session->token ||
        packet->mode != session->mode || packet->selector != session->selector ||
        packet->target != session->target) return VISION_EVENT_NONE;
    if ((packet->flags & (VISION_FLAG_VALID | VISION_FLAG_STABLE)) !=
            (VISION_FLAG_VALID | VISION_FLAG_STABLE) || packet->value[2] < 60) {
        session->confirmations = 0U;
        if (session->waitForTarget) {
            session->misses = 0U;
            session->state = VISION_STATE_REQUEST;
            return VISION_EVENT_REQUEST;
        }
        return vision_session_miss(session, "UNCONFIRMED");
    }
    session->misses = 0U;
    vision_map_error(calibration,
                     (int16_t)(packet->value[0] - calibration->anchorU),
                     (int16_t)(packet->value[1] - calibration->anchorV),
                     &forward, &right);
    session->forwardMm = forward;
    session->rightMm = right;
    absForward = forward < 0.0f ? -forward : forward;
    absRight = right < 0.0f ? -right : right;
    if (absForward <= 5.0f && absRight <= 5.0f) {
        if (++session->confirmations >= 2U) {
            session->state = VISION_STATE_ALIGNED;
            return VISION_EVENT_ALIGNED;
        }
        session->state = VISION_STATE_REQUEST;
        return VISION_EVENT_REQUEST;
    }
    session->confirmations = 0U;
    if (++session->iteration > 12U) {
        session->state = VISION_STATE_FAILED;
        session->fault = "NOT_CONVERGED";
        return VISION_EVENT_FAILED;
    }
    vision_limit_step(&session->forwardMm, &session->rightMm, 20.0f);
    absForward = session->forwardMm < 0.0f ? -session->forwardMm : session->forwardMm;
    absRight = session->rightMm < 0.0f ? -session->rightMm : session->rightMm;
    session->cumulativeMm += absForward + absRight;
    if (session->cumulativeMm > 100.0f) {
        session->state = VISION_STATE_FAILED;
        session->fault = "CUMULATIVE_LIMIT";
        return VISION_EVENT_FAILED;
    }
    session->state = VISION_STATE_MOVE;
    return VISION_EVENT_MOVE;
}

static void vision_session_move_complete(VisionSession *session, uint32_t now)
{
    session->state = VISION_STATE_SETTLE;
    session->settleUntil = now + 300U;
}

static VisionEvent vision_session_tick(VisionSession *session, uint32_t now)
{
    if (session->state == VISION_STATE_WAIT && now - session->requestAt >= 1800U) {
        return vision_session_miss(session, "CAMERA_TIMEOUT");
    }
    if (session->state == VISION_STATE_SETTLE &&
        (int32_t)(now - session->settleUntil) >= 0) {
        session->state = VISION_STATE_REQUEST;
        return VISION_EVENT_REQUEST;
    }
    return VISION_EVENT_NONE;
}

static void vision_session_pause(VisionSession *session)
{
    session->state = VISION_STATE_PAUSED;
}

#endif
