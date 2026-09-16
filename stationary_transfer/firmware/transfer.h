#ifndef TRANSFER_H
#define TRANSFER_H
#include "config.h"
#include "protocol.h"
typedef enum {IDLE, CLEAR_Z,CLEAR_X,TRAY,TURN_PICK,EXTEND_PICK,ALIGN_PICK,
    LOWER_PICK,CLOSE_GRIP,LIFT_PICK,VERIFY_HELD,RETRACT_PICK,TURN_PLACE,
    EXTEND_PLACE,ALIGN_PLACE,LOWER_PLACE,OPEN_GRIP,LIFT_PLACE,RETRACT_PLACE,
    VERIFY_PLACED,NEXT_ITEM,FINISH,FAILED} TransferState;
typedef struct {
    TransferState state;
    uint8_t colors[3],rings[3],index,placed,holding,entered,pending,attempts;
    uint32_t token,request_at,state_at;
    float dx,dtheta;
    Packet request;
    const char *fault;
} Transfer;
extern Transfer transfer;
void transfer_init(void);
int transfer_start(const uint8_t colors[3],const uint8_t rings[3]);
void transfer_tick(void);
void transfer_observation(const Packet *p);
void transfer_abort(const char *reason);
int transfer_active(void);
#endif
