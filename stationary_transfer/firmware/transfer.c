#include "transfer.h"
#include "platform.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>
Transfer transfer;
/* Token sequence is never reset between runs. No queued prior request is reused. */
static uint32_t sequence;
static void enter(TransferState s) {
    transfer.state=s; transfer.entered=0; transfer.pending=0;transfer.attempts=0;
    transfer.state_at=platform_ms();
}
void transfer_init(void) { memset(&transfer,0,sizeof(transfer)); transfer.fault="NONE"; }
int transfer_active(void) {return transfer.state!=IDLE&&transfer.state!=FAILED&&transfer.state!=FINISH;}
void transfer_abort(const char *reason) {
    platform_stop(); transfer.fault=reason; enter(FAILED);platform_log(reason);
}
int transfer_start(const uint8_t colors[3],const uint8_t rings[3]) {
    unsigned i,j;
    if(transfer.state!=IDLE||!config_valid(&config)||!platform_referenced()||platform_busy()||platform_fault()) return 0;
    for(i=0;i<3;i++) {
        if(colors[i]<1||colors[i]>6||rings[i]<1||rings[i]>3) return 0;
        for(j=0;j<i;j++) if(colors[i]==colors[j]||rings[i]==rings[j]) return 0;
    }
    memcpy(transfer.colors,colors,3);memcpy(transfer.rings,rings,3);
    transfer.index=transfer.placed=transfer.holding=0;transfer.dx=transfer.dtheta=0;
    enter(CLEAR_Z);return 1;
}
static void axis_step(unsigned a,float v,TransferState next) {
    if(!transfer.entered) {transfer.entered=1;if(!platform_axis(a,v)) transfer_abort("AXIS_REJECTED");}
    else if(!platform_busy()) enter(next);
}
static void servo_step(unsigned a,float v,TransferState next) {
    if(!transfer.entered) {transfer.entered=1;if(!platform_servo(a,v)) transfer_abort("SERVO_REJECTED");}
    else if(!platform_busy()) enter(next);
}
static void observe(uint8_t mode,const View *v) {
    Packet *p=&transfer.request;
    const WorkPose *pose;const float *slope;float dx,dt,u,iv;
    if(platform_busy()||transfer.pending) return;
    memset(p,0,sizeof(*p));p->type=MSG_REQUEST;p->token=++sequence;
    p->mode=mode;p->color=transfer.colors[transfer.index];p->target=transfer.rings[transfer.index];
    p->value[0]=v->x;p->value[1]=v->y;p->value[2]=v->w;p->value[3]=v->h;
    pose=(mode==MODE_MATERIAL||mode==MODE_HELD)?&config.pick:&config.place[transfer.rings[transfer.index]-1];
    slope=(mode==MODE_HELD||mode==MODE_PLACED)?pose->verify_slope:pose->align_slope;
    dx=platform_position(0)-(mode==MODE_PLACED?config.clear_x:pose->x);
    dt=platform_position(4)-pose->theta;
    u=v->u+slope[0]*dx+slope[1]*dt;iv=v->v+slope[2]*dx+slope[3]*dt;
    if(u<v->x||u>=v->x+v->w||iv<v->y||iv>=v->y+v->h) {transfer_abort("TCP_OUTSIDE_ROI");return;}
    p->value[4]=(int16_t)lroundf(u);p->value[5]=(int16_t)lroundf(iv);
    transfer.token=p->token;transfer.request_at=platform_ms();transfer.pending=1;
    platform_camera_send(p);
}
void transfer_observation(const Packet *p) {
    const WorkPose *pose; int du,dv;float dx,dt;
    if(!transfer.pending||p->type!=MSG_RESULT||p->token!=transfer.token||p->mode!=transfer.request.mode||
       p->color!=transfer.request.color||p->target!=transfer.request.target) return;
    if(platform_ms()-transfer.request_at>config.vision_timeout_ms) {transfer_abort("VISION_STALE");return;}
    transfer.pending=0;
    if((p->flags&(FLAG_VALID|FLAG_STABLE))!=(FLAG_VALID|FLAG_STABLE)||p->value[2]<60||
       p->value[0]<0||p->value[0]>=320||p->value[1]<0||p->value[1]>=240) {
        if(++transfer.attempts>=3) transfer_abort("VISION_UNCONFIRMED");
        return;
    }
    if(p->value[0]<transfer.request.value[0]||p->value[0]>=transfer.request.value[0]+transfer.request.value[2]||
       p->value[1]<transfer.request.value[1]||p->value[1]>=transfer.request.value[1]+transfer.request.value[3]) {
        transfer_abort("VISION_OUTSIDE_ROI");return;
    }
    if((transfer.state==VERIFY_HELD||transfer.state==VERIFY_PLACED)&&
        (abs(p->value[0]-transfer.request.value[4])>6||abs(p->value[1]-transfer.request.value[5])>6)) {
        transfer_abort("VERIFY_POSITION");return;
    }
    if(transfer.state==VERIFY_HELD) {transfer.holding=1;enter(RETRACT_PICK);return;}
    if(transfer.state==VERIFY_PLACED) {transfer.holding=0;transfer.placed++;enter(NEXT_ITEM);return;}
    if(transfer.state!=ALIGN_PICK&&transfer.state!=ALIGN_PLACE) return;
    pose=transfer.state==ALIGN_PICK?&config.pick:&config.place[transfer.rings[transfer.index]-1];
    du=p->value[0]-transfer.request.value[4];dv=p->value[1]-transfer.request.value[5];
    if(abs(du)<=config.tolerance_px&&abs(dv)<=config.tolerance_px) {
        enter(transfer.state==ALIGN_PICK?LOWER_PICK:LOWER_PLACE);return;
    }
    if(++transfer.attempts>12) {transfer_abort("ALIGN_NOT_CONVERGED");return;}
    dx=pose->correction[0]*du+pose->correction[1]*dv;
    dt=pose->correction[2]*du+pose->correction[3]*dv;
    /* Correct one joint per fresh image, so the moving camera is re-observed. */
    if(fabsf(dx)>config.max_dx) dx=copysignf(config.max_dx,dx);
    if(fabsf(dt)>config.max_dtheta) dt=copysignf(config.max_dtheta,dt);
    if(fabsf(dx)/config.max_dx>=fabsf(dt)/config.max_dtheta) {
        transfer.dx+=dx;
        if(fabsf(transfer.dx)>config.max_total_dx||!platform_axis(0,platform_position(0)+dx)) transfer_abort("ALIGN_X_LIMIT");
    } else {
        transfer.dtheta+=dt;
        if(fabsf(transfer.dtheta)>config.max_total_dtheta||!platform_servo(2,platform_position(4)+dt)) transfer_abort("ALIGN_ANGLE_LIMIT");
    }
}
void transfer_tick(void) {
    const WorkPose *p;
    if(!transfer_active()) return;
    if(platform_fault()) {transfer_abort("HARDWARE_FAULT");return;}
    if(platform_ms()-transfer.state_at>60000) {transfer_abort("STATE_TIMEOUT");return;}
    if(transfer.pending&&platform_ms()-transfer.request_at>config.vision_timeout_ms) {transfer_abort("CAMERA_TIMEOUT");return;}
    p=&config.place[transfer.rings[transfer.index]-1];
    switch(transfer.state) {
    case CLEAR_Z: axis_step(1,config.safe_z,CLEAR_X);break;
    case CLEAR_X: axis_step(0,config.clear_x,TRAY);break;
    case TRAY: servo_step(1,config.tray_angles[transfer.index],TURN_PICK);break;
    case TURN_PICK: servo_step(2,config.pick.theta,EXTEND_PICK);break;
    case EXTEND_PICK:
        if(!transfer.entered) {if(!platform_servo(0,80)) {transfer_abort("GRIP_OPEN");break;}transfer.entered=1;}
        else if(transfer.entered==1&&!platform_busy()) {if(!platform_axis(0,config.pick.x)) {transfer_abort("PICK_X");break;}transfer.entered=2;}
        else if(transfer.entered==2&&!platform_busy()) {transfer.dx=transfer.dtheta=0;enter(ALIGN_PICK);}
        break;
    case ALIGN_PICK: observe(MODE_MATERIAL,&config.pick.align);break;
    case LOWER_PICK: axis_step(1,config.pick.z,CLOSE_GRIP);break;
    case CLOSE_GRIP: servo_step(0,30,LIFT_PICK);break;
    case LIFT_PICK: axis_step(1,config.safe_z,VERIFY_HELD);break;
    case VERIFY_HELD: observe(MODE_HELD,&config.pick.verify);break;
    case RETRACT_PICK: axis_step(0,config.clear_x,TURN_PLACE);break;
    case TURN_PLACE: servo_step(2,p->theta,EXTEND_PLACE);break;
    case EXTEND_PLACE:
        axis_step(0,p->x,ALIGN_PLACE);transfer.dx=transfer.dtheta=0;break;
    case ALIGN_PLACE: observe(MODE_RING,&p->align);break;
    case LOWER_PLACE: axis_step(1,p->z,OPEN_GRIP);break;
    case OPEN_GRIP: servo_step(0,80,LIFT_PLACE);break;
    case LIFT_PLACE: axis_step(1,config.safe_z,RETRACT_PLACE);break;
    case RETRACT_PLACE: axis_step(0,config.clear_x,VERIFY_PLACED);break;
    case VERIFY_PLACED: observe(MODE_PLACED,&p->verify);break;
    case NEXT_ITEM:
        if(++transfer.index<3) enter(CLEAR_Z);else {transfer.index=2;enter(FINISH);platform_log("DONE 3/3");}break;
    default: transfer_abort("BAD_STATE");break;
    }
}
