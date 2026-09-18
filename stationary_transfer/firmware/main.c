#include "platform.h"
#include "transfer.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
static uint32_t diagnostic_token=0x80000000u;
static Packet diagnostic;
/* Commissioning console: USART1 PA9 TX / PA10 RX, 115200 8N1. */
static void command(char *line) {
    float a,b,c,d,e;unsigned n,co[3],ri[3],i;uint8_t colors[3],rings[3];char extra;char msg[160];
    if(!strcmp(line,"STOP")){transfer_abort("USER_STOP");return;}
    if(!strcmp(line,"STATUS")) {
        snprintf(msg,sizeof(msg),"state=%u item=%u placed=%u held=%u ref=%d busy=%d fault=%s hw=%d x=%.2f z=%.2f grip=%.1f tray=%.1f theta=%.1f",
        transfer.state,transfer.index,transfer.placed,transfer.holding,platform_referenced(),platform_busy(),transfer.fault,platform_fault(),
        (double)platform_position(0),(double)platform_position(1),(double)platform_position(2),(double)platform_position(3),(double)platform_position(4));platform_log(msg);return;
    }
    if(transfer_active()||platform_busy()){platform_log("BUSY: STOP or STATUS only");return;}
    if(sscanf(line,"ZERO %f %f %f %f %f %c",&a,&b,&c,&d,&e,&extra)==5) {
        if(platform_zero(a,b,c,d,e)) {transfer_init();platform_log("REFERENCE_ACCEPTED (manual physical placement required)");}
        else platform_log("ZERO_REJECTED");
        return;
    }
    if(sscanf(line,"AXIS %u %f %c",&n,&a,&extra)==2) {
        if(n>1||!isfinite(a)||fabsf(a-platform_position(n))>2)platform_log("JOG_LIMIT: max 2 mm");
        else platform_log(platform_axis(n,a)?"OK":"REJECTED");
        return;
    }
    if(sscanf(line,"SERVO %u %f %c",&n,&a,&extra)==2) {
        if(n>2||!isfinite(a)||fabsf(a-platform_position(n+2))>5)platform_log("JOG_LIMIT: max 5 degrees");
        else platform_log(platform_servo(n,a)?"OK":"REJECTED");
        return;
    }
    if(sscanf(line,"SNAP %u %u %u %c",&n,&co[0],&ri[0],&extra)==3) {
        const View *v;
        if(n<1||n>4||co[0]<1||co[0]>6||ri[0]<1||ri[0]>3){platform_log("BAD_SNAP");return;}
        v=n==1?&config.pick.align:n==3?&config.pick.verify:n==2?&config.place[ri[0]-1].align:&config.place[ri[0]-1].verify;
        memset(&diagnostic,0,sizeof(diagnostic));diagnostic.type=MSG_REQUEST;
        diagnostic.token=++diagnostic_token;diagnostic.mode=(uint8_t)n;diagnostic.color=(uint8_t)co[0];diagnostic.target=(uint8_t)ri[0];
        diagnostic.value[0]=v->x;diagnostic.value[1]=v->y;diagnostic.value[2]=v->w;diagnostic.value[3]=v->h;
        diagnostic.value[4]=v->u;diagnostic.value[5]=v->v;platform_camera_send(&diagnostic);platform_log("SNAP_SENT");return;
    }
    if(sscanf(line,"START %u %u %u %u %u %u %c",&co[0],&ri[0],&co[1],&ri[1],&co[2],&ri[2],&extra)==6) {
        for(i=0;i<3;i++){if(co[i]<1||co[i]>6||ri[i]<1||ri[i]>3){platform_log("BAD_PLAN");return;}colors[i]=(uint8_t)co[i];rings[i]=(uint8_t)ri[i];}
        platform_log(transfer_start(colors,rings)?"STARTED":"START_REJECTED: calibration/reference/plan/state");return;
    }
    platform_log("Commands: STATUS | ZERO x z theta tray grip | AXIS 0/1 absolute_mm | SERVO 0/1/2 degrees | SNAP mode color ring | START c1 r1 c2 r2 c3 r3 | STOP");
}
int main(void) {
    Parser parser={0};Packet packet;uint8_t b;char line[128];int stopped=0;
    platform_init();transfer_init();platform_log("StationaryTransfer v1; PWM initially off; AUTO requires calibrated config.");
    for(;;) {
        platform_poll();
        if(platform_stop_pressed()) {if(!stopped){transfer_abort("PC9_STOP");stopped=1;}}else stopped=0;
        if(platform_line(line,sizeof(line))) {if(!stopped) command(line);}
        while(platform_camera_byte(&b))if(protocol_feed(&parser,b,&packet)) {
            if(packet.type==MSG_RESULT&&packet.token==diagnostic.token&&!transfer_active()) {
                char msg[100];snprintf(msg,sizeof(msg),"VISION flags=%u u=%d v=%d quality=%d w=%d h=%d",packet.flags,
                    packet.value[0],packet.value[1],packet.value[2],packet.value[3],packet.value[4]);platform_log(msg);
            } else transfer_observation(&packet);
        }
        transfer_tick();
        if(platform_fault()&&platform_referenced())transfer_abort("HARDWARE_FAULT");
    }
}
