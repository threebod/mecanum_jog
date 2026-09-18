#include "config.h"
#include <math.h>
/* ALL positions below are placeholders, not measured settings.
   Firmware builds with both ready flags OFF. Read CALIBRATION.md first.
   Axis order: 0 = extension motor 6; 1 = lift motor 5; z positive DOWN.
   Ring array index is PHYSICAL ring ID - 1, not current image order. */
#define VIEW {0,0,320,240,160,120}
#define POSE {0,180,0,VIEW,VIEW,{0,0,0,0},{0,0,0,0},{0,0,0,0}}
const Config config={
    .mechanics_ready=0, .vision_ready=0,
    .positive_dir={0,0}, .steps_mm={25.465f,80.0f},
    .axis_min={-122,0}, .axis_max={65,135}, .ms_mm={150,150},
    .safe_z=0, .clear_x=-48, .home_theta=180,
    .tray_angles={20,139,256},
    .servo_span={270,270,360}, .servo_min={30,0,0}, .servo_max={80,270,360},
    .servo_ms_degree=30,
    .max_dx=1, .max_dtheta=1, .max_total_dx=8, .max_total_dtheta=8,
    .motor_rpm=100, .motor_acc=100,
    .settle_ms=500, .vision_timeout_ms=1800, .tolerance_px=3,
    .pick=POSE, .place={POSE,POSE,POSE}
};
static int bounded(float x,float lo,float hi) { return isfinite(x)&&x>=lo&&x<=hi; }
static int view_ok(const View *v) {
    return v->x>=0&&v->y>=0&&v->w>0&&v->h>0&&v->x+v->w<=320&&v->y+v->h<=240&&
        v->u>=v->x&&v->u<v->x+v->w&&v->v>=v->y&&v->v<v->y+v->h;
}
int config_valid(const Config *c) {
    unsigned i,j; const WorkPose *p;
    if(!c->mechanics_ready||!c->vision_ready||c->motor_rpm==0||c->motor_rpm>300||
       c->vision_timeout_ms<1000||c->vision_timeout_ms>10000||c->settle_ms<100||c->settle_ms>5000||
       !bounded(c->servo_ms_degree,1,100)||c->tolerance_px<1||c->tolerance_px>10) return 0;
    for(i=0;i<2;i++) if(!bounded(c->steps_mm[i],1,10000)||!bounded(c->ms_mm[i],10,2000)||
        !bounded(c->axis_min[i],-1000,1000)||!bounded(c->axis_max[i],-1000,1000)||
        c->axis_min[i]>=c->axis_max[i]||c->positive_dir[i]>1) return 0;
    for(i=0;i<3;i++) if(!bounded(c->servo_span[i],90,360)||
        !bounded(c->servo_min[i],0,c->servo_span[i])||!bounded(c->servo_max[i],c->servo_min[i],c->servo_span[i])) return 0;
    if(!bounded(30,c->servo_min[0],c->servo_max[0])||!bounded(80,c->servo_min[0],c->servo_max[0])||
       !bounded(c->safe_z,c->axis_min[1],c->axis_max[1])||!bounded(c->clear_x,c->axis_min[0],c->axis_max[0])||
       !bounded(c->home_theta,c->servo_min[2],c->servo_max[2])||
       !bounded(c->max_dx,0.01f,2)||!bounded(c->max_dtheta,0.01f,2)||
       !bounded(c->max_total_dx,c->max_dx,15)||!bounded(c->max_total_dtheta,c->max_dtheta,15)) return 0;
    for(i=0;i<3;i++) if(!bounded(c->tray_angles[i],c->servo_min[1],c->servo_max[1])) return 0;
    for(i=0;i<4;i++) {
        p=i?&c->place[i-1]:&c->pick;
        if(!bounded(p->x,c->axis_min[0],c->axis_max[0])||!bounded(p->z,c->safe_z,c->axis_max[1])||
           !bounded(p->theta,c->servo_min[2],c->servo_max[2])||!view_ok(&p->align)||!view_ok(&p->verify)) return 0;
        for(j=0;j<4;j++) if(!bounded(p->correction[j],-10,10)||
            !bounded(p->align_slope[j],-100,100)||!bounded(p->verify_slope[j],-100,100)) return 0;
        if(fabsf(p->correction[0]*p->correction[3]-p->correction[1]*p->correction[2])<0.000001f) return 0;
    }
    return 1;
}
