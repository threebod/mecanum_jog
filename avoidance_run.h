#ifndef MECANUM_AVOIDANCE_RUN_H
#define MECANUM_AVOIDANCE_RUN_H
#include <string.h>
#include "avoidance_plan.h"
#include "avoidance_protocol.h"

typedef enum { AV_IDLE,AV_CAMERA,AV_PLAN,AV_TURN,AV_MOVE,AV_CONFIRM,AV_RETREAT,AV_DWELL,AV_DONE,AV_FAULT } AvPhase;
typedef struct {
    uint16_t blocked, token, generation, replans, rpm;
    uint8_t active, zone, stage, heading, region, camera, dirty, retreat, requestConfirm, regionRetries;
    unsigned count,index,historyCount;
    AvPoint path[AV_PATH_MAX],history[AV_PATH_MAX], retreatTarget;
    float x,y;
    uint32_t lastRx,started,deadline,legStarted,requestAt;
    AvPhase phase;
    const char *reason;
} AvRun;
static const char *av_phase_name(AvPhase phase)
{
    static const char *names[]={"IDLE","CAMERA","PLAN","TURN","MOVE","CONFIRM","RETREAT","DWELL","DONE","FAULT"};
    return names[phase];
}
static const char *av_target_name(uint8_t stage)
{
    static const char *names[]={"QR","RAW_1","COARSE_1","TEMP_1","RAW_2","COARSE_2","TEMP_2","HOME"};
    return stage<8?names[stage]:"HOME";
}
static AvPoint av_target(const AvRun *run)
{
    static const uint8_t nodes[]={NAV_QR,NAV_RAW,NAV_COARSE,NAV_TEMP,NAV_RAW,NAV_COARSE,NAV_TEMP};
    NavPoint n=navPoint(run->stage<7?nodes[run->stage]:navStartNode(run->zone));
    AvPoint p={0,0,4};p.x=n.x;p.y=n.y;p.heading=run->stage==0?3:4;return p;
}
static void av_new_request(AvRun *run,uint8_t region,uint8_t confirm,uint32_t now)
{
    if(++run->token==0) ++run->token;
    run->region=region;run->requestConfirm=confirm;run->requestAt=now;run->dirty=1;
}
static void av_fail(AvRun *run,const char *reason)
{ run->phase=AV_FAULT;run->active=0;run->reason=reason;run->dirty=1; }
static void av_run_stop(AvRun *run,const char *reason)
{ run->active=0;run->phase=AV_IDLE;run->reason=reason;run->camera=0;run->dirty=1; }
static uint8_t av_run_start(AvRun *run,uint8_t zone,uint16_t rpm,uint32_t now)
{
    uint16_t blocked=run->blocked,token=run->token,generation=run->generation;NavPoint start;
    if(run->active || (zone!=1 && zone!=2) || rpm<10 || rpm>250) return 0;
    start=navPoint(navStartNode(zone));
    if(!av_point_clear(start.x,start.y,blocked)) return 0;
    memset(run,0,sizeof(*run));run->blocked=blocked;run->token=token;run->generation=generation;
    run->zone=zone;run->rpm=rpm;run->active=1;run->phase=AV_CAMERA;run->reason="NONE";
    run->x=start.x;run->y=start.y;run->started=run->lastRx=now;
    run->history[0].x=start.x;run->history[0].y=start.y;run->history[0].heading=0;run->historyCount=1;
    av_new_request(run,0,0,now);return 1;
}
static AvPoint av_leg_target(const AvRun *run)
{ return run->retreat?run->retreatTarget:run->path[run->index]; }
static void av_prepare_leg(AvRun *run,uint32_t now)
{
    AvPoint target=av_leg_target(run);
    run->phase=target.heading!=run->heading?AV_TURN:run->retreat?AV_RETREAT:AV_MOVE;
    run->legStarted=now;run->dirty=1;
}
static void av_run_tick(AvRun *run,uint32_t now)
{
    if(!run->active) return;
    if(!run->camera && now-run->started>=1500) {av_fail(run,"CAMERA_INIT_TIMEOUT");return;}
    if(run->camera && now-run->lastRx>500) {run->camera=0;av_fail(run,"CAMERA_LOST");return;}
    if(run->phase==AV_DWELL && (int32_t)(now-run->deadline)>=0) {
        if(++run->stage>=8) {run->phase=AV_DONE;run->active=0;run->reason="NONE";run->dirty=1;return;}
        run->phase=AV_PLAN;
    }
    if(run->phase==AV_PLAN) {
        AvPoint from={(int16_t)(run->x+0.5f),(int16_t)(run->y+0.5f),run->heading};
        run->count=av_plan(from,av_target(run),run->blocked,run->path,AV_PATH_MAX);
        ++run->generation;run->dirty=1;
        if(!run->count) {av_fail(run,"NO_PATH");return;}
        run->index=1;
        if(run->count==1) {run->phase=AV_DWELL;run->deadline=now+500;}
        else av_prepare_leg(run,now);
    }
    if(run->phase==AV_CONFIRM && (int32_t)(now-run->deadline)>=0 && !run->requestConfirm)
        av_new_request(run,run->region,1,now);
    if(run->phase!=AV_CONFIRM) {
        uint8_t region=run->phase==AV_MOVE?av_view_region(run->x,run->y,run->heading):0;
        if(run->phase==AV_MOVE) {
            AvPoint here={(int16_t)(run->x+0.5f),(int16_t)(run->y+0.5f),run->heading};
            AvPoint target=av_leg_target(run);
            if(av_direction(here,target)!=run->heading || !av_segment_regions(here,target)) region=0;
        }
        if(region!=run->region || run->requestConfirm) av_new_request(run,region,0,now);
    }
}
static void av_run_observe(AvRun *run,const AvObservation *packet,uint32_t now)
{
    if(!run->active || packet->token!=run->token || packet->region!=run->region) return;
    run->camera=1;run->lastRx=now;
    if(run->phase==AV_CAMERA) {run->phase=AV_PLAN;run->dirty=1;return;}
    if(run->phase==AV_MOVE && packet->seen && run->region) {
        run->phase=AV_CONFIRM;run->deadline=now+300;run->reason="CANDIDATE";
        run->regionRetries=0;
        av_new_request(run,run->region,0,now);return;
    }
    if(run->phase==AV_CONFIRM && run->requestConfirm && packet->stable) {
        uint8_t region;unsigned i;
        if(!packet->seen) {run->reason="NONE";run->phase=AV_MOVE;av_new_request(run,run->region,0,now);return;}
        region=av_obstacle_region(run->x,run->y,run->heading,packet->bottom);
        if(!region || region!=run->region) {
            if(av_front_distance(packet->bottom)<0) {av_fail(run,"DISTANCE_RANGE");return;}
            if(!run->regionRetries++) {
                run->reason="REGION_RECHECK";run->deadline=now+300;
                av_new_request(run,run->region,0,now);return;
            }
            av_fail(run,"REGION_AMBIGUOUS");return;
        }
        run->blocked |= (uint16_t)(1U<<(region-1));++run->replans;run->dirty=1;
        /* Retreat only along the collinear, already travelled history. Never turn inside a newly blocked zone. */
        for(i=run->historyCount;i>0;--i) {
            AvPoint p=run->history[i-1];AvPoint here={(int16_t)(run->x+0.5f),(int16_t)(run->y+0.5f),run->heading};
            if(p.heading!=run->heading || (p.x!=here.x && p.y!=here.y)) break;
            if(av_point_clear(p.x,p.y,run->blocked) && av_segment_clear(here,p,(uint16_t)(run->blocked & ~(1U<<(region-1))))) {
                run->retreatTarget=p;run->historyCount=i;run->retreat=1;run->reason="BLOCKED";
                ++run->generation;run->count=2;run->index=1;run->path[0]=here;run->path[1]=p;
                av_prepare_leg(run,now);av_new_request(run,0,0,now);return;
            }
        }
        av_fail(run,"NO_SAFE_RETREAT");
    }
}
static void av_run_arrived(AvRun *run,uint32_t now)
{
    AvPoint target=av_leg_target(run);
    if(run->phase==AV_TURN) {
        run->heading=target.heading;av_prepare_leg(run,now);return;
    }
    run->x=target.x;run->y=target.y;
    if(run->retreat) {run->retreat=0;run->phase=AV_PLAN;run->reason="NONE";run->dirty=1;return;}
    if(run->historyCount>=AV_PATH_MAX) {av_fail(run,"HISTORY_FULL");return;}
    run->history[run->historyCount++]=target;
    if(++run->index>=run->count) {run->phase=AV_DWELL;run->deadline=now+500;run->dirty=1;}
    else av_prepare_leg(run,now);
}
#endif
