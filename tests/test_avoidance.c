#include <assert.h>
#include <stdio.h>
#include "../avoidance_run.h"

static void walk(AvRun *run,uint32_t *now)
{
    unsigned guard=0;
    while(run->active && guard++<1000) {
        AvObservation packet={0};packet.token=run->token;packet.region=run->region;
        av_run_observe(run,&packet,*now);av_run_tick(run,*now);
        if(run->phase==AV_TURN || run->phase==AV_MOVE || run->phase==AV_RETREAT) av_run_arrived(run,*now);
        if(run->stage==0 && run->phase==AV_DWELL) assert(run->heading==3);
        *now+=100;
    }
    if(run->phase!=AV_DONE) fprintf(stderr,"phase=%s reason=%s stage=%s xy=%.0f,%.0f h=%u mask=%u\n",av_phase_name(run->phase),run->reason,av_target_name(run->stage),run->x,run->y,run->heading,run->blocked);
    assert(guard<1000 && run->phase==AV_DONE);
}
static uint8_t swept_reference(AvPoint a,AvPoint b,uint16_t blocked)
{
    int x=a.x,y=a.y,dx=(b.x>a.x)-(b.x<a.x),dy=(b.y>a.y)-(b.y<a.y);
    if(a.x!=b.x && a.y!=b.y) return 0;
    for(;;) {
        if(!av_point_clear((int16_t)x,(int16_t)y,blocked)) return 0;
        if(x==b.x && y==b.y) return 1;
        x+=dx;y+=dy;
    }
}
int main(void)
{
    AvPoint path[AV_PATH_MAX];AvRun run={0};unsigned zone,h,region,i,count;uint32_t now=0;
    AvParser parser={0};AvObservation observation={0};char line[96],request[96];unsigned size,received=0;
#ifdef _WIN32
    _set_error_mode(_OUT_TO_STDERR);
#endif
    assert(av_crc16("123456789",9)==0x29B1);
    size=(unsigned)sprintf(line,"AVR:17:3:1:1:320:407:2300");
    sprintf(line+size,":%04X\n",(unsigned)av_crc16(line,size));
    for(i=0;line[i];++i)received+=av_parser_feed(&parser,(uint8_t)line[i],&observation);
    assert(received==1 && observation.token==17 && observation.pixels==2300);
    line[5]='8';assert(!av_decode(line,&observation));
    assert(av_encode_request(request,17,3,1)>0 && strstr(request,"AVQ:17:3:1:")==request);
    assert(av_front_distance(462)==235 && av_front_distance(456)==280);
    assert(av_front_distance(407)==365 && av_front_distance(375)==370 && av_front_distance(352)==490);
    assert(av_front_distance(463)==-1 && av_front_distance(351)==-1);
    assert(av_front_distance(453)==285);
    assert(av_view_region(2100,1700,2)==3); /* keep observing near the road exit */
    assert(!av_turn_clear((AvPoint){1200,1600,0},0));
    assert(av_turn_clear((AvPoint){1200,1200,0},0));
    assert(av_turn_clear((AvPoint){2100,1200,3},1U<<2));
    for(i=0;i<AV_GRID_NODES;++i)for(h=0;h<AV_GRID_NODES;++h) {
        AvPoint a=av_node((uint8_t)i,0),b=av_node((uint8_t)h,0);
        assert(av_segment_clear(a,b,0)==swept_reference(a,b,0));
        assert(av_segment_clear(a,b,5)==swept_reference(a,b,5));
        assert(av_segment_clear(a,b,511)==swept_reference(a,b,511));
    }
    for(zone=1;zone<=2;++zone)for(h=0;h<4;++h) {
        NavPoint start=navPoint(navStartNode((uint8_t)zone));NavPoint target=navPoint(NAV_RAW);
        count=av_plan((AvPoint){start.x,start.y,(uint8_t)h},(AvPoint){target.x,target.y,0},0,path,AV_PATH_MAX);
        assert(count);
        assert(count>1 && path[1].x==2100 && path[1].y==start.y);
        for(i=1;i<count;++i) {
            if(path[i].heading!=path[i-1].heading) assert(av_turn_clear(path[i],0));
            else assert(av_segment_clear(path[i-1],path[i],0));
        }
    }
    for(region=1;region<=9;++region) {
        count=av_plan((AvPoint){2100,2080,2},(AvPoint){1200,400,0},(uint16_t)(1U<<(region-1)),path,AV_PATH_MAX);
        assert(count);
        for(i=1;i<count;++i)assert(av_segment_clear(path[i-1],path[i],(uint16_t)(1U<<(region-1))));
    }
    for(region=0;region<4;++region) {
        static const uint16_t masks[]={20,12,48,192}; /* R3+R5, R3+R4, R5+R6, R7+R8 */
        count=av_plan((AvPoint){2100,2080,2},(AvPoint){1200,400,0},masks[region],path,AV_PATH_MAX);
        assert(count);
        for(i=1;i<count;++i) {
            assert(av_segment_clear(path[i-1],path[i],masks[region]));
            if(path[i].heading!=path[i-1].heading)assert(av_turn_clear(path[i],masks[region]));
        }
    }
    for(zone=1;zone<=2;++zone) {
        NavPoint home=navPoint(navStartNode((uint8_t)zone));
        count=av_plan((AvPoint){400,1200,2},(AvPoint){home.x,home.y,4},0,path,AV_PATH_MAX);
        assert(count>=2 && path[count-2].x==2100 && path[count-2].y==home.y);
        for(i=0;i<count;++i) {
            if(path[i].x==2250) assert(path[i].y==home.y);
            if(path[i].y==150 || path[i].y==2250) assert(path[i].x>=2100);
        }
    }
    assert(!av_plan((AvPoint){2100,2080,2},(AvPoint){1200,400,0},511,path,AV_PATH_MAX));
    assert(!av_run_start(&run,1,9,0) && !av_run_start(&run,1,251,0));
    for(zone=1;zone<=2;++zone) {
        assert(av_run_start(&run,(uint8_t)zone,zone==1?10:250,now));walk(&run,&now);
    }
    for(zone=1;zone<=2;++zone)for(h=0;h<4;++h) {
        assert(av_run_start(&run,(uint8_t)zone,30,now));
        run.heading=(uint8_t)h;run.history[0].heading=(uint8_t)h;walk(&run,&now);
    }
    run.blocked=1U<<2;assert(av_run_start(&run,1,30,now));walk(&run,&now);assert(run.blocked==(1U<<2));
    assert(av_run_start(&run,1,30,now));av_run_tick(&run,now+1500);assert(run.phase==AV_FAULT);
    assert(run.blocked==(1U<<2));
    av_run_stop(&run,"STOPPED");assert(run.blocked==(1U<<2));
    run.blocked=0;run.active=1;run.phase=AV_MOVE;run.camera=1;run.heading=2;run.x=2100;run.y=2100;
    run.token=30;run.region=3;run.historyCount=1;run.history[0]=(AvPoint){2100,2100,2};
    observation=(AvObservation){29,3,1,0,320,407,2300};av_run_observe(&run,&observation,now);assert(run.phase==AV_MOVE);
    observation.token=30;av_run_observe(&run,&observation,now);assert(run.phase==AV_CONFIRM && !run.blocked);
    av_run_tick(&run,now+300);assert(run.requestConfirm);
    observation.token=run.token;observation.stable=1;av_run_observe(&run,&observation,now+300);
    assert(run.blocked==(1U<<2) && run.replans==1 && run.phase==AV_RETREAT);
    av_run_arrived(&run,now+300);assert(run.phase==AV_PLAN);
    run.blocked=0;run.active=1;run.phase=AV_CONFIRM;run.requestConfirm=1;run.regionRetries=0;
    run.x=2100;run.y=2250;run.region=3;run.heading=2;
    observation.token=run.token;observation.bottom=462;
    av_run_observe(&run,&observation,now+400);
    assert(run.active && run.phase==AV_CONFIRM && !run.blocked && !run.requestConfirm);
    av_run_tick(&run,now+700);observation.token=run.token;
    av_run_observe(&run,&observation,now+700);
    assert(!run.active && run.phase==AV_FAULT && !run.blocked && !strcmp(run.reason,"REGION_AMBIGUOUS"));
    puts("PASS: CRC, calibration, heading-aware routes, two laps, memory, pause and retreat");
    return 0;
}
