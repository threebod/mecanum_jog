#ifndef MECANUM_AVOIDANCE_PLAN_H
#define MECANUM_AVOIDANCE_PLAN_H
#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include "map_navigation.h"
#include "avoidance_regions.h"

#define AV_PATH_MAX 180U
#define AV_GRID_NODES 45U
#define AV_STATES (AV_GRID_NODES * 4U)
typedef struct { int16_t x,y; uint8_t heading; } AvPoint;
/* Shared with the upper computer through this header; headings 0 up,1 left,2 down,3 right. */
static const int16_t avGridX[5] = {300,400,1200,2100,2250};
static const int16_t avGridY[9] = {150,300,400,1150,1200,1250,2080,2100,2250};
static const AvRegion avFixedRegions[]={{550,550,1000,1000},{1400,550,1850,1000},{550,1400,1000,1850},
    {1400,1400,1850,1850},{0,910,150,1490},{910,0,1490,150}};
static AvPoint av_node(uint8_t node, uint8_t heading)
{
    AvPoint p; p.x = avGridX[node % 5]; p.y = avGridY[node / 5]; p.heading = heading; return p;
}
static int av_node_at(int16_t x, int16_t y)
{
    unsigned n; for (n=0; n<AV_GRID_NODES; ++n) { AvPoint p=av_node((uint8_t)n,0); if(p.x==x && p.y==y) return (int)n; }
    return -1;
}
static uint8_t av_rect_overlap(int x0,int y0,int x1,int y1,const AvRegion *r)
{ return (uint8_t)(x0 < r->x1 && x1 > r->x0 && y0 < r->y1 && y1 > r->y0); }
static uint8_t av_point_clear(int16_t x,int16_t y,uint16_t blocked)
{
    unsigned i; if(!navPointClear(x,y)) return 0;
    for(i=0;i<AV_REGION_COUNT;++i) if((blocked & (1U<<i)) && av_rect_overlap(x-150,y-150,x+150,y+150,&avRegions[i])) return 0;
    return 1;
}
static uint8_t av_segment_clear(AvPoint a,AvPoint b,uint16_t blocked)
{
    int x0=a.x<b.x?a.x:b.x,x1=a.x>b.x?a.x:b.x,y0=a.y<b.y?a.y:b.y,y1=a.y>b.y?a.y:b.y;
    unsigned i;
    if((a.x!=b.x && a.y!=b.y) || !av_point_clear(a.x,a.y,blocked) || !av_point_clear(b.x,b.y,blocked)) return 0;
    /* An axis-aligned square swept along an axis is exactly this rectangle. */
    for(i=0;i<sizeof(avFixedRegions)/sizeof(avFixedRegions[0]);++i)
        if(av_rect_overlap(x0-150,y0-150,x1+150,y1+150,&avFixedRegions[i])) return 0;
    for(i=0;i<AV_REGION_COUNT;++i)
        if((blocked&(1U<<i)) && av_rect_overlap(x0-150,y0-150,x1+150,y1+150,&avRegions[i])) return 0;
    return 1;
}
static uint8_t av_circle_rect(int x,int y,const AvRegion *r)
{
    int dx=x<r->x0?r->x0-x:x>r->x1?x-r->x1:0;
    int dy=y<r->y0?r->y0-y:y>r->y1?y-r->y1:0;
    return (uint8_t)(dx*dx+dy*dy < 45000); /* circumradius sqrt(150^2+150^2) */
}
static uint8_t av_turn_clear(AvPoint p,uint16_t blocked)
{
    unsigned i;
    if(p.x<213 || p.x>2187 || p.y<213 || p.y>2187) return 0;
    for(i=0;i<sizeof(avFixedRegions)/sizeof(avFixedRegions[0]);++i) if(av_circle_rect(p.x,p.y,&avFixedRegions[i])) return 0;
    /* Work stations are guaranteed obstacle-free; a coarse road closure must not forbid their turns. */
    for(i=NAV_QR;i<NAV_NODE_COUNT;++i) {
        if((i==NAV_QR || i==NAV_RAW || i==NAV_COARSE || i==NAV_TEMP) &&
           p.x==navPoints[i].x && p.y==navPoints[i].y && av_point_clear(p.x,p.y,blocked)) return 1;
    }
    for(i=0;i<AV_REGION_COUNT;++i) if((blocked&(1U<<i)) && av_circle_rect(p.x,p.y,&avRegions[i])) return 0;
    return 1;
}
static uint8_t av_segment_regions(AvPoint a,AvPoint b)
{
    unsigned i; int x0=a.x<b.x?a.x:b.x,x1=a.x>b.x?a.x:b.x,y0=a.y<b.y?a.y:b.y,y1=a.y>b.y?a.y:b.y;
    for(i=0;i<AV_REGION_COUNT;++i) if(av_rect_overlap(x0-150,y0-150,x1+150,y1+150,&avRegions[i])) return 1;
    return 0;
}
static uint8_t av_direction(AvPoint a,AvPoint b)
{ return b.x>a.x?3:b.x<a.x?1:b.y>a.y?0:2; }
static unsigned av_plan(AvPoint start,AvPoint target,uint16_t blocked,AvPoint *path,unsigned capacity)
{
    /* Single main-loop planner; static workspace avoids overflowing the MCU stack. */
    static uint32_t distance[AV_STATES]; static int16_t previous[AV_STATES]; static uint8_t visited[AV_STATES];
    static uint16_t reverse[AV_STATES]; unsigned i,count=0; int startNode=av_node_at(start.x,start.y),targetNode=av_node_at(target.x,target.y),finish=-1;
    if(startNode<0 || targetNode<0 || start.heading>3 || !av_point_clear(start.x,start.y,blocked) || !av_point_clear(target.x,target.y,blocked)) return 0;
    for(i=0;i<AV_STATES;++i) {distance[i]=0xFFFFFFFFUL;previous[i]=-1;visited[i]=0;}
    distance[startNode*4+start.heading]=0;
    for(;;) {
        int best=-1; uint32_t cost=0xFFFFFFFFUL; unsigned n,h,j; AvPoint p;
        for(i=0;i<AV_STATES;++i) if(!visited[i] && distance[i]<cost) {best=(int)i;cost=distance[i];}
        if(best<0) break;
        n=(unsigned)best/4;h=(unsigned)best%4;p=av_node((uint8_t)n,(uint8_t)h);
        if((int)n==targetNode && (target.heading>3 || h==target.heading)) {finish=best;break;}
        visited[best]=1;
        if(av_turn_clear(p,blocked)) for(j=0;j<4;++j) if(j!=h) {
            unsigned s=n*4+j; uint32_t next=cost+120U*(j==(h+2)%4?2U:1U);
            if(next<distance[s]) {distance[s]=next;previous[s]=(int16_t)best;}
        }
        for(j=0;j<AV_GRID_NODES;++j) {
            AvPoint q=av_node((uint8_t)j,(uint8_t)h); unsigned s=j*4+h; uint32_t next;
            /* Only adjacent grid intersections; obstacles can remove an edge, never shortcut it. */
            if(!((j/5==n/5 && (j+1==n || n+1==j)) || (j%5==n%5 && (j+5==n || n+5==j)))) continue;
            /* Boundary anchors belong only to the two start-zone entrances.
               Travel along the field uses the existing inner lanes instead. */
            if((p.x==2250 || q.x==2250) && !(p.y==q.y && (p.y==150 || p.y==2250))) continue;
            if((p.y==150 || p.y==2250) && p.x<2100) continue;
            if((q.y==150 || q.y==2250) && q.x<2100) continue;
            if(av_direction(p,q)!=h && av_segment_regions(p,q)) continue;
            if(!av_segment_clear(p,q,blocked)) continue;
            next=cost+(uint32_t)(abs(q.x-p.x)+abs(q.y-p.y));
            if(next<distance[s]) {distance[s]=next;previous[s]=(int16_t)best;}
        }
    }
    if(finish<0) return 0;
    for(;;) { if(count>=AV_STATES) return 0;reverse[count++]=(uint16_t)finish;if(previous[finish]<0) break;finish=previous[finish]; }
    if(count>capacity) return 0;
    for(i=0;i<count;++i) path[i]=av_node((uint8_t)(reverse[count-1-i]/4),(uint8_t)(reverse[count-1-i]%4));
    return count;
}
static int av_front_distance(uint16_t bottom)
{
    static const int row[5]={17,23,72,104,127},mm[5]={235,280,365,370,490};
    int value=479-(int)bottom; unsigned i;
    if(value<row[0] || value>row[4]) return -1;
    for(i=1;i<5;++i) if(value<=row[i]) return mm[i-1]+(value-row[i-1])*(mm[i]-mm[i-1])/(row[i]-row[i-1]);
    return -1;
}
static uint8_t av_region_at(int x,int y)
{
    unsigned i,count=0;uint8_t region=0;
    for(i=0;i<AV_REGION_COUNT;++i) {const AvRegion *r=&avRegions[i];if(x>=r->x0 && x<=r->x1 && y>=r->y0 && y<=r->y1) {region=(uint8_t)(i+1);++count;}}
    return count==1?region:0;
}
static uint8_t av_obstacle_region(float x,float y,uint8_t heading,uint16_t bottom)
{
    int distance=av_front_distance(bottom); if(distance<0) return 0; distance+=150;
    if(heading==0)y+=distance;else if(heading==1)x-=distance;else if(heading==2)y-=distance;else x+=distance;
    return av_region_at((int)(x+0.5f),(int)(y+0.5f));
}
static uint8_t av_view_region(float x,float y,uint8_t heading)
{
    unsigned d; for(d=385;d<=640;d+=10) {
        int px=(int)(x+0.5f),py=(int)(y+0.5f);uint8_t region;
        if(heading==0)py+=(int)d;else if(heading==1)px-=(int)d;else if(heading==2)py-=(int)d;else px+=(int)d;
        region=av_region_at(px,py);if(region) return region;
    }
    return av_region_at((int)(x+0.5f),(int)(y+0.5f));
}
#endif
