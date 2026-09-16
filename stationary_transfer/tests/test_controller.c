#include "transfer.h"
#include "platform.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#define V {10,10,300,220,160,120}
#define P {10,180,20,V,V,{0.1f,0,0,0.1f},{2,0,0,0},{2,0,0,0}}
const Config config={1,1,{0,0},{25.465f,80},{-122,0},{65,135},{150,150},
    0,-48,180,{20,139,256},{270,270,360},{30,0,0},{80,270,360},30,
    1,1,8,8,100,100,500,1800,3,P,{P,P,P}};
extern const Config default_config;
static uint32_t now,until;static int ref,hw,stop_count,moves,tray_moves;static float q[5];
static Packet latest;
uint32_t platform_ms(void){return now;}
void platform_log(const char *s){(void)s;}
int platform_referenced(void){return ref;}
int platform_fault(void){return hw;}
int platform_busy(void){return (int32_t)(until-now)>0;}
void platform_stop(void){ref=0;until=now;stop_count++;}
float platform_position(unsigned c){return q[c];}
void platform_camera_send(const Packet *p){latest=*p;}
int platform_axis(unsigned a,float target){
    assert(!platform_busy());assert(ref);assert(a<2);
    if(target<config.axis_min[a]||target>config.axis_max[a])return 0;
    q[a]=target;until=now+20;moves++;return 1;
}
int platform_servo(unsigned s,float v){
    assert(!platform_busy());assert(ref);assert(s<3);
    if(s==1){assert(q[0]==config.clear_x&&q[1]==config.safe_z);assert(!transfer.holding);tray_moves++;}
    if(s==2)assert(q[1]==config.safe_z);
    q[s+2]=v;until=now+20;moves++;return 1;
}
static void reset(void){now=until=0;ref=1;hw=stop_count=moves=tray_moves=0;memset(q,0,sizeof(q));transfer_init();}
static Packet response(void){Packet p=latest;p.type=MSG_RESULT;p.flags=3;p.value[0]=latest.value[4];p.value[1]=latest.value[5];p.value[2]=80;return p;}
static void await_request(void){unsigned i;for(i=0;i<500&&!transfer.pending;i++){now+=20;transfer_tick();}assert(transfer.pending);}
static void start(void){uint8_t c[]={1,5,6},r[]={2,1,3};assert(transfer_start(c,r));}
static void success(void){
    unsigned i;Packet p;reset();start();
    for(i=0;i<3000&&transfer_active();i++){
        now+=20;transfer_tick();
        if(transfer.pending){p=response();transfer_observation(&p);transfer_observation(&p);}
    }
    assert(transfer.state==FINISH&&transfer.placed==3&&!transfer.holding&&tray_moves==3&&!stop_count);
    assert(!transfer_start(transfer.colors,transfer.rings));
}
int main(void){
    uint8_t c[]={1,5,6},r[]={1,2,3},bad[]={1,1,3};Packet p,out;
    uint8_t data[PACKET_SIZE];Parser parser={0};unsigned i,j;int got;Config changed;
    assert(protocol_crc((const uint8_t*)"123456789",9)==0x29b1);
    memset(&p,0,sizeof(p));p.type=MSG_RESULT;p.token=0x12345678;p.mode=3;p.color=5;p.target=2;p.flags=3;
    p.value[0]=-20;p.value[1]=120;p.value[2]=80;
    protocol_encode(&p,data);printf("GOLDEN:");for(i=0;i<PACKET_SIZE;i++)printf("%02x",data[i]);puts("");
    got=0;for(i=0;i<PACKET_SIZE;i++)got+=protocol_feed(&parser,data[i],&out);
    assert(got==1&&out.token==p.token&&out.value[0]==-20);
    for(j=0;j<PACKET_SIZE;j++){
        memset(&parser,0,sizeof(parser));data[j]^=0x40;got=0;
        for(i=0;i<PACKET_SIZE;i++)got+=protocol_feed(&parser,data[i],&out);
        assert(!got);data[j]^=0x40;
        for(i=0;i<PACKET_SIZE;i++)got+=protocol_feed(&parser,data[i],&out);
        assert(got==1);
    }
    assert(config_valid(&config)&&!config_valid(&default_config));
    changed=config;changed.pick.correction[0]=NAN;assert(!config_valid(&changed));
    changed=config;changed.pick.z=-1;assert(!config_valid(&changed));
    changed=config;changed.place[1].align.w=1000;assert(!config_valid(&changed));
    reset();assert(!transfer_start(c,bad));ref=0;assert(!transfer_start(c,r));
    reset();hw=1;assert(!transfer_start(c,r));
    success();
    reset();start();await_request();p=response();p.token--;transfer_observation(&p);assert(transfer.pending);
    p=response();p.mode=4;transfer_observation(&p);assert(transfer.pending);
    now+=config.vision_timeout_ms+1;transfer_tick();assert(transfer.state==FAILED&&!ref&&stop_count==1);
    reset();start();await_request();p=response();now+=config.vision_timeout_ms+1;transfer_observation(&p);assert(transfer.state==FAILED);
    reset();start();for(i=0;i<3;i++){await_request();p=response();p.flags=0;transfer_observation(&p);}assert(transfer.state==FAILED&&!transfer.placed);
    reset();start();await_request();p=response();p.value[0]=0;transfer_observation(&p);assert(transfer.state==FAILED);
    reset();start();await_request();p=response();p.value[0]+=20;transfer_observation(&p);
    assert(platform_busy()&&!transfer.pending); /* fresh image required after correction */
    for(i=0;i<20&&transfer_active();i++){await_request();p=response();p.value[0]+=20;transfer_observation(&p);}
    assert(transfer.state==FAILED); /* correction travel budget */
    reset();start();await_request();
    /* Camera does not translate with extension: fixed material u=170.
       TCP projection must move as extension changes, allowing convergence. */
    for(i=0;i<12&&transfer.state==ALIGN_PICK;i++){
        p=response();p.value[0]=170;transfer_observation(&p);
        if(transfer.state==ALIGN_PICK)await_request();
    }
    assert(transfer.state==LOWER_PICK&&q[0]>config.pick.x);
    reset();start();for(i=0;i<2000&&transfer.state!=VERIFY_HELD;i++){
        now+=20;transfer_tick();if(transfer.pending){p=response();transfer_observation(&p);}
    }
    await_request();p=response();p.value[0]+=10;transfer_observation(&p);assert(transfer.state==FAILED&&!transfer.placed);
    reset();start();transfer.holding=1;transfer_abort("test");j=moves;
    for(i=0;i<100;i++){now+=20;transfer_tick();}assert(moves==(int)j&&transfer.holding);
    puts("PASS controller: protocol corruption/recovery, configuration gates, full 3-slot flow, stale/wrong/duplicate observations, timeout, visual failure, bounded correction, stop");
    return 0;
}
