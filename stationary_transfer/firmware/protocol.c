#include "protocol.h"
#include <string.h>
uint16_t protocol_crc(const uint8_t *p, unsigned n) {
    uint16_t crc=0xffff; unsigned i;
    while(n--) { crc^=(uint16_t)*p++<<8; for(i=0;i<8;i++) crc=(crc&0x8000)?(uint16_t)((crc<<1)^0x1021):(uint16_t)(crc<<1); }
    return crc;
}
void protocol_encode(const Packet *p, uint8_t out[PACKET_SIZE]) {
    unsigned i; uint16_t crc;
    memset(out,0,PACKET_SIZE); out[0]=0xa5; out[1]=0x5a; out[2]=1; out[3]=p->type;
    for(i=0;i<4;i++) out[4+i]=(uint8_t)(p->token>>(8*i));
    out[8]=p->mode; out[9]=p->color; out[10]=p->target; out[11]=p->flags;
    for(i=0;i<8;i++) { out[12+2*i]=(uint8_t)p->value[i]; out[13+2*i]=(uint8_t)((uint16_t)p->value[i]>>8); }
    crc=protocol_crc(out,28); out[28]=(uint8_t)crc; out[29]=(uint8_t)(crc>>8);
}
int protocol_feed(Parser *p, uint8_t byte, Packet *out) {
    unsigned i; uint16_t crc;
    p->bytes[p->count++]=byte;
    while(p->count && (p->bytes[0]!=0xa5 || (p->count>1 && p->bytes[1]!=0x5a))) {
        memmove(p->bytes,p->bytes+1,--p->count);
    }
    if(p->count<PACKET_SIZE) return 0;
    crc=(uint16_t)(p->bytes[28]|((uint16_t)p->bytes[29]<<8));
    if(p->bytes[2]!=1 || crc!=protocol_crc(p->bytes,28)) {
        memmove(p->bytes,p->bytes+1,--p->count); return 0;
    }
    out->type=p->bytes[3]; out->token=0;
    for(i=0;i<4;i++) out->token|=(uint32_t)p->bytes[4+i]<<(8*i);
    out->mode=p->bytes[8]; out->color=p->bytes[9]; out->target=p->bytes[10]; out->flags=p->bytes[11];
    for(i=0;i<8;i++) out->value[i]=(int16_t)(p->bytes[12+2*i]|((uint16_t)p->bytes[13+2*i]<<8));
    p->count=0; return 1;
}
