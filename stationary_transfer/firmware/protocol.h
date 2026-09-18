#ifndef TRANSFER_PROTOCOL_H
#define TRANSFER_PROTOCOL_H
#include <stdint.h>
#define PACKET_SIZE 30
#define MSG_REQUEST 1
#define MSG_RESULT 2
#define MODE_MATERIAL 1
#define MODE_RING 2
#define MODE_HELD 3
#define MODE_PLACED 4
#define FLAG_VALID 1
#define FLAG_STABLE 2
typedef struct {
    uint32_t token;
    uint8_t type, mode, color, target, flags;
    int16_t value[8];
} Packet;
typedef struct { uint8_t bytes[PACKET_SIZE]; unsigned count; } Parser;
uint16_t protocol_crc(const uint8_t *p, unsigned n);
void protocol_encode(const Packet *p, uint8_t out[PACKET_SIZE]);
int protocol_feed(Parser *p, uint8_t byte, Packet *out);
#endif
