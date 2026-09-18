#ifndef TRANSFER_PLATFORM_H
#define TRANSFER_PLATFORM_H
#include <stdint.h>
#include "config.h"
#include "protocol.h"
void platform_init(void);
uint32_t platform_ms(void);
void platform_poll(void);
void platform_log(const char *s);
int platform_line(char *dst,unsigned capacity);
int platform_camera_byte(uint8_t *byte);
void platform_camera_send(const Packet *p);
int platform_fault(void);
int platform_stop_pressed(void);
/* Coordinates are commanded estimates; no encoder confirmation is claimed. */
int platform_zero(float x,float z,float theta,float tray,float grip);
int platform_axis(unsigned axis,float target);
int platform_servo(unsigned servo,float degrees);
int platform_busy(void);
void platform_stop(void);
int platform_referenced(void);
float platform_position(unsigned channel); /* 0 x,1 z,2 grip,3 tray,4 theta */
#endif
