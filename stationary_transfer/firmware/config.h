#ifndef TRANSFER_CONFIG_H
#define TRANSFER_CONFIG_H
#include <stdint.h>
typedef struct { int16_t x,y,w,h,u,v; } View;
typedef struct {
    float x,theta,z;
    View align,verify;
    /* Signed inverse image Jacobian: dq = matrix * (observed - anchor).
       x in mm, theta in degrees. Calibrate WITH moving camera at this pose. */
    float correction[4];
    /* Expected target pixel (TCP projection) slope with x/theta.
       [du/dx, du/dtheta, dv/dx, dv/dtheta]. Separate verification height. */
    float align_slope[4], verify_slope[4];
} WorkPose;
typedef struct {
    uint8_t mechanics_ready,vision_ready;
    uint8_t positive_dir[2];
    float steps_mm[2], axis_min[2],axis_max[2],ms_mm[2];
    float safe_z,clear_x,home_theta,tray_angles[3];
    float servo_span[3],servo_min[3],servo_max[3],servo_ms_degree;
    float max_dx,max_dtheta,max_total_dx,max_total_dtheta;
    uint16_t motor_rpm; uint8_t motor_acc;
    uint32_t settle_ms,vision_timeout_ms;
    int16_t tolerance_px;
    WorkPose pick,place[3];
} Config;
extern const Config config;
int config_valid(const Config *c);
#endif
