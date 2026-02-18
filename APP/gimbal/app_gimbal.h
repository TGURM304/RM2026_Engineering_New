//
// Created by fish on 2024/11/16.
//

#pragma once

#include "app_conf.h"

typedef struct {
    bool angle_upd;
    float end_angle;
    float tar_xyz[3];
    float tar_rpy[3];
    float q_data[6];
    float q_d[6];
    float q_dd[6];
} gimbal_arm_t;

#ifdef __cplusplus
extern "C" {
#endif

void app_gimbal_init();
void app_gimbal_task(void *args);
const gimbal_arm_t *gimbal_arm_data();

#ifdef __cplusplus
}
#endif