//
// Created by guan on 2026/2/7.
//
#ifndef APP_MSG_DEF_H
#define APP_MSG_DEF_H

#include <cstdint>

struct app_msg_gimbal_to_chassis {
    float vx, vy, rotate;
    int8_t save_state[2];

} __attribute__((packed));

struct app_msg_chassis_to_gimbal {
    uint16_t d_msg, e_msg;

} __attribute__((packed));

#endif //APP_MSG_DEF_H