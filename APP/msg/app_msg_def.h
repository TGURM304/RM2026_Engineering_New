//
// Created by guan on 2026/2/7.
//
#ifndef APP_MSG_DEF_H
#define APP_MSG_DEF_H

#include <cstdint>

struct app_msg_gimbal_to_chassis {
    int16_t vx, vy, rotate;
    int8_t save_state[2];

} __attribute__((packed));

struct app_msg_chassis_to_gimbal {
    bool open_done_L, close_done_L;
    bool open_done_R, close_done_R;

} __attribute__((packed));

#endif //APP_MSG_DEF_H