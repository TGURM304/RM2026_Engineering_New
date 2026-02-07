//
// Created by guan on 2026/1/23.
//
#include "app_arm.h"

#include "app_sys.h"
#include "bsp_uart.h"
#include "sys_task.h"
#include "dev_motor_dm.h"
#include "app_msg.h"
#include "ctrl_motor_base_pid.h"

#include "app_sys.h"
#include "sys_task.h"

#include <sys/types.h>

#include "bsp_rng.h"

#ifdef COMPILE_ARM

arm::Kinematics arm_(360, -90, 14.64, 250,
                     Matrixf<4, 4>().translation(0.0f, 0.0f, 80.0f),
                     Matrixf<4, 4>().translation(0.0f, 0.0f, 170.0f + 250.0f));

arm::Kinematics arm__(360, -90, 14.64, 250,
                     Matrixf<4, 4>().translation(0.0f, 0.0f, 80.0f),
                     Matrixf<4, 4>().translation(0.0f, 0.0f, 170.0f + 250.0f));

auto arm_data = arm_.app_arm_data();
auto arm_data_ = arm__.app_arm_data();

// 静态任务，在 CubeMX 中配置
void app_arm_task(void *args) {
    // Wait for system init.
    while(!app_sys_ready()) OS::Task::SleepMilliseconds(10);

    OS::Task::SleepMilliseconds(100);

    float q_data[6] = {
        50.2f * M_PI / 180, 40.5f * M_PI / 180, 112.17f * M_PI / 180, 40.12f * M_PI / 180, 56.21f * M_PI / 180,
        70.7f * M_PI / 180
    };

    float count = 0;
    int8_t rng[6] = {};

    while(true) {

        if(++count == 360) count = 0;

        float Lim[6][2] = {
            {-M_PI, M_PI},
            {0.0f, M_PI},
            {0.0f, 150.0f * M_PI/180.0f},
            {-M_PI, M_PI},
            {-150.0f * M_PI/180.0f, 150.0f * M_PI/180.0f},
            {-M_PI, M_PI}
        };

        for(uint8_t i = 0; i < 6; i++) {
            rng[i] = bsp_rng_random(-5, 5);
            q_data[i] += rng[i] * 0.1 * M_PI / 180;
            if (q_data[i] > Lim[i][1]) q_data[i] = Lim[i][1];
            if (q_data[i] < Lim[i][0]) q_data[i] = Lim[i][0];
        }

        Matrixf<6, 1> tmp_q(q_data);
        arm_.arm_forward_clc(tmp_q);
        arm_.arm_inverse_clc(arm_data->T_arm_end);
        arm_.arm_jacobi_clc();
        arm_.select_angle();
        arm__.arm_forward_clc(arm_data->upd_angle);

        app_msg_vofa_send(E_UART_DEBUG,
                          // arm_data->cur_angle[arm_.best_idx_t][3] * 180 / M_PI,
                          // arm_data->cur_angle[arm_.best_idx_t][4] * 180 / M_PI,
                          // arm_data->cur_angle[arm_.best_idx_t][5] * 180 / M_PI,
                          arm_data->upd_angle[3][0] * 180 / M_PI,
                          arm_data->upd_angle[4][0] * 180 / M_PI,
                          arm_data->upd_angle[5][0] * 180 / M_PI,
                          tmp_q[3][0] * 180 / M_PI,
                          tmp_q[4][0] * 180 / M_PI,
                          tmp_q[5][0] * 180 / M_PI,
                          arm_.diff_tmp[0],
                          arm_.diff_tmp[1],
                          arm_.diff_tmp[2],
                          arm_.diff_tmp[3],
                          arm_.diff_tmp[4],
                          arm_.diff_tmp[5],
                          arm_.diff_tmp[arm_.best_idx_t]
                          // arm_data->T_arm_end[0][2] - arm_data_->T_arm_end[0][2],
                          // arm_data->T_arm_end[1][2] - arm_data_->T_arm_end[1][2],
                          // arm_data->T_arm_end[2][2] - arm_data_->T_arm_end[2][2],
                          // arm_.clc_time[0],
                          // arm_.clc_time[1],
                          // arm_.clc_time[2],
                          // arm_.clc_time[3]
        );

        OS::Task::SleepMilliseconds(1);
    }
}

void app_arm_init() {

}

#endif
