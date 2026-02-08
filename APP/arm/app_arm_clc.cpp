//
// Created by guan on 2026/1/23.
//
#include "app_arm_clc.h"

#include "app_sys.h"
#include "bsp_uart.h"
#include "sys_task.h"
#include "dev_motor_dm.h"
#include "app_msg.h"
#include "app_arm_def.h"
#include "ctrl_motor_base_pid.h"

#include "app_sys.h"
#include "sys_task.h"

#include <sys/types.h>

#include "bsp_rng.h"

#ifdef COMPILE_ARM

arm::Kinematics arm_(arm_a2, arm_a3, arm_d2, arm_d4,
                     Matrixf<4, 4>().translation(chassis_x, 0.0f, arm_j0_h + chassis_h),
                     Matrixf<4, 4>().translation(0.0f, 0.0f, arm_end_z));

auto arm_data = arm_.app_arm_get_data();

// 静态任务，在 CubeMX 中配置
void app_arm_task(void *args) {
    // Wait for system init.
    while(!app_sys_ready()) OS::Task::SleepMilliseconds(10);

    OS::Task::SleepMilliseconds(100);

    float q_data[6] = {
        50.2f * M_PI / 180, 40.5f * M_PI / 180, 112.17f * M_PI / 180,
        40.12f * M_PI / 180, 56.21f * M_PI / 180, 70.7f * M_PI / 180
    };

    int8_t rng[6] = {};

    while(true) {

        // for(uint8_t i = 0; i < 6; i++) {
        //     rng[i] = bsp_rng_random(-5, 5);
        //     q_data[i] += rng[i] * 0.1 * M_PI / 180;
        //     if (q_data[i] > arm::ARM_JOINT_LIMITS.J[i].max_val) q_data[i] = arm::ARM_JOINT_LIMITS.J[i].max_val;
        //     if (q_data[i] < arm::ARM_JOINT_LIMITS.J[i].min_val) q_data[i] = arm::ARM_JOINT_LIMITS.J[i].min_val;
        // }

        Matrixf<6, 1> tmp_q(q_data);
        arm_.arm_forward_clc(tmp_q);
        arm_.arm_inverse_clc(arm_data->T_arm_end);
        arm_.arm_jacobi_clc();
        arm_.select_angle();

        OS::Task::SleepMilliseconds(1);
    }
}

void app_arm_init() {

}

const arm::app_Arm_data_t *app_arm_data() {
    return arm_data;
}

#endif
