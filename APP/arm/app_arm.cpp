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

#ifdef COMPILE_ARM

arm::Kinematics arm_(360, -90, 14.64, 250,
    Matrixf<4,4>().translation(0.0f, 0.0f, 80.0f),
    Matrixf<4,4>().translation(0.0f, 0.0f, 170.0f+250.0f));

// 静态任务，在 CubeMX 中配置
void app_arm_task(void *args) {
    // Wait for system init.
    while(!app_sys_ready()) OS::Task::SleepMilliseconds(10);

    OS::Task::SleepMilliseconds(100);

    float q_data [6] = {50.2f*M_PI/180, 40.5f*M_PI/180, 112.17f*M_PI/180, 40.12f*M_PI/180, 56.21f*M_PI/180, 70.7f*M_PI/180};

    float count = 0;

    while(true) {

        if(++count == 360) count = 0;

        q_data[0] = 50.0f*M_PI/180 + 20.0f*M_PI/180 * sinf(count*M_PI/180);
        q_data[1] = 50.0f*M_PI/180 + 30.0f*M_PI/180 * cosf(count*M_PI/180);

        Matrixf<6,1>tmp_q(q_data);
        Matrixf<4,4> T_end_t = arm_.arm_forward_clc(tmp_q);
        Matrixf<8, 6> Theta = arm_.arm_inverse_clc(T_end_t);
        Matrixf<6, 6> Jacobi = arm_.arm_jacobi_clc(tmp_q);

        app_msg_vofa_send(E_UART_DEBUG,
            // T_end_t[0][0],
            // T_end_t[1][1],
            // T_end_t[2][2],
            Theta[0][0]*180/M_PI,
            Theta[0][1]*180/M_PI,
            Theta[0][2]*180/M_PI,
            Theta[0][3]*180/M_PI,
            Theta[0][4]*180/M_PI,
            Theta[0][5]*180/M_PI,
            arm_.clc_time[0],
            arm_.clc_time[1],
            arm_.clc_time[2]
            // Jacobi[0][0],
            // Jacobi[0][1],
            // Jacobi[0][2],
            // Jacobi[1][0],
            // Jacobi[1][1],
            // Jacobi[1][2],
            // Jacobi[2][0],
            // Jacobi[2][1],
            // Jacobi[2][2]
            );

        OS::Task::SleepMilliseconds(1);
    }
}

void app_arm_init() {

}

#endif
