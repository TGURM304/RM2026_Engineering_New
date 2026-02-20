//
// Created by guan on 2026/1/23.
//
#include "app_arm_clc.h"

#include "app_sys.h"
#include "bsp_uart.h"
#include "bsp_rng.h"
#include "sys_task.h"
#include "app_msg.h"
#include "app_arm_def.h"
#include "ctrl_motor_base_pid.h"

#include "app_sys.h"
#include "sys_task.h"

#include <sys/types.h>

#include "app_gimbal.h"
#include "robotics.h"

#ifdef COMPILE_ARM

static arm::Link g_arm_links[6];
static arm::Arm_link* arm_ = nullptr;

static void arm_links_init() {
    const float e = 1e-9f;
    const float c_ = M_PI/180.0f;

    // 惯性张量（单位：kg·m²）
    Matrixf<3, 3> I1, I2, I3, I4, I5, I6;

    // I1
    I1[0][0] = 2929412.0f;  I1[0][1] = 21896.0f;   I1[0][2] = 22349.0f;
    I1[1][0] = 21896.0f;    I1[1][1] = 3326646.0f; I1[1][2] = 53289.0f;
    I1[2][0] = 22349.0f;    I1[2][1] = 53289.0f;   I1[2][2] = 1922344.0f;
    I1 *= e;

    // I2
    I2[0][0] = 379480.0f;   I2[0][1] = -174.0f;    I2[0][2] = 127079.0f;
    I2[1][0] = -174.0f;     I2[1][1] = 17864293.0f; I2[1][2] = -15.0f;
    I2[2][0] = 127079.0f;   I2[2][1] = -15.0f;     I2[2][2] = 18071288.0f;
    I2 *= e;

    // I3
    I3[0][0] = 3787022.0f;  I3[0][1] = 320615.0f;  I3[0][2] = -646029.0f;
    I3[1][0] = 320615.0f;   I3[1][1] = 5019218.0f; I3[1][2] = -217418.0f;
    I3[2][0] = -646029.0f;  I3[2][1] = -217418.0f; I3[2][2] = 6405916.0f;
    I3 *= e;

    // I4
    I4[0][0] = 1833108.0f;  I4[0][1] = -211.0f;    I4[0][2] = -795.0f;
    I4[1][0] = -211.0f;     I4[1][1] = 1746121.0f; I4[1][2] = -31949.0f;
    I4[2][0] = -795.0f;     I4[2][1] = -31949.0f;  I4[2][2] = 795712.0f;
    I4 *= e;

    // I5
    I5[0][0] = 1066176.0f;  I5[0][1] = 15431.0f;   I5[0][2] = 6961.0f;
    I5[1][0] = 15431.0f;    I5[1][1] = 334383.0f;  I5[1][2] = 11307.0f;
    I5[2][0] = 6961.0f;     I5[2][1] = 11307.0f;   I5[2][2] = 1051975.0f;
    I5 *= e;

    // I6
    I6[0][0] = 772509.0f;   I6[0][1] = -13949.0f;  I6[0][2] = 290.0f;
    I6[1][0] = -13949.0f;   I6[1][1] = 511879.0f;  I6[1][2] = -468.0f;
    I6[2][0] = 290.0f;      I6[2][1] = -468.0f;    I6[2][2] = 429308.0f;
    I6 *= e;

    Matrixf<3, 1> pc1, pc2, pc3, pc4, pc5, pc6;
    pc1[0][0] = 0.00058f; pc1[1][0] = 0.00711f; pc1[2][0] = 0.04869f;
    pc2[0][0] = 0.185f;   pc2[1][0] = 0.0f;     pc2[2][0] = -0.0051f;
    pc3[0][0] = -0.0019f; pc3[1][0] = 0.0018f;  pc3[2][0] = -0.0011f;
    pc4[0][0] = 0.0035f;  pc4[1][0] = 0.005f;   pc4[2][0] = 0.048f;
    pc5[0][0] = -0.0013f; pc5[1][0] = -0.0017f; pc5[2][0] = -0.0025f;
    pc6[0][0] = 0.00001f; pc6[1][0] = 0.00018f; pc6[2][0] = 0.0456f;

    float m1 = 1.143f, m2 = 0.777f + 0.3f, m3 = 1.82f + 0.2f, m4 = 0.596f + 0.3f, m5 = 0.471f + 0.1f, m6 = 0.376f + 0.2f;

    g_arm_links[0] = arm::Link(0.0f,   0.0f,    0.0f,   0.0f, arm::Revolute, 0.0f,   -240.0f*c_, 245.0f*c_, m1, pc1, I1);
    g_arm_links[1] = arm::Link(0.0f,   M_PI_2,  arm_d2, 0.0f, arm::Revolute, 0.0f,     67.0f*c_, 135.0f*c_, m2, pc2, I2);
    g_arm_links[2] = arm::Link(arm_a2, 0.0f,    0.0f,   0.0f, arm::Revolute, M_PI_2, -137.0f*c_,  13.0f*c_, m3, pc3, I3);
    g_arm_links[3] = arm::Link(arm_a3, M_PI_2,  arm_d4, 0.0f, arm::Revolute, 0.0f,   -175.0f*c_, 115.0f*c_, m4, pc4, I4);
    g_arm_links[4] = arm::Link(0.0f,   -M_PI_2, 0.0f,   0.0f, arm::Revolute, 0.0f,    -80.0f*c_,  82.0f*c_, m5, pc5, I5);
    g_arm_links[5] = arm::Link(0.0f,   M_PI_2,  0.0f,   0.0f, arm::Revolute, 0.0f,        -M_PI,      M_PI, m6, pc6, I6);
}

const arm::app_Arm_data_t* arm_data = nullptr;
auto gimbal_data = gimbal_arm_data();

// 静态任务，在 CubeMX 中配置
void app_arm_task(void *args) {
    // Wait for system init.
    while(!app_sys_ready()) OS::Task::SleepMilliseconds(10);

    OS::Task::SleepMilliseconds(100);

    float q_data[6] = {
        50.2f * M_PI / 180, 40.5f * M_PI / 180, 112.17f * M_PI / 180,
        40.12f * M_PI / 180, 56.21f * M_PI / 180, 70.7f * M_PI / 180
    };
    float tar_q[6] = {0, 0, 0, 0, 0, 0};
    float q_d[6] = {0, 1, 0, 1, 0, 0};
    float q_dd[6] = {0, 0, 0, 1, 0, 0};
    float xyz[3] = {}, rpy[3] = {};

    // int8_t rng[6] = {};

    while(true) {

        // for(uint8_t i = 0; i < 6; i++) {
        //     rng[i] = bsp_rng_random(-5, 5);
        //     q_data[i] += rng[i] * 0.1 * M_PI / 180;
        //     if (q_data[i] > arm::ARM_JOINT_LIMITS.J[i].max_val) q_data[i] = arm::ARM_JOINT_LIMITS.J[i].max_val;
        //     if (q_data[i] < arm::ARM_JOINT_LIMITS.J[i].min_val) q_data[i] = arm::ARM_JOINT_LIMITS.J[i].min_val;
        // }

        if(gimbal_data->angle_upd) {
            memcpy(q_data, gimbal_data->q_data, sizeof(q_data));
            memcpy(q_d, gimbal_data->q_d, sizeof(q_data));
            memcpy(q_dd, gimbal_data->q_dd, sizeof(q_data));
            memcpy(xyz, gimbal_data->tar_xyz, sizeof(xyz));
            memcpy(rpy, gimbal_data->tar_rpy, sizeof(rpy));
            arm_->change_a_upd_state(true);
        }else arm_->change_a_upd_state(false);

        Matrixf<6, 1> tmp_q(q_data);
        Matrixf<6, 1> tmp_qd(q_d);
        Matrixf<6, 1> tmp_qdd(q_dd);
        Matrixf<3, 1> t_p(xyz);
        Matrixf<3, 1> t_rpy(rpy);
        Matrixf<3, 3> t_R = robotics::rpy2r(rpy);
        Matrixf<4, 4> tar_T = robotics::rp2t(t_R, t_p);

        arm_->arm_forward_clc(tmp_q);
        // arm_->arm_jacobi_clc();
        arm_->arm_inverse_clc(tar_T);
        arm_->select_angle();
        arm_->arm_newton_euler_clc(tmp_q, tmp_qd, tmp_qdd);

        // app_msg_vofa_send(E_UART_DEBUG,
        //     gimbal_data->angle_upd,
        //     gimbal_data->q_data[0],
        //     gimbal_data->q_data[1],
        //     gimbal_data->q_data[2],
        //     gimbal_data->q_data[3],
        //     gimbal_data->q_data[4],
        //     gimbal_data->q_data[5]
        //     );

        OS::Task::SleepMilliseconds(1);
    }
}

void app_arm_init() {
    arm_links_init();
    static arm::Arm_link arm_obj(g_arm_links,
        Matrixf<4, 4>().translation(chassis_x, 0.0f, arm_j0_h + chassis_h),
        Matrixf<4, 4>().translation(0.0f, 0.0f, arm_end_z));
    arm_ = &arm_obj;
    arm_data = arm_->app_arm_get_data();
}

const arm::app_Arm_data_t *app_arm_data() {
    return arm_data;
}

#endif
