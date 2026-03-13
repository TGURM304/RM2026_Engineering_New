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

// static void arm_links_init() {
//     const float e = 1e-9f;
//
//     // 惯性张量（单位：kg·m²）
//     Matrixf<3, 3> I1, I2, I3, I4, I5, I6;
//
//     // I1
//     I1[0][0] = 3851115.32f;  I1[0][1] = 48716.4f;     I1[0][2] = 23697.02f;
//     I1[1][0] = 48716.4f;     I1[1][1] = 4514902.84f;  I1[1][2] = -65688.79f;
//     I1[2][0] = 23697.02f;    I1[2][1] = -65688.79f;   I1[2][2] = 2352842.69f;
//     I1 *= e;
//
//     // I2
//     I2[0][0] = 783522.26f;   I2[0][1] = -328.72f;     I2[0][2] = -203486.02f;
//     I2[1][0] = -328.72f;     I2[1][1] = 23937546.86f; I2[1][2] = -41.73f;
//     I2[2][0] = -203486.02f;  I2[2][1] = -41.73f;      I2[2][2] = 24004699.35f;
//     I2 *= e;
//
//     // I3
//     I3[0][0] = 10047985.38f; I3[0][1] = 3517941.27f;  I3[0][2] = 1648446.23f;
//     I3[1][0] = 3517941.27f;  I3[1][1] = 11295497.25f; I3[1][2] = 1882049.72f;
//     I3[2][0] = 1648446.23f;  I3[2][1] = 1882049.72f;  I3[2][2] = 11478405.51f;
//     I3 *= e;
//
//     // I4
//     I4[0][0] = 1633171.41f;  I4[0][1] = -9408.46f;    I4[0][2] = 311.33f;
//     I4[1][0] = -9408.46f;    I4[1][1] = 1554994.96f;  I4[1][2] = 39837.17f;
//     I4[2][0] = 311.33f;      I4[2][1] = 39837.17f;    I4[2][2] = 798585.34f;
//     I4 *= e;
//
//     // I5
//     I5[0][0] = 412978.93f;   I5[0][1] = 16348.19f;    I5[0][2] = -614.6f;
//     I5[1][0] = 16348.19f;    I5[1][1] = 229625.28f;   I5[1][2] = -5848.72f;
//     I5[2][0] = -614.6f;      I5[2][1] = -5848.72f;    I5[2][2] = 389801.96;
//     I5 *= e;
//
//     // I6
//     I6[0][0] = 858165.56f;   I6[0][1] = -13104.78f;   I6[0][2] = 20.93f;
//     I6[1][0] = -13104.78f;   I6[1][1] = 600938.51f;   I6[1][2] = -1484.55f;
//     I6[2][0] = 20.93f;       I6[2][1] = -1484.55f;    I6[2][2] = 502263.56f;
//     I6 *= e;
//
//     Matrixf<3, 1> pc1, pc2, pc3, pc4, pc5, pc6;
//     pc1[0][0] =  0.00075f; pc1[1][0] =  0.00362f; pc1[2][0] =  0.08834f;
//     pc2[0][0] =  0.18728f; pc2[1][0] = -0.00001f; pc2[2][0] = -0.05947f;
//     pc3[0][0] = -0.01792f; pc3[1][0] = -0.00668f; pc3[2][0] =  0.00728f;
//     pc4[0][0] =  0.00044f; pc4[1][0] =  0.00614f; pc4[2][0] =  0.05728f;
//     pc5[0][0] = -0.00104f; pc5[1][0] = -0.02705f; pc5[2][0] = -0.00226f;
//     pc6[0][0] =  0.00001f; pc6[1][0] =  0.0f;     pc6[2][0] =  0.04709f;
//
//     float m1 = 1.62106f, m2 = 0.9753f, m3 = 2.44157f, m4 = 0.76625f, m5 = 0.45733f, m6 = 0.55557f;
//
//     g_arm_links[0] = arm::Link(0.0f,   0.0f,    0.0f,   0.0f, arm::Revolute, 0.0f,   arm::ARM_JOINT_LIMITS.J[arm::ARM_JOINT_0].min_val, arm::ARM_JOINT_LIMITS.J[arm::ARM_JOINT_0].max_val, m1, pc1, I1);
//     g_arm_links[1] = arm::Link(0.0f,   M_PI_2,  arm_d2, 0.0f, arm::Revolute, 0.0f,   arm::ARM_JOINT_LIMITS.J[arm::ARM_JOINT_1].min_val, arm::ARM_JOINT_LIMITS.J[arm::ARM_JOINT_1].max_val, m2, pc2, I2);
//     g_arm_links[2] = arm::Link(arm_a2, 0.0f,    0.0f,   0.0f, arm::Revolute, M_PI_2, arm::ARM_JOINT_LIMITS.J[arm::ARM_JOINT_2].min_val, arm::ARM_JOINT_LIMITS.J[arm::ARM_JOINT_2].max_val, m3, pc3, I3);
//     g_arm_links[3] = arm::Link(arm_a3, M_PI_2,  arm_d4, 0.0f, arm::Revolute, 0.0f,   arm::ARM_JOINT_LIMITS.J[arm::ARM_JOINT_3].min_val, arm::ARM_JOINT_LIMITS.J[arm::ARM_JOINT_3].max_val, m4, pc4, I4);
//     g_arm_links[4] = arm::Link(0.0f,   -M_PI_2, 0.0f,   0.0f, arm::Revolute, 0.0f,   arm::ARM_JOINT_LIMITS.J[arm::ARM_JOINT_4].min_val, arm::ARM_JOINT_LIMITS.J[arm::ARM_JOINT_4].max_val, m5, pc5, I5);
//     g_arm_links[5] = arm::Link(0.0f,   M_PI_2,  0.0f,   0.0f, arm::Revolute, 0.0f,   arm::ARM_JOINT_LIMITS.J[arm::ARM_JOINT_5].min_val, arm::ARM_JOINT_LIMITS.J[arm::ARM_JOINT_5].max_val, m6, pc6, I6);
// }
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
    float q_d[6] = {0, 0, 0, 0, 0, 0};
    float q_dd[6] = {0, 0, 0, 0, 0, 0};
    float xyz[3] = {}, rpy[3] = {};
    float jo_angle = 0.0f;
    Matrixf<4,4> T_c = Matrixf<4, 4>().translation(chassis_x, 0.0f, 0.0f);

    while(true) {

        if(gimbal_data->angle_upd) {
            memcpy(q_data, gimbal_data->q_data, sizeof(q_data));
            memcpy(q_d, gimbal_data->q_d, sizeof(q_data));
            memcpy(q_dd, gimbal_data->q_dd, sizeof(q_data));
            memcpy(xyz, gimbal_data->tar_xyz, sizeof(xyz));
            memcpy(rpy, gimbal_data->tar_rpy, sizeof(rpy));
            arm_->change_a_upd_state(true);
            jo_angle = gimbal_data->j0_rc;
        }else arm_->change_a_upd_state(false);

        Matrixf<6, 1> tmp_q(q_data);
        Matrixf<6, 1> tmp_qd(q_d);
        Matrixf<6, 1> tmp_qdd(q_dd);
        Matrixf<4,4> tar_T =
            Matrixf<4, 4>().translation(xyz[0], xyz[1], xyz[2]) *
            Matrixf<4, 4>().rot_ypr(rpy[2], rpy[1], rpy[0]) *
            Matrixf<4, 4>().rot_rpy(M_PI, M_PI_2, 0.0f);
        Matrixf<4,4> Rz_j0 = Matrixf<4, 4>().rot_z(jo_angle);
        Matrixf<4,4> T_c_inv = matrixf::inv(T_c);
        tar_T = T_c * Rz_j0 * T_c_inv * tar_T;

        arm_->arm_forward_clc(tmp_q);
        // arm_->arm_jacobi_clc();
        arm_->arm_inverse_clc(tar_T);
        arm_->select_angle();
        arm_->arm_newton_euler_clc(tmp_q, tmp_qd, tmp_qdd);

        // app_msg_vofa_send(E_UART_DEBUG,
        //     rpy[0], rpy[1], rpy[2]
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
