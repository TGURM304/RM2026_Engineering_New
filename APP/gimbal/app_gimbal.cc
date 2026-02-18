//
// Created by fish on 2024/11/17.
//

#include "app_gimbal.h"
#include "app_msg.h"
#include "app_referee.h"
#include "app_referee_def.h"
#include "ctrl_motor_base_pid.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include "app_ins.h"
#include "app_motor.h"
#include "app_msg_def.h"
#include "app_sys.h"
#include "dev_motor_dji.h"
#include "sys_task.h"

#include "app_arm_control.h"
#include "app_arm_clc.h"
#include "dev_motor_dm.h"

#ifdef COMPILE_GIMBAL

using namespace Motor;
using namespace Controller;

DMMotor DM_Joint0("Joint0",DMMotor::J4310,{
    .slave_id = 0x00,
    .master_id = 0x10,
    .port = E_CAN3,
    .mode = DMMotor::MIT,
    .p_max = 12.5, .v_max = 30, .t_max = 10, .kp_max = 500, .kd_max = 5
});
//J10010
DMMotor DM_Joint1("Joint1",DMMotor::J4310,{
    .slave_id = 0x01,
    .master_id = 0x11,
    .port = E_CAN2,
    .mode = DMMotor::MIT,
    .p_max = 12.5, .v_max = 20, .t_max = 200, .kp_max = 12.5, .kd_max = 5
});
DMMotor DM_Joint2("Joint2",DMMotor::J8009P,{
    .slave_id = 0x02,
    .master_id = 0x12,
    .port = E_CAN3,
    .mode = DMMotor::MIT,
    .p_max = 12.5, .v_max = 45, .t_max = 54, .kp_max = 12.5, .kd_max = 5
});
//J4340
DMMotor DM_Joint3("Joint3",DMMotor::J4310,{
    .slave_id = 0x03,
    .master_id = 0x13,
    .port = E_CAN2,
    .mode = DMMotor::MIT,
    .p_max = 12.5, .v_max = 10, .t_max = 28, .kp_max = 500, .kd_max = 5
});
DMMotor DM_Joint4("Joint4",DMMotor::J4310,{
    .slave_id = 0x04,
    .master_id = 0x14,
    .port = E_CAN1,
    .mode = DMMotor::MIT,
    .p_max = 12.5, .v_max = 30, .t_max = 10, .kp_max = 500, .kd_max = 5
});
DMMotor DM_Joint5("Joint5",DMMotor::J4310,{
    .slave_id = 0x05,
    .master_id = 0x15,
    .port = E_CAN1,
    .mode = DMMotor::MIT,
    .p_max = 12.5, .v_max = 30, .t_max = 10, .kp_max = 500, .kd_max = 5
});
DMMotor DM_Joint_End("Joint_end",DMMotor::J4310,{
    .slave_id = 0x06,
    .master_id = 0x16,
    .port = E_CAN1,
    .mode = DMMotor::MIT,
    .p_max = 12.5, .v_max = 30, .t_max = 10, .kp_max = 500, .kd_max = 5
});

static const float start_deg_q[6] = {
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
};
static const float waiting_deg_q[6] = {
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
};

static arm::arm_parm g_arm_parm = {
    .J_parm = {
            { .Kp = 20.0f, .Kd = 3.0f, .speed_max = 0.0f, .tor_max = 10.0f, .tor_min = -10.0f },
            { .Kp =  0.0f, .Kd = 0.0f, .speed_max = 0.0f, .tor_max = 200.0f, .tor_min = -200.0f },
            { .Kp =  0.0f, .Kd = 0.0f, .speed_max = 0.0f, .tor_max = 54.0f, .tor_min = -54.0f },
            { .Kp =  0.0f, .Kd = 0.0f, .speed_max = 0.0f, .tor_max = 28.0f, .tor_min = -28.0f },
            { .Kp =  0.0f, .Kd = 0.0f, .speed_max = 0.0f, .tor_max = 10.0f, .tor_min = -10.0f },
            { .Kp =  0.0f, .Kd = 0.0f, .speed_max = 0.0f, .tor_max = 10.0f, .tor_min = -10.0f },
            { .Kp =  5.0f, .Kd = 2.0f, .speed_max = 0.0f, .tor_max = 10.0f, .tor_min = -10.0f },
    },
    .start_deg = Matrixf<6, 1>(const_cast<float*>(start_deg_q)),
    .waiting_deg = Matrixf<6, 1>(const_cast<float*>(waiting_deg_q))
};

arm::ArmController g_arm_controller(
    g_arm_parm,
    &DM_Joint0, &DM_Joint1, &DM_Joint2, &DM_Joint3,
    &DM_Joint4, &DM_Joint5, &DM_Joint_End
);

const auto ins = app_ins_data();
const auto rc = bsp_rc_data();
const auto arm_data = g_arm_controller.app_arm_ctr_data();
arm::ctrl_out_data_t arm_out;
gimbal_arm_t gimbal_arm = {
    .angle_upd = false,
    .end_angle = 0,
    .tar_xyz = {0, 0, 0},
    .tar_rpy = {0, 0, 0},
    .q_data = {0, 0, 0, 0, 0, 0},
    .q_d = {0, 0, 0, 0, 0, 0},
    .q_dd = {0, 0, 0, 0, 0, 0}
};

const gimbal_arm_t *gimbal_arm_data() {
    return &gimbal_arm;
}

static void get_DM_angle() {
    if(arm_data->arm_state == arm::ArmState::Relax) {
        gimbal_arm.angle_upd = false;
        memset(gimbal_arm.q_data, 0, sizeof(gimbal_arm.q_data));
    }else if(arm_data->arm_state == arm::ArmState::Working || arm_data->arm_state == arm::ArmState::Waiting) {
        gimbal_arm.q_data[0] = DM_Joint0.status.pos;
        gimbal_arm.q_data[1] = -(DM_Joint1.status.pos - 90.0f * M_PI / 180);
        gimbal_arm.q_data[2] = DM_Joint2.status.pos - 90.0f * M_PI / 180;
        gimbal_arm.q_data[3] = DM_Joint3.status.pos;
        gimbal_arm.q_data[4] = DM_Joint4.status.pos;
        gimbal_arm.q_data[5] = DM_Joint5.status.pos;
        gimbal_arm.end_angle = DM_Joint_End.status.pos;
        gimbal_arm.angle_upd = true;
    }
}

float chassis_vx = 0, chassis_vy = 0;
float chassis_rotate = 0;
// 0: lift	1: right
int8_t chassis_save_state[2] = {};

//双板通信
//收
app_msg_can_receiver <app_msg_chassis_to_gimbal> chassis(E_CAN3, 0x033);
//发
void send_msg_to_chassis() {
    app_msg_gimbal_to_chassis pkg = {
        .vx = chassis_vx,
        .vy = chassis_vy,
        .rotate = chassis_rotate,
        .save_state = {chassis_save_state[0], chassis_save_state[1]}
    };
    app_msg_can_send(E_CAN3, 0x066, pkg);
}

// 静态任务，在 CubeMX 中配置
void app_gimbal_task(void *args) {
    // Wait for system init.
    while(!app_sys_ready())
        OS::Task::SleepMilliseconds(10);

    const auto arm_clc = app_arm_data();

    while((DM_Joint0.status.err & DM_Joint1.status.err & DM_Joint2.status.err & DM_Joint3.status.err &
        DM_Joint4.status.err & DM_Joint5.status.err & DM_Joint_End.status.err) != 1 ) {
        g_arm_controller.enable();
        OS::Task::SleepMilliseconds(20);
    }

    chassis.init();

    float j0_q = 0;

    while(true) {

        if (bsp_time_get_ms() - rc->timestamp < 100) {
            chassis_vx = rc->rc_l[0] * 1.67f;
            chassis_vy = rc->rc_l[1] * 1.67f;
            j0_q += rc->rc_r[0] * 0.000005f;
            j0_q = math::limit(j0_q, arm::ARM_JOINT_LIMITS.J[0].min_val, arm::ARM_JOINT_LIMITS.J[0].max_val);
            chassis_rotate = 3.0f * rc->reserved;
            chassis_save_state[0] = rc->s_l;
            chassis_save_state[1] = rc->s_r;
        } else {
            j0_q = chassis_vx = chassis_vy = chassis_rotate = 0;
            chassis_save_state[0] = chassis_save_state[1] = false;
            gimbal_arm.angle_upd = false;
        }

        get_DM_angle();
        arm_out.g_tor_ref = arm_clc->upd_tar;
        arm_out.g_tor_ref[1][0] *= -1.1, arm_out.g_tor_ref[4][0] *= 1.1;
        arm_out.pos_ref = matrixf::zeros<6, 1>();
        arm_out.pos_ref[0][0] = j0_q;
        g_arm_controller.update(arm_out);

        // DM_Joint0.control(j0_q, 0, 20, 3, arm_clc->upd_tar[0][0]);
        // DM_Joint1.control(0, 0, 0, 0, -arm_clc->upd_tar[1][0]*1.1);
        // DM_Joint2.control(0, 0, 0, 0, arm_clc->upd_tar[2][0]*1.0);
        // DM_Joint3.control(0, 0, 0, 0, arm_clc->upd_tar[3][0]*1.0);
        // DM_Joint4.control(0, 0, 0, 0, arm_clc->upd_tar[4][0]*1.1);
        // DM_Joint5.control(0, 0, 0, 0, arm_clc->upd_tar[5][0]*1.0);
        // DM_Joint_End.control(0, 0, 0, 0, 0);

        app_msg_vofa_send(E_UART_DEBUG,
            gimbal_arm.q_data[0] * 180/M_PI,
            gimbal_arm.q_data[1] * 180/M_PI,
            gimbal_arm.q_data[2] * 180/M_PI,
            gimbal_arm.q_data[3] * 180/M_PI,
            gimbal_arm.q_data[4] * 180/M_PI,
            gimbal_arm.q_data[5] * 180/M_PI,
            arm_data->pos[0][0] * 180/M_PI,
            arm_data->pos[1][0] * 180/M_PI,
            arm_data->pos[2][0] * 180/M_PI
            // chassis_save_state[1]
            // arm_clc->upd_tar[0][0],
            // arm_clc->upd_tar[1][0],
            // arm_clc->upd_tar[2][0],
            // arm_clc->upd_tar[3][0],
            // arm_clc->upd_tar[4][0],
            // arm_clc->upd_tar[5][0]
        );

        OS::Task::SleepMilliseconds(1);
        send_msg_to_chassis();
    }
}

void app_gimbal_init() {
    g_arm_controller.init();
}

#endif
