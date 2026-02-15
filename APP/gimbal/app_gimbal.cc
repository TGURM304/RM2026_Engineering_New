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

const auto ins = app_ins_data();
const auto rc = bsp_rc_data();
const auto arm_clc = app_arm_data();

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

    while((DM_Joint0.status.err & DM_Joint1.status.err & DM_Joint2.status.err & DM_Joint3.status.err &
        DM_Joint4.status.err & DM_Joint5.status.err & DM_Joint_End.status.err) != 1 ) {
        DM_Joint0.enable(), OS::Task::SleepMilliseconds(1);
        DM_Joint1.enable(), OS::Task::SleepMilliseconds(1);
        DM_Joint2.enable(), OS::Task::SleepMilliseconds(1);
        DM_Joint3.enable(), OS::Task::SleepMilliseconds(1);
        DM_Joint4.enable(), OS::Task::SleepMilliseconds(1);
        DM_Joint5.enable(), OS::Task::SleepMilliseconds(1);
        DM_Joint_End.enable();
        OS::Task::SleepMilliseconds(20);
    }

    chassis.init();

    while(true) {

        if (bsp_time_get_ms() - rc->timestamp < 100) {
            chassis_vx = rc->rc_l[0] * 1.67f;
            chassis_vy = rc->rc_l[1] * 1.67f;
            chassis_rotate = 3.0f * rc->reserved;
            chassis_save_state[0] = rc->s_l;
            chassis_save_state[1] = rc->s_r;
        } else {
            chassis_vx = chassis_vy = chassis_rotate = 0;
            chassis_save_state[0] = chassis_save_state[1] = false;
        }

        DM_Joint0.control(0, 0, 0, 0, 0);
        DM_Joint1.control(0, 0, 0, 0, 0);
        DM_Joint2.control(0, 0, 0, 0, 0);
        DM_Joint3.control(0, 0, 0, 0, 0);
        DM_Joint4.control(0, 0, 0, 0, 0);
        DM_Joint5.control(0, 0, 0, 0, 0);
        DM_Joint_End.control(0, 0, 0, 0, 0);

        // DM_Joint0.control(0, 0, 0, 0, 0);
        // DM_Joint1.control(0, 0, 0, 0, 2.66);
        // DM_Joint2.control(0, 0, 0, 0, 3.6362);
        // DM_Joint3.control(0, 0, 0, 0, 0);
        // DM_Joint4.control(0, 0, 0, 0, 0.1742);
        // DM_Joint5.control(0, 0, 0, 0, 0);
        // DM_Joint_End.control(0, 0, 0, 0, 0);

        app_msg_vofa_send(E_UART_DEBUG,
            DM_Joint0.status.pos,
            DM_Joint1.status.pos,
            DM_Joint2.status.pos,
            DM_Joint3.status.pos,
            DM_Joint4.status.pos,
            DM_Joint5.status.pos,
            DM_Joint_End.status.pos,
            rc->s_r,
            chassis_save_state[1]
            // arm_clc->upd_angle[0][0] * 180/M_PI,
            // arm_clc->upd_angle[1][0] * 180/M_PI,
            // arm_clc->upd_angle[2][0] * 180/M_PI,
            // arm_clc->upd_angle[3][0] * 180/M_PI,
            // arm_clc->upd_angle[4][0] * 180/M_PI,
            // arm_clc->upd_angle[5][0] * 180/M_PI
        );

        OS::Task::SleepMilliseconds(1);
        send_msg_to_chassis();
    }
}

void app_gimbal_init() {
    DM_Joint0.init(), OS::Task::SleepMilliseconds(1);
    DM_Joint1.init(), OS::Task::SleepMilliseconds(1);
    DM_Joint2.init(), OS::Task::SleepMilliseconds(1);
    DM_Joint3.init(), OS::Task::SleepMilliseconds(1);
    DM_Joint4.init(), OS::Task::SleepMilliseconds(1);
    DM_Joint5.init(), OS::Task::SleepMilliseconds(1);
    DM_Joint_End.init(), OS::Task::SleepMilliseconds(1);
}

#endif
