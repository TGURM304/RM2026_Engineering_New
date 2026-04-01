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
#include "app_servo.h"
#include "app_sys.h"
#include "alg_filter.h"
#include "dev_motor_dji.h"
#include "sys_task.h"

#include "app_arm_control.h"
#include "app_arm_clc.h"
#include "app_auto_mine.h"
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

static Algorithm::LowPassFilter pos_lpf[3] = {
    Algorithm::LowPassFilter(3.0),
    Algorithm::LowPassFilter(3.0),
    Algorithm::LowPassFilter(3.0)
};
static Algorithm::LowPassFilter rpy_lpf[3] = {
    Algorithm::LowPassFilter(3.0),
    Algorithm::LowPassFilter(3.0),
    Algorithm::LowPassFilter(3.0)
};

static const float start_deg_q[6] = {
    0.0f, -45.0f*M_PI/180, -47.0f*M_PI/180, 0.0f, 0.0f, 0.0f
};
static const float waiting_deg_q[6] = {
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
};

static const float save1_deg_st[6] = {
    192.61f*M_PI/180, 13.45f*M_PI/180, -31.75f*M_PI/180,
    -92.66f*M_PI/180, 92.99f*M_PI/180, 44.12f*M_PI/180
};

static const float save1_deg_mid[6] = {
    192.01f*M_PI/180, 37.32f*M_PI/180, -36.16f*M_PI/180,
    -99.17f*M_PI/180, 93.02f*M_PI/180, 13.74f*M_PI/180
};

static const float save1_deg_end[6] = {
    154.69f*M_PI/180, 27.15f*M_PI/180, -32.84f*M_PI/180,
    -118.58f*M_PI/180, 94.87f*M_PI/180, 18.63f*M_PI/180
};

static const float save2_deg_st[6] = {
    -187.13f*M_PI/180, 3.74f*M_PI/180, -44.43f*M_PI/180,
    67.48f*M_PI/180, 67.77f*M_PI/180, -39.44f*M_PI/180
};

static const float save2_deg_mid[6] = {
    -177.01f*M_PI/180, 37.32f*M_PI/180, -33.50f*M_PI/180,
    117.30f*M_PI/180, 95.79f*M_PI/180, -20.38f*M_PI/180
};

static const float save2_deg_end[6] = {
    -150.43f*M_PI/180, 6.45f*M_PI/180, -30.47f*M_PI/180,
    140.66f*M_PI/180, 73.77f*M_PI/180, -50.11f*M_PI/180
};

// static const float save2_deg_end_2[6] = {
//     -135.13f*M_PI/180, 16.95f*M_PI/180, -44.46f*M_PI/180,
//     109.75f*M_PI/180, 82.29f*M_PI/180, -30.87f*M_PI/180
// };

// 一键存矿：三段 q + clamp_states[3]/save_dirs[3]；一键取矿可共用 q_*，只改两数组顺序或 wait_* 等。
static mine::AutoMineFsm::Config left_mine_cfg = {
    .q_st = Matrixf<6, 1>(const_cast<float*>(save1_deg_st)),
    .q_mid = Matrixf<6, 1>(const_cast<float*>(save1_deg_mid)),
    .q_end = Matrixf<6, 1>(const_cast<float*>(save1_deg_end)),
    .save_index = 0,
    .clamp_states = {arm::ClampState::Close, arm::ClampState::Open, arm::ClampState::Open},
    .save_dirs = {0, 0, 0},
    .wait_save_done = {false, false, false},
    .step_timeout_ms = 20000, // 单步过久（未到位）则退出，避免永久卡在 ToMid/回预姿态等
};
static mine::AutoMineFsm::Config right_mine_cfg = {
    .q_st = Matrixf<6, 1>(const_cast<float*>(save2_deg_st)),
    .q_mid = Matrixf<6, 1>(const_cast<float*>(save2_deg_mid)),
    .q_end = Matrixf<6, 1>(const_cast<float*>(save2_deg_end)),
    .save_index = 1,
    .clamp_states = {arm::ClampState::Close, arm::ClampState::Open, arm::ClampState::Open},
    .save_dirs = {0, 0, 0},
    .wait_save_done = {false, false, false},
    .step_timeout_ms = 20000,
};
static mine::AutoMineFsm left_mine_fsm(left_mine_cfg);
static mine::AutoMineFsm right_mine_fsm(right_mine_cfg);

static servo::Servo servo_yaw;
static servo::Servo servo_pitch;

static arm::arm_parm g_arm_parm = {
    .J_parm = {
            { .use_mit_pd = false,
                    .joint_pos_pid = {6, 0.08f, 0, 3, 0},
                    .joint_speed_pid = {1.3, 1.0f/1000.f, 0, 5, 3},
                    .Kp = 0.0f, .Kd = 0.5f, .speed_max = 0.0f, .tor_max = 10.0f, .tor_min = -10.0f },
            { .use_mit_pd = false,
                    .joint_pos_pid = {17, 0, 0, 1, 0},
                    .joint_speed_pid = {16, 2.5f/1000.f, 0.5f, 26, 10},
                    .Kp =  0.0f, .Kd = 0.0f, .speed_max = 0.0f, .tor_max = 200.0f, .tor_min = -200.0f },
            { .use_mit_pd = false,
                    .joint_pos_pid = {14, 0, 0, 1, 0},
                    .joint_speed_pid = {13, 3.0f/1000.f, 0, 16, 6},
                    .Kp =  0.0f, .Kd = 0.0f, .speed_max = 0.0f, .tor_max = 54.0f, .tor_min = -54.0f },
            { .use_mit_pd = false,
                    .joint_pos_pid = {11, 0, 0, 6, 0},
                    .joint_speed_pid = {10, 3.5f/1000.f, 0.5f, 10, 5},
                    .Kp =  0.0f, .Kd = 0.0f, .speed_max = 0.0f, .tor_max = 28.0f, .tor_min = -28.0f },
            { .use_mit_pd = true,
                    .joint_pos_pid = {1, 0, 0, 1, 0},
                    .joint_speed_pid = {2, 0.02f/1000.f, 0.02f, 5, 3},
                    .Kp =  25.0f, .Kd = 1.0f, .speed_max = 0.0f, .tor_max = 10.0f, .tor_min = -10.0f },
            { .use_mit_pd = true,
                    .joint_pos_pid = {0, 0, 0, 0, 0},
                    .joint_speed_pid = {0, 0, 0, 0, 0},
                    .Kp =  10.0f, .Kd = 1.0f, .speed_max = 0.0f, .tor_max = 10.0f, .tor_min = -10.0f },
            { .use_mit_pd = true,
                    .joint_pos_pid = {0, 0, 0, 0, 0},
                    .joint_speed_pid = {0, 0, 0, 0, 0},
                    .Kp =  5.0f, .Kd = 1.0f, .speed_max = 0.0f, .tor_max = 10.0f, .tor_min = -10.0f },
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
const auto referee = app_referee_data();
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

static void get_DM_angle(float pos[3], float rpy[3], float& j0_rc_angle) {
    if(arm_data->arm_state == arm::ArmState::Relax) {
        gimbal_arm.angle_upd = false;
        gimbal_arm.j0_rc = 0.0f;
        memset(gimbal_arm.tar_rpy, 0, sizeof(gimbal_arm.tar_rpy));
        memset(gimbal_arm.tar_xyz, 0, sizeof(gimbal_arm.tar_xyz));
        memset(gimbal_arm.q_data, 0, sizeof(gimbal_arm.q_data));
        gimbal_arm.end_angle = 0.0f;
    }else if(arm_data->arm_state == arm::ArmState::Working || arm_data->arm_state == arm::ArmState::Waiting ||
            arm_data->arm_state == arm::ArmState::Float) {
        gimbal_arm.angle_upd = true;
        if (!std::isfinite(j0_rc_angle)) j0_rc_angle = 0.0f;
        gimbal_arm.j0_rc = j0_rc_angle;
        // else gimbal_arm.j0_rc = 0.0f;
        for (int i = 0; i < 3; i++) {
            if (!std::isfinite(pos[i])) pos[i] = 0.0f;
            if (!std::isfinite(rpy[i])) rpy[i] = 0.0f;
        }
        memcpy(gimbal_arm.tar_rpy, rpy, sizeof(gimbal_arm.tar_rpy));
        memcpy(gimbal_arm.tar_xyz, pos, sizeof(gimbal_arm.tar_xyz));

        gimbal_arm.q_data[0] = DM_Joint0.status.pos / 1.5f;
        gimbal_arm.q_data[1] = -(DM_Joint1.status.pos - 90.0f * M_PI / 180);
        gimbal_arm.q_data[2] = DM_Joint2.status.pos - 90.0f * M_PI / 180;
        gimbal_arm.q_data[3] = DM_Joint3.status.pos;
        gimbal_arm.q_data[4] = DM_Joint4.status.pos;
        gimbal_arm.q_data[5] = arm::wrapPi(DM_Joint5.status.pos);
        gimbal_arm.end_angle = DM_Joint_End.status.pos;
    }
}

static Matrixf<6, 1> get_clc_angle(const arm::app_Arm_data_t* data_) {
    if(data_ && data_->angle_upd) {
        Matrixf<6, 1> tmp_pos_t = data_->upd_angle;
        tmp_pos_t[0][0] = tmp_pos_t[0][0] * 1.5f;
        tmp_pos_t[1][0] = -tmp_pos_t[1][0] + 90.0f * M_PI / 180;
        tmp_pos_t[2][0] = tmp_pos_t[2][0] + 90.0f * M_PI / 180;
        return tmp_pos_t;
    }else return g_arm_controller.get_waiting_deg();
}

int16_t chassis_vx = 0, chassis_vy = 0;
int16_t chassis_rotate = 0;
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
        .save_state = {chassis_save_state[1], chassis_save_state[0]}
    };
    app_msg_can_send(E_CAN3, 0x066, pkg);
}

bool gimbal_init = 0;
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

    bool use_delta = false;
    bool tmp_use = false;
    bool lpf_inited = false;
    uint8_t send_count = 0;
    float pos[3], rpy[3];
    float j0_rc_angle = {};
    float lst_pos[3] = {}, lst_rpy[3] = {};
    float DM_pos[6] = {};
    Matrixf<6, 1> tmp_pos = matrixf::zeros<6, 1>();
    Matrixf<6, 1> pos_ref_tmp = matrixf::zeros<6, 1>();
    bool trigger_left = false, trigger_right = false;

    OS::Task::SleepMilliseconds(3000);
    gimbal_init = 1;
    OS::Task::SleepMilliseconds(2000);

    while(true) {

        if (bsp_time_get_ms() - rc->timestamp < 100) {
            chassis_vx = rc->rc_l[0];
            chassis_vy = rc->rc_l[1];
            chassis_rotate = rc->reserved;
            // chassis_save_state[0] = chassis_save_state[1] = rc->s_r;
            j0_rc_angle -= rc->rc_r[0] * (M_PI / 660.0f) * 0.0005f;
            j0_rc_angle = math::limit(j0_rc_angle,
                arm::ARM_JOINT_LIMITS.J[arm::ARM_JOINT_0].min_val,
                arm::ARM_JOINT_LIMITS.J[arm::ARM_JOINT_0].max_val);
            //末端状态
            if(rc->s_r == 1) trigger_left = true;
            else trigger_left = false;
            if(rc->s_r == -1) trigger_right = true;
            else trigger_right = false;
            arm_out.clamp_state = arm::ClampState::Close;
            // if(rc->s_r == 1) arm_out.clamp_state = arm::ClampState::Close;
            // else if(rc->s_r == -1) arm_out.clamp_state = arm::ClampState::Open;
            // else arm_out.clamp_state = arm::ClampState::SetZero;
            //机械臂状态
            if(rc->s_l == -1) g_arm_controller.setState(arm::ArmState::Float);
            else g_arm_controller.setState(arm::ArmState::Working);
            if(rc->s_l == 1) use_delta = true, tmp_use = false;
            else if(rc->s_l == -1) use_delta = false, tmp_use = true;
            else use_delta = tmp_use = false;
        } else {
            g_arm_controller.setState(arm::ArmState::Float);
            chassis_vx = chassis_vy = chassis_rotate = 0.0f;
            chassis_save_state[0] = chassis_save_state[1] = false;
            j0_rc_angle = 0.0f;
            arm_out.clamp_state = arm::ClampState::Open;
            gimbal_arm.angle_upd = false;
            use_delta = false;
            trigger_left = trigger_right = false;
            left_mine_fsm.reset();
            right_mine_fsm.reset();
        }

        if(bsp_time_get_ms() - referee->timestamp < 200) {
            if(bsp_time_get_ms() - referee->custom_controller_timestamp < 200) {
                float pos_raw[3], rpy_raw[3];
                pos_raw[0] = -referee->custom_controller.pos_data[0]*3.0f + 0.550f;
                pos_raw[1] = -referee->custom_controller.pos_data[1]*3.0f;
                pos_raw[2] = (referee->custom_controller.pos_data[2] + 0.233f)*4.0f + 0.610f;

                // rpy_raw[0] = (-referee->custom_controller.rpy_data[2] + 180.0f) * M_PI / 180.0f;
                // rpy_raw[1] = (referee->custom_controller.rpy_data[1] + 90.0f) * M_PI / 180.0f;
                // rpy_raw[2] = -referee->custom_controller.rpy_data[0] * M_PI / 180.0f;
                rpy_raw[0] = -referee->custom_controller.rpy_data[0] * M_PI / 180.0f;
                rpy_raw[1] =  referee->custom_controller.rpy_data[1] * M_PI / 180.0f;
                rpy_raw[2] = -referee->custom_controller.rpy_data[2] * M_PI / 180.0f;

                if (!lpf_inited) {
                    for (uint8_t i = 0; i < 3; ++i) {
                        pos_lpf[i].reset(pos_raw[i]);
                        rpy_lpf[i].reset(rpy_raw[i]);
                    }
                    lpf_inited = true;
                }
                for (uint8_t i = 0; i < 3; ++i) {
                    pos[i] = static_cast<float>(pos_lpf[i].update(pos_raw[i], 0.001));
                    rpy[i] = static_cast<float>(rpy_lpf[i].update(rpy_raw[i], 0.001));
                }
                memcpy(lst_pos, pos, sizeof(lst_pos));
                memcpy(lst_rpy, rpy, sizeof(lst_rpy));
            }else {
                lpf_inited = false;
                pos[0] = pos[1] = pos[2] = lst_pos[0] = lst_pos[1] = lst_pos[2] = 0.0f;
                rpy[0] = rpy[1] = rpy[2] = lst_rpy[0] = lst_rpy[1] = lst_rpy[2] = 0.0f;
            }
        }else {
            lpf_inited = false;
            pos[0] = pos[1] = pos[2] = lst_pos[0] = lst_pos[1] = lst_pos[2] = 0.0f;
            rpy[0] = rpy[1] = rpy[2] = lst_rpy[0] = lst_rpy[1] = lst_rpy[2] = 0.0f;
        }

        DM_pos[0] = DM_Joint0.status.pos;
        DM_pos[1] = DM_Joint1.status.pos;
        DM_pos[2] = DM_Joint2.status.pos;
        DM_pos[3] = DM_Joint3.status.pos;
        DM_pos[4] = DM_Joint4.status.pos;
        DM_pos[5] = DM_Joint5.status.pos;

        get_DM_angle(pos, rpy, j0_rc_angle);
        tmp_pos = get_clc_angle(arm_clc);
        for (int i = 0; i < 6; ++i) {
            if (!std::isfinite(tmp_pos[i][0])) {
                tmp_pos = g_arm_controller.get_waiting_deg();
                break;
            }
        }
        arm_out.g_tor_ref = arm_clc->upd_tar;
        arm_out.g_tor_ref[1][0] *= -1.0, arm_out.g_tor_ref[4][0] *= 1.1;
        // if(use_delta) {
        //     // pos_ref_tmp = tmp_pos;
        //     arm_out.pos_ref = tmp_pos;
        //     // arm_out.use_Tline = true;
        //     // arm_out.pos_Tline = Matrixf<6, 1>(const_cast<float*>(start_deg_q));
        //     // arm_out.pos_ref = Matrixf<6, 1>(const_cast<float*>(save1_end1_deg_q));
        // // }else if(tmp_use) {
        //     // arm_out.use_Tline = false;
        //     // arm_out.pos_ref = Matrixf<6, 1>(const_cast<float*>(save1_end2_deg_q));
        //     // arm_out.pos_ref = g_arm_controller.get_waiting_deg();
        // }else {
        //     arm_out.pos_ref = g_arm_controller.get_waiting_deg();
        //     // arm_out.pos_ref = Matrixf<6, 1>(const_cast<float*>(save1_deg_q));
        // }
        if(use_delta) {
            arm_out.pos_ref = tmp_pos;
        }else {
            arm_out.pos_ref = g_arm_controller.get_waiting_deg();
        }

        const bool mine_allowed = (g_arm_controller.getState() == arm::ArmState::Working);
        if (!mine_allowed || (!trigger_left && !trigger_right)) {
            left_mine_fsm.reset();
            right_mine_fsm.reset();
        }

        int8_t manual_save[2] = {0, 0};  // 手动模式存矿槽，可按需映射遥控器
        chassis_save_state[0] = manual_save[0];
        chassis_save_state[1] = manual_save[1];

        // 存矿到位标志来自底盘 CAN；save_fb_valid = chassis.valid()，超时丢包时不采信标志位，FSM 内仅靠延时推进
        mine::SaveFeedback save_fb = {false, false, false, false};
        bool save_fb_valid = false;
        if (const auto *p = chassis()) {
            save_fb = {p->open_done_L, p->close_done_L, p->open_done_R, p->close_done_R};
            save_fb_valid = chassis.valid();
        }
        float pos_ref_buf_l[6], pos_ref_buf_r[6];
        arm::ClampState clamp_buf_l, clamp_buf_r;
        int8_t save_buf_l[2], save_buf_r[2];
        bool force_work_l = false, force_work_r = false;
        const uint32_t now_ms = bsp_time_get_ms();
        // 左右对称每周期更新（避免原先「左激活时右完全不 update」导致 last_trigger 等状态不同步）
        left_mine_fsm.update(mine_allowed && trigger_left, now_ms, DM_pos,
            arm_out.clamp_state, manual_save, save_fb, save_fb_valid,
            pos_ref_buf_l, clamp_buf_l, save_buf_l, force_work_l);
        right_mine_fsm.update(mine_allowed && trigger_right, now_ms, DM_pos,
            arm_out.clamp_state, manual_save, save_fb, save_fb_valid,
            pos_ref_buf_r, clamp_buf_r, save_buf_r, force_work_r);

        if (left_mine_fsm.isActive()) {
            for (int i = 0; i < 6; i++) arm_out.pos_ref[i][0] = pos_ref_buf_l[i];
            arm_out.clamp_state = clamp_buf_l;
            chassis_save_state[0] = save_buf_l[0];
            chassis_save_state[1] = save_buf_l[1];
        } else if (right_mine_fsm.isActive()) {
            for (int i = 0; i < 6; i++) arm_out.pos_ref[i][0] = pos_ref_buf_r[i];
            arm_out.clamp_state = clamp_buf_r;
            chassis_save_state[0] = save_buf_r[0];
            chassis_save_state[1] = save_buf_r[1];
        }else {
            chassis_save_state[0] = chassis_save_state[1] = 0;
            if(use_delta) {
                arm_out.pos_ref = tmp_pos;
            }else {
                arm_out.pos_ref = g_arm_controller.get_waiting_deg();
            }
        }

        g_arm_controller.update(arm_out);

        servo_yaw.set_angle(180.0f);
        servo_pitch.set_angle(95.0f);

        app_msg_vofa_send(E_UART_DEBUG,
            // gimbal_arm.tar_xyz[0] * 1000,
            // gimbal_arm.tar_xyz[1] * 1000,
            // gimbal_arm.tar_xyz[2] * 1000,
            // gimbal_arm.tar_rpy[0] * 180/M_PI,
            // gimbal_arm.tar_rpy[1] * 180/M_PI,
            // gimbal_arm.tar_rpy[2] * 180/M_PI,
            // bsp_time_get_ms() - referee->timestamp,
            // bsp_time_get_ms() - referee->custom_controller_timestamp,
            // arm_clc->validCount,
            // arm_clc->best_idx_t
            // arm_clc->upd_angle[0][0] * 180/M_PI,
            // arm_clc->upd_angle[1][0] * 180/M_PI,
            // arm_clc->upd_angle[2][0] * 180/M_PI,
            // arm_clc->upd_angle[3][0] * 180/M_PI,
            // arm_clc->upd_angle[4][0] * 180/M_PI,
            // arm_clc->upd_angle[5][0] * 180/M_PI
            // arm_data->pos[0][0] * 180/M_PI,
            // arm_data->vel[0][0],
            // g_arm_controller.joint(arm::ARM_JOINT_0)->status.torque,
            // g_arm_controller.joint(arm::ARM_JOINT_0)->status.vel
            pos[0] * 1000,
            pos[1] * 1000,
            pos[2] * 1000,
            rpy[0] * 180/M_PI,
            rpy[1] * 180/M_PI,
            rpy[2] * 180/M_PI,
            arm_clc->T_arm_end[0][3] * 1000,
            arm_clc->T_arm_end[1][3] * 1000,
            arm_clc->T_arm_end[2][3] * 1000,
            // arm_out.pos_ref[0][0] * 180/M_PI,
            // arm_out.pos_ref[1][0] * 180/M_PI,
            // arm_out.pos_ref[2][0] * 180/M_PI,
            // arm_out.pos_ref[3][0] * 180/M_PI,
            // arm_out.pos_ref[4][0] * 180/M_PI,
            // arm_out.pos_ref[5][0] * 180/M_PI,
            // trigger_left,
            // trigger_right,
            g_arm_controller.joint(arm::ARM_JOINT_0)->status.pos * 180/M_PI,
            g_arm_controller.joint(arm::ARM_JOINT_1)->status.pos * 180/M_PI,
            g_arm_controller.joint(arm::ARM_JOINT_2)->status.pos * 180/M_PI,
            g_arm_controller.joint(arm::ARM_JOINT_3)->status.pos * 180/M_PI,
            g_arm_controller.joint(arm::ARM_JOINT_4)->status.pos * 180/M_PI,
            g_arm_controller.joint(arm::ARM_JOINT_5)->status.pos * 180/M_PI
            // left_mine_fsm.isActive(),
            // right_mine_fsm.isActive(),
            // left_mine_fsm.step(),
            // right_mine_fsm.step()
            // tmp_pos[0][0] * 180/M_PI,
            // tmp_pos[1][0] * 180/M_PI,
            // tmp_pos[2][0] * 180/M_PI,
            // tmp_pos[3][0] * 180/M_PI,
            // tmp_pos[4][0] * 180/M_PI,
            // tmp_pos[5][0] * 180/M_PI
            // arm_clc->T_arm_end[0][3] * 1000,
            // arm_clc->T_arm_end[1][3] * 1000,
            // arm_clc->T_arm_end[2][3] * 1000
        );

        OS::Task::SleepMilliseconds(1);
        if(++ send_count == 10) {
        	send_count = 0;
        	send_msg_to_chassis();
        }
    }
}

void app_gimbal_init() {
    g_arm_controller.init();
    g_arm_controller.setUseFri(arm::ARM_JOINT_3, 0.6, 2.6);
    g_arm_controller.setUseSumAngle(arm::ARM_JOINT_5);
    g_arm_controller.setUseSumAngle(arm::ARM_JOINT_3);

    servo_yaw.init(&htim2, TIM_CHANNEL_1, 50.0, 180.0f, 270.0f, 0.0f);
    servo_pitch.init(&htim2, TIM_CHANNEL_3, 50.0, 95.0f, 270.0f, 0.0f);
}

bool app_gimbal_ready() {
    return gimbal_init;
}

#endif
