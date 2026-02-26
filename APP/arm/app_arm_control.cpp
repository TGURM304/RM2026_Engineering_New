//
// Created by guan on 2026/2/8.
//

#include "app_arm_control.h"

#include "bsp_time.h"
#include <cmath>

#include "sys_task.h"

namespace arm {

    static float calc_delta(float full, float current, float target) {
        float dt = target - current;
        if(2 * dt >  full) dt -= full;
        if(2 * dt < -full) dt += full;
        return dt;
    }

    void ArmController::init() {
        for (uint8_t i = 0; i < 7; i++) {
            if(joints_[i]) {
                joints_[i]->init();
                OS::Task::SleepMilliseconds(1);
            }
        }
        arm_state_ = ArmState::Waiting;
    }

    void ArmController::disable() {
        for (uint8_t i = 0; i < 7; i++) {
            if(joints_[i]) {
                joints_[i]->disable();
                OS::Task::SleepMilliseconds(1);
            }
        }
        arm_state_ = ArmState::Relax;
    }

    void ArmController::enable() {
        for (uint8_t i = 0; i < 7; i++) {
            if(joints_[i]) {
                joints_[i]->enable();
                OS::Task::SleepMilliseconds(1);
            }
        }
        arm_state_ = ArmState::Working;
    }

    void ArmController::update(const ctrl_out_data_t& arm_cmd) {
        if (!valid_) return;

        // 从电机读反馈，更新当前状态（与 param 单位一致，为弧度）
        for (uint8_t j = 0; j < 6; j++) {
            if (joints_[j]) {
                cur_q[j][0] = joints_[j]->status.pos;
                data_.vel[j][0] = joints_[j]->status.vel;
                if(use_sum_angle_[j]) {
                    data_.pos[j][0] -= calc_delta(25, cur_q[j][0], lst_q[j][0]);
                    lst_q[j][0] = cur_q[j][0];
                } else {
                    data_.pos[j][0] = cur_q[j][0];
                }
            }
        }
        if (joints_[6]) data_.clamp_pos = joints_[6]->status.pos;
        data_.arm_state = arm_state_;
        data_.clamp_state = clamp_state_;

        // 暂未完善保护

        // 位置超限失能保护
        float lim = 5*M_PI/180;
        for (uint8_t i = 0; i < 6; i++) {
            if (!joints_[i]) continue;
            float pos = data_.pos[i][0];
            if (pos < ARM_JOINT_RAW_LIMITS.J[i].min_val - lim || pos > ARM_JOINT_RAW_LIMITS.J[i].max_val + lim) {
                joints_[i]->control(0,0,0,0,0);
                joints_[i]->disable();
                return;
            }
            if (last_arm_state_ != arm_state_) {
                for (uint8_t j = 0; j < 6; j++) {
                    joint_filter[j].reset(data_.pos[j][0]);
                }
                parm_.J_parm[i].joint_pos_pid.clear();
                parm_.J_parm[i].joint_speed_pid.clear();
            }
        }
        last_arm_state_ = arm_state_;

        if (arm_state_ == ArmState::Relax) {
            q_ref_ = g_ref_ = matrixf::zeros<6, 1>();
            for (uint8_t i = 0; i < 6; i++) {
                tline_[i].trajectory_active_ = false;
            }
            disable();
            return;
        }
        g_ref_ = arm_cmd.g_tor_ref;

        if (arm_state_ == ArmState::Waiting) {
            q_ref_ = parm_.waiting_deg;
            clamp_state_ = ClampState::Open;
            for (uint8_t i = 0; i < 6; i++) {
                tline_[i].trajectory_active_ = false;
            }
            applyJointLimits(q_ref_);
        }else {
            q_ref_ = arm_cmd.pos_ref;
            applyJointLimits(q_ref_);
            clamp_state_ = arm_cmd.clamp_state;
            // Matrixf<6, 1> target = arm_cmd.pos_ref;
            // applyJointLimits(target);
            //
            // const float pos_thresh = 1e-2f;
            // bool t_change[6] = {false, false, false, false, false, false};
            // for (uint8_t j = 0; j < 6; j++) {
            //     if (fabsf(target[j][0] - q_goal_[j][0]) > pos_thresh) {
            //         t_change[j] = true;
            //     }
            //     if (t_change[j] && use_tline_) {
            //         q_goal_[j][0] = target[j][0];
            //         tline_[j].t_start_us_ = bsp_time_get_us();
            //         tline_[j].trajectory_active_ = true;
            //         planTline(j, data_.pos[j][0], target[j][0], data_.vel[j][0], 0.0f);
            //     }
            //     if (tline_[j].trajectory_active_ && use_tline_) {
            //         float t_elapsed = static_cast<float>(bsp_time_get_us() - tline_[j].t_start_us_) * 1e-6f;
            //         bool all_done = true;
            //         if (t_elapsed < tline_[j].T) all_done = false;
            //         tline_[j].tmp_t = t_elapsed;
            //         float q_now, qd_now;
            //         evalTline(j, t_elapsed, &q_now, &qd_now);
            //         q_ref_[j][0] = q_now;
            //         if (all_done) tline_[j].trajectory_active_ = false;
            //     } else {
            //         q_ref_[j][0] = target[j][0];
            //     }
            // }
        }

        for (uint8_t j = 0; j < 6; j++) {
            if (!joints_[j]) continue;
            q_ref_[j][0] = joint_filter[j].update(q_ref_[j][0], 0.001f);
            if(use_joint_fri_[j]) {
                float fri_tor = parm_.J_parm[j].joint_fri * tanhf(parm_.J_parm[j].k_f * data_.vel[j][0]);
                g_ref_[j][0] += fri_tor;
            }
            if(arm_state_ == ArmState::Float) {
                joints_[j]->control(0, 0, 0, 0, g_ref_[j][0]);
                continue;
            }
            if(parm_.J_parm[j].use_mit_pd) {
                joints_[j]->control(q_ref_[j][0], parm_.J_parm[j].speed_max,
                    parm_.J_parm[j].Kp, parm_.J_parm[j].Kd, g_ref_[j][0]);
            }else {
                float vel_ref = parm_.J_parm[j].joint_pos_pid.update(data_.pos[j][0], q_ref_[j][0]);
                float tor_ref = parm_.J_parm[j].joint_speed_pid.update(data_.vel[j][0], vel_ref);
                tor_ref += g_ref_[j][0];
                tor_ref = math::limit(tor_ref, parm_.J_parm[j].tor_min, parm_.J_parm[j].tor_max);
                joints_[j]->control(0, 0, 0, 0, tor_ref);
            }
        }

        if (joints_[ARM_JOINT_END]) {
            float clamp_pos = 0.0f;
            switch (arm_cmd.clamp_state) {
                case ClampState::Open:    clamp_pos = ARM_JOINT_RAW_LIMITS.J_end.min_val; break;
                case ClampState::Close:   clamp_pos = ARM_JOINT_RAW_LIMITS.J_end.max_val; break;
                case ClampState::SetZero: clamp_pos = ARM_JOINT_RAW_LIMITS.J_end.min_val/3; break;
            }
            joints_[ARM_JOINT_END]->control(clamp_pos, parm_.J_parm[ARM_JOINT_END].speed_max,
                parm_.J_parm[ARM_JOINT_END].Kp, parm_.J_parm[ARM_JOINT_END].Kd, 0);
        }
    }

    // T型轨迹规划
    void ArmController::planTline(uint8_t j, float q0, float q1, float v0, float v1) {
        if (j >= 6) return;
        float dq = q1 - q0;
        const float eps = 1e-6f;
        if (fabsf(dq) < eps && fabsf(v0) < eps && fabsf(v1) < eps) {
            tline_[j].q0 = q0, tline_[j].q1 = q1;
            tline_[j].v0 = v0, tline_[j].v1 = v1;
            tline_[j].T = 0.f;
            tline_[j].sign_ = 1.0f;
            tline_[j].valid = false;
            return;
        }

        float sign = (dq >= 0.f) ? 1.0f : -1.0f;
        float q0_p = sign * q0, q1_p = sign * q1;
        float v0_p = sign * v0, v1_p = sign * v1;
        float dq_p = q1_p - q0_p;

        float vmax = parm_.J_parm[j].vmax_traj;
        float amax = parm_.J_parm[j].amax_traj;
        if (vmax <= 0.f) vmax = 2.0f;
        if (amax <= 0.f) amax = 2.0f;

        float inner = 2.0f*amax*dq_p + (v1_p*v1_p + v0_p*v0_p)*0.5f;
        if (inner < eps) inner = 0.0f;
        float v_temp = sqrtf(inner);
        float vlin_p = (v_temp < vmax) ? v_temp : vmax;

        float Ta = (vlin_p - v0_p) / amax;
        float Sa = v0_p*Ta + amax*Ta*Ta*0.5f;

        float accel_dist = (2.0f*vlin_p*vlin_p - v0_p*v0_p - v1_p*v1_p) / (2.0f*amax);
        if (vlin_p < eps) vlin_p = 0.0f;
        float Tv = (dq_p - accel_dist) / vlin_p;
        float Sv = vlin_p * Tv;

        float Td = (vlin_p - v1_p) / amax;

        tline_[j].q0 = q0_p, tline_[j].q1 = q1_p;
        tline_[j].v0 = v0_p, tline_[j].v1 = v1_p;
        tline_[j].Ta = Ta, tline_[j].Tv = Tv, tline_[j].Td = Td;
        tline_[j].T = Ta + Tv + Td;
        tline_[j].vlin = vlin_p;
        tline_[j].Sa = Sa, tline_[j].Sv = Sv;
        tline_[j].amax = amax;
        tline_[j].sign_ = sign;
        tline_[j].valid = true;
    }

    void ArmController::evalTline(uint8_t j, float t, float* q_out, float* qd_out) {
        if (j >= 6 || !q_out || !qd_out) return;
        if (!tline_[j].valid) {
            *q_out = tline_[j].q0;
            *qd_out = tline_[j].v0;
            return;
        }

        tline_->state[0] = tline_->state[1] = tline_->state[2] = false;

        if (t <= 0.f) {
            *q_out = tline_[j].q0;
            *qd_out = tline_[j].v0;
            return;
        }
        // if (t > tline_[j].T) {
        //     *q_out = tline_[j].q1;
        //     *qd_out = tline_[j].v1;
        //     return;
        // }
        if (t < tline_[j].Ta) {
            tline_->state[0] = true;
            *q_out = tline_[j].q0 + tline_[j].v0*t + tline_[j].amax*t*t*0.5f;
            *qd_out = tline_[j].v0 + tline_[j].amax*t;
        } else if (t < tline_[j].Ta + tline_[j].Tv) {
            tline_->state[1] = true;
            *q_out = tline_[j].q0 + tline_[j].Sa + tline_[j].vlin*(t - tline_[j].Ta);
            *qd_out = tline_[j].vlin;
        } else {
            tline_->state[2] = true;
            float t_dec = t - tline_[j].Ta - tline_[j].Tv;
            *q_out = tline_[j].q0 + tline_[j].Sa + tline_[j].Sv + tline_[j].vlin*t_dec - tline_[j].amax*t_dec*t_dec*0.5f;
            *qd_out = tline_[j].vlin - tline_[j].amax*t_dec;
        }
        *q_out *= tline_[j].sign_;
        *qd_out *= tline_[j].sign_;
    }
}
