//
// Created by guan on 2026/2/8.
//

#include "app_arm_control.h"

#include <cmath>

namespace arm {

    void ArmController::init() {
        for (uint8_t i = 0; i < 7; i++) {
            if(joints_[i]) joints_[i]->init();
        }
    }

    void ArmController::disable() {
        for (uint8_t i = 0; i < 7; i++) {
            if(joints_[i]) joints_[i]->disable();
        }
    }

    void ArmController::enable() {
        for (uint8_t i = 0; i < 7; i++) {
            if(joints_[i]) joints_[i]->enable();
        }
    }

    void ArmController::update(const ctrl_out_data_t& arm_cmd) {
        if (!valid_) return;

        // 从电机读反馈，更新当前状态（与 param 单位一致，一般为弧度）
        for (uint8_t j = 0; j < 6; j++) {
            if (joints_[j]) data_.pos[j][0] = joints_[j]->status.pos;
        }
        if (joints_[6]) data_.clamp_pos = joints_[6]->status.pos;
        data_.arm_state = arm_state_;
        data_.clamp_state = clamp_state_;

        // 暂未完善保护

        // 位置超限失能保护
        for (uint8_t i = 0; i < 6; i++) {
            if (!joints_[i]) continue;
            float pos = joints_[i]->status.pos;
            if (pos < ARM_JOINT_LIMITS.J[i].min_val || pos > ARM_JOINT_LIMITS.J[i].max_val) {
                joints_[i]->disable();
                return;
            }
        }
        if (arm_state_ == ArmState::Relax) {
            disable();
            return;
        }

        if (arm_state_ == ArmState::Waiting) {
            q_ref_ = parm_.waiting_deg;
            clamp_state_ = ClampState::Open;
        }else {
            q_ref_ = arm_cmd.pos_ref;
            clamp_state_ = arm_cmd.clamp_state;
        }

        applyJointLimits(q_ref_);

        // 暂未加入重力补偿
        for (uint8_t j = 0; j < 6; j++) {
            if (joints_[j])
                joints_[j]->control(q_ref_[j][0], parm_.J_parm[j].speed_max,
                    parm_.J_parm[j].Kp, parm_.J_parm[j].Kd, 0);
        }

        if (joints_[ARM_JOINT_END]) {
            float clamp_pos = 0.0f;
            switch (arm_cmd.clamp_state) {
                case ClampState::Open:    clamp_pos = ARM_JOINT_LIMITS.J_end.max_val; break;
                case ClampState::Close:   clamp_pos = ARM_JOINT_LIMITS.J_end.min_val; break;
                case ClampState::SetZero: clamp_pos = ARM_JOINT_LIMITS.J_end.max_val/2; break;
            }
            joints_[ARM_JOINT_END]->control(clamp_pos, parm_.J_parm[ARM_JOINT_END].speed_max,
                parm_.J_parm[ARM_JOINT_END].Kp, parm_.J_parm[ARM_JOINT_END].Kd, 0);
        }
    }

}
