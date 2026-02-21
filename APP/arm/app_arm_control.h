//
// Created by guan on 2026/2/8.
//

#pragma once

#include "app_conf.h"

#include "app_arm_def.h"
#include "app_motor.h"
#include "dev_motor_dm.h"
#include <matrix.h>
#include <array>
#include <memory>
#include <cstdint>

#include "ctrl_pid.h"

#ifdef __cplusplus

namespace arm {
    //统一使用弧度制

    struct pid_struct{
        bool use_mit_pd;
        Controller::PID joint_pos_pid, joint_speed_pid;
        float Kp, Kd;
        float speed_max, tor_max, tor_min;
        float joint_fri, k_f;
    };

    struct arm_parm{
        pid_struct J_parm[7];
        Matrixf<6, 1> start_deg;
        Matrixf<6, 1> waiting_deg;
    };

    enum class ArmState {
        Relax, Working, Waiting
    };

    enum class ClampState {
        Open, Close, SetZero
    };

    // 当前状态
    struct arm_data_t {
        Matrixf<6, 1> pos;
        Matrixf<6, 1> vel;
        float clamp_pos{0.f};
        ArmState arm_state{ArmState::Relax};
        ClampState clamp_state{ClampState::Open};
    };
    // 目标状态
    struct ctrl_out_data_t {
        Matrixf<6, 1> g_tor_ref;
        Matrixf<6, 1> pos_ref;
        ClampState clamp_state{ClampState::Open};
    };

    class ArmController {
    public:
        ArmController() = default;
        ArmController(arm_parm parm, Motor::DMMotor* joint0, Motor::DMMotor* joint1, Motor::DMMotor* joint2,
            Motor::DMMotor* joint3, Motor::DMMotor* joint4, Motor::DMMotor* joint5, Motor::DMMotor* j_end)
        : parm_(parm),
          joints_{joint0, joint1, joint2, joint3, joint4, joint5, j_end} {
            valid_ = (joints_[0] && joints_[1] && joints_[2] && joints_[3] && joints_[4] && joints_[5] && joints_[6]);
        }

        Motor::DMMotor* joint(ArmJointModel j) {
            if (j >= ARM_JOINT_COUNT) return nullptr;
            return joints_[j];
        }
        const Motor::DMMotor* joint(ArmJointModel j) const {
            if (j >= ARM_JOINT_COUNT) return nullptr;
            return joints_[j];
        }

        void init();
        void disable();
        void enable();
        void update(const ctrl_out_data_t& arm_cmd);

        const Matrixf<6, 1>& getCurrentQRef() const { return q_ref_; }

        void setUseFri(ArmJointModel l, float fri, float k_f) {
            use_joint_fri[l] = true;
            parm_.J_parm[l].joint_fri = fri;
            parm_.J_parm[l].k_f = k_f;
        }

        void setState(ArmState s) { arm_state_ = s; }
        ArmState getState() const { return arm_state_; }

        arm_data_t *app_arm_ctr_data() {
            return &data_;
        }

    private:
        void applyJointLimits(Matrixf<6, 1>& q) const {
            for (uint8_t i = 0; i < 6; i++) {
                if (q[i][0] > ARM_JOINT_RAW_LIMITS.J[i].max_val) q[i][0] = ARM_JOINT_RAW_LIMITS.J[i].max_val;
                if (q[i][0] < ARM_JOINT_RAW_LIMITS.J[i].min_val) q[i][0] = ARM_JOINT_RAW_LIMITS.J[i].min_val;
            }
        }

        Motor::DMMotor* joints_[7]{};
        Matrixf<6, 1> q_ref_ = matrixf::zeros<6, 1>();
        Matrixf<6, 1> g_ref_ = matrixf::zeros<6, 1>();

        bool valid_{false};
        bool use_joint_fri[ARM_JOINT_COUNT]{};            // 是否使用关节摩擦力补偿
        arm_parm parm_;
        arm_data_t data_;   // 当前状态
        ArmState arm_state_{ArmState::Relax};
        ClampState clamp_state_{ClampState::Open};
    };
}

#endif
