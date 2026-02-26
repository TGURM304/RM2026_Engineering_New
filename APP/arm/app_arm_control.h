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
#include <cmath>
#include <memory>
#include <cstdint>

#include "alg_filter.h"
#include "ctrl_pid.h"
#include "bsp_time.h"

#ifdef __cplusplus

namespace arm {
    //统一使用弧度制

    struct pid_struct{
        bool use_mit_pd;
        Controller::PID joint_pos_pid, joint_speed_pid;
        float Kp, Kd;
        float speed_max, tor_max, tor_min;
        float joint_fri, k_f;
        float vmax_traj{2.0f};   // 轨迹最大速度 rad/s
        float amax_traj{2.0f};   // 轨迹最大加速度 rad/s²
    };

    // T型轨迹参数（单关节）
    struct TlineParams {
        float q0{0.f}, q1{0.f}, v0{0.f}, v1{0.f};
        float Ta{0.f}, Tv{0.f}, Td{0.f}, T{0.f};
        float vlin{0.f}, Sa{0.f}, Sv{0.f};
        float amax{0.f};
        float sign_{1.f};
        uint64_t t_start_us_{0};
        bool valid{false};
        bool trajectory_active_{false};                 //T轨迹规划单次初始化
        bool state[3]{false, false, false};    // 加速、匀速、减速阶段标志
        float tmp_t;
    };

    struct arm_parm{
        pid_struct J_parm[7];
        Matrixf<6, 1> start_deg;
        Matrixf<6, 1> waiting_deg;
    };

    enum class ArmState {
        Relax, Working, Waiting, Float
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
        // T型轨迹
        void planTline(uint8_t j, float q0, float q1, float v0, float v1);
        void evalTline(uint8_t j, float t, float* q_out, float* qd_out);

        const Matrixf<6, 1>& getCurrentQRef() const { return q_ref_; }
        ArmState getState() const { return arm_state_; }
        void setState(ArmState s) { arm_state_ = s; }
        void setUseTline(bool use) { use_tline_ = use; }
        void setUseSumAngle(ArmJointModel l) { use_sum_angle_[l] = true; }
        void setUseFri(ArmJointModel l, float fri, float k_f) {
            use_joint_fri_[l] = true;
            parm_.J_parm[l].joint_fri = fri;
            parm_.J_parm[l].k_f = k_f;
        }

        arm_data_t *app_arm_ctr_data() {
            return &data_;
        }

        Matrixf<6, 1> q_ref_ = matrixf::zeros<6, 1>();
        Matrixf<6, 1> q_goal_ = matrixf::zeros<6, 1>();
        TlineParams tline_[6]{};

    private:
        void applyJointLimits(Matrixf<6, 1>& q) const {
            for (uint8_t i = 0; i < 6; i++) {
                if (i == 5 && use_sum_angle_[i]) {
                    // 将目标[-π,π]换算到距当前cur最近的等价角度
                    float target = q[i][0], cur = data_.pos[i][0];
                    const float two_pi = 2.0f * M_PI;
                    float k = roundf((cur - target) / two_pi);
                    q[i][0] = target + k * two_pi;
                } else {
                    if (q[i][0] > ARM_JOINT_RAW_LIMITS.J[i].max_val) q[i][0] = ARM_JOINT_RAW_LIMITS.J[i].max_val;
                    if (q[i][0] < ARM_JOINT_RAW_LIMITS.J[i].min_val) q[i][0] = ARM_JOINT_RAW_LIMITS.J[i].min_val;
                }
            }
        }

        Algorithm::LowPassFilter joint_filter[6] = {
            Algorithm::LowPassFilter(3.0),
            Algorithm::LowPassFilter(3.0),
            Algorithm::LowPassFilter(3.0),
            Algorithm::LowPassFilter(3.0),
            Algorithm::LowPassFilter(3.0),
            Algorithm::LowPassFilter(3.0)
        };

        Motor::DMMotor* joints_[7]{};
        // TlineParams tline_[6]{};
        // Matrixf<6, 1> q_ref_ = matrixf::zeros<6, 1>();
        Matrixf<6, 1> g_ref_ = matrixf::zeros<6, 1>();
        // Matrixf<6, 1> q_goal_ = matrixf::zeros<6, 1>();
        Matrixf<6, 1> cur_q = matrixf::zeros<6, 1>();
        Matrixf<6, 1> lst_q = matrixf::zeros<6, 1>();

        bool use_tline_{true};
        bool valid_{false};
        bool use_joint_fri_[ARM_JOINT_COUNT]{};             // 是否使用关节摩擦力补偿
        bool use_sum_angle_[ARM_JOINT_COUNT]{};             // 是否使用总角度计算：目标[-π,π]换算到距当前cur最近的角度
        arm_parm parm_;
        arm_data_t data_;   // 当前状态
        ArmState arm_state_{ArmState::Relax};
        ArmState last_arm_state_{ArmState::Relax};
        ClampState clamp_state_{ClampState::Open};
    };
}

#endif
