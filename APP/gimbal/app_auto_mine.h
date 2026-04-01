//
// Created by guan on 2026/3/22.
//
#pragma once

#include <cstdint>
#include <cmath>

#include "app_arm_control.h"
#include "app_arm_def.h"
#include <matrix.h>

namespace mine {

// 存矿机构到位标志（底盘 app_msg_chassis_to_gimbal，与 chassis.valid() 成对使用）
struct SaveFeedback {
    bool open_done_L = false, close_done_L = false;
    bool open_done_R = false, close_done_R = false;
};

// 三段 q + clamp_states[3]；存矿槽建议由底盘独立控制；save_dirs/wait_save_done 仅保留兼容
class AutoMineFsm {
    public:
        enum class Step : uint8_t {
            Idle,
            ToSt,
            DoSt,
            ToMid,
            DoMid,
            ToEnd,
            DoEnd,
            ReturnToPrePose,
            ExitHold,
        };

        struct Config {
            // 到位判定（当前关节角误差连续满足该次数才认为到位）
            float pos_eps_rad = 4.0f * M_PI/180.0f;
            uint16_t reached_count_req = 30;

            // 到位后切换夹爪/存矿槽并等待的时间
            uint32_t after_reached_delay_ms = 300;
            // 回到进入姿态后保持一小段时间再退出自动模式
            uint32_t exit_hold_ms = 120;

            // 三段目标关节角（单位：弧度）
            Matrixf<6, 1> q_st = matrixf::zeros<6, 1>();
            Matrixf<6, 1> q_mid = matrixf::zeros<6, 1>();
            Matrixf<6, 1> q_end = matrixf::zeros<6, 1>();

            // 云台 chassis_save_state 下标；发到底盘时 save_state 会交换，故 0→底盘 dir_R(Save_R)，1→dir_L(Save_L)
            uint8_t save_index = 0;

            // 三段：ST / MID / END 在「Do* + 移向下一顶点」期间输出的夹爪与存矿槽方向
            arm::ClampState clamp_states[3] = {
                arm::ClampState::Close,
                arm::ClampState::Open,
                arm::ClampState::Open,
            };
            // 存矿槽由底盘自行「开到堵转后保持」；云台不再发开/关指令，也不等底盘到位标志
            int8_t save_dirs[3] = {0, 0, 0};

            // 各段 Do* 是否等待底盘存矿到位标志（false 则该段仅靠 after_reached_delay_ms）
            bool wait_save_done[3] = {false, false, false};

            // 单步最大停留时间（ms），超时则退出 FSM，避免未到位永远卡在 To* / Return 等；0 表示关闭
            uint32_t step_timeout_ms = 0;
        };

        AutoMineFsm() = default;
        explicit AutoMineFsm(const Config& cfg) : cfg_(cfg) {}

        bool isActive() const { return active_; }
        Step step() const { return step_; }

        void reset() {
            active_ = false;
            step_ = Step::Idle;
            last_trigger_ = false;
            reached_count_ = 0;
            state_enter_ms_ = 0;
        }

        // trigger：遥控器用于启动自动存矿的条件（由调用方决定怎么映射）
        // now_ms：当前毫秒时间（用于计时，不在 FSM 内调用 bsp_time_get_ms）
        // cur_q：当前关节角（弧度），按关节 0..5
        // manual_clamp / manual_save：进入自动模式前的模式
        // save_fb：底盘 open_done_*/close_done_*；save_fb_valid==false（如 CAN 超时）时不采信标志位，Do* 只靠延时
        void update(bool trigger, uint32_t now_ms,
            const float cur_q[6],
            arm::ClampState manual_clamp, const int8_t manual_save[2],
            const SaveFeedback& save_fb, bool save_fb_valid,
            float pos_ref_out[6], arm::ClampState& clamp_out, int8_t save_out[2],
            bool& force_working_out) {

            // 默认：不启用自动模式，关节目标由调用方决定
            clamp_out = manual_clamp;
            save_out[0] = manual_save[0];
            save_out[1] = manual_save[1];
            for (int i = 0; i < 6; i++) pos_ref_out[i] = cur_q[i];
            force_working_out = false;

            if (!active_) {
                // 上升沿触发：开始三段动作
                if (trigger && !last_trigger_) {
                    active_ = true;
                    step_ = Step::ToSt;
                    state_enter_ms_ = now_ms;
                    reached_count_ = 0;

                    // 备份进入自动存矿前的状态（用于最后回到原始姿态）
                    for (int i = 0; i < 6; i++) pre_q_[i] = cur_q[i];
                    pre_clamp_ = manual_clamp;
                    pre_save_[0] = manual_save[0];
                    pre_save_[1] = manual_save[1];
                }

                last_trigger_ = trigger;
                return;
            }

            // active 状态：接管输出
            force_working_out = true;

            if (cfg_.step_timeout_ms > 0 &&
                (now_ms - state_enter_ms_) >= cfg_.step_timeout_ms) {
                active_ = false;
                step_ = Step::Idle;
                reached_count_ = 0;
                state_enter_ms_ = 0;
                last_trigger_ = trigger;
                force_working_out = false;
                return;
            }

            // 由于调用方 pos_ref_out 在 active 时一定会被接管，所以这里始终输出本 FSM 的目标
            switch (step_) {
                case Step::ToSt: {
                    setPosTarget(pos_ref_out, cfg_.q_st);
                    clamp_out = pre_clamp_;
                    // 到位前先不动作存矿槽电机，避免在机械臂移动过程中与机构联动
                    save_out[cfg_.save_index] = 0;
                    if (isReached(cur_q, cfg_.q_st)) stepToDoOrWait(now_ms, Step::DoSt, reached_count_);
                    else reached_count_ = 0;
                    break;
                }
                case Step::DoSt: {
                    setPosTarget(pos_ref_out, cfg_.q_st);
                    applySegOut(0, clamp_out, save_out);
                    if (segmentDoOk(0, save_fb, save_fb_valid, now_ms)) {
                        step_ = Step::ToMid;
                        reached_count_ = 0;
                        state_enter_ms_ = now_ms;
                    }
                    break;
                }
                case Step::ToMid: {
                    setPosTarget(pos_ref_out, cfg_.q_mid);
                    applySegOut(0, clamp_out, save_out);
                    if (isReached(cur_q, cfg_.q_mid)) stepToDoOrWait(now_ms, Step::DoMid, reached_count_);
                    else reached_count_ = 0;
                    break;
                }
                case Step::DoMid: {
                    setPosTarget(pos_ref_out, cfg_.q_mid);
                    applySegOut(1, clamp_out, save_out);
                    if (segmentDoOk(1, save_fb, save_fb_valid, now_ms)) {
                        step_ = Step::ToEnd;
                        reached_count_ = 0;
                        state_enter_ms_ = now_ms;
                    }
                    break;
                }
                case Step::ToEnd: {
                    setPosTarget(pos_ref_out, cfg_.q_end);
                    applySegOut(1, clamp_out, save_out);
                    if (isReached(cur_q, cfg_.q_end)) stepToDoOrWait(now_ms, Step::DoEnd, reached_count_);
                    else reached_count_ = 0;
                    break;
                }
                case Step::DoEnd: {
                    setPosTarget(pos_ref_out, cfg_.q_end);
                    applySegOut(2, clamp_out, save_out);
                    if (segmentDoOk(2, save_fb, save_fb_valid, now_ms)) {
                        step_ = Step::ReturnToPrePose;
                        reached_count_ = 0;
                        state_enter_ms_ = now_ms;
                    }
                    break;
                }
                case Step::ReturnToPrePose: {
                    // 回到进入自动存矿前的关节角
                    for (int i = 0; i < 6; i++) pos_ref_out[i] = pre_q_[i];
                    clamp_out = pre_clamp_;
                    save_out[0] = pre_save_[0];
                    save_out[1] = pre_save_[1];
                    if (isReachedArray(cur_q, pre_q_)) {
                        reached_count_++;
                        if (reached_count_ >= cfg_.reached_count_req) {
                            step_ = Step::ExitHold;
                            state_enter_ms_ = now_ms;
                            reached_count_ = 0;
                        }
                    } else {
                        reached_count_ = 0;
                    }
                    break;
                }
                case Step::ExitHold: {
                    for (int i = 0; i < 6; i++) pos_ref_out[i] = pre_q_[i];
                    clamp_out = pre_clamp_;
                    save_out[0] = pre_save_[0];
                    save_out[1] = pre_save_[1];
                    if (now_ms - state_enter_ms_ >= cfg_.exit_hold_ms) {
                        // 退出自动模式（调用方会在下一周期恢复手动 pos_ref）
                        active_ = false;
                        step_ = Step::Idle;
                    }
                    break;
                }
                default:
                    break;
            }

            last_trigger_ = trigger;
        }

    private:
        void applySegOut(uint8_t seg, arm::ClampState& clamp_out, int8_t save_out[2]) const {
            clamp_out = cfg_.clamp_states[seg];
            save_out[cfg_.save_index] = cfg_.save_dirs[seg];
        }

        // Do*：本段存矿到位（wait_save_done 且 save_valid 时才看标志位）且超过 after_reached_delay_ms
        bool segmentDoOk(uint8_t seg, const SaveFeedback& save_fb, bool save_valid, uint32_t now_ms) const {
            const bool save_ok = !cfg_.wait_save_done[seg] || isSaveReadySeg(seg, save_fb, save_valid);
            return save_ok && (now_ms - state_enter_ms_ >= cfg_.after_reached_delay_ms);
        }

        void setPosTarget(float pos_ref_out[6], const Matrixf<6, 1>& target) const {
            for (int i = 0; i < 6; i++) pos_ref_out[i] = target[i][0];
        }

        bool isReached(const float cur_q[6], const Matrixf<6, 1>& target) const {
            float t[6];
            for (int i = 0; i < 6; i++) t[i] = target[i][0];
            return isReachedArray(cur_q, t);
        }

        // 与 Matrixf 目标共用同一套判定，pre_q_ 等数组目标也走这里，避免与控制器 3/5 轴卷绕不一致
        bool isReachedArray(const float cur_q[6], const float target_q[6]) const {
            constexpr float two_pi = 2.0f * static_cast<float>(M_PI);
            for (int i = 0; i < 6; i++) {
                if (!std::isfinite(cur_q[i]) || !std::isfinite(target_q[i])) return false;
                float diff = cur_q[i] - target_q[i];
                // 关节 3/5 在控制器里开启了 use_sum_angle，cur_q 可能与 target 存在 2*pi 等价偏差
                if (i == 3 || i == 5) diff -= std::round(diff / two_pi) * two_pi;
                // J5（索引 5）控制/传动误差较大，一键存矿到位判定放宽为 2 倍
                const float eps = (i == 5) ? (3.0f * cfg_.pos_eps_rad) : cfg_.pos_eps_rad;
                if (std::fabs(diff) > eps) return false;
            }
            return true;
        }

        bool isSaveReadySeg(int seg, const SaveFeedback& fb, bool valid) const {
            if (!valid) return true;
            const int8_t d = cfg_.save_dirs[seg];
            if (d == 0) return true;
            // 与 app_gimbal send_msg_to_chassis 中 save_state 交换一致：save_index 0 对应底盘 R 路反馈
            const bool open_done = (cfg_.save_index == 0) ? fb.open_done_R : fb.open_done_L;
            const bool close_done = (cfg_.save_index == 0) ? fb.close_done_R : fb.close_done_L;
            return (d == 1 && open_done) || (d == -1 && close_done);
        }

        // helper：ToX 状态达到后，连续 reached_count_ 达到阈值则进入 DoX
        void stepToDoOrWait(uint32_t now_ms, Step do_step, uint16_t& reached_count_ref) {
            reached_count_ref++;
            if (reached_count_ref >= cfg_.reached_count_req) {
                step_ = do_step;
                state_enter_ms_ = now_ms;
                reached_count_ref = 0;
            }
        }

        Config cfg_{};
        bool active_{false};
        bool last_trigger_{false};
        Step step_{Step::Idle};

        float pre_q_[6]{};
        arm::ClampState pre_clamp_{arm::ClampState::Open};
        int8_t pre_save_[2]{};

        uint16_t reached_count_{0};
        uint32_t state_enter_ms_{0};
    };
} // namespace mine

