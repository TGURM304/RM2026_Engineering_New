//
// Created by guan on 2026/3/31.
//

#pragma once
#include "bsp_tim.h"

namespace servo {
    class Servo {
        public:
        void init(TIM_HandleTypeDef *htim, uint32_t channel, double freq, float init_angle_deg, float lim_max, float lim_min) {
            htim_ = htim;
            pwm_freq_hz_ = freq;
            pwm_channel_ = channel;
            lim_max_ = lim_max, lim_min_ = lim_min;
            bsp_tim_config(htim_, freq);
            bsp_tim_pwm_enable(htim_, pwm_channel_);
            bsp_tim_set_blank(htim_, pwm_channel_, angle_to_blank(init_angle_deg));
            inited_ = true;
        }

        void set_angle(float angle) {
            if(!inited_) return;
            bsp_tim_set_blank(htim_, pwm_channel_, angle_to_blank(angle));
        }

        void open() {
            set_angle(lim_max_);
        }

        void close() {
            set_angle(lim_min_);
        }

        bool get_state() { return inited_; }

    private:
        double angle_to_blank(float angle_deg) const {
            if(angle_deg < lim_min_) angle_deg = lim_min_;
            if(angle_deg > lim_max_) angle_deg = lim_max_;
            const double pulse_us = 500.0 + (2000.0 * angle_deg / 270.0);
            return pulse_us * pwm_freq_hz_ / 1e6;
        }

        bool inited_ = false;
        TIM_HandleTypeDef *htim_ = nullptr;
        uint32_t pwm_channel_ = 0;
        double pwm_freq_hz_ = 50.0;
        float lim_max_ = 270.0f, lim_min_ = 0.0f;
    };
}