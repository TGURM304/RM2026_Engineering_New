//
// Created by guan on 2026/2/8.
//

#pragma once

#include <cmath>

#include "matrix.h"
#include "SJTU_Matrix/utils.h"

#define chassis_x   0.116f
#define chassis_h   0.215f
#define arm_j0_h    0.091f
#define arm_a2      0.350f
#define arm_a3     -0.090f
#define arm_d2      0.0227f
#define arm_d4      0.22571f
#define arm_end_z   0.150f

#ifdef __cplusplus

namespace arm {
    //统一使用弧度制

    enum ArmClcMode {
        FORWARD, INVERSE, JACOBI, NEWTON_EULER, SELECT_ANGLE
    };

    enum ArmJointModel {
        ARM_JOINT_0, ARM_JOINT_1, ARM_JOINT_2, ARM_JOINT_3,
        ARM_JOINT_4, ARM_JOINT_5, ARM_JOINT_END, ARM_JOINT_COUNT
    };

    struct JointLimit {
        float min_val;
        float max_val;
    };

    struct ArmJointLimits {
        JointLimit J[6];
        JointLimit J_end;
    };

    inline const ArmJointLimits ARM_JOINT_RAW_LIMITS = {
        {{-240.0f * M_PI / 180.0f, 245.0f * M_PI / 180.0f},
         { -45.0f * M_PI / 180.0f,  23.0f * M_PI / 180.0f},
         { -47.0f * M_PI / 180.0f, 103.0f * M_PI / 180.0f},
         {-175.0f * M_PI / 180.0f, 115.0f * M_PI / 180.0f},
         { -80.0f * M_PI / 180.0f,  82.0f * M_PI / 180.0f},
         {-65535, 65535}},
        {-80.0f * M_PI / 180.0f, -0.0f * M_PI / 180.0f}
    };

    inline const ArmJointLimits ARM_JOINT_LIMITS = {
        {{-240.0f * M_PI / 180.0f, 245.0f * M_PI / 180.0f},
         {  67.0f * M_PI / 180.0f, 135.0f * M_PI / 180.0f},
         {-137.0f * M_PI / 180.0f,  13.0f * M_PI / 180.0f},
         {-175.0f * M_PI / 180.0f, 115.0f * M_PI / 180.0f},
         { -80.0f * M_PI / 180.0f,  82.0f * M_PI / 180.0f},
         {-65535, 65535}},
        {-80.0f * M_PI / 180.0f, -0.0f * M_PI / 180.0f}
    };

    // 角度归一化到 [-π, π]
    inline float wrapPi(float x) {
        return atan2f(sinf(x), cosf(x));
    }

    // 改进 DH 旋转矩阵
    // 参数：a - 连杆长度, alpha - 连杆扭角, d - 连杆偏移, theta - 关节角度
    inline Matrixf<4, 4> dh_trans(float a, float alpha, float d, float theta) {
        Matrixf<4, 4> DHTrans;

        float cos_theta = cosf(theta), sin_theta = sinf(theta);
        float cos_alpha = cosf(alpha), sin_alpha = sinf(alpha);

        DHTrans[0][0] = cos_theta;
        DHTrans[0][1] = -sin_theta;
        DHTrans[0][2] = 0.0f;
        DHTrans[0][3] = a;

        DHTrans[1][0] = sin_theta * cos_alpha;
        DHTrans[1][1] = cos_theta * cos_alpha;
        DHTrans[1][2] = -sin_alpha;
        DHTrans[1][3] = -d * sin_alpha;

        DHTrans[2][0] = sin_theta * sin_alpha;
        DHTrans[2][1] = cos_theta * sin_alpha;
        DHTrans[2][2] = cos_alpha;
        DHTrans[2][3] = d * cos_alpha;

        DHTrans[3][0] = DHTrans[3][1] = DHTrans[3][2] = 0.0f;
        DHTrans[3][3] = 1.0f;

        return DHTrans;
    }

    enum JointType {
        Revolute, Prismatic
    };

    // (MDH) + (m, rc：质心坐标, I：惯量张量)
    class Link {
        public:
        Link() = default;
        Link(const float a, const float alpha, const float d, const float theta0, const JointType type = Revolute,
             const float offset = 0.0f,  const float qmin = 0.0f, const float qmax = 0.0f, const float m = 1.0f,
             const Matrixf<3, 1> rc = matrixf::zeros<3, 1>(), const Matrixf<3, 3> I = matrixf::zeros<3, 3>())
            : a_(a), alpha_(alpha), d_(d), theta0_(theta0), type_(type), offset_(offset),
            qmin_(qmin), qmax_(qmax), m_(m), rc_(rc), I_(I) {}

        Link(const Link& link)
            : a_(link.a_), alpha_(link.alpha_), d_(link.d_), theta0_(link.theta0_),
            type_(link.type_), offset_(link.offset_), qmin_(link.qmin_), qmax_(link.qmax_),
            m_(link.m_), rc_(link.rc_), I_(link.I_) {}

        Link& operator = (Link link) {
            a_ = link.a_;
            alpha_ = link.alpha_;
            d_ = link.d_;
            theta0_ = link.theta0_;
            type_ = link.type_;
            offset_ = link.offset_;
            qmin_ = link.qmin_;
            qmax_ = link.qmax_;
            m_ = link.m_;
            rc_ = link.rc_;
            I_ = link.I_;
            return *this;
        }

        Matrixf<4, 4> T(float q) const {
            float theta, d;

            if (type_ == Revolute) {
                theta = math::limit(q, qmin_, qmax_);
                theta += offset_;
                d = d_;
            } else {
                theta = theta0_;
                float q_eff = q + offset_;
                d = math::limit(q_eff, qmin_, qmax_);
            }
            return dh_trans(a_, alpha_, d, theta);
        }

        float a() const { return a_; }
        float alpha() const { return alpha_; }
        float d() const { return d_; }
        float theta0() const { return theta0_; }
        float offset() const { return offset_; }
        float m() const { return m_; }
        const Matrixf<3, 1>& rc() const { return rc_; }
        const Matrixf<3, 3>& I() const { return I_; }
        JointType type() const { return type_; }

    private:
        float a_, alpha_, d_, theta0_;
        JointType type_{Revolute};
        bool use_ang_lim_{false};
        float offset_{0.0f}, qmin_{0.0f}, qmax_{0.0f};
        float m_{1.0f};
        Matrixf<3, 1> rc_;
        Matrixf<3, 3> I_;
    };

} // namespace arm

#endif