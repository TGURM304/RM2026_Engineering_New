//
// Created by guan on 2026/2/8.
//

#pragma once

#include <cmath>

#define chassis_x   116.0f
#define chassis_h   215.0f
#define arm_j0_h    91.0f
#define arm_a2      350.0f
#define arm_a3      -90.0f
#define arm_d2      22.7f
#define arm_d4      225.71f
#define arm_end_z   150.0f

#ifdef __cplusplus

namespace arm {
    //统一使用弧度制

    struct JointLimit {
        float min_val;
        float max_val;
    };

    enum ArmJointModel {
        ARM_JOINT_0, ARM_JOINT_1, ARM_JOINT_2, ARM_JOINT_3,
        ARM_JOINT_4, ARM_JOINT_5, ARM_JOINT_END, ARM_JOINT_COUNT
    };

    struct ArmJointLimits {
        JointLimit J[6];
        JointLimit J_end;
    };

    inline const ArmJointLimits ARM_JOINT_LIMITS = {
        {{-M_PI, M_PI},
         {0.0f, M_PI},
         {0.0f, 150.0f * M_PI / 180.0f},
         {-M_PI, M_PI},
         {-150.0f * M_PI / 180.0f, 150.0f * M_PI / 180.0f},
         {-M_PI, M_PI}},
        {-M_PI, M_PI}
    };

} // namespace arm

#endif