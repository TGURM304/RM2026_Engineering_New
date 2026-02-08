//
// Created by guan on 2026/1/23.
//

#pragma once

#include "app_conf.h"
#include "bsp_time.h"

#include <matrix.h>

#include "app_arm_def.h"

#define Joint_Max   0
#define Joint_Min   1

#ifdef __cplusplus
/*
 *
 */
namespace arm {
    //统一使用弧度制

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

    // 获取Position
    inline Matrixf<3, 1> pos_from_T(const Matrixf<4, 4>& T) {
        Matrixf<3, 1> p;
        p[0][0] = T[0][3]; p[1][0] = T[1][3]; p[2][0] = T[2][3];
        return p;
    }
    // 获取Z轴方向
    inline Matrixf<3, 1> z_from_T(const Matrixf<4, 4>& T) {
        Matrixf<3, 1> z;
        z[0][0] = T[0][2]; z[1][0] = T[1][2]; z[2][0] = T[2][2];
        return z;
    }

    // 角度归一化到 [-π, π]
    inline float wrapPi(float x) {
        return atan2f(sinf(x), cosf(x));
    }

    struct app_Arm_data_t {
        bool range_state{false};
        uint8_t validCount{};
        Matrixf<4, 4> T_arm_end;
        Matrixf<6, 6> J_arm;
        Matrixf<8, 6> raw_data, cur_angle;
        Matrixf<6, 1> upd_angle;
    };

    class Kinematics {
        public:
        Kinematics();
        Kinematics(float const a2_, float const a3_, float const d2_, float const d4_,
            const Matrixf<4, 4>& base_, const Matrixf<4, 4>& tool_)
            : a2(a2_), a3(a3_), d2(d2_), d4(d4_), Base(base_), Tool(tool_) {
            arm_theta.cur_angle = arm_theta.raw_data = matrixf::zeros<8, 6>();
            Base_inv = matrixf::inv(Base), Tool_inv = matrixf::inv(Tool);
        }

        // 正运动学解算
        void arm_forward_clc(const Matrixf<6, 1>& tem_q) {
            lst_clc_time[0] = bsp_time_get_us();
            for(uint8_t i =0; i<6; i++) {
                cur_q[i][0] = tem_q[i][0];
            }
            cur_q[2][0] += M_PI_2;

            T_joint[0] = dh_trans(0.0f,    0.0f,       0.0f,    cur_q[0][0]);
            T_joint[1] = dh_trans(0.0f,    M_PI_2,     d2,      cur_q[1][0]);
            T_joint[2] = dh_trans(a2,      0.0f,       0.0f,    cur_q[2][0]);
            T_joint[3] = dh_trans(a3,      M_PI_2,     d4,      cur_q[3][0]);
            T_joint[4] = dh_trans(0.0f,    -M_PI_2,    0.0f,    cur_q[4][0]);
            T_joint[5] = dh_trans(0.0f,    M_PI_2,     0.0f,    cur_q[5][0]);

            T_ = T_joint[0] * T_joint[1] * T_joint[2] * T_joint[3] * T_joint[4] * T_joint[5];
            T_end = Base * T_ * Tool;
            arm_theta.T_arm_end = T_end;

            clc_time[0] = bsp_time_get_us() - lst_clc_time[0];
        }

        // 雅可比矩阵计算
        void arm_jacobi_clc() {
            lst_clc_time[2] = bsp_time_get_us();

            Matrixf<4, 4> T01 = T_joint[0];
            Matrixf<4, 4> T02 = T01 * T_joint[1];
            Matrixf<4, 4> T03 = T02 * T_joint[2];
            Matrixf<4, 4> T04 = T03 * T_joint[3];
            Matrixf<4, 4> T05 = T04 * T_joint[4];
            Matrixf<4, 4> T06 = T_;

            Matrixf<3, 1> P01 = pos_from_T(T01), P02 = pos_from_T(T02), P03 = pos_from_T(T03);
            Matrixf<3, 1> P04 = pos_from_T(T04), P05 = pos_from_T(T05), P06 = pos_from_T(T06);
            Matrixf<3, 1> Z1 = z_from_T(T01), Z2 = z_from_T(T02), Z3 = z_from_T(T03);
            Matrixf<3, 1> Z4 = z_from_T(T04), Z5 = z_from_T(T05), Z6 = z_from_T(T06);

            set_col(Jacobi, 0, vector3f::cross(Z1, P06 - P01), Z1);
            set_col(Jacobi, 1, vector3f::cross(Z2, P06 - P02), Z2);
            set_col(Jacobi, 2, vector3f::cross(Z3, P06 - P03), Z3);
            set_col(Jacobi, 3, vector3f::cross(Z4, P06 - P04), Z4);
            set_col(Jacobi, 4, vector3f::cross(Z5, P06 - P05), Z5);
            set_col(Jacobi, 5, matrixf::zeros<3, 1>(), Z6);
            arm_theta.J_arm = Jacobi;

            clc_time[2] = bsp_time_get_us() - lst_clc_time[2];
        }

        // 逆运动学解算
        void arm_inverse_clc(const Matrixf<4, 4>& T_target) {
            lst_clc_time[1] = bsp_time_get_us();

            Matrixf<4, 4> T06 = Base_inv * T_target * Tool_inv;

            float nx = T06[0][0], ny = T06[1][0], nz = T06[2][0];
            // float ox = T06[0][1], oy = T06[1][1], oz = T06[2][1];
            float ax = T06[0][2], ay = T06[1][2], az = T06[2][2];
            float px = T06[0][3], py = T06[1][3], pz = T06[2][3];

            float ForJudgment = px*px + py*py - d2*d2;

            if (ForJudgment < -1e-6) {
                arm_theta.raw_data = matrixf::zeros<8, 6>();
                arm_theta.range_state = false;
                return;
            }
            if (ForJudgment >= -1e-6 && ForJudgment < 0.0) {
                ForJudgment = 0.0;
            }
            arm_theta.range_state = true;

            // theta1 (2个)
            float theta1[2];
            float sqrt_ForJudgment = sqrtf(ForJudgment);
            theta1[0] = atan2f(py, px) - atan2f(-d2, sqrt_ForJudgment);
            theta1[1] = atan2f(py, px) - atan2f(-d2, -sqrt_ForJudgment);
            // theta3 (2个)
            float theta3[2];
            float k_t = (px*px + py*py + pz*pz - a2*a2 - a3*a3 - d2*d2 - d4*d4) / (2.0f*a2);
            float sqrt_k = sqrtf(a3*a3 + d4*d4 - k_t*k_t);
            theta3[0] = -atan2f(a3, d4) + atan2f(k_t, sqrt_k);
            theta3[1] = -atan2f(a3, d4) + atan2f(k_t, -sqrt_k);

            // theta2, theta4, theta5, theta6 (4个)
            float theta23[4], theta2[4], theta4[4], theta5[4], theta6[4];
            uint8_t id1[4] = {0, 1, 0, 1};
            uint8_t id3[4] = {0, 0, 1, 1};

            for (uint8_t k = 0; k < 4; k++) {
                float tem1 = theta1[id1[k]];
                float tem3 = theta3[id3[k]];

                float s1 = sinf(tem1), c1 = cosf(tem1);
                float s3 = sinf(tem3), c3 = cosf(tem3);

                // theta2
                float num = (a2*c3 + a3) * pz + (c1*px + s1*py) * (a2*s3 + d4);
                float den = -(d4 + a2*s3) * pz + (c1*px + s1*py) * (a2*c3 + a3);
                theta23[k] = atan2f(num, den);
                theta2[k] = theta23[k] - tem3;

                // theta4
                float tem23 = theta2[k] + tem3;
                float s23 = sinf(tem23), c23 = cosf(tem23);
                
                float tem4_1 = ax*s1 - ay*c1;
                float tem4_2 = ax*c1*c23 + ay*s1*c23 + az*s23;

                if (fabsf(tem4_1) < 1e-9 && fabsf(tem4_2) < 1e-9) {
                    theta4[k] = theta5[k] = theta6[k] = 0.0f;
                    continue;
                } else theta4[k] = atan2f(tem4_1, tem4_2);

                float s4 = sinf(theta4[k]), c4 = cosf(theta4[k]);

                // theta5
                float tem5_1 = ax * (c1*c23*c4 + s1*s4) + ay * (s1*c23*c4 - c1*s4) + az * (s23*c4);
                float tem5_2 = ax*c1*s23 + ay*s1*s23 - az*c23;

                if (fabsf(tem5_1) < 1e-9f && fabsf(tem5_2) < 1e-9f) {
                    theta5[k] = 0.0f;
                } else {
                    theta5[k] = atan2f(tem5_1, tem5_2);
                }

                float s5 = sinf(theta5[k]), c5 = cosf(theta5[k]);

                // theta6
                float tem6_1 = -nx * (c1*c23*s4 - s1*c4) - ny * (s1*c23*s4 + c1*c4) - nz * s23 * s4;
                float tem6_2 = nx * ((c1*c23*c4 + s1*s4) * c5 - c1*s23*s5) +
                            ny * ((s1*c23*c4 - c1*s4) * c5 - s1*s23*s5) +
                            nz * (s23*c4*c5 + c23*s5);

                if (fabsf(tem6_1) < 1e-9f && fabsf(tem6_2) < 1e-9f) {
                    theta6[k] = 0.0f;
                } else {
                    theta6[k] = atan2f(tem6_1, tem6_2);
                }
            }

            // 扩展为8个解
            float thetax_8[6][8];
            for (uint8_t i = 0; i < 4; i++) {
                thetax_8[0][i] = theta1[id1[i]],   thetax_8[0][i + 4] = theta1[id1[i]];
                thetax_8[1][i] = theta2[i],        thetax_8[1][i + 4] = theta2[i];
                thetax_8[2][i] = theta3[id3[i]],   thetax_8[2][i + 4] = theta3[id3[i]];
                thetax_8[3][i] = theta4[i],        thetax_8[3][i + 4] = theta4[i] + M_PI;
                thetax_8[4][i] = theta5[i],        thetax_8[4][i + 4] = -theta5[i];
                thetax_8[5][i] = theta6[i],        thetax_8[5][i + 4] = theta6[i] + M_PI;
            }

            // 应用偏移量并归一化
            const float offset[6] = {0.0f, 0.0f, M_PI_2, 0.0f, 0.0f, 0.0f};
            // 角度限幅
            validCount = 0;

            for (uint8_t i = 0; i < 8; i++) {
                float q_tmp[6];
                for (uint8_t j = 0; j < 6; j++) {
                    q_tmp[j] = wrapPi(thetax_8[j][i] - offset[j]);
                    arm_theta.raw_data[i][j] = q_tmp[j];
                    if (q_tmp[j] < ARM_JOINT_LIMITS.J[j].min_val || q_tmp[j] > ARM_JOINT_LIMITS.J[j].max_val) {
                        goto next_solution;
                    }
                }
                for (uint8_t k = 0; k < 6; k++) {
                    arm_theta.cur_angle[validCount][k] = q_tmp[k];
                }
                validCount++;

                next_solution:;
            }
            arm_theta.validCount = validCount;

            clc_time[1] = bsp_time_get_us() - lst_clc_time[1];
        }

        // 解的选择
        void select_angle() {
            lst_clc_time[3] = bsp_time_get_us();
            if (validCount == 0) arm_theta.upd_angle = matrixf::zeros<6, 1>();

            float min_dist = 1e10f;
            uint8_t best_idx = 0;

            for (uint8_t i = 0; i < 8; i++) {
                diff_tmp[i] = 0;
            }
            for (uint8_t i = 0; i < validCount; i++) {
                float dist = 0.0f;
                for (uint8_t j = 0; j < 6; j++) {
                    float diff = arm_theta.cur_angle[i][j] - cur_q[j][0];
                    // 处理角度周期性：选择最短路径
                    if (diff > M_PI) diff -= 2.0f * M_PI;
                    if (diff < -M_PI) diff += 2.0f * M_PI;
                    dist += diff * diff;
                }

                diff_tmp[i] = dist;

                if (dist < min_dist) {
                    min_dist = dist;
                    best_idx = i;
                }
            }

            best_idx_t = best_idx;
            arm_theta.upd_angle = arm_theta.cur_angle.row(best_idx).trans();
            clc_time[3] = bsp_time_get_us() - lst_clc_time[3];
        }

        const app_Arm_data_t *app_arm_get_data() {
            return &arm_theta;
        }

        uint32_t clc_time[4] = {};
        uint32_t lst_clc_time[4] = {};
        int16_t diff_tmp[8] = {};
        uint8_t best_idx_t = 0;

    private:
        static void set_col(Matrixf<6, 6>& J, uint8_t col,
                   const Matrixf<3, 1>& lin, const Matrixf<3, 1>& ang) {
            J[0][col] = lin[0][0]; J[1][col] = lin[1][0]; J[2][col] = lin[2][0];
            J[3][col] = ang[0][0]; J[4][col] = ang[1][0]; J[5][col] = ang[2][0];
        }

        float a2, a3, d2, d4;
        uint8_t validCount = 0;
        Matrixf<4,4> Base, Tool;
        Matrixf<4,4> Base_inv, Tool_inv;
        Matrixf<4,4> T_joint[6];
        Matrixf<4, 4> T_, T_end;
        Matrixf<6, 6> Jacobi;
        Matrixf<6,1> cur_q;
        app_Arm_data_t arm_theta;
    };

    /* 暂无动力学模型 */

    // class Dynamic {
    //     public:
    // private:
    // };
}
#endif

#ifdef __cplusplus
extern "C" {
#endif

    void app_arm_init();
    void app_arm_task(void *argument);
    const arm::app_Arm_data_t *app_arm_data();

#ifdef __cplusplus
}
#endif
