//
// Created by guan on 2026/1/23.
//

#pragma once

#include "app_conf.h"
#include "bsp_time.h"

#include <matrix.h>

#define Joint_Max   0
#define Joint_Min   1

#ifdef __cplusplus
extern "C" {
#endif

    void app_arm_init();
    void app_arm_task(void *argument);

#ifdef __cplusplus
}
#endif

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

    inline float wrapPi(float x) {
        return atan2f(sinf(x), cosf(x));
    }

    enum JointModel {
        Joint0, Joint1, Joint2, Joint3, Joint4, Joint5
    };

    struct app_Arm_Theta_t {
        bool range_state{false};
        uint8_t out_id{};
        Matrixf<8,6> raw_data;
        Matrixf<8,6> cur_solutions;
    };

    class Kinematics {
        public:
        Kinematics();
        Kinematics(float const a2_, float const a3_, float const d2_, float const d4_,
            const Matrixf<4, 4>& base_, const Matrixf<4, 4>& tool_)
            : a2(a2_), a3(a3_), d2(d2_), d4(d4_), Base(base_), Tool(tool_) {
            arm_theta.cur_solutions = arm_theta.raw_data = matrixf::zeros<8,6>();
            Base_inv = matrixf::inv(Base), Tool_inv = matrixf::inv(Tool);
        }

        Matrixf<4,4> arm_forward_clc(const Matrixf<6, 1>& tem_q) {
            lst_cle_time[0] = bsp_time_get_us();
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


            clc_time[0] = bsp_time_get_us() - lst_cle_time[0];
            return T_end;
        }

        Matrixf<8, 6> arm_inverse_clc(const Matrixf<4, 4>& T_target) {
            lst_cle_time[1] = bsp_time_get_us();
            Matrixf<8, 6> AllSloverTheta = matrixf::zeros<8, 6>();

            Matrixf<4, 4> T06 = Base_inv * T_target * Tool_inv;

            float nx = T06[0][0], ny = T06[1][0], nz = T06[2][0];
            // float ox = T06[0][1], oy = T06[1][1], oz = T06[2][1];
            float ax = T06[0][2], ay = T06[1][2], az = T06[2][2];
            float px = T06[0][3], py = T06[1][3], pz = T06[2][3];

            float ForJudgment = px*px + py*py - d2*d2;

            if (ForJudgment < -1e-6) {
                arm_theta.raw_data = matrixf::zeros<8, 6>();
                arm_theta.range_state = false;
                return AllSloverTheta;
            }
            if (ForJudgment >= -1e-6 && ForJudgment < 0.0) {
                arm_theta.range_state = false;
                ForJudgment = 0.0;
            }

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
            int id1[4] = {0, 1, 0, 1};
            int id3[4] = {0, 0, 1, 1};

            for (uint8_t k = 0; k < 4; k++) {
                float tem1 = theta1[id1[k]];
                float tem3 = theta3[id3[k]];

                // theta2
                float num = (a2*cosf(tem3) + a3) * pz + (cosf(tem1)*px + sinf(tem1)*py) * (a2*sinf(tem3) + d4);
                float den = -(d4 + a2*sinf(tem3)) * pz + (cosf(tem1)*px + sinf(tem1)*py) * (a2*cosf(tem3) + a3);
                theta23[k] = atan2f(num, den);
                theta2[k] = theta23[k] - tem3;

                // theta4
                float tem23 = theta2[k] + tem3;
                float tem4_1 = ax*sinf(tem1) - ay*cosf(tem1);
                float tem4_2 = ax*cosf(tem1)*cosf(tem23) + ay*sinf(tem1)*cosf(tem23) + az*sinf(tem23);

                if (fabsf(tem4_1) < 1e-9 && fabsf(tem4_2) < 1e-9) {
                    theta4[k] = theta5[k] = theta6[k] = 0.0f;
                    continue;
                } else theta4[k] = atan2f(tem4_1, tem4_2);

                // theta5
                float tem5_1 = ax * (cosf(tem1)*cosf(tem23)*cosf(theta4[k]) + sinf(tem1)*sinf(theta4[k])) +
                       ay * (sinf(tem1)*cosf(tem23)*cosf(theta4[k]) - cosf(tem1)*sinf(theta4[k])) +
                       az * (sinf(tem23)*cosf(theta4[k]));
                float tem5_2 = ax*cosf(tem1)*sinf(tem23) + ay*sinf(tem1)*sinf(tem23) - az*cosf(tem23);

                if (fabsf(tem5_1) < 1e-9f && fabsf(tem5_2) < 1e-9f) {
                    theta5[k] = 0.0f;
                } else {
                    theta5[k] = atan2f(tem5_1, tem5_2);
                }

                // theta6
                float tem6_1 = -nx * (cosf(tem1)*cosf(tem23)*sinf(theta4[k]) - sinf(tem1)*cosf(theta4[k])) -
                       ny * (sinf(tem1)*cosf(tem23)*sinf(theta4[k]) + cosf(tem1)*cosf(theta4[k])) -
                       nz * sinf(tem23) * sinf(theta4[k]);
                float tem6_2 = nx * ((cosf(tem1)*cosf(tem23)*cosf(theta4[k]) + sinf(tem1)*sinf(theta4[k])) * cosf(theta5[k]) -
                             cosf(tem1)*sinf(tem23)*sinf(theta5[k])) +
                       ny * ((sinf(tem1)*cosf(tem23)*cosf(theta4[k]) - cosf(tem1)*sinf(theta4[k])) * cosf(theta5[k]) -
                             sinf(tem1)*sinf(tem23)*sinf(theta5[k])) +
                       nz * (sinf(tem23)*cosf(theta4[k])*cosf(theta5[k]) + cosf(tem23)*sinf(theta5[k]));

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
            float offset[6] = {0.0f, 0.0f, M_PI_2, 0.0f, 0.0f, 0.0f};
            float AllSolutions[8][6];

            for (uint8_t i = 0; i < 8; i++) {
                for (uint8_t k = 0; k < 6; k++) {
                    AllSolutions[i][k] = wrapPi(thetax_8[k][i] - offset[k]);
                }
            }

            // 角度限幅
            uint8_t validCount = 0;

            for (uint8_t i = 0; i < 8; i++) {
                float q_tmp[6];
                for (uint8_t j = 0; j < 6; j++) {
                    q_tmp[j] = wrapPi(thetax_8[j][i] - offset[j]);
                    arm_theta.raw_data[i][j] = q_tmp[j];
                    if (q_tmp[j] < Lim[j][0] || q_tmp[j] > Lim[j][1]) {
                        goto next_solution;
                    }
                }
                for (uint8_t k = 0; k < 6; k++) {
                    AllSloverTheta[validCount][k] = q_tmp[k];
                    arm_theta.cur_solutions[validCount][k] = q_tmp[k];
                }
                validCount++;

                next_solution:;
            }

            clc_time[1] = bsp_time_get_us() - lst_cle_time[1];
            return AllSloverTheta;
        }

        const app_Arm_Theta_t *app_Arm_Theta() {
            return &arm_theta;
        }

        uint32_t clc_time[2] = {};
        uint32_t lst_cle_time[2] = {};

    private:
        float a2, a3, d2, d4;
        Matrixf<4,4> Base, Tool;
        Matrixf<4,4> Base_inv, Tool_inv;
        Matrixf<4,4> T_joint[6];
        Matrixf<4, 4> T_, T_end;
        Matrixf<6,1> cur_q;
        app_Arm_Theta_t arm_theta;

        const float Lim[6][2] = {
            {-M_PI, M_PI},
            {0.0f, M_PI},
            {0.0f, 150.0f * M_PI/180.0f},
            {-M_PI, M_PI},
            {-150.0f * M_PI/180.0f, 150.0f * M_PI/180.0f},
            {-M_PI, M_PI}
        };
    };
    // class Dynamic {
    //     public:
    //     Dynamic();
    //     Dynamic(float const R_, float const r_, float const L_, float const l_): R(R_), r(r_), L(L_), l(l_) {
    //         float data1[3] = {R_,0,0}, data2[3] = {r_, 0, 0};
    //         Matrixf<3,1> temp1(data1), temp2(data2);
    //         for(uint8_t i = 0; i < 3; i++) {
    //             pos_oc[i] = temp1.rot_z(phi[i]) * temp1;
    //             pos_pa[i] = temp2.rot_z(phi[i]) * temp2;
    //             theta_i[i] = 0;
    //         }
    //     }
    //
    //     Matrixf<3, 1>  tor_clc(const Matrixf<3, 1>& _theta, const Matrixf<3, 1>& vector_OP, const Matrixf<3, 1>& force) {
    //         Matrixf<3, 1> vec_op = vector_OP, vec_force = force;
    //         theta_i[0] = _theta[0][0] ,theta_i[1] = _theta[1][0] ,theta_i[2] = _theta[2][0];
    //         Matrixf<1,3> t_s[3];
    //         for(uint8_t i = 0; i < 3; i++) {
    //             get_bi(theta_i[i],phi[i],&pos_b[i]);
    //             get_cbi(theta_i[i],phi[i],&pos_cb[i]);
    //             pos_s[i] = (vec_op + pos_pa[i] - pos_oc[i] - pos_cb[i])/1000;
    //             t_s[i] = pos_s[i].trans();
    //         }
    //
    //         Matrixf<3,3> temp1 = matrixf::vertcat(matrixf::vertcat(t_s[0], t_s[1]), t_s[2]);
    //         temp1 = matrixf::inv(temp1);
    //         Matrixf<3,3> temp2 = matrixf::zeros<3,3>();
    //         for(uint8_t i = 0; i < 3; i++) {
    //             Matrixf<1,1> temp = t_s[i] * pos_b[i] / 1000;
    //             temp2[i][i] = temp[0][0];
    //         }
    //         Matrixf<3,3> jacobi = (temp1)*(temp2);
    //         Matrixf<1,3> f = vec_force.trans();
    //         tor = f*jacobi;
    //
    //         return tor.trans();
    //     }
    //
    // private:
    //     void get_bi(float _theta, float _phi, Matrixf<3,1> *mat) const {
    //         //这里正负号存疑
    //         float data[3] = {L*sin(_theta),0,L*cos(_theta)};
    //         Matrixf<3,1> temp(data);
    //         *mat = temp.rot_z(_phi) * temp;
    //     }
    //     void get_cbi(float _theta, float _phi, Matrixf<3,1> *mat) const {
    //         float data[3] = {L*cos(_theta),0,-L*sin(_theta)};
    //         Matrixf<3,1> temp(data);
    //         *mat = temp.rot_z(_phi) * temp;
    //     }
    //     float R,r,L,l;
    //     const float phi[3] = {0,
    //                     2.0f * M_PI / 3.0f,
    //                     4.0f * M_PI / 3.0f,};
    //     float theta_i[3];
    //     Matrixf<3,1> pos_oc[3];
    //     Matrixf<3,1> pos_b[3];          //x+bθ其中的b
    //     Matrixf<3,1> pos_cb[3];
    //     Matrixf<3,1> pos_pa[3];
    //     Matrixf<3,1> pos_s[3];
    //     Matrixf<1,3> tor;
    // };
}
#endif

