//
// Created by fish on 2024/12/18.
//

#include "app_chassis.h"
#include "app_msg.h"
#include "app_referee.h"
#include "app_referee_def.h"
#include "ctrl_motor_base_pid.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include "app_ins.h"
#include "app_motor.h"
#include "app_msg_def.h"
#include "app_sys.h"
#include "dev_motor_dji.h"
#include "sys_task.h"

#ifdef COMPILE_CHASSIS

/*
 *  适用于麦克纳姆轮
 *  实现了基础的旋转、平移
 */

/*
 *  麦克纳姆轮
 *  ^ vy
 *  |       LU              RU
 *  |           O ------ O
 *  |           |        |
 *  |           |        |
 *  |           O ------ O
 *  |       LD              RD
 *  O------------------------------> vx
 *
 *  定义每个轮子的正速度为 vy 方向的速度，故 RU、RD 需要 reverse 一下
 *
 *  v_LU =  rotate * sqrt(2) + vy + vx * sqrt(2)
 *  v_LD =  rotate * sqrt(2) + vy + vx * sqrt(2)
 *  v_RU = -rotate * sqrt(2) + vy - vx * sqrt(2)
 *  v_RD = -rotate * sqrt(2) + vy - vx * sqrt(2)
 */

using namespace Motor;
using namespace Controller;

MotorController LU(std::make_unique <DJIMotor> (
	"w_lu",
	DJIMotor::M3508,
	(DJIMotor::Param) { 0x01, E_CAN2, DJIMotor::CURRENT }
));
MotorController LD(std::make_unique <DJIMotor> (
	"w_ld",
	DJIMotor::M3508,
	(DJIMotor::Param) { 0x02, E_CAN2, DJIMotor::CURRENT }
));
MotorController RD(std::make_unique <DJIMotor> (
	"w_rd",
	DJIMotor::M3508,
	(DJIMotor::Param) { 0x03, E_CAN2, DJIMotor::CURRENT }
));
MotorController RU(std::make_unique <DJIMotor> (
	"w_ru",
	DJIMotor::M3508,
	(DJIMotor::Param) { 0x04, E_CAN2, DJIMotor::CURRENT }
));
MotorController Save_L(std::make_unique <DJIMotor> (
	"save_left",
	DJIMotor::M2006,
	(DJIMotor::Param) { 0x01, E_CAN1, DJIMotor::CURRENT }
));
MotorController Save_R(std::make_unique <DJIMotor> (
	"save_right",
	DJIMotor::M2006,
	(DJIMotor::Param) { 0x02, E_CAN1, DJIMotor::CURRENT }
));

const auto ins = app_ins_data();
const auto rc = bsp_rc_data();

// 直角坐标系下的底盘速度，符合人类直觉，y 轴正方向为机体前进方向。
double vx = 0, vy = 0;
// 旋转速度
double rotate = 0;

float target = 0;
void set_target(bsp_uart_e e, uint8_t *s, uint16_t l) {
	sscanf((char *) s, "%f", &target);
}

void motor_update(double vx_t, double vy_t, double r_t) {

	auto w1 = r_t + vy_t * M_SQRT2 + vx_t * M_SQRT2;
	auto w2 = r_t - vy_t * M_SQRT2 + vx_t * M_SQRT2;
	auto w3 = r_t - vy_t * M_SQRT2 - vx_t * M_SQRT2;
	auto w4 = r_t + vy_t * M_SQRT2 - vx_t * M_SQRT2;

	LU.update(w1), RU.update(w2), RD.update(w3), LD.update(w4);
}

static bool open_done_L = false;
static bool close_done_L = false;
static bool open_done_R = false;
static bool close_done_R = false;
//双板通信
//收
app_msg_can_receiver <app_msg_gimbal_to_chassis> gimbal(E_CAN3, 0x066);
//发
void send_msg_to_gimbal() {
	app_msg_chassis_to_gimbal pkg = {
		.open_done_L = open_done_L,
		.close_done_L = close_done_L,
		.open_done_R = open_done_R,
		.close_done_R = close_done_R
	};
	app_msg_can_send(E_CAN3, 0x033, pkg);
}

// 静态任务，在 CubeMX 中配置
void app_chassis_task(void *args) {
	// Wait for system init.
	while(!app_sys_ready()) OS::Task::SleepMilliseconds(10);

	gimbal.init();
	// bsp_uart_set_callback(E_UART_DEBUG, set_target);
	uint8_t send_count = 0;
	int8_t save_state[2];
	// 0: lift	1: right
	static int8_t last_dir_L = 0;
	static int8_t last_dir_R = 0;
	constexpr double kSaveCurrent = 1000.0;

	while(true) {

		if (bsp_time_get_ms() - gimbal.timestamp < 100)
		{
			if(gimbal()->vx < 700 && gimbal()->vy < 700 && gimbal()->rotate < 700) {
				vx = gimbal()->vx * 1.67f;
				vy = gimbal()->vy * 1.67f;
				rotate = 3.0f * gimbal()->rotate;
			}else {
				vx = vy = rotate = 0;
			}
			save_state[0] = gimbal()->save_state[0];
			save_state[1] = gimbal()->save_state[1];
		}else {
			vx = vy = rotate = 0;
			save_state[0] = save_state[1] = false;
		}

		const int8_t dir_L = save_state[0];
		const int8_t dir_R = save_state[1];

		// lift/right 方向切换时，清空各自“到位完成”标记
		if (dir_L != last_dir_L) {
			last_dir_L = dir_L;
			open_done_L = false;
			close_done_L = false;
		}
		if (dir_R != last_dir_R) {
			last_dir_R = dir_R;
			open_done_R = false;
			close_done_R = false;
		}

		auto theta = std::atan2(vy, vx), r = std::sqrt((vx * vx) + (vy * vy));
		theta -= M_PI / 4096;//无解算
		// theta -= ins->yaw / 180 * M_PI;//底盘陀螺仪的小陀螺结算
		vx = r * std::cos(theta), vy = r * std::sin(theta);

		// lift
		if (dir_L == 1) {
			if (open_done_L) Save_L.update(0);
			else Save_L.update(kSaveCurrent);
		} else if (dir_L == -1) {
			if (close_done_L) Save_L.update(0);
			else Save_L.update(-kSaveCurrent);
		} else {
			open_done_L = close_done_L = false;
			Save_L.update(0);
		}

		// right
		if (dir_R == 1) {
			if (open_done_R) Save_R.update(0);
			else Save_R.update(kSaveCurrent);
		} else if (dir_R == -1) {
			if (close_done_R) Save_R.update(0);
			else Save_R.update(-kSaveCurrent);
		} else {
			open_done_R = close_done_R = false;
			Save_R.update(0);
		}

		const bool stall_L = (Save_L.error_code & APP_MOTOR_ERROR_STALL) != 0;
		const bool stall_R = (Save_R.error_code & APP_MOTOR_ERROR_STALL) != 0;
		if (!open_done_L && dir_L == 1 && stall_L) open_done_L = true;
		if (!close_done_L && dir_L == -1 && stall_L) close_done_L = true;
		if (!open_done_R && dir_R == 1 && stall_R) open_done_R = true;
		if (!close_done_R && dir_R == -1 && stall_R) close_done_R = true;

		motor_update(vx, vy, rotate);

		app_msg_vofa_send(E_UART_DEBUG,
			vx,
			vy,
			rotate,
			gimbal()->vx,
			gimbal()->vy,
			gimbal()->rotate,
			gimbal()->save_state[0],
			gimbal()->save_state[1],
			Save_L.current,
			Save_R.current
		);

		OS::Task::SleepMilliseconds(1);
		if(++ send_count == 10) {
			send_count = 0;
			send_msg_to_gimbal();
		}
	}
}

void app_chassis_init() {
	LU.init(); LD.init(); RU.init(); RD.init();
	Save_L.init(); Save_R.init();

	LU.add_controller(std::make_unique <MotorBasePID> (
		MotorBasePID::PID_SPEED,
		std::make_unique <PID> (14.5, 0.08, 0.03, 16384, 1000),
		nullptr
		));
	LD.add_controller(std::make_unique <MotorBasePID> (
		MotorBasePID::PID_SPEED,
		std::make_unique <PID> (14.5, 0.08, 0.03, 16384, 1000),
		nullptr
		));
	RU.add_controller(std::make_unique <MotorBasePID> (
		MotorBasePID::PID_SPEED,
		std::make_unique <PID> (14.5, 0.08, 0.03, 16384, 1000),
		nullptr
		));
	RD.add_controller(std::make_unique <MotorBasePID> (
		MotorBasePID::PID_SPEED,
		std::make_unique <PID> (14.5, 0.08, 0.03, 16384, 1000),
		nullptr
		));

	Save_L.add_controller(std::make_unique <MotorBasePID> (
		MotorBasePID::PID_SPEED,
		std::make_unique <PID> (10.5, 0.08, 0.03, 16384, 1000),
		nullptr
	));
	Save_R.add_controller(std::make_unique <MotorBasePID> (
		MotorBasePID::PID_SPEED,
		std::make_unique <PID> (10.5, 0.08, 0.03, 16384, 1000),
		nullptr
	));

	Save_L.use_stall_detect = true;
	Save_R.use_stall_detect = true;
	Save_L.stall_detector_time_threshold = 120;
	Save_R.stall_detector_time_threshold = 120;
	// 设置堵转阈值
	Save_L.stall_detector_current_threshold = 2000.0f;
	Save_R.stall_detector_current_threshold = 2000.0f;

	// LU.relax(); LD.relax(); RU.relax(); RD.relax();

}

#endif