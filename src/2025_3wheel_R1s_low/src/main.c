#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/motor.h>
#include <zephyr/drivers/sbus.h>
#include <zephyr/drivers/chassis.h>
#include <ares/board/init.h>
#include <ares/ekf/imu_task.h>
#include <ares/interface/usb/usb_bulk.h>
#include <ares/protocol/dual/dual_protocol.h>
#include <ares/ares_comm.h>
#include <sys/_stdint.h>
#include "ares/ekf/QuaternionEKF.h"
#include "devices.h"
#include <arm_math.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);
#ifndef M_PI
#define M_PI 3.14159265f
#endif

int pub_cnt = 0;
#define CHASSIS_SRC_SBUS
static volatile bool yunqiu_in_progress = false;  // 运球进行中标志
static volatile bool allow_rotation_only = false; // 仅允许旋转标志

void Sensor_update_cb(QEKF_INS_t *QEKF)
{
	struct pos_data pos = {0};

	pos.accel[0] = QEKF->Accel[X];
	pos.accel[1] = QEKF->Accel[Y];
	pos.accel[2] = QEKF->Accel[Z];
	pos.Yaw = QEKF->Yaw;
	
	if (pub_cnt++ % 2) {
		// chassis_update_sensor(chassis, &pos);
	}
}


void yunqiu_low(void)
{
	LOG_INF("Yunqiu low operation...");

	// 设置运球进行中标志 - 允许旋转，禁止移动
	yunqiu_in_progress = true;
	allow_rotation_only = true;

	k_msleep(250);

	// 第一阶段：准备
	motor_set_torque(yq1, 9.5);
	motor_set_torque(yq3, -9.5);
	k_msleep(150);

	// 第二阶段：运动
	motor_set_speed(yq1, -150);
	motor_set_speed(yq3, 150);
	k_msleep(350);

	// 停止
	motor_set_speed(yq1, 0);
	motor_set_speed(yq3, 0);

	// 第一阶段：准备
	k_msleep(350);

	motor_set_torque(yq1, 9.5);
	motor_set_torque(yq3, -9.5);
	k_msleep(150);

	// 第二阶段：运动
	motor_set_speed(yq1, -150);
	motor_set_speed(yq3, 150);
	k_msleep(350);

	// 停止
	motor_set_speed(yq1, 0);
	motor_set_speed(yq3, 0);
	// k_msleep(100);

	// 清除标志 - 恢复完全控制
	yunqiu_in_progress = false;
	allow_rotation_only = false;

	LOG_INF("Yunqiu low operation completed");
}

void takein_low()
{
	LOG_INF("Takein low...");

	// 取球时也可以允许旋转
	yunqiu_in_progress = true;
	allow_rotation_only = true;

	motor_set_speed(yq1, -150);
	motor_set_speed(yq3, 150);
	k_msleep(200);
	motor_set_speed(yq1, 0);
	motor_set_speed(yq3, 0);

	// 恢复控制
	yunqiu_in_progress = false;
	allow_rotation_only = false;

	LOG_INF("Takein low completed");
}

void console_feedback(void *arg1, void *arg2, void *arg3)
{
	float angvel = 0;
	int cnt = 0;
	bool zeroed = false;

	while (1) {
		k_msleep(4);

#ifdef CHASSIS_SRC_SBUS
		angvel = -sbus_get_percent(sbus, 0);
		float X = -sbus_get_percent(sbus, 3);
		float Y = -sbus_get_percent(sbus, 1);

		// 死区处理
		float linear_magnitude = sqrtf(X * X + Y * Y);
		bool in_deadzone = ((linear_magnitude < 0.06f) && (fabsf(angvel) < 0.06f));
		bool rotation_only = (linear_magnitude < 0.06f) && (fabsf(angvel) >= 0.06f);

		// 检查运球状态并相应控制底盘
		if (yunqiu_in_progress && allow_rotation_only) {
			// 运球进行中：禁止移动，允许旋转
			if (rotation_only) {
				// 只有旋转输入，允许旋转
				chassis_set_static(chassis, false);
				chassis_set_speed(chassis, 0, 0); // 强制零移动
				chassis_set_gyro(chassis, angvel * 2.0f);
				zeroed = false;
				LOG_DBG("Yunqiu mode: Rotation allowed, X=0, Y=0, Gyro=%.2f",
					(double)angvel);
			} else if (fabsf(angvel) < 0.06f) {
				// 无旋转输入，完全静止
				chassis_set_static(chassis, true);
				zeroed = true;
			} else {
				// 有移动和旋转输入，只执行旋转
				chassis_set_static(chassis, false);
				chassis_set_speed(chassis, 0, 0); // 强制零移动
				chassis_set_gyro(chassis, angvel * 2.0f);
				zeroed = false;
				LOG_DBG("Yunqiu mode: Only rotation, Gyro=%.2f", (double)angvel);
			}
		} else if (in_deadzone) {
			// 正常死区处理
			chassis_set_static(chassis, true);
			zeroed = true;
		} else {
			// 正常底盘控制（移动+旋转）
			chassis_set_static(chassis, false);
			chassis_set_speed(chassis, -X * 2.5f, -Y * 2.5f);
			chassis_set_gyro(chassis, angvel * 2.5f);
			zeroed = false;
		}

		if (cnt++ % 2000 == 0) {
			if (yunqiu_in_progress && allow_rotation_only) {
				LOG_INF("Manual: ROTATION ONLY (yunqiu) - Gyro=%.1f",
					(double)angvel);
			} else if (yunqiu_in_progress) {
				LOG_INF("Manual: BLOCKED (yunqiu in progress)");
			} else {
				LOG_INF("Manual: X=%.1f Y=%.1f Gyro=%.1f", (double)X, (double)Y,
					(double)angvel);
			}
		}
#endif
	}
}


K_THREAD_DEFINE(feedback_thread, 4096, console_feedback, NULL, NULL, NULL, 2, 0, 100);
uint32_t log_cnt;

int main(void)
{
	chassis_set_enabled(chassis, false);
	IMU_Sensor_trig_init(accel_dev, gyro_dev);
	k_msleep(100); // 减少等待时间
	IMU_Sensor_set_update_cb(Sensor_update_cb);
	chassis_set_enabled(chassis, true);
	chassis_set_gyro(chassis, 0);
	static bool sbus_states[6] = {false}; // 简化状态管理

	while (1) {
		// 读取SBUS
		float sbus_vals[6] = {
			sbus_get_percent(sbus, 4), // takein_up
			sbus_get_percent(sbus, 5), // yunqiu
			sbus_get_percent(sbus, 6), // shoot
		};

		// 统一的SBUS处理逻辑
		if (sbus_vals[0] > 0.0f && !sbus_states[0]) {
			yunqiu_low();
		
			
			sbus_states[0] = true;
		} else if (sbus_vals[0] <= 0.5f) {
			sbus_states[0] = false;
		}

		if (sbus_vals[1] > 0.0f && !sbus_states[1]) {
			takein_low();
			sbus_states[1] = true;
		} else if (sbus_vals[1] <= 0.5f) {
			sbus_states[1] = false;
		}

		if (sbus_vals[2] > 0.0f && !sbus_states[2]) {
			
			sbus_states[2] = true;
		} else if (sbus_vals[2] <= 0.5f) {
			sbus_states[2] = false;
		}

	

		k_msleep(10);

		// 简化日志
		if (++log_cnt % 2000 == 0) {
			const char *status = "";
			if (yunqiu_in_progress && allow_rotation_only) {
				status = " [ROTATION_ONLY]";
			} else if (yunqiu_in_progress) {
				status = " [BLOCKED]";
			}
			LOG_DBG("SBUS: [%.1f %.1f %.1f ]%s", (double)sbus_vals[0],
				(double)sbus_vals[1], (double)sbus_vals[2],  status);
		}
	}

	return 0;
}
