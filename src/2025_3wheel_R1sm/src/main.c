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
void getChassisYaw(float32_t *q_imu, float32_t *pResultYaw)
{
	// IMU的Y轴朝上，需要绕X轴旋转-90度来校正到底盘坐标系
	// 绕X轴旋转-90度的四元数：cos(-45°), sin(-45°), 0, 0
	// = cos(45°), -sin(45°), 0, 0
	// = 0.70710678f, -0.70710678f, 0, 0
	float32_t q_rotation[4] = {0.70710678f, -0.70710678f, 0.0f, 0.0f};

	// 应用旋转到IMU四元数
	float32_t q_chassis[4];
	arm_quaternion_product_f32(q_rotation, q_imu, q_chassis, 1);

	// 提取四元数分量
	float32_t w = q_chassis[0];
	float32_t x = q_chassis[1];
	float32_t y = q_chassis[2];
	float32_t z = q_chassis[3];

	// 计算yaw角（绕Z轴旋转）
	float32_t yaw_rad = atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));

	// 弧度转角度并转换到 0-360° 范围
	float32_t yaw_deg_180 = yaw_rad * (180.0f / M_PI);

	if (yaw_deg_180 < 0) {
		*pResultYaw = yaw_deg_180 + 360.0f;
	} else {
		*pResultYaw = yaw_deg_180;
	}
}
// =============================================================================
// 全局控制标志 - 修改为更精确的控制
// =============================================================================
static volatile bool yunqiu_in_progress = false;  // 运球进行中标志
static volatile bool allow_rotation_only = false; // 仅允许旋转标志

void Sensor_update_cb(QEKF_INS_t *QEKF)
{
	struct pos_data pos = {0};

	pos.accel[0] = QEKF->Accel[X];
	pos.accel[1] = QEKF->Accel[Y];
	pos.accel[2] = QEKF->Accel[Z];
	// pos.Yaw = QEKF->Yaw;
	// int ret = 0;
	getChassisYaw(QEKF->q, &pos.Yaw);
	if (pub_cnt++ % 2) {
		// chassis_update_sensor(chassis, &pos);

		// ret = usb_trans_sync_flush(Q_pack);
		// ret = usb_trans_sync_flush(A_pack);
		// ret = usb_trans_sync_flush(G_pack);
	}
}

void yunqiu_simple(void)
{
	LOG_INF("Yunqiu operation...");

	// 设置运球进行中标志 - 允许旋转，禁止移动
	yunqiu_in_progress = true;
	allow_rotation_only = true;

	k_msleep(300);

	// 第一阶段：准备
	motor_set_torque(yq1, 6);
	motor_set_torque(yq3, -6.9);
	k_msleep(200);

	// 第二阶段：运动
	motor_set_speed(yq1, -240);
	motor_set_speed(yq3, 240);
	k_msleep(400);

	// 第三阶段：收尾
	motor_set_speed(yq1, 110);
	motor_set_speed(yq3, 100);
	motor_set_speed(yq2, 110);
	k_msleep(400);

	// 停止
	motor_set_speed(yq1, 0);
	motor_set_speed(yq3, 0);
	motor_set_speed(yq2, 0);
	k_msleep(300);

	// 清除标志 - 恢复完全控制
	yunqiu_in_progress = false;
	allow_rotation_only = false;

	LOG_INF("Yunqiu operation completed");
}

void yunqiu_low(void)
{
	LOG_INF("Yunqiu low operation...");

	// 设置运球进行中标志 - 允许旋转，禁止移动
	yunqiu_in_progress = true;
	allow_rotation_only = true;

	k_msleep(200);

	// 第一阶段：准备
	motor_set_torque(yq1, 9.5);
	motor_set_torque(yq3, -9.5);
	k_msleep(200);

	// 第二阶段：运动
	motor_set_speed(yq1, -150);
	motor_set_speed(yq3, 150);
	k_msleep(350);

	// 停止
	motor_set_speed(yq1, 0);
	motor_set_speed(yq3, 0);
	k_msleep(200);

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

void takein_up_simple(void)
{
	LOG_INF("Takein up...");
	motor_set_speed(yq1, -150);
	motor_set_speed(yq3, 150);
	k_msleep(300);
	motor_set_speed(yq1, 110);
	motor_set_speed(yq3, 100);
	motor_set_speed(yq2, 110);
	k_msleep(300);
	motor_set_speed(yq1, 0);
	motor_set_speed(yq3, 0);
	motor_set_speed(yq2, 0);
}

void takein_down_simple(void)
{
	LOG_INF("Takein down...");
	motor_set_speed(yq2, -150);
	motor_set_speed(yq1, 150);
	k_msleep(200);
	motor_set_speed(yq1, 110);
	motor_set_speed(yq3, 100);
	motor_set_speed(yq2, 110);
	k_msleep(300);
	motor_set_speed(yq1, 0);
	motor_set_speed(yq3, 0);
	motor_set_speed(yq2, 0);
}

void shoot_simple(void)
{
	LOG_INF("Shooting...");

	// 发射时完全阻止底盘控制（可选）
	yunqiu_in_progress = true;
	allow_rotation_only = false; // 完全禁止移动

	motor_set_torque(yq1, -10);
	motor_set_torque(yq2, 10);
	k_msleep(1000);
	motor_set_speed(yq1, 0);
	motor_set_speed(yq3, 0);
	motor_set_speed(yq2, 0);

	// 恢复控制
	yunqiu_in_progress = false;
	allow_rotation_only = false;

	LOG_INF("Shooting completed");
}

// =============================================================================
// 修改底盘控制回调函数 - 支持旋转控制
// =============================================================================
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
			chassis_set_speed(chassis, -X * 2.0f, -Y * 2.0f);
			chassis_set_gyro(chassis, angvel * 2.0f);
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

// =============================================================================
// 线程和协议定义
// =============================================================================
K_THREAD_DEFINE(feedback_thread, 4096, console_feedback, NULL, NULL, NULL, 2, 0, 100);
uint32_t log_cnt;

// =============================================================================
// 修改主函数中的按钮处理
// =============================================================================
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
			sbus_get_percent(sbus, 7), // task
			sbus_get_percent(sbus, 8), // takein_down
			sbus_get_percent(sbus, 10) // unused
		};

		// 统一的SBUS处理逻辑
		if (sbus_vals[5] > 0.0f && !sbus_states[5]) {
			takein_low();
			sbus_states[5] = true;
		} else if (sbus_vals[5] <= 0.5f) {
			sbus_states[5] = false;
		}

		if (sbus_vals[1] > 0.0f && !sbus_states[1]) {
			// 双重运球操作 - 允许旋转
			yunqiu_low();
			yunqiu_low();
			sbus_states[1] = true;
		} else if (sbus_vals[1] <= 0.5f) {
			sbus_states[1] = false;
		}

		if (sbus_vals[2] > 0.5f && !sbus_states[2]) {
			shoot_simple();
			sbus_states[2] = true;
		} else if (sbus_vals[2] <= 0.5f) {
			sbus_states[2] = false;
		}

		if (sbus_vals[3] > 0.5f && !sbus_states[3]) {
			// 空的任务处理
			sbus_states[3] = true;
		} else if (sbus_vals[3] <= 0.5f) {
			sbus_states[3] = false;
		}

		if (sbus_vals[4] > 0.5f && !sbus_states[4]) {
			// 双重运球操作 - 允许旋转
			yunqiu_low();
			k_msleep(50);
			yunqiu_low();
			sbus_states[4] = true;
		} else if (sbus_vals[4] <= 0.5f) {
			sbus_states[4] = false;
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
			LOG_DBG("SBUS: [%.1f %.1f %.1f %.1f %.1f]%s", (double)sbus_vals[0],
				(double)sbus_vals[1], (double)sbus_vals[2], (double)sbus_vals[3],
				(double)sbus_vals[4], status);
		}
	}

	return 0;
}
