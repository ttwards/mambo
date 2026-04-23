/*
 * Copyright (c) 2024 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdbool.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/chassis.h>
#include <zephyr/drivers/pid.h>
#include <zephyr/drivers/wheel.h>
#include <zephyr/logging/log.h>
#include "chassis.h"
#include "zephyr/kernel.h"
#include <arm_math.h>
#include "zephyr/kernel/thread.h"
#include "zephyr/sys/time_units.h"
#include "zephyr/toolchain.h"
#include <math.h>

#define DT_DRV_COMPAT ares_chassis

#define RAD2DEG(x) 57.2957795131f * x

LOG_MODULE_REGISTER(chassis, CONFIG_MOTOR_LOG_LEVEL);

#ifndef M_PI_4
#define M_PI_4 0.78539816339744830962f
#define M_PI_2 1.57079632679489661923f
#endif

void chassis_timer_cb(struct k_timer *timer);

// struct k_thread chassis_thread_data;
// K_THREAD_STACK_DEFINE(chassis_stack_area, CHASSIS_STACK_SIZE);

K_TIMER_DEFINE(chassis_timer, chassis_timer_cb, NULL);

int cchassis_init(const struct device *dev)
{
	chassis_data_t *data = dev->data;
	const chassis_cfg_t *cfg = dev->config;
	for (int idx = 0; idx < CHASSIS_WHEEL_COUNT; idx++) {
		float arc;
		arm_atan2_f32(cfg->pos_Y_offset[idx], cfg->pos_X_offset[idx], &arc);

		data->angle_to_center[idx] = -RAD2DEG(arc) + 90.0f;
		arm_sqrt_f32(cfg->pos_X_offset[idx] * cfg->pos_X_offset[idx] +
				     cfg->pos_Y_offset[idx] * cfg->pos_Y_offset[idx],
			     &data->distance_to_center[idx]);
	}
	data->currTime = k_cycle_get_32();
	data->prevTime = data->currTime - 100;

	// k_thread_create(&chassis_thread_data, chassis_stack_area,
	// 		K_THREAD_STACK_SIZEOF(chassis_stack_area), chassis_thread, (void *)dev,
	// 		NULL, NULL, -2, 0, K_NO_WAIT);
	k_timer_user_data_set(&chassis_timer, (void *)dev);
	k_timer_start(&chassis_timer, K_MSEC(100), K_MSEC(8));
	return 0;
}

void cchassis_set_angle(const struct device *dev, float angle)
{
	chassis_data_t *data = dev->data;
	data->target_status.angle = angle;
	data->angleControl = true;
}

void cchassis_set_speed(const struct device *dev, float x_speed, float y_speed)
{
	chassis_data_t *data = dev->data;
	data->target_status.speedX = x_speed;
	data->target_status.speedY = y_speed;
}

void cchassis_set_gyro(const struct device *dev, float gyro)
{
	chassis_data_t *data = dev->data;
	data->target_status.gyro = gyro;
	data->angleControl = false;
}

int cchassis_set_static(const struct device *dev, bool static_angle)
{
	chassis_data_t *data = dev->data;
	data->static_angle = static_angle;
	data->target_status.speedX = 0;
	data->target_status.speedY = 0;
	if (fabsf(data->set_status.speedX) > 0.08f || fabsf(data->set_status.speedY) > 0.08f) {
		return -EBUSY;
	}
	return 0;
}

chassis_status_t *cchassis_get_status(const struct device *dev)
{
	chassis_data_t *data = dev->data;
	return &data->chassis_status;
}

void cchassis_resolve(chassis_data_t *data, const chassis_cfg_t *cfg)
{
	// data->targetRollSpeed = 0;

	for (int idx = 0; idx < CHASSIS_WHEEL_COUNT; idx++) {

		float rollSpeedX = -cfg->pos_Y_offset[idx] * data->set_status.gyro;
		float rollSpeedY = cfg->pos_X_offset[idx] * data->set_status.gyro;
		float speedX, speedY;
		if (data->static_angle &&
		    (data->set_status.speedX * data->set_status.speedX +
		     data->set_status.speedY * data->set_status.speedY) < 0.000001f) {
			int err = wheel_set_static(cfg->wheels[idx], data->angle_to_center[idx]);
			if (err < 0) {
				wheel_set_speed(cfg->wheels[idx], 0, data->angle_to_center[idx]);
			}
			continue;
		}
		if (!data->track_angle) {
			speedX = rollSpeedX + data->set_status.speedX;
			speedY = rollSpeedY + data->set_status.speedY;
		} else {
			// 将角度转换为弧度
			float radians = -data->chassis_sensor_data.Yaw * PI / 180.0f;
			// 计算旋转后的新速度
			speedX = data->set_status.speedX * cosf(radians) -
				 data->set_status.speedY * sinf(radians) + rollSpeedX;
			speedY = data->set_status.speedX * sinf(radians) +
				 data->set_status.speedY * cosf(radians) + rollSpeedY;
		}
		float steerwheel_speed;
		arm_sqrt_f32(speedX * speedX + speedY * speedY, &steerwheel_speed);
		float steerwheel_angle = 0;
		arm_atan2_f32(speedY, speedX, &steerwheel_angle);

		steerwheel_angle = -RAD2DEG(steerwheel_angle) + 90.0f;

		wheel_set_speed(cfg->wheels[idx], steerwheel_speed, steerwheel_angle);
		k_sleep(K_USEC(130));
	}
}

K_SEM_DEFINE(chassis_sem, 0, 1);
void chassis_timer_cb(struct k_timer *timer)
{
	const struct device *dev = (const struct device *)k_timer_user_data_get(timer);
	chassis_data_t *data = dev->data;
	if (data->enabled) {
		k_sem_give(&chassis_sem);
	}
}

void chassis_thread_entry(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	k_thread_name_set(k_current_get(), "chassis");

	const struct device *dev = (const struct device *)arg1;
	if (dev == NULL) {
		return;
	}

	chassis_data_t *data = dev->data;
	const chassis_cfg_t *cfg = dev->config;

	while (!data->enabled) {
		for (int i = 0; i < CHASSIS_WHEEL_COUNT; i++) {
			wheel_set_static(cfg->wheels[i], 90.0f);
		}
	}

	while (1) {
		k_sem_take(&chassis_sem, K_MSEC(5));
		data->prevTime = data->currTime;
		data->currTime = k_cycle_get_32();

		// LOG_INF("Yaw: %f, targetYaw: %f, delta_Yaw: %f\n", data->currentYaw,
		// data->target_status.angle,
		//        delta_Yaw);
		if ((data->chassis_sensor_data.accel[2] - 9.8f) > 0.4f) {
			// We are in the air
		}
		int32_t deltaTimeUs = k_cyc_to_us_floor32(data->currTime - data->prevTime);
		float error = 0;

		if (!isnanf(data->chassis_sensor_data.Yaw) && data->angleControl) {
			float delta_Yaw = data->chassis_sensor_data.Yaw - data->target_status.angle;
			delta_Yaw = fmodf(delta_Yaw, 360.0f);
			if (delta_Yaw > 180) {
				delta_Yaw -= 360.0f;
			} else if (delta_Yaw < -180) {
				delta_Yaw += 360.0f;
			}
			if (fabsf(delta_Yaw) > 0.8f) {
				error = delta_Yaw;
			}
		}

		// float currentSpeedX[CHASSIS_WHEEL_COUNT] = {0};
		// float currentSpeedY[CHASSIS_WHEEL_COUNT] = {0};
		data->chassis_status.speedX = 0;
		data->chassis_status.speedY = 0;
		data->chassis_status.gyro = 0;
		// float currentGyro = 0;

		// for (int idx = 0; cfg->wheels[idx] != NULL; idx++) {
		// 	// 由于我们的轮电机使用的是速度环PID控制
		// 	// 我不认为它们会疯转，但可能存在遇到障碍
		// 	wheel_status_t *status = wheel_get_speed(cfg->wheels[idx]);
		// 	// float angle = status->angle;
		// 	float speed = status->speed;

		// 	float sin_theta, cos_theta;

		// 	arm_sin_cos_f32(data->angle_to_center[idx] + data->chassis_sensor_data.Yaw,
		// 			&sin_theta, &cos_theta);
		// 	currentSpeedX[idx] = speed * cos_theta;
		// 	currentSpeedY[idx] = speed * sin_theta;

		// 	data->chassis_status.speedX += currentSpeedX[idx];
		// 	data->chassis_status.speedY += currentSpeedY[idx];
		// }

		// data->chassis_status.speedX /= CHASSIS_WHEEL_COUNT;
		// data->chassis_status.speedY /= CHASSIS_WHEEL_COUNT;

		float delta_speed_X = data->target_status.speedX - data->set_status.speedX;
		float delta_speed_Y = data->target_status.speedY - data->set_status.speedY;
		float deltaTime = (float)deltaTimeUs * 0.000001f;

		if (deltaTime > 1e-6f) {
			// Calculate ground acceleration, including centripetal acceleration,
			// expressed in chassis frame
			float ax_d = delta_speed_X / deltaTime; // desired linear acceleration x
			float ay_d = delta_speed_Y / deltaTime; // desired linear acceleration y

			float ax_total, ay_total;

			if (data->track_angle) {
				// When track_angle is true, speeds and accelerations are in the
				// ground frame.
				ax_total = ax_d;
				ay_total = ay_d;
			} else {
				// When track_angle is false, speeds are in the chassis frame.
				// To get the total acceleration in the ground frame (expressed in
				// chassis coords), we must add the centripetal acceleration.
				float vx = data->set_status.speedX;
				float vy = data->set_status.speedY;
				float omega = data->target_status.gyro;

				// Centripetal acceleration component: a_c = omega x v
				float ax_c = -omega * vy;
				float ay_c = omega * vx;

				// Total ground acceleration (in chassis frame)
				ax_total = ax_d + ax_c;
				ay_total = ay_d + ay_c;
			}

			float a_total_mag;
			arm_sqrt_f32(ax_total * ax_total + ay_total * ay_total, &a_total_mag);

			if (a_total_mag > cfg->max_lin_accel) {
				float scale = cfg->max_lin_accel / a_total_mag;
				if (data->track_angle) {
					ax_d *= scale;
					ay_d *= scale;
				} else {
					// The new limited total acceleration
					ax_total *= scale;
					ay_total *= scale;

					// We can only control the linear acceleration part (ax_d,
					// ay_d), so we subtract the centripetal part to find the
					// new ax_d and ay_d.
					float vx = data->set_status.speedX;
					float vy = data->set_status.speedY;
					float omega = data->target_status.gyro;
					float ax_c = -omega * vy;
					float ay_c = omega * vx;
					ax_d = ax_total - ax_c;
					ay_d = ay_total - ay_c;
				}

				// Update delta_speed
				delta_speed_X = ax_d * deltaTime;
				delta_speed_Y = ay_d * deltaTime;
			}
		}

		data->set_status.speedX += delta_speed_X;
		data->set_status.speedY += delta_speed_Y;

		// float currentSpeed = sqrtf(data->chassis_status.speedX *
		// data->chassis_status.speedX +
		//    data->chassis_status.speedY * data->chassis_status.speedY);

		if (!data->angleControl) {
			data->set_status.gyro = data->target_status.gyro;
		} else {
			data->set_status.gyro = pid_calc_in(cfg->angle_pid, error, deltaTimeUs);
		}

		data->set_status.gyro = data->set_status.gyro > cfg->max_gyro ? cfg->max_gyro
					: data->set_status.gyro < -cfg->max_gyro
						? -cfg->max_gyro
						: data->set_status.gyro;

		cchassis_resolve(data, cfg);
	}
}

K_THREAD_DEFINE(chassis_thread, CHASSIS_STACK_SIZE, chassis_thread_entry,
		DEVICE_DT_GET(DT_DRV_INST(0)), NULL, NULL, 1, 0, 0);

struct chassis_driver_api chassis_driver_api = {
	.set_angle = cchassis_set_angle,
	.set_speed = cchassis_set_speed,
	.set_gyro = cchassis_set_gyro,
	.get_status = cchassis_get_status,
	.set_static = cchassis_set_static,
};

DT_INST_FOREACH_STATUS_OKAY(CHASSIS_INIT)
