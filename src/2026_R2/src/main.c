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
// #include "ares/ekf/QuaternionEKF.h"
#include "devices.h"
#include <arm_math.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);
#ifndef M_PI
#define M_PI 3.14159265f
#endif

int pub_cnt = 0;
#define CHASSIS_SRC_SBUS

// void Sensor_update_cb(QEKF_INS_t *QEKF)
// {
// 	struct pos_data pos = {0};

// 	pos.accel[0] = QEKF->Accel[X];
// 	pos.accel[1] = QEKF->Accel[Y];
// 	pos.accel[2] = QEKF->Accel[Z];
// 	pos.Yaw = QEKF->Yaw;

// 	if (pub_cnt++ % 2) {
// 		// chassis_update_sensor(chassis, &pos);
// 	}
// }

int rank[4] = {1, 1, -1, -1};
bool ups[4] = {true, true, true, true};
const struct device *dmmotor[4];
void run()
{
	for (int id = 0; id < 4; id++) {
		motor_set_mit(dmmotor[id], 10.0f, -50.0f * rank[id], 0.0);
		ups[id] = true;
		k_sleep(K_USEC(130));
	}
}
void lift(int id, bool up)
{
	if (up && (!ups[id])) {
		motor_set_mit(dmmotor[id], 10.0f, 0.0f * rank[id], 0.0);
		ups[id] = true;
	} else if ((!up) && (ups[id])) {
		motor_set_mit(dmmotor[id], 10.0f, -800.0f * rank[id], 0.0);
		ups[id] = false;
	}
	k_sleep(K_USEC(130));
}

void console_feedback(void *arg1, void *arg2, void *arg3)
{
	float angvel = 0;
	int cnt = 0;
	bool zeroed = false;

	while (1) {
		k_msleep(5);
		// chassis_set_static(chassis, false);
		// chassis_set_speed(chassis,  2.5f, 2.5f);
#ifdef CHASSIS_SRC_SBUS
		if (sbus_get_percent(sbus, 4) > 0.5f) {
			lift(0, false);
			lift(1, false);
			lift(2, false);
			lift(3, false);

		} else if (sbus_get_percent(sbus, 4) < -0.5f) {
			run();
		}
		if (sbus_get_percent(sbus, 5) > 0.5f) {
			lift(2, false);
			lift(1, false);
		} else if (sbus_get_percent(sbus, 5) < -0.5f) {
			lift(2, true);
			lift(1, true);
		}
		if (sbus_get_percent(sbus, 6) > 0.5f) {
			lift(0, false);
			lift(3, false);
		} else if (sbus_get_percent(sbus, 6) < -0.5f) {
			lift(0, true);
			lift(3, true);
		}
		angvel = -sbus_get_percent(sbus, 0);
		float X = -sbus_get_percent(sbus, 3);
		float Y = -sbus_get_percent(sbus, 1);
		float linear_magnitude = sqrtf(X * X + Y * Y);
		bool in_deadzone = ((linear_magnitude < 0.06f) && (fabsf(angvel) < 0.06f));
		if (linear_magnitude < 0.06f) {
			X = 0;
			Y = 0;
		}
		if (in_deadzone) {
			chassis_set_static(chassis, true);
			zeroed = true;
		} else {
			chassis_set_static(chassis, false);
			chassis_set_speed(chassis, -X * 1.0f, Y * 1.0f);
			chassis_set_gyro(chassis, angvel * 10.0f);
			zeroed = false;
		}

		// if (cnt++ % 500 == 0) {
		// 		// LOG_INF("Manual: X=%.1f Y=%.1f Gyro=%.1f", (double)X, (double)Y,
		// 		// 	(double)angvel);
		// 		LOG_INF("Chassis: X=%.1f Y=%.1f Gyro=%.1f",
		// (double)sbus_get_percent(sbus, 4), (double)sbus_get_percent(sbus, 5),
		// (double)sbus_get_percent(sbus, 6));

		// }
#endif
	}
}

K_THREAD_DEFINE(feedback_thread, 4096, console_feedback, NULL, NULL, NULL, 2, 0, 100);
uint32_t log_cnt;

int main(void)
{
	k_msleep(2000);

	dmmotor[0] = dm_motor1;
	dmmotor[1] = dm_motor2;
	dmmotor[2] = dm_motor3;
	dmmotor[3] = dm_motor4;
	motor_control(dm_motor1, ENABLE_MOTOR);
	motor_control(dm_motor2, ENABLE_MOTOR);
	motor_control(dm_motor3, ENABLE_MOTOR);
	motor_control(dm_motor4, ENABLE_MOTOR);

	motor_set_mode(dm_motor1, MIT);
	motor_set_mode(dm_motor2, MIT);
	motor_set_mode(dm_motor3, MIT);
	motor_set_mode(dm_motor4, MIT);

	k_msleep(2000);
	motor_control(wheel_motor1, ENABLE_MOTOR);
	motor_control(wheel_motor2, ENABLE_MOTOR);
	motor_control(wheel_motor3, ENABLE_MOTOR);
	motor_control(wheel_motor4, ENABLE_MOTOR);
	k_msleep(100); // 等待电机使能
	motor_set_mode(wheel_motor1, ML_SPEED);
	motor_set_mode(wheel_motor2, ML_SPEED);
	motor_set_mode(wheel_motor3, ML_SPEED);
	motor_set_mode(wheel_motor4, ML_SPEED);
	motor_control(steer_motor1, ENABLE_MOTOR);
	motor_control(steer_motor2, ENABLE_MOTOR);
	motor_control(steer_motor3, ENABLE_MOTOR);
	motor_control(steer_motor4, ENABLE_MOTOR);
	chassis_set_enabled(chassis, false);
	// IMU_Sensor_trig_init(accel_dev, gyro_dev);
	k_msleep(100);
	// IMU_Sensor_set_update_cb(Sensor_update_cb);
	chassis_set_enabled(chassis, true);
	chassis_set_gyro(chassis, 0);

	while (1) {
		k_msleep(2000);
	}

	return 0;
}
