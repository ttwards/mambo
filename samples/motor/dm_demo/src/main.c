/*
 * 达妙DM电机控制示例 - 简化版
 * 上电直接设置转速100 RPM
 *
 * Copyright (c) 2024 ttwards
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/motor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(dm_motor_demo, LOG_LEVEL_INF);

// 电机设备定义
static const struct device *dm_motor = DEVICE_DT_GET(DT_PATH(motor, dm_motor));

/**
 * @brief 电机状态监控线程
 */
void motor_monitor_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	while (1) {
		k_msleep(1000); // 每秒显示一次状态

		motor_status_t status;
		int ret = motor_get(dm_motor, &status);

		if (ret != 0) {
			LOG_ERR("读取电机状态失败: %d", ret);
			continue;
		}

		// 显示电机状态信息
		LOG_INF("电机状态 - 扭矩: %.2f Nm, 速度: %.1f RPM, 角度: %.1f°", status.torque,
			status.rpm, status.angle);
	}
}

K_THREAD_DEFINE(motor_monitor, 1024, motor_monitor_thread, NULL, NULL, NULL, 5, 0, 0);

/**
 * @brief 主函数
 */
int main(void)
{
	LOG_INF("=== 达妙DM电机控制示例 - 简化版 ===");


	// 检查电机设备是否就绪
	if (!device_is_ready(dm_motor)) {
		LOG_ERR("达妙DM电机设备未就绪");
		return -ENODEV;
	}


	// motor_control(dm_motor, SET_ZERO);
	motor_control(dm_motor, ENABLE_MOTOR);
	LOG_INF("电机已启用");
	motor_set_mode(dm_motor, MIT);

	k_msleep(1000);


	motor_set_mit(dm_motor, 10.0f,720.0f,0.0);
	k_msleep(500);




	while (1) {
		// motor_set_mit(dm_motor, 10.0f,90.0f,0.0);
		// k_msleep(1000);
		// motor_set_mit(dm_motor, 10.0f,180.0f,0.0);
		// k_msleep(1000);
		// motor_set_mit(dm_motor, 10.0f,360.0f,0.0);
		k_msleep(1000);
		// motor_set_speed(dm_motor, 200.0f);
	}

	return 0;
}
