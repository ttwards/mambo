/**
 * RS02 BLDC Demo Application
 *
 * @author: EclipseaHime017
 * @create: 2025-12-18
 * @update: 2026-03-16
 * Description: Demonstrates control of the Robstride RS02 brushless DC motor using Zephyr RTOS.
 */

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/motor.h>
#include <zephyr/logging/log.h>
#include <math.h>

LOG_MODULE_REGISTER(rs02_demo, LOG_LEVEL_INF);

#define HIGH_BYTE(x)           ((x) >> 8)
#define LOW_BYTE(x)            ((x) & 0xFF)
#define COMBINE_HL8(HIGH, LOW) ((HIGH << 8) + LOW)

// 电机设备定义
#define MOTOR_NODE DT_INST(0, rs_motor)
const struct device *motor = DEVICE_DT_GET(MOTOR_NODE);

/**
 * @brief 电机状态监控线程
 */
void motor_monitor_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	while (1) {
		motor_status_t status;
		int ret = motor_get(motor, &status);

		while (ret != 0) {
			LOG_ERR("读取电机状态失败: %d", ret);
			k_msleep(1000); // 1秒后重试
			ret = motor_get(motor, &status);
			continue;
		}

		LOG_INF("motor torque: %.2f, speed: %.2f, angle: %.2f", status.torque, status.rpm,
			status.angle);

		k_msleep(200);
	}
}

K_THREAD_DEFINE(motor_monitor, 1024, motor_monitor_thread, NULL, NULL, NULL, 5, 0, 0);

/**
 * @brief 主函数
 */
int main(void)
{
	LOG_INF("===  灵足时代无刷直流电机控制示例  ===");

	// 检查电机设备是否就绪
	if (!device_is_ready(motor)) {
		LOG_ERR("电机设备未就绪");
		return -ENODEV;
	}

	// 启用电机
	motor_control(motor, ENABLE_MOTOR);
	k_msleep(100); // 等待电机使能

	motor_control(motor, SET_ZERO);
	k_msleep(100);

	motor_set_mode(motor, MIT);
	LOG_INF("电机已启用");
	k_msleep(1000);

	/* Start Feedback thread*/
	while (1) {

		// 位置测试
		motor_set_mit(motor, 0.0f, -30.0f, 0.0f);
		k_msleep(2000);
		motor_set_mit(motor, 0.0f, 0.0f, 0.0f);
		k_msleep(2000);
		motor_set_mit(motor, 0.0f, 30.0f, 0.0f);
		k_msleep(2000);
		// 测试结束
	}

	return 0;
}
