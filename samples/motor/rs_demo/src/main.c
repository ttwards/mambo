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

#define PID_CONTROLLER DT_NODELABEL(mit_pid_1)
const struct device *pid_controller = DEVICE_DT_GET(PID_CONTROLLER);

// 测试序列状态
typedef enum {
	STATE_SET_ZERO_1 = 0, // 设置零点1
	STATE_SPEED_TEST,     // 100rpm转10秒
	STATE_STOP,           // 停下
	STATE_SET_ZERO_2,     // 设置零点2
	STATE_ANGLE_TEST,     // 正转180度
	STATE_MAX
} test_state_t;

static test_state_t current_state = STATE_SET_ZERO_1;
static uint32_t state_start_time = 0;

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

	motor_control(motor, AUTO_REPORT_ENABLE);
	k_msleep(100); // 等待自动报告使能

	motor_control(motor, SET_ZERO);
	k_msleep(100);

	motor_set_mode(motor, MIT);
	LOG_INF("电机已启用");
	k_msleep(1000);
	
	
	struct pid_config pid_params;
	pid_get_params(pid_controller, &pid_params);
	/* Start Feedback thread*/
	while (1) {

		// 重置PID参数
		pid_params.k_p = 20.0f;
		pid_params.k_d = 1.0f;
		pid_set_params(pid_controller, &pid_params);
		// 位置环测试
		motor_set_mit(motor, 0.0f, -90.0f, 0.0f);
		k_msleep(2000);
		motor_set_mit(motor, 0.0f, 0.0f, 0.0f);
		k_msleep(2000);
		motor_set_mit(motor, 0.0f, 90.0f, 0.0f);
		k_msleep(2000);
		// // 速度环测试
		pid_params.k_p = 0.0f;
		pid_set_params(pid_controller, &pid_params);
		motor_set_mit(motor, 100.0f, 0.0f, 0.0f);
		k_msleep(1000);
		// 测试结束
	}

	return 0;
}
