#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/motor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(lk_test_sched_demo, LOG_LEVEL_INF);

static const struct device *const lk_motor = DEVICE_DT_GET(DT_PATH(motor, lk_motor));

/**
 * @brief 详细的电机状态日志
 */
static void log_status(void)
{
	motor_status_t status = {0};
	int ret = motor_get(lk_motor, &status);

	if (ret != 0) {
		LOG_ERR("motor_get failed: %d", ret);
		return;
	}

	LOG_INF("motor state: mode=%d angle=%.2f° rpm=%.2f torque=%.3fNm temp=%.1f°C "
		"limits: speed=%.0frpm torque=%.3fNm",
		status.mode, (double)status.angle, (double)status.rpm,
		(double)status.torque, (double)status.temperature,
		(double)status.speed_limit[0], (double)status.torque_limit[0]);
}

/**
 * @brief 测试命令发送
 */
static void test_commands(void)
{
	LOG_INF("--- Testing motor commands ---");

	/* 1. 测试清除错误 */
	LOG_INF("1. Clear error");
	motor_control(lk_motor, CLEAR_ERROR);
	LOG_INF("   Clear error sent (async)");
	k_msleep(100);

	/* 2. 测试设置零点 */
	LOG_INF("2. Set zero position");
	motor_control(lk_motor, SET_ZERO);
	LOG_INF("   Set zero sent (async)");
	k_msleep(200);

	LOG_INF("--- Command testing completed ---");
}

/**
 * @brief 测试不同控制模式
 */
static void test_control_modes(void)
{
	motor_status_t target = {0};
	int ret;

	LOG_INF("--- Testing control modes ---");

	/* 测试速度控制模式 */
	LOG_INF("1. Testing SPEED mode");
	target.mode = ML_SPEED;
	target.rpm = 600.0f;  /* 600 RPM */
	target.torque_limit[0] = 2.0f;  /* 2 Nm */

	ret = motor_set(lk_motor, &target);
	if (ret != 0) {
		LOG_ERR("   SPEED mode set failed: %d", ret);
	} else {
		LOG_INF("   SPEED mode set successfully");
		k_msleep(1000);
		log_status();
	}

	/* 测试转矩控制模式 */
	LOG_INF("2. Testing TORQUE mode");
	target.mode = ML_TORQUE;
	target.torque = 1.0f;  /* 1 Nm */

	ret = motor_set(lk_motor, &target);
	if (ret != 0) {
		LOG_ERR("   TORQUE mode set failed: %d", ret);
	} else {
		LOG_INF("   TORQUE mode set successfully");
		k_msleep(1000);
		log_status();
	}

	/* 切回角度控制模式 */
	LOG_INF("3. Switching back to ANGLE mode");
	target.mode = ML_ANGLE;
	target.angle = 0.0f;
	target.speed_limit[0] = 800.0f;

	ret = motor_set(lk_motor, &target);
	if (ret != 0) {
		LOG_ERR("   ANGLE mode set failed: %d", ret);
	} else {
		LOG_INF("   ANGLE mode set successfully");
		k_msleep(1000);
		log_status();
	}

	LOG_INF("--- Control mode testing completed ---");
}

/**
 * @brief 测试启停循环
 */
static void test_enable_disable_cycle(void)
{
	int cycle_count = 0;

	LOG_INF("--- Testing enable/disable cycle ---");

	for (cycle_count = 0; cycle_count < 3; cycle_count++) {
		LOG_INF("Cycle %d: Disable motor", cycle_count + 1);
		motor_control(lk_motor, DISABLE_MOTOR);
		k_msleep(500);

		LOG_INF("Cycle %d: Enable motor", cycle_count + 1);
		motor_control(lk_motor, ENABLE_MOTOR);
		k_msleep(500);

		log_status();
	}

	LOG_INF("--- Enable/disable cycle completed ---");
}

int main(void)
{
	if (!device_is_ready(lk_motor)) {
		LOG_ERR("%s not ready", lk_motor->name);
		return -ENODEV;
	}

	LOG_INF("╔══════════════════════════════╗");
	LOG_INF("   LK Motor Functional Test Demo");
	LOG_INF("   Device: %s", lk_motor->name);
	LOG_INF("╚══════════════════════════════╝");

	/* 等待启动稳定 */
	k_msleep(200);

	/* 测试 1: 命令功能测试 */
	test_commands();

	/* 测试 2: 使能电机 */
	LOG_INF("Enabling motor for functional test");
	motor_control(lk_motor, ENABLE_MOTOR);
	k_msleep(100);

	/* 测试 3: 控制模式测试 */
	test_control_modes();

	/* 测试 4: 启停循环测试 */
	test_enable_disable_cycle();

	/* 重新使能电机进入主测试循环 */
	motor_control(lk_motor, ENABLE_MOTOR);
	k_msleep(500);

	/* 主测试循环：持续位置控制 */
	LOG_INF("╔══════════════════════════════╗");
	LOG_INF("   Main Test Loop: Continuous Angle Control");
	LOG_INF("╚════════════════════════════════╝");

	motor_status_t target = {
		.mode = ML_ANGLE,
		.angle = 0.0f,
		.speed_limit = {800.0f, 800.0f},
	};
	float direction = 1.0f;
	int loop_count = 0;

	while (1) {
		int ret;

		loop_count++;

		/* 每 10 次循环输出一次状态 */
		if (loop_count % 10 == 0) {
			LOG_INF("Loop %d: angle=%.2f° direction=%+.1f", loop_count,
					(double)target.angle, (double)direction);
			log_status();
		}

		/* 更新目标位置 */
		target.angle += 45.0f * direction;

		/* 边界检查 */
		if (target.angle >= 180.0f) {
			target.angle = 180.0f;
			direction = -1.0f;
			LOG_INF("Reached +180°, changing direction to -1");
		} else if (target.angle <= -180.0f) {
			target.angle = -180.0f;
			direction = 1.0f;
			LOG_INF("Reached -180°, changing direction to +1");
		}

		/* 发送目标 */
		ret = motor_set(lk_motor, &target);
		if (ret != 0) {
			LOG_ERR("motor_set failed at angle=%.2f°: %d", (double)target.angle, ret);
		}

		/* 控制循环时间 */
		k_msleep(100);
	}

	return 0;
}
