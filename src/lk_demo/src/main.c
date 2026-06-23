

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/motor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(dm_motor_demo, LOG_LEVEL_INF);

// 电机设备定义
static const struct device *dm_motor = DEVICE_DT_GET(DT_PATH(motor, lk_motor));

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
struct motor_driver_data status = {
	.angle = 90.0f,
	.speed_limit = {-100.0f, 100.0f},
	.mode = ML_ANGLE,

};
struct motor_driver_data status1 = {
	// .torque_limit = {-12.0f, 12.0f},
	// .rpm = 100,
	.torque=0.8f,
	.mode = ML_TORQUE,
	
};
K_THREAD_DEFINE(motor_monitor, 1024, motor_monitor_thread, NULL, NULL, NULL, 5, 0, 0);

/**
 * @brief 主函数
 */
int main(void)
{
	LOG_INF("=== 达妙DM电机控制示例 - 简化版 ===");
	LOG_INF("上电直接设置转速100 RPM");

	// 检查电机设备是否就绪
	// if (!device_is_ready(dm_motor)) {
	// 	LOG_ERR("达妙DM电机设备未就绪");
	// 	return -ENODEV;
	// }
motor_control(dm_motor, SET_ZERO);
	// 启用电机
	LOG_INF("电机已启用");motor_control(dm_motor, ENABLE_MOTOR);
	
	
	// motor_set_mode(dm_motor, VO);
	// 等待电机初始化完成
	k_msleep(1000);

	// 设置电机转速为100 RPM
	
	motor_set(dm_motor, &status);
	k_msleep(500);
	// motor_set_vo(dm_motor, 800.0f);
	LOG_INF("电机转速已设置为100 RPM");

	// 主循环 - 保持程序运行
	while (1) {
		// motor_control(dm_motor, SET_ZERO);
		status.angle += 90.0f;
		motor_set(dm_motor, &status);
		k_msleep(1000);
		// motor_set(dm_motor, &status);
		// motor_set_speed(dm_motor, 200.0f);
	}

	return 0;
}
