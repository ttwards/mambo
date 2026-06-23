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
#include "ares/ekf/QuaternionEKF.h"
#include "devices.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

// =============================================================================
// 常量定义
// =============================================================================
#define CONTROL_PERIOD_MS 10 // 统一控制周期: 10ms (100Hz)
#define CHASSIS_SRC_SBUS
// =============================================================================
// 简化的PD控制器
// =============================================================================
typedef struct {
	float kp;
	float kd;
	float last_error;
	float last_output; // 用于输出平滑
	bool first_run;
} simple_pd_t;

static simple_pd_t yaw_pd = {.kp = 0.4f,  // 降低P增益，减少跳变
			     .kd = 0.03f, // 降低D增益，增加平滑性
			     .last_error = 0.0f,
			     .last_output = 0.0f,
			     .first_run = true};

// =============================================================================
// 全局变量 - 添加线程安全的yaw值
// =============================================================================
static uint8_t taskcut = 1;
static bool yaw_initialized = false;
static float initial_yaw = 0.0f;
static int pub_cnt = 0;
static int usb_cnt = 0;
static uint8_t log_cnt = 0;

// 改为使用回调更新的yaw值
static float current_yaw = 0.0f; // 当前Yaw角 (由回调更新)
static bool yaw_updated = false; // yaw值是否已更新标志

// =============================================================================
// 工具函数 - 修改为使用回调更新的yaw
// =============================================================================
float angle_diff(float target, float current)
{
	float diff = target - current;
	while (diff > 180.0f)
		diff -= 360.0f;
	while (diff < -180.0f)
		diff += 360.0f;
	return diff;
}

// 修改：使用回调更新的yaw值
float get_current_yaw(void)
{
	return current_yaw; // 使用回调中更新的值
}

// 检查yaw是否有效
bool is_yaw_ready(void)
{
	return yaw_updated;
}

// =============================================================================
// 平滑PD控制器（防跳变）
// =============================================================================
float smooth_yaw_control(simple_pd_t *pd, float target, float current)
{
	float error = angle_diff(target, current);

	if (pd->first_run) {
		pd->last_error = error;
		pd->last_output = 0.0f;
		pd->first_run = false;
		return 0.0f; // 第一次输出为0，避免跳变
	}

	// 简单PD计算
	float p_term = pd->kp * error;
	float d_term = pd->kd * (error - pd->last_error) / 0.01f; // 固定dt

	float raw_output = p_term + d_term;

	// 输出限幅
	if (raw_output > 0.4f)
		raw_output = 0.4f; // 降低最大输出
	if (raw_output < -0.4f)
		raw_output = -0.4f;

	// 输出平滑滤波，防止跳变
	float alpha = 0.7f; // 平滑系数
	float output = alpha * raw_output + (1.0f - alpha) * pd->last_output;

	// 更新历史值
	pd->last_error = error;
	pd->last_output = output;

	return output;
}

void reset_yaw_control(simple_pd_t *pd)
{
	pd->first_run = true;
	pd->last_error = 0.0f;
	pd->last_output = 0.0f;
}

// =============================================================================
// 快速角度修正函数 - 分层修正策略
// =============================================================================
void correct_angle_precise(float target_yaw, uint32_t max_time_ms)
{
	LOG_INF("=== Fast angle correction to %.1f ===", (double)target_yaw);

	// 等待yaw值有效
	uint32_t wait_start = k_uptime_get_32();
	while (!is_yaw_ready() && (k_uptime_get_32() - wait_start) < 1000) {
		LOG_WRN("Waiting for yaw data...");
		k_msleep(50);
	}

	if (!is_yaw_ready()) {
		LOG_ERR("Yaw data not ready, skipping correction");
		return;
	}

	// 确保完全停止
	chassis_set_speed(chassis, 0, 0);
	chassis_set_gyro(chassis, 0);
	k_msleep(50); // 减少等待时间

	float initial_yaw = get_current_yaw();
	float initial_error = angle_diff(target_yaw, initial_yaw);
	LOG_INF("Initial error: %.2f degrees", (double)initial_error);

	// 快速跳出条件 - 扩大可接受范围
	if (fabsf(initial_error) <= 1.5f) {
		LOG_INF("Error within acceptable range (%.2f deg), skipping correction",
			(double)initial_error);
		return;
	}

	// 分层修正策略
	simple_pd_t fast_pd = {.kp = 0.8f,  // 更高的P增益用于快速修正
			       .kd = 0.05f, // 适中的D增益
			       .last_error = 0.0f,
			       .last_output = 0.0f,
			       .first_run = true};

	uint32_t start_time = k_uptime_get_32();
	uint32_t last_log_time = start_time;
	bool fast_mode = true;
	uint32_t stable_count = 0;

	while ((k_uptime_get_32() - start_time) < max_time_ms) {
		float current_yaw_val = get_current_yaw();
		float error = angle_diff(target_yaw, current_yaw_val);

		// 快速跳出条件 - 动态调整
		float acceptable_error = fast_mode ? 1.5f : 0.8f;

		if (fabsf(error) <= acceptable_error) {
			stable_count++;
			if (stable_count >= 3) { // 连续3次都在范围内才退出
				LOG_INF("Angle corrected quickly, final error: %.2f deg (stable %d "
					"times)",
					(double)error, stable_count);
				break;
			}
		} else {
			stable_count = 0; // 重置稳定计数
		}

		// 动态调整修正策略
		if (fabsf(error) > 5.0f && fast_mode) {
			// 大误差：快速修正模式
			fast_pd.kp = 1.0f;  // 高增益
			fast_pd.kd = 0.02f; // 低阻尼
		} else if (fabsf(error) <= 5.0f && fast_mode) {
			// 中误差：切换到精确模式
			fast_pd.kp = 0.6f;  // 中等增益
			fast_pd.kd = 0.04f; // 中等阻尼
			fast_mode = false;
			LOG_DBG("Switched to precision mode");
		} else if (!fast_mode) {
			// 小误差：精确修正模式
			fast_pd.kp = 0.4f;  // 低增益
			fast_pd.kd = 0.06f; // 高阻尼
		}

		// 确保底盘不移动，只修正角度
		chassis_set_speed(chassis, 0, 0);
		float correction = smooth_yaw_control(&fast_pd, target_yaw, current_yaw_val);

		// 输出限制 - 根据误差大小动态调整
		float max_output = fabsf(error) > 10.0f ? 0.6f : 0.4f;
		if (correction > max_output)
			correction = max_output;
		if (correction < -max_output)
			correction = -max_output;

		chassis_set_gyro(chassis, correction);

		// 减少日志频率，提高响应速度
		uint32_t current_time = k_uptime_get_32();
		if (current_time - last_log_time >= 300) { // 300ms打印一次
			LOG_DBG("Correcting... Error: %.2f deg, Correction: %.3f, Mode: %s",
				(double)error, (double)correction,
				fast_mode ? "Fast" : "Precision");
			last_log_time = current_time;
		}

		k_msleep(CONTROL_PERIOD_MS);
	}

	// 完全停止修正
	chassis_set_speed(chassis, 0, 0);
	chassis_set_gyro(chassis, 0);
	k_msleep(50); // 减少稳定时间

	// 最终检查
	float final_yaw = get_current_yaw();
	float final_error = angle_diff(target_yaw, final_yaw);

	if (fabsf(final_error) > 2.0f) {
		LOG_WRN("Fast correction incomplete, final error: %.2f deg", (double)final_error);
	} else {
		LOG_INF("Fast correction completed, final error: %.2f deg", (double)final_error);
	}
}

// =============================================================================
// 简化运动函数 - 移除内部的开始/结束修正，只保留运动中的微调
// =============================================================================
void chassis_move_smooth_with_correction(float x, float y, float target_yaw, uint32_t total_time_ms)
{
	LOG_INF("=== Movement Execution ===");
	LOG_INF("Target: (%.1f, %.1f), Yaw=%.1f, Time=%dms", (double)x, (double)y,
		(double)target_yaw, total_time_ms);

	// 检查yaw数据是否有效
	if (!is_yaw_ready()) {
		LOG_ERR("Yaw data not ready, cannot start movement");
		return;
	}

	// 计算运动参数
	uint32_t total_steps = total_time_ms / CONTROL_PERIOD_MS;
	uint32_t accel_steps = total_steps / 4;                         // 25% 加速
	uint32_t decel_steps = total_steps / 4;                         // 25% 减速
	uint32_t const_steps = total_steps - accel_steps - decel_steps; // 50% 匀速

	// 使用更激进的微调参数
	simple_pd_t micro_pd = {.kp = 0.08f, // 更小的微调增益
				.kd = 0.01f, // 很小的微调阻尼
				.last_error = 0.0f,
				.last_output = 0.0f,
				.first_run = true};

	LOG_DBG("Movement phase started");

	// 1. 加速阶段 - 微调
	for (uint32_t i = 0; i < accel_steps; i++) {
		float t = (float)i / (float)accel_steps;
		float speed_factor = 3.0f * t * t - 2.0f * t * t * t;

		// 更宽松的微调条件
		float current_yaw_val = get_current_yaw();
		float error = angle_diff(target_yaw, current_yaw_val);
		float micro_correction = 0;

		// 只有当误差超过5度时才进行微调
		if (fabsf(error) > 5.0f) {
			micro_correction =
				smooth_yaw_control(&micro_pd, target_yaw, current_yaw_val);
			// 严格限制微调幅度
			if (micro_correction > 0.1f)
				micro_correction = 0.1f;
			if (micro_correction < -0.1f)
				micro_correction = -0.1f;
		}

		chassis_set_speed(chassis, x * speed_factor, y * speed_factor);
		chassis_set_gyro(chassis, micro_correction);

		k_msleep(CONTROL_PERIOD_MS);
	}

	// 2. 匀速阶段 - 微调
	for (uint32_t i = 0; i < const_steps; i++) {
		float current_yaw_val = get_current_yaw();
		float error = angle_diff(target_yaw, current_yaw_val);
		float micro_correction = 0;

		if (fabsf(error) > 5.0f) {
			micro_correction =
				smooth_yaw_control(&micro_pd, target_yaw, current_yaw_val);
			if (micro_correction > 0.1f)
				micro_correction = 0.1f;
			if (micro_correction < -0.1f)
				micro_correction = -0.1f;
		}

		chassis_set_speed(chassis, x, y);
		chassis_set_gyro(chassis, micro_correction);

		k_msleep(CONTROL_PERIOD_MS);
	}

	// 3. 减速阶段 - 微调
	for (uint32_t i = 0; i < decel_steps; i++) {
		float t = (float)i / (float)decel_steps;
		float speed_factor = 1.0f - (3.0f * t * t - 2.0f * t * t * t);

		float current_yaw_val = get_current_yaw();
		float error = angle_diff(target_yaw, current_yaw_val);
		float micro_correction = 0;

		if (fabsf(error) > 5.0f) {
			micro_correction =
				smooth_yaw_control(&micro_pd, target_yaw, current_yaw_val);
			if (micro_correction > 0.1f)
				micro_correction = 0.1f;
			if (micro_correction < -0.1f)
				micro_correction = -0.1f;
		}

		chassis_set_speed(chassis, x * speed_factor, y * speed_factor);
		chassis_set_gyro(chassis, micro_correction);

		k_msleep(CONTROL_PERIOD_MS);
	}

	// 4. 快速停止
	for (int i = 0; i < 8; i++) {
		float speed_factor = (8 - i) / 8.0f * 0.15f;
		chassis_set_speed(chassis, x * speed_factor, y * speed_factor);
		chassis_set_gyro(chassis, 0); // 停止过程中不修正角度
		k_msleep(20);
	}

	// 5. 完全停止
	chassis_set_speed(chassis, 0, 0);
	chassis_set_gyro(chassis, 0);
	k_msleep(50);

	float final_yaw = get_current_yaw();
	float final_error = angle_diff(target_yaw, final_yaw);
	LOG_INF("=== Movement Completed ===");
	LOG_INF("Movement error: %.2f deg", (double)final_error);
}

// =============================================================================
// 优化的任务执行函数 - 确保每个task开始和结束都有角度修正
// =============================================================================
void execute_task_unified(void)
{
	if (!is_yaw_ready()) {
		LOG_ERR("Yaw data not ready, cannot execute task");
		return;
	}

	// 记录初始Yaw角
	if (!yaw_initialized) {
		k_msleep(100);
		initial_yaw = get_current_yaw();
		yaw_initialized = true;
		LOG_INF("=== Fast Task Sequence Started ===");
		LOG_INF("Initial Yaw: %.1f degrees", (double)initial_yaw);

		// 初始快速校正
		LOG_INF("=== Initial System Calibration ===");
		correct_angle_precise(initial_yaw, 800);
		k_msleep(150);
	}

	switch (taskcut) {
	case 1: // 左上
		LOG_INF("========================================");
		LOG_INF("=== Task 1: Left-Up Movement ===");
		LOG_INF("========================================");

		// Task 1 开始前角度修正
		LOG_INF("--- Task 1 Start: Angle Correction ---");
		correct_angle_precise(initial_yaw, 600);
		k_msleep(100);

		yunqiu_simple();
		yunqiu_simple();

		// 运动执行
		chassis_move_smooth_with_correction(0.8, -0.8, initial_yaw, 1200);

		// Task 1 结束后角度修正
		LOG_INF("--- Task 1 End: Angle Correction ---");
		correct_angle_precise(initial_yaw, 600);
		k_msleep(100);

		yunqiu_simple();
		yunqiu_simple();
		LOG_INF("=== Task 1 Completed ===");
		break;

	case 2: // 左
		LOG_INF("========================================");
		LOG_INF("=== Task 2: Left Movement ===");
		LOG_INF("========================================");

		// Task 2 开始前角度修正
		LOG_INF("--- Task 2 Start: Angle Correction ---");
		correct_angle_precise(initial_yaw, 600);
		k_msleep(100);

		// 运动执行
		chassis_move_smooth_with_correction(1.0, 0, initial_yaw, 1700);

		// Task 2 结束后角度修正
		LOG_INF("--- Task 2 End: Angle Correction ---");
		correct_angle_precise(initial_yaw, 600);
		k_msleep(100);

		yunqiu_simple();
		yunqiu_simple();
		LOG_INF("=== Task 2 Completed ===");
		break;

	case 3: // 下
		LOG_INF("========================================");
		LOG_INF("=== Task 3: Down Movement ===");
		LOG_INF("========================================");

		// Task 3 开始前角度修正
		LOG_INF("--- Task 3 Start: Angle Correction ---");
		correct_angle_precise(initial_yaw, 600);
		k_msleep(100);

		// 运动执行
		chassis_move_smooth_with_correction(0, 1.0, initial_yaw, 2200);

		// Task 3 结束后角度修正
		LOG_INF("--- Task 3 End: Angle Correction ---");
		correct_angle_precise(initial_yaw, 600);
		k_msleep(100);

		yunqiu_simple();
		yunqiu_simple();
		LOG_INF("=== Task 3 Completed ===");
		break;

	case 4: // 下
		LOG_INF("========================================");
		LOG_INF("=== Task 4: Down Again Movement ===");
		LOG_INF("========================================");

		// Task 4 开始前角度修正
		LOG_INF("--- Task 4 Start: Angle Correction ---");
		correct_angle_precise(initial_yaw, 600);
		k_msleep(100);

		// 运动执行
		chassis_move_smooth_with_correction(0, 1.0, initial_yaw, 2200);

		// Task 4 结束后角度修正
		LOG_INF("--- Task 4 End: Angle Correction ---");
		correct_angle_precise(initial_yaw, 600);
		k_msleep(100);

		yunqiu_simple();
		yunqiu_simple();
		LOG_INF("=== Task 4 Completed ===");
		break;

	case 5: // 右
		LOG_INF("========================================");
		LOG_INF("=== Task 5: Right Movement ===");
		LOG_INF("========================================");

		// Task 5 开始前角度修正
		LOG_INF("--- Task 5 Start: Angle Correction ---");
		correct_angle_precise(initial_yaw, 600);
		k_msleep(100);

		// 运动执行
		chassis_move_smooth_with_correction(-1.0, 0, initial_yaw, 1700);

		// Task 5 结束后角度修正
		LOG_INF("--- Task 5 End: Angle Correction ---");
		correct_angle_precise(initial_yaw, 600);
		k_msleep(100);

		yunqiu_simple();
		yunqiu_simple();
		LOG_INF("=== Task 5 Completed ===");
		break;

	case 6: // 右上
		LOG_INF("========================================");
		LOG_INF("=== Task 6: Right-Up Movement ===");
		LOG_INF("========================================");

		// Task 6 开始前角度修正
		LOG_INF("--- Task 6 Start: Angle Correction ---");
		correct_angle_precise(initial_yaw, 600);
		k_msleep(100);

		// 运动执行
		chassis_move_smooth_with_correction(-0.8, -0.8, initial_yaw, 1200);

		// Task 6 结束后角度修正
		LOG_INF("--- Task 6 End: Angle Correction ---");
		correct_angle_precise(initial_yaw, 600);
		k_msleep(100);

		yunqiu_simple();
		yunqiu_simple();

		// 所有任务完成后的最终角度修正
		LOG_INF("=== Final System Calibration ===");
		correct_angle_precise(initial_yaw, 1000); // 最终修正时间更长

		LOG_INF("========================================");
		LOG_INF("=== ALL TASKS COMPLETED SUCCESSFULLY! ===");
		LOG_INF("========================================");

		taskcut = 0;
		yaw_initialized = false;
		break;

	default:
		LOG_WRN("Invalid task: %d", taskcut);
		break;
	}
}

// =============================================================================
// 简化的机械臂操作
// =============================================================================
void yunqiu_simple(void)
{
	LOG_INF("Yunqiu operation...");
	k_msleep(300); // 减少等待时间
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
	k_msleep(300); // 减少等待时间
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
	motor_set_torque(yq1, -10);
	motor_set_torque(yq2, 10);
	k_msleep(1000);
	motor_set_speed(yq1, 0);
	motor_set_speed(yq3, 0);
	motor_set_speed(yq2, 0);
}

// =============================================================================
// 简化的回调函数
// =============================================================================
void console_feedback(void *arg1, void *arg2, void *arg3)
{
	float angvel = 0;
	int cnt = 0;
	bool zeroed = false;

	while (1) {
		k_msleep(4);

#ifdef CHASSIS_SRC_SBUS
		angvel = sbus_get_percent(sbus, 0);
		float X = sbus_get_percent(sbus, 3);
		float Y = sbus_get_percent(sbus, 1);

		// 死区处理
		float linear_magnitude = sqrtf(X * X + Y * Y);
		bool in_deadzone = (linear_magnitude < 0.06f) && (fabsf(angvel) < 0.06f);

		if (in_deadzone) {
			// if (!zeroed) {
				// chassis_set_speed(chassis, 0, 0);
				// chassis_set_gyro(chassis, 0);
				chassis_set_static(chassis, true);
				zeroed = true;
			// }
		} else {
			// if (zeroed) {
			// 	chassis_set_static(chassis, false);
			// 	zeroed = false;
			// }

			chassis_set_speed(chassis, -X * 2.0f, -Y * 2.0f);
			chassis_set_gyro(chassis, angvel * 2.0f);
		}

		if (cnt++ % 2000 == 0) {
			LOG_INF("Manual: X=%.1f Y=%.1f Gyro=%.1f", (double)X, (double)Y,
				(double)angvel);
		}
#endif
	}
}

void Sensor_update_cb(QEKF_INS_t *QEKF)
{
	struct pos_data pos = {0};
	pos.Yaw = -QEKF->Yaw;
	pos.accel[0] = QEKF->Accel[X];
	pos.accel[1] = QEKF->Accel[Y];
	pos.accel[2] = QEKF->Accel[Z];

	// 更新全局yaw值 (线程安全)
	current_yaw = QEKF->Yaw;
	yaw_updated = true; // 标记yaw已更新

	pub_cnt++;
	// chassis_update_sensor(chassis, &pos);

	// if (pub_cnt % 1000 == 0) {
	// 	LOG_DBG("IMU Update: Yaw=%.1f Roll=%.1f Pitch=%.1f", (double)QEKF->Yaw,
	// 		(double)QEKF->Roll, (double)QEKF->Pitch);
	// }
}



// =============================================================================
// 线程和协议定义
// =============================================================================
K_THREAD_DEFINE(feedback_thread, 4096, console_feedback, NULL, NULL, NULL, -1, 0, 100);
DUAL_PROPOSE_PROTOCOL_DEFINE(dual_protocol);
ARES_BULK_INTERFACE_DEFINE(usb_bulk_interface);

// =============================================================================
// 简化的主函数
// =============================================================================
int main(void)
{
	k_msleep(100); // 减少初始化等待时间
	// 初始化
	// ares_bind_interface(&usb_bulk_interface, &dual_protocol);
	// dual_func_add(&dual_protocol, 0x1, (dual_trans_func_t)vel_func_cb);

	chassis_set_enabled(chassis, false);
	IMU_Sensor_trig_init(accel_dev, gyro_dev);
	k_msleep(100); // 减少等待时间
	IMU_Sensor_set_update_cb(Sensor_update_cb);
	chassis_set_enabled(chassis, true);
	chassis_set_gyro(chassis, 0);

	LOG_INF("System initializing...");
	k_msleep(350);

	// 等待yaw数据有效
	// LOG_INF("Waiting for IMU data...");
	// uint32_t wait_start = k_uptime_get_32();
	// while (!is_yaw_ready() && (k_uptime_get_32() - wait_start) < 5000) {
	// 	k_msleep(100);
	// }

	// if (is_yaw_ready()) {
	// 	LOG_INF("IMU data ready, current yaw: %.1f", (double)get_current_yaw());
	// } else {
	// 	LOG_ERR("IMU data timeout, some functions may not work properly");
	// }

	// 状态标志
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
		if (sbus_vals[0] > 0.5f && !sbus_states[0]) {
			takein_up_simple();
			sbus_states[0] = true;
		} else if (sbus_vals[0] <= 0.5f) {
			sbus_states[0] = false;
		}

		if (sbus_vals[1] > 0.5f && !sbus_states[1]) {
			yunqiu_simple();
			yunqiu_simple();
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

		// 使用新的统一任务执行函数
		if (sbus_vals[3] > 0.5f && !sbus_states[3]) {
			// execute_task_unified(); // 使用修改后的统一函数
			taskcut++;
			sbus_states[3] = true;
		} else if (sbus_vals[3] <= 0.5f) {
			sbus_states[3] = false;
		}

		if (sbus_vals[4] > 0.5f && !sbus_states[4]) {
			takein_down_simple();
			sbus_states[4] = true;
		} else if (sbus_vals[4] <= 0.5f) {
			sbus_states[4] = false;
		}

		k_msleep(CONTROL_PERIOD_MS);

		// 简化日志
		if (++log_cnt % 2000 == 0) {
			LOG_DBG("SBUS: [%.1f %.1f %.1f %.1f %.1f] Yaw: %.1f", (double)sbus_vals[0],
				(double)sbus_vals[1], (double)sbus_vals[2], (double)sbus_vals[3],
				(double)sbus_vals[4], (double)get_current_yaw());
		}
	}

	return 0;
}
