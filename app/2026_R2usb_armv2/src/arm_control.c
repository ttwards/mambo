#include "arm_control.h"

#include <errno.h>
#include <math.h>
#include <stdbool.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/motor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(arm_control, LOG_LEVEL_DBG);

#define ARM_JOINT1_NODE DT_NODELABEL(arm_rs_motor1)
#define ARM_JOINT2_NODE DT_NODELABEL(arm_dm_motor2)
#define ARM_JOINT3_NODE DT_NODELABEL(arm_rs_motor3)
#define ARM_JOINT4_NODE DT_NODELABEL(arm_rs_motor4)
#define EMVALVE1_NODE DT_NODELABEL(emvalve1)
#define EMVALVE2_NODE DT_NODELABEL(emvalve2)

#define M1_MIN_DEG 0.0f
#define M1_MAX_DEG 180.0f
#define M2_MIN_DEG (-135.0f)
#define M2_MAX_DEG 0.0f
#define M3_MIN_DEG (-127.0f)
#define M3_MAX_DEG 153.0f
#define M4_MIN_DEG (-180.0f)
#define M4_MAX_DEG 32.7f

#define POSE_INTERVAL_MS 100
#define TRAJ_DT_MS 20
#define TRAJ_MIN_SEGMENT_MS 900
#define TRAJ_DEFAULT_SPEED_DPS 40.0f
#define TRAJ_MIN_SPEED_DPS 1.0f

static const struct device *motor1 = DEVICE_DT_GET(ARM_JOINT1_NODE);
static const struct device *motor2 = DEVICE_DT_GET(ARM_JOINT2_NODE);
static const struct device *motor3 = DEVICE_DT_GET(ARM_JOINT3_NODE);
static const struct device *motor4 = DEVICE_DT_GET(ARM_JOINT4_NODE);

static const struct gpio_dt_spec emvalve1 = GPIO_DT_SPEC_GET(EMVALVE1_NODE, gpios);
static const struct gpio_dt_spec emvalve2 = GPIO_DT_SPEC_GET(EMVALVE2_NODE, gpios);

static bool arm_feedback_enabled;
static float m1_fb;
static float m2_fb;
static float m3_fb;
static float m4_fb;

static int check_device_ready(const struct device *dev, const char *name)
{
	if (dev == NULL || !device_is_ready(dev)) {
		LOG_ERR("device not ready: %s", name);
		return -ENODEV;
	}

	return 0;
}

static float clamp_float(float val, float min, float max)
{
	if (val < min) {
		return min;
	}
	if (val > max) {
		return max;
	}
	return val;
}

static float max_pose_diff(const motor_angles_t *start, const motor_angles_t *end)
{
	float diff1 = fabsf(end->motor1_angle - start->motor1_angle);
	float diff2 = fabsf(end->motor2_angle - start->motor2_angle);
	float diff3 = fabsf(end->motor3_angle - start->motor3_angle);
	float diff4 = fabsf(end->motor4_angle - start->motor4_angle);
	float max_diff = diff1;

	if (diff2 > max_diff) {
		max_diff = diff2;
	}
	if (diff3 > max_diff) {
		max_diff = diff3;
	}
	if (diff4 > max_diff) {
		max_diff = diff4;
	}

	return max_diff;
}

static motor_angles_t clamp_pose(const motor_angles_t *pose)
{
	motor_angles_t out = *pose;

	out.motor1_angle = clamp_float(out.motor1_angle, M1_MIN_DEG, M1_MAX_DEG);
	out.motor2_angle = clamp_float(out.motor2_angle, M2_MIN_DEG, M2_MAX_DEG);
	out.motor3_angle = clamp_float(out.motor3_angle, M3_MIN_DEG, M3_MAX_DEG);
	out.motor4_angle = clamp_float(out.motor4_angle, M4_MIN_DEG, M4_MAX_DEG);

	return out;
}

static void warn_pose_limits(const motor_angles_t *pose, size_t index)
{
	if (pose->motor1_angle < M1_MIN_DEG || pose->motor1_angle > M1_MAX_DEG) {
		LOG_WRN("Path %u: motor1 %.1f out of range [%.1f, %.1f]",
			(unsigned int)index, (double)pose->motor1_angle,
			(double)M1_MIN_DEG, (double)M1_MAX_DEG);
	}
	if (pose->motor2_angle < M2_MIN_DEG || pose->motor2_angle > M2_MAX_DEG) {
		LOG_WRN("Path %u: motor2 %.1f out of range [%.1f, %.1f]",
			(unsigned int)index, (double)pose->motor2_angle,
			(double)M2_MIN_DEG, (double)M2_MAX_DEG);
	}
	if (pose->motor3_angle < M3_MIN_DEG || pose->motor3_angle > M3_MAX_DEG) {
		LOG_WRN("Path %u: motor3 %.1f out of range [%.1f, %.1f]",
			(unsigned int)index, (double)pose->motor3_angle,
			(double)M3_MIN_DEG, (double)M3_MAX_DEG);
	}
	if (pose->motor4_angle < M4_MIN_DEG || pose->motor4_angle > M4_MAX_DEG) {
		LOG_WRN("Path %u: motor4 %.1f out of range [%.1f, %.1f]",
			(unsigned int)index, (double)pose->motor4_angle,
			(double)M4_MIN_DEG, (double)M4_MAX_DEG);
	}
}

static void set_mit_pose(const motor_angles_t *pose, const motor_angles_t *speed)
{
	motor_set_mit(motor1, speed->motor1_angle, pose->motor1_angle, 0.0f);
	motor_set_mit(motor2, speed->motor2_angle, pose->motor2_angle, 0.0f);
	motor_set_mit(motor3, speed->motor3_angle, pose->motor3_angle, 0.0f);
	motor_set_mit(motor4, speed->motor4_angle, pose->motor4_angle, 0.0f);
}

static void execute_path_segment(const motor_angles_t *start, const motor_angles_t *end,
				 float speed_dps)
{
	motor_angles_t start_clamped = clamp_pose(start);
	motor_angles_t end_clamped = clamp_pose(end);
	float max_diff = max_pose_diff(&start_clamped, &end_clamped);
	const motor_angles_t zero_speed = {0.0f, 0.0f, 0.0f, 0.0f};

	if (max_diff < 0.001f) {
		set_mit_pose(&end_clamped, &zero_speed);
		return;
	}

	float segment_ms = (max_diff / speed_dps) * 1000.0f;

	if (segment_ms < TRAJ_MIN_SEGMENT_MS) {
		segment_ms = TRAJ_MIN_SEGMENT_MS;
	}

	size_t steps = (size_t)ceilf(segment_ms / (float)TRAJ_DT_MS);
	if (steps < 1) {
		steps = 1;
	}

	float segment_s = ((float)steps * (float)TRAJ_DT_MS) / 1000.0f;
	float inv_steps = 1.0f / (float)steps;
	float inv_segment_s = 1.0f / segment_s;
	motor_angles_t delta = {
		.motor1_angle = end_clamped.motor1_angle - start_clamped.motor1_angle,
		.motor2_angle = end_clamped.motor2_angle - start_clamped.motor2_angle,
		.motor3_angle = end_clamped.motor3_angle - start_clamped.motor3_angle,
		.motor4_angle = end_clamped.motor4_angle - start_clamped.motor4_angle,
	};
	motor_angles_t delta_speed = {
		.motor1_angle = delta.motor1_angle * inv_segment_s,
		.motor2_angle = delta.motor2_angle * inv_segment_s,
		.motor3_angle = delta.motor3_angle * inv_segment_s,
		.motor4_angle = delta.motor4_angle * inv_segment_s,
	};

	for (size_t i = 0; i <= steps; i++) {
		float r = (float)i * inv_steps;
		float one_minus_r = 1.0f - r;
		float r2 = r * r;
		float r3 = r2 * r;
		float s = r3 * (10.0f + r * (-15.0f + 6.0f * r));
		float sd = 30.0f * r2 * one_minus_r * one_minus_r;
		motor_angles_t pose = {
			.motor1_angle = start_clamped.motor1_angle + delta.motor1_angle * s,
			.motor2_angle = start_clamped.motor2_angle + delta.motor2_angle * s,
			.motor3_angle = start_clamped.motor3_angle + delta.motor3_angle * s,
			.motor4_angle = start_clamped.motor4_angle + delta.motor4_angle * s,
		};
		motor_angles_t speed = {
			.motor1_angle = delta_speed.motor1_angle * sd,
			.motor2_angle = delta_speed.motor2_angle * sd,
			.motor3_angle = delta_speed.motor3_angle * sd,
			.motor4_angle = delta_speed.motor4_angle * sd,
		};

		set_mit_pose(&pose, &speed);
		if (i < steps) {
			k_msleep(TRAJ_DT_MS);
		}
	}
}

void execute_path(const motor_angles_t *poses, size_t count, float speed_dps)
{
	if (poses == NULL || count < 2) {
		return;
	}

	if (speed_dps < TRAJ_MIN_SPEED_DPS) {
		speed_dps = TRAJ_DEFAULT_SPEED_DPS;
	}

	for (size_t i = 0; i < count; i++) {
		warn_pose_limits(&poses[i], i);
	}

	for (size_t i = 0; i + 1 < count; i++) {
		execute_path_segment(&poses[i], &poses[i + 1], speed_dps);
	}
}

void execute_pose_sequence(const motor_angles_t *poses, size_t count)
{
	for (size_t i = 0; i < count; i++) {
		motor_angles_t pose = clamp_pose(&poses[i]);

		warn_pose_limits(&poses[i], i);
		motor_set_mit(motor1, 0.0f, pose.motor1_angle, 0.0f);
		motor_set_mit(motor2, 0.0f, pose.motor2_angle, 0.0f);
		motor_set_mit(motor3, 0.0f, pose.motor3_angle, 0.0f);
		motor_set_mit(motor4, 0.0f, pose.motor4_angle, 0.0f);
		k_msleep(POSE_INTERVAL_MS);
	}
}

void xipan_on(void)
{
	gpio_pin_set_dt(&emvalve1, 1);
	gpio_pin_set_dt(&emvalve2, 1);
	LOG_INF("Suction ON");
}

void xipan_off(void)
{
	gpio_pin_set_dt(&emvalve1, 0);
	gpio_pin_set_dt(&emvalve2, 0);
	LOG_INF("Suction OFF");
}

static int xipan_init(void)
{
	int ret;

	if (!gpio_is_ready_dt(&emvalve1) || !gpio_is_ready_dt(&emvalve2)) {
		LOG_ERR("suction GPIO not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&emvalve1, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		return ret;
	}

	ret = gpio_pin_configure_dt(&emvalve2, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		return ret;
	}

	LOG_INF("Suction GPIO initialized (PE7/PE8, active-low)");
	return 0;
}

static int arm_motor_enable_mit(const struct device *motor, const char *name)
{
	int ret;

	ret = check_device_ready(motor, name);
	if (ret < 0) {
		return ret;
	}

	motor_control(motor, ENABLE_MOTOR);
	k_msleep(100);

	ret = motor_set_mit(motor, 0.0f, motor_get_angle(motor), 0.0f);
	if (ret < 0) {
		LOG_ERR("failed to set %s MIT target: %d", name, ret);
		return ret;
	}
	k_msleep(100);

	return 0;
}

static void arm_feedback_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	while (1) {
		if (arm_feedback_enabled) {
			m1_fb = motor_get_angle(motor1);
			m2_fb = motor_get_angle(motor2);
			m3_fb = motor_get_angle(motor3);
			m4_fb = motor_get_angle(motor4);
		}

		k_msleep(10);
	}
}

K_THREAD_DEFINE(arm_feedback_tid, 4096, arm_feedback_thread, NULL, NULL, NULL, 2, 0, 100);

int arm_control_prepare(void)
{
	int ret;

	ret = arm_motor_enable_mit(motor1, "arm_rs_motor1");
	if (ret < 0) {
		return ret;
	}
	ret = arm_motor_enable_mit(motor2, "arm_dm_motor2");
	if (ret < 0) {
		return ret;
	}
	ret = arm_motor_enable_mit(motor3, "arm_rs_motor3");
	if (ret < 0) {
		return ret;
	}
	ret = arm_motor_enable_mit(motor4, "arm_rs_motor4");
	if (ret < 0) {
		return ret;
	}

	ret = xipan_init();
	if (ret < 0) {
		return ret;
	}

	k_msleep(500);
	execute_pose_sequence(init, init_count);
	arm_feedback_enabled = true;

	LOG_INF("arm control ready");
	return 0;
}
