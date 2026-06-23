/*
 * 机械臂区域控制板 — 串口通信 + 三路继电器控制 + 位姿序列回放
 *
 * 通过 ARES UART 双协议与上位机进行双向通信 (2 float),
 * 同时控制 PE9/PE11/PE13 三路继电器和 3 路 CAN 电机。
 *
 * 接收 (上位机 -> 本板):
 *   [0] 动作类型: 0=init_to_1, 1=init_to_2, 2=one_to_up, 3=two_to_up, 4=up_to_put, 5=关继电器
 *   [1] 保留
 *
 * 发送 (本板 -> 上位机):
 *   [0] 动作完成标志 (magic number, 抗干扰)
 *   [1] 保留
 */

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/motor.h>
#include <zephyr/logging/log.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include <ares/interface/uart/uart.h>
#include <ares/protocol/dual/dual_protocol.h>
#include <ares/ares_comm.h>

LOG_MODULE_REGISTER(relay_comm, LOG_LEVEL_DBG);

/* ==================== 位姿数据结构 ==================== */

typedef struct {
	float joint1;
	float joint2;
	float joint3;
	float joint4;
} arm_pose_t;

#define POSE_INTERVAL_MS  100

/* ==================== 6 动作模式 ==================== */

typedef enum {
	MODE_INIT_TO_1,    /* 0: 初始→位置1 */
	MODE_INIT_TO_2,    /* 1: 初始→位置2 */
	MODE_ONE_TO_UP,    /* 2: 位置1→举起 */
	MODE_TWO_TO_UP,    /* 3: 位置2→举起 */
	MODE_UP_TO_PUT,    /* 4: 举起→放置 */
	MODE_RELAY_OFF,    /* 5: 关继电器 PE9 */
} RobotMode;

/* ==================== 协议常量 ==================== */

/*
 * 动作完成标志: 使用不易被串口干扰误判的 magic number.
 * 通过 union 将 uint32 特征值映射为 float, 避免直接用 0.0/1.0 这类易被噪声命中的值.
 */
typedef union {
	float    f;
	uint32_t u;
} flag_val_t;

/* 0x4B0CD0F1 = "DONE-ish", 0x4E0DD0F1 = "NOTD-ish" */
#define FLAG_DONE     ((flag_val_t){ .u = 0x4B0CD0F1 }).f
#define FLAG_PENDING  ((flag_val_t){ .u = 0x4E0DD0F1 }).f

/* 继电器电平标志 */
#define RELAY_HIGH 1.0f
#define RELAY_LOW  0.0f

/* ==================== 电机设备 ==================== */

#define MOTOR1_NODE DT_NODELABEL(rs_motor1)
#define MOTOR2_NODE DT_NODELABEL(rs_motor2)
#define MOTOR3_NODE DT_NODELABEL(rs_motor3)

static const struct device *motor1 = DEVICE_DT_GET(MOTOR1_NODE);
static const struct device *motor2 = DEVICE_DT_GET(MOTOR2_NODE);
static const struct device *motor3 = DEVICE_DT_GET(MOTOR3_NODE);
static const struct device *dm_motor = DEVICE_DT_GET(DT_PATH(motor, dm_motor));

/* ==================== 继电器 ==================== */

/* PE9, PE11, PE13 */
static const struct gpio_dt_spec relay1 = GPIO_DT_SPEC_GET(DT_NODELABEL(relay_1), gpios);
static const struct gpio_dt_spec relay2 = GPIO_DT_SPEC_GET(DT_NODELABEL(relay_2), gpios);
static const struct gpio_dt_spec relay3 = GPIO_DT_SPEC_GET(DT_NODELABEL(relay_3), gpios);

static const struct gpio_dt_spec *relay_gpios[] = {&relay1, &relay2, &relay3};

static int relay_init(void)
{
	for (int i = 0; i < 3; i++) {
		if (!gpio_is_ready_dt(relay_gpios[i])) {
			LOG_ERR("继电器 %d GPIO 未就绪", i + 1);
			return -ENODEV;
		}
		gpio_pin_configure_dt(relay_gpios[i], GPIO_OUTPUT_INACTIVE);
		gpio_pin_set_dt(relay_gpios[i], 0);
	}
	LOG_INF("三路继电器初始化完成 (PE9/PE11/PE13)");
	return 0;
}

static void relay_set(int index, int on)
{
	if (index < 0 || index > 2) {
		return;
	}
	gpio_pin_set_dt(relay_gpios[index], on ? 1 : 0);
}

static int relay_get(int index)
{
	if (index < 0 || index > 2) {
		return 0;
	}
	return gpio_pin_get_dt(relay_gpios[index]);
}

/* ==================== 电机初始化 (按 2026_arm_death 方式) ==================== */

static int motor_init(void)
{
	if (!device_is_ready(motor1) || !device_is_ready(motor2) ||
	    !device_is_ready(motor3) || !device_is_ready(dm_motor)) {
		LOG_ERR("电机设备未就绪");
		return -ENODEV;
	}

	motor_control(motor1, ENABLE_MOTOR);
	k_msleep(100);
	motor_control(motor1, ENABLE_MOTOR);
	k_msleep(100);
	motor_set_mode(motor1, MIT);
	k_msleep(100);
	motor_control(motor1, SET_ZERO);
	k_msleep(100);

	motor_control(motor2, ENABLE_MOTOR);
	k_msleep(100);
	motor_control(motor2, ENABLE_MOTOR);
	k_msleep(100);
	motor_set_mode(motor2, MIT);
	k_msleep(100);
	motor_control(motor2, SET_ZERO);
	k_msleep(100);

	motor_control(motor3, ENABLE_MOTOR);
	k_msleep(100);
	motor_control(motor3, ENABLE_MOTOR);
	k_msleep(100);
	motor_set_mode(motor3, MIT);
	k_msleep(100);
	motor_control(motor3, SET_ZERO);
	k_msleep(100);

	motor_control(dm_motor, ENABLE_MOTOR);
	k_msleep(100);
	motor_set_mode(dm_motor, MIT);
	k_msleep(100);

	motor_set_mit(motor1, 0.0f, 0.0f, 0.0f);
	k_msleep(1000);

	LOG_INF("电机初始化完成 (MIT 模式, 已归零)");
	return 0;
}

/* ==================== 位姿序列 (来自 2026_arm_death) ==================== */

arm_pose_t init_to_1[] = {
	{0.0f,   0.00f,   0.00f,   0.00f},
	{0.0f,   2.27f,  -0.54f,   0.94f},
	{0.0f,   2.27f,  -0.56f,   1.88f},
	{0.0f,   2.38f,  -0.58f,   2.82f},
	{0.0f,   3.77f,  -1.88f,   3.76f},
	{0.0f,   6.34f,  -4.43f,   4.69f},
	{0.0f,   8.78f,  -6.87f,   5.63f},
	{0.0f,  12.76f, -10.65f,   6.57f},
	{0.0f,  16.23f, -13.97f,   7.51f},
	{0.0f,  18.58f, -15.97f,   8.45f},
	{0.0f,  22.78f, -20.12f,   9.39f},
	{0.0f,  25.86f, -23.42f,  10.33f},
	{0.0f,  26.01f, -24.91f,  11.27f},
	{0.0f,  25.99f, -27.95f,  12.21f},
	{0.0f,  24.32f, -30.58f,  13.15f},
	{0.0f,  21.37f, -30.89f,  14.08f},
	{0.0f,  18.12f, -31.11f,  15.02f},
	{0.0f,  14.80f, -31.33f,  15.96f},
	{0.0f,  11.64f, -33.27f,  16.90f},
	{0.0f,   7.88f, -34.34f,  17.84f},
	{0.0f,   2.58f, -35.18f,  18.78f},
	{0.0f,  -3.48f, -37.05f,  19.72f},
	{0.0f, -10.01f, -38.06f,  20.66f},
	{0.0f, -16.87f, -39.66f,  21.60f},
	{0.0f, -22.98f, -40.45f,  22.54f},
	{0.0f, -26.72f, -40.61f,  23.47f},
	{0.0f, -30.94f, -41.46f,  24.41f},
	{0.0f, -37.31f, -43.05f,  25.35f},
	{0.0f, -43.57f, -44.17f,  26.29f},
	{0.0f, -47.24f, -44.15f,  27.23f},
	{0.0f, -50.17f, -44.43f,  28.17f},
	{0.0f, -51.75f, -44.15f,  29.11f},
	{0.0f, -52.72f, -44.27f,  30.05f},
	{0.0f, -52.46f, -44.91f,  31.99f},
	{0.0f, -52.43f, -45.94f,  33.93f},
	{0.0f, -52.10f, -46.94f,  35.87f},
	{0.0f, -52.30f, -48.87f,  37.80f},
};

arm_pose_t init_to_2[] = {
	{0.0f,   0.30f,  -0.05f,  -2.50f},
	{0.0f,   1.46f,  -0.19f,  -2.51f},
	{0.0f,   3.03f,  -1.19f,  -2.81f},
	{0.0f,   5.23f,  -2.22f,  -3.11f},
	{0.0f,   8.16f,  -3.53f,  -3.41f},
	{0.0f,  10.50f,  -4.53f,  -3.41f},
	{0.0f,  12.69f,  -5.80f,  -3.40f},
	{0.0f,  14.33f,  -6.72f,  -3.41f},
	{0.0f,  15.87f,  -10.02f, -3.41f},
	{0.0f,  15.92f,  -11.28f, -5.03f},
	{0.0f,  16.06f,  -12.40f, -5.03f},
	{0.0f,  15.98f,  -13.44f, -6.98f},
	{0.0f,  14.11f,  -14.44f, -9.05f},
	{0.0f,  11.44f,  -15.17f, -9.41f},
	{0.0f,   7.40f,  -15.50f, -10.29f},
	{0.0f,   2.79f,  -16.59f, -13.62f},
	{0.0f,  -2.92f,  -17.14f, -15.69f},
	{0.0f,  -6.24f,  -17.74f, -15.69f},
	{0.0f,  -9.63f,  -18.47f, -15.69f},
	{0.0f, -12.96f,  -19.38f, -17.22f},
	{0.0f, -16.32f,  -20.07f, -19.10f},
	{0.0f, -19.67f,  -21.46f, -21.17f},
	{0.0f, -23.48f,  -22.23f, -24.55f},
	{0.0f, -27.90f,  -24.22f, -26.54f},
	{0.0f, -32.19f,  -25.30f, -28.34f},
	{0.0f, -36.51f,  -27.06f, -31.38f},
	{0.0f, -40.49f,  -28.61f, -33.06f},
	{0.0f, -44.34f,  -30.04f, -33.61f},
	{0.0f, -47.77f,  -31.30f, -34.76f},
	{0.0f, -51.14f,  -32.80f, -36.35f},
	{0.0f, -54.01f,  -34.37f, -38.14f},
	{0.0f, -56.98f,  -36.12f, -39.74f},
	{0.0f, -59.89f,  -38.06f, -40.84f},
	{0.0f, -62.59f,  -39.88f, -41.87f},
	{0.0f, -65.25f,  -41.80f, -42.74f},
	{0.0f, -67.16f,  -44.06f, -43.08f},
	{0.0f, -68.88f,  -46.02f, -44.22f},
	{0.0f, -70.38f,  -48.08f, -44.22f},
	{0.0f, -71.76f,  -50.06f, -45.28f},
	{0.0f, -73.20f,  -52.58f, -46.08f},
	{0.0f, -74.54f,  -54.96f, -46.40f},
	{0.0f, -75.85f,  -57.76f, -46.88f},
	{0.0f, -76.69f,  -59.75f, -47.93f},
	{0.0f, -76.72f,  -61.47f, -47.71f},
	{0.0f, -76.59f,  -62.68f, -47.79f},
	{0.0f, -76.59f,  -63.49f, -48.11f},
	{0.0f, -76.86f,  -64.30f, -49.98f},
	{0.0f, -76.88f,  -64.93f, -51.14f},
	{0.0f, -76.46f,  -65.66f, -54.19f},
};

arm_pose_t one_to_up[] = {
	{0.0f, -58.30f, -42.87f,  37.80f},
	{0.0f, -57.55f, -44.15f,  34.59f},
	{0.0f, -53.20f, -46.15f,  30.91f},
	{0.0f, -49.75f, -48.15f,  27.23f},
	{0.0f, -45.49f, -48.12f,  25.54f},
	{0.0f, -41.05f, -47.66f,  24.86f},
	{0.0f, -37.33f, -47.66f,  24.17f},
	{0.0f, -33.77f, -47.33f,  23.49f},
	{0.0f, -29.49f, -46.96f,  22.81f},
	{0.0f, -23.70f, -45.16f,  22.12f},
	{0.0f, -19.46f, -44.80f,  21.44f},
	{0.0f, -15.37f, -44.80f,  20.75f},
	{0.0f, -11.04f, -44.80f,  20.07f},
	{0.0f,  -7.15f, -44.80f,  19.39f},
	{0.0f,  -3.90f, -44.91f,  18.70f},
	{0.0f,   0.56f, -46.96f,  18.02f},
	{0.0f,   4.98f, -51.97f,  17.33f},
	{0.0f,   8.63f, -55.18f,  16.65f},
	{0.0f,  11.26f, -58.30f,  15.97f},
	{0.0f,  14.50f, -61.55f,  15.28f},
	{0.0f,  18.39f, -63.53f,  14.60f},
	{0.0f,  20.87f, -65.29f,  13.91f},
	{0.0f,  22.34f, -65.91f,  13.23f},
	{0.0f,  25.13f, -66.37f,  12.55f},
	{0.0f,  26.52f, -66.34f,  11.86f},
	{0.0f,  27.05f, -66.50f,  11.18f},
	{0.0f,  27.40f, -66.48f,  10.49f},
	{0.0f,  27.57f, -66.45f,   9.81f},
	{0.0f,  27.68f, -66.45f,   9.13f},
	{0.0f,  27.77f, -66.45f,   8.44f},
	{0.0f,  27.84f, -66.45f,   7.76f},
	{0.0f,  28.14f, -66.45f,   7.07f},
	{0.0f,  28.45f, -66.48f,   6.39f},
	{0.0f,  29.05f, -66.48f,   5.71f},
	{0.0f,  29.24f, -66.45f,   5.02f},
	{0.0f,  29.31f, -66.52f,   4.34f},
	{0.0f,  29.62f, -67.97f,   3.65f},
	{0.0f,  29.90f, -68.08f,   2.97f},
	{0.0f,  29.79f, -68.06f,   2.29f},
	{0.0f,  29.81f, -68.06f,   1.60f},
	{0.0f,  29.71f, -68.06f,   0.92f},
	{0.0f,  29.31f, -68.04f,   0.23f},
	{0.0f,  29.22f, -68.06f,  -0.45f},
	{0.0f,  29.20f, -68.06f,  -1.14f},
	{0.0f,  29.22f, -70.06f,  -1.82f},
	{0.0f,  28.30f, -73.14f,  -2.48f},
};

arm_pose_t two_to_up[] = {
	{0.0f, -76.46f, -65.66f, -54.19f},
	{0.0f, -76.46f, -63.66f, -53.18f},
	{0.0f, -76.46f, -62.40f, -52.16f},
	{0.0f, -76.08f, -60.36f, -51.15f},
	{0.0f, -72.15f, -59.38f, -50.13f},
	{0.0f, -66.92f, -59.27f, -49.12f},
	{0.0f, -62.34f, -59.25f, -48.11f},
	{0.0f, -57.22f, -59.25f, -47.09f},
	{0.0f, -51.86f, -59.27f, -46.08f},
	{0.0f, -46.10f, -59.27f, -45.06f},
	{0.0f, -40.01f, -59.27f, -44.05f},
	{0.0f, -34.34f, -59.27f, -43.04f},
	{0.0f, -27.81f, -59.09f, -42.02f},
	{0.0f, -24.06f, -59.33f, -41.01f},
	{0.0f, -18.94f, -59.71f, -39.99f},
	{0.0f, -10.47f, -59.88f, -38.98f},
	{0.0f,  -4.52f, -59.88f, -37.97f},
	{0.0f,  -1.40f, -59.90f, -36.95f},
	{0.0f,   3.07f, -60.01f, -35.94f},
	{0.0f,  10.98f, -63.09f, -34.92f},
	{0.0f,  15.86f, -65.58f, -33.91f},
	{0.0f,  17.66f, -67.96f, -32.89f},
	{0.0f,  20.54f, -69.25f, -31.88f},
	{0.0f,  24.39f, -71.22f, -30.87f},
	{0.0f,  25.35f, -73.22f, -29.85f},
	{0.0f,  25.46f, -73.22f, -28.84f},
	{0.0f,  26.12f, -73.25f, -27.82f},
	{0.0f,  26.83f, -73.22f, -26.81f},
	{0.0f,  27.02f, -73.18f, -25.80f},
	{0.0f,  27.00f, -73.14f, -24.78f},
	{0.0f,  27.05f, -73.18f, -23.77f},
	{0.0f,  27.05f, -73.14f, -22.75f},
	{0.0f,  27.20f, -73.16f, -21.74f},
	{0.0f,  27.42f, -73.16f, -20.72f},
	{0.0f,  27.46f, -73.18f, -19.71f},
	{0.0f,  27.44f, -73.14f, -18.70f},
	{0.0f,  27.55f, -73.16f, -17.68f},
	{0.0f,  27.55f, -73.16f, -16.67f},
	{0.0f,  27.68f, -73.18f, -15.65f},
	{0.0f,  27.81f, -73.18f, -14.64f},
	{0.0f,  28.14f, -73.20f, -13.63f},
	{0.0f,  28.32f, -73.18f, -12.61f},
	{0.0f,  28.30f, -73.16f, -11.60f},
	{0.0f,  28.30f, -73.18f, -10.58f},
	{0.0f,  28.32f, -73.18f,  -9.57f},
	{0.0f,  28.32f, -73.18f,  -8.55f},
	{0.0f,  28.30f, -73.16f,  -7.54f},
	{0.0f,  28.30f, -73.18f,  -6.53f},
	{0.0f,  28.30f, -73.16f,  -5.51f},
	{0.0f,  28.28f, -73.16f,  -4.50f},
	{0.0f,  28.30f, -73.14f,  -3.48f},
	{0.0f,  28.30f, -73.14f,  -2.48f},
};

arm_pose_t up_to_put[] = {
	{0.0f,  28.30f, -73.14f,  -2.48f},
	{0.0f,  26.31f, -73.97f,  -2.50f},
	{0.0f,  24.32f, -74.80f,  -2.52f},
	{0.0f,  22.33f, -75.62f,  -2.55f},
	{0.0f,  20.34f, -76.45f,  -2.57f},
	{0.0f,  18.35f, -77.28f,  -2.59f},
	{0.0f,  16.36f, -78.11f,  -2.61f},
	{0.0f,  14.37f, -78.93f,  -2.64f},
	{0.0f,  12.38f, -79.76f,  -2.66f},
	{0.0f,  10.39f, -80.59f,  -2.68f},
	{0.0f,   8.40f, -81.42f,  -2.70f},
	{0.0f,   6.41f, -82.24f,  -2.73f},
	{0.0f,   4.42f, -83.07f,  -2.75f},
	{0.0f,   2.43f, -83.90f,  -2.77f},
	{0.0f,   0.44f, -84.73f,  -2.79f},
	{0.0f,  -1.55f, -85.55f,  -2.82f},
	{0.0f,  -3.54f, -86.38f,  -2.84f},
	{0.0f,  -5.53f, -87.21f,  -2.86f},
	{0.0f,  -7.52f, -88.04f,  -2.88f},
	{0.0f,  -9.51f, -88.86f,  -2.91f},
	{0.0f, -11.50f, -89.69f,  -2.93f},
	{0.0f, -13.49f, -90.52f,  -2.95f},
	{0.0f, -15.48f, -91.35f,  -2.97f},
	{0.0f, -17.47f, -92.18f,  -3.00f},
	{0.0f, -19.46f, -93.01f,  -3.01f},
};

/* ==================== 位姿执行 ==================== */

static void execute_pose_sequence(const arm_pose_t *poses, size_t count)
{
	for (size_t i = 0; i < count; i++) {
		motor_set_mit(motor2, 0.0f, poses[i].joint2, 0.0f);
		motor_set_mit(motor3, 0.0f, poses[i].joint3, 0.0f);
		motor_set_mit(dm_motor, 0.0f, poses[i].joint4, 0.0f);
		k_msleep(POSE_INTERVAL_MS);
		/* 每 10 步 (~1s) 打印进度，首尾必打，避免刷屏 */
		if (i == 0 || i == count - 1 || i % 10 == 0) {
			LOG_INF("  位姿进度 %u/%u: J2=%.1f J3=%.1f J4=%.1f",
				(unsigned)i + 1, (unsigned)count,
				(double)poses[i].joint2, (double)poses[i].joint3,
				(double)poses[i].joint4);
		}
	}
}

/* ==================== 动作完成脉冲计数器 ==================== */

#define DONE_PULSE_FRAMES 10
static uint8_t done_pulse;

/* ==================== 轨迹执行线程同步 ==================== */

static RobotMode pending_mode;
static volatile bool trajectory_running;
K_SEM_DEFINE(trajectory_sem, 0, 1);

static void trajectory_thread_fn(void *arg1, void *arg2, void *arg3)
{
	while (1) {
		k_sem_take(&trajectory_sem, K_FOREVER);
		trajectory_running = true;

		switch (pending_mode) {
		case MODE_INIT_TO_1:
			LOG_INF(">>> 开始执行 [init_to_1: 初始→位置1] (%u步)",
				ARRAY_SIZE(init_to_1));
			relay_set(0, 1);
			execute_pose_sequence(init_to_1, ARRAY_SIZE(init_to_1));
			break;
		case MODE_INIT_TO_2:
			LOG_INF(">>> 开始执行 [init_to_2: 初始→位置2] (%u步)", ARRAY_SIZE(init_to_2));
			relay_set(0, 1);
			execute_pose_sequence(init_to_2, ARRAY_SIZE(init_to_2));
			break;
		case MODE_ONE_TO_UP:
			LOG_INF(">>> 开始执行 [one_to_up: 位置1→举起] (%u步)", ARRAY_SIZE(one_to_up));
			execute_pose_sequence(one_to_up, ARRAY_SIZE(one_to_up));
			break;
		case MODE_TWO_TO_UP:
			LOG_INF(">>> 开始执行 [two_to_up: 位置2→举起] (%u步)", ARRAY_SIZE(two_to_up));
			execute_pose_sequence(two_to_up, ARRAY_SIZE(two_to_up));
			break;
		case MODE_UP_TO_PUT:
			LOG_INF(">>> 开始执行 [up_to_put: 举起→放置] (%u步)", ARRAY_SIZE(up_to_put));
			execute_pose_sequence(up_to_put, ARRAY_SIZE(up_to_put));
			break;
		case MODE_RELAY_OFF:
			relay_set(0, 0);
			LOG_INF(">>> 关继电器 PE9");
			break;
		default:
			break;
		}

		LOG_INF("动作执行完毕。");
		done_pulse = DONE_PULSE_FRAMES;
		trajectory_running = false;
	}
}

K_THREAD_DEFINE(trajectory_tid, 4096, trajectory_thread_fn, NULL, NULL, NULL,
		2, 0, 0);

/* ==================== 串口通信 ==================== */

/* UART 设备 */
#define UART_NODE DT_NODELABEL(usart6)
static const struct device *uart_dev = DEVICE_DT_GET(UART_NODE);

ARES_UART_INTERFACE_DEFINE(uart_interface);
DUAL_PROPOSE_PROTOCOL_DEFINE(uart_protocol);

/* 双向数据通道 */
#define SYNC_ID_TX 0x0100 /* 本板 -> 上位机 */
#define SYNC_ID_RX 0x0200 /* 上位机 -> 本板 */

/* 2 个 float = 8 字节 */
#define DATA_LEN 8
static float tx_data[2]; /* [完成标志, 保留] */
static float rx_data[2]; /* [动作类型, 子模式] */

static sync_table_t *tx_pack;
static sync_table_t *rx_pack;

/* 统计 */
static uint32_t rx_cnt;
static uint32_t tx_cnt;
static uint32_t tx_fail;

/* 新数据到达标志 */
static bool rx_new_data;

static void rx_cb(int status)
{
	if (status == SYNC_PACK_STATUS_DONE) {
		rx_cnt++;
		rx_new_data = true;
		/* 确认链路存活: 前 5 包 + 之后每 500 包打印一次 */
		if (rx_cnt <= 5 || rx_cnt % 500 == 0) {
			LOG_INF("[RX] rx=%u raw=[%02x %02x %02x %02x %02x %02x %02x %02x]",
				rx_cnt,
				((uint8_t *)rx_data)[0], ((uint8_t *)rx_data)[1],
				((uint8_t *)rx_data)[2], ((uint8_t *)rx_data)[3],
				((uint8_t *)rx_data)[4], ((uint8_t *)rx_data)[5],
				((uint8_t *)rx_data)[6], ((uint8_t *)rx_data)[7]);
		}
	} else if (status != 0) {
		/* 非 DONE 非 0 说明协议层有异常，但不刷屏 */
		static uint32_t rx_err_cnt;
		static uint32_t rx_err_last_log;
		rx_err_cnt++;
		if (rx_err_last_log == 0 ||
		    rx_err_cnt - rx_err_last_log >= 500) {
			LOG_WRN("[RX_ERR] status=%d err_cnt=%u", status, rx_err_cnt);
			rx_err_last_log = rx_err_cnt;
		}
	}
}

static int uart_comm_init(void)
{
	if (!device_is_ready(uart_dev)) {
		LOG_ERR("UART 设备未就绪");
		return -ENODEV;
	}

	ares_uart_init_dev(&uart_interface, uart_dev);
	int ret = ares_bind_interface(&uart_interface, &uart_protocol);
	if (ret < 0) {
		LOG_ERR("UART 绑定失败: %d", ret);
		return ret;
	}

	tx_pack = dual_sync_add(&uart_protocol, SYNC_ID_TX, (uint8_t *)tx_data,
				DATA_LEN, NULL);
	rx_pack = dual_sync_add(&uart_protocol, SYNC_ID_RX, (uint8_t *)rx_data,
				DATA_LEN, rx_cb);

	LOG_INF("串口通信初始化完成 (TX:0x%04X RX:0x%04X len=%d)",
		SYNC_ID_TX, SYNC_ID_RX, DATA_LEN);
	return 0;
}

static int uart_tx_flush(void)
{
	return dual_sync_flush(&uart_protocol, tx_pack);
}

/* ==================== 指令处理 ==================== */

static void process_rx_command(void)
{
	int cmd = (int)rx_data[0];
	static const char *mode_names[] = {
		[MODE_INIT_TO_1] = "init_to_1",
		[MODE_INIT_TO_2] = "init_to_2",
		[MODE_ONE_TO_UP] = "one_to_up",
		[MODE_TWO_TO_UP] = "two_to_up",
		[MODE_UP_TO_PUT] = "up_to_put",
		[MODE_RELAY_OFF] = "relay_off",
	};

	switch (cmd) {
	case MODE_INIT_TO_1:
	case MODE_INIT_TO_2:
	case MODE_ONE_TO_UP:
	case MODE_TWO_TO_UP:
	case MODE_UP_TO_PUT:
		LOG_INF("[CMD] rx=%u 动作=%s(%d) sub=%.0f raw=[%02x %02x %02x %02x %02x %02x %02x %02x]",
			rx_cnt, mode_names[cmd], cmd, (double)rx_data[1],
			((uint8_t *)rx_data)[0], ((uint8_t *)rx_data)[1],
			((uint8_t *)rx_data)[2], ((uint8_t *)rx_data)[3],
			((uint8_t *)rx_data)[4], ((uint8_t *)rx_data)[5],
			((uint8_t *)rx_data)[6], ((uint8_t *)rx_data)[7]);
		pending_mode = (RobotMode)cmd;
		trajectory_running = true;
		k_sem_give(&trajectory_sem);
		break;

	case MODE_RELAY_OFF:
		LOG_INF("[CMD] rx=%u 动作=%s(%d) raw=[%02x %02x %02x %02x %02x %02x %02x %02x]",
			rx_cnt, mode_names[cmd], cmd,
			((uint8_t *)rx_data)[0], ((uint8_t *)rx_data)[1],
			((uint8_t *)rx_data)[2], ((uint8_t *)rx_data)[3],
			((uint8_t *)rx_data)[4], ((uint8_t *)rx_data)[5],
			((uint8_t *)rx_data)[6], ((uint8_t *)rx_data)[7]);
		pending_mode = MODE_RELAY_OFF;
		k_sem_give(&trajectory_sem);
		break;

	default:
		LOG_WRN("[CMD] rx=%u 未知动作=%d sub=%.0f raw=[%02x %02x %02x %02x %02x %02x %02x %02x]",
			rx_cnt, cmd, (double)rx_data[1],
			((uint8_t *)rx_data)[0], ((uint8_t *)rx_data)[1],
			((uint8_t *)rx_data)[2], ((uint8_t *)rx_data)[3],
			((uint8_t *)rx_data)[4], ((uint8_t *)rx_data)[5],
			((uint8_t *)rx_data)[6], ((uint8_t *)rx_data)[7]);
		break;
	}
}

/* ==================== 主函数 ==================== */

int main(void)
{
	LOG_INF("===== 机械臂区域控制板启动 =====");
	k_msleep(1000);
	if (relay_init() < 0) {
		return -1;
	}

	if (uart_comm_init() < 0) {
		return -1;
	}

	if (motor_init() < 0) {
		LOG_WRN("电机初始化失败, 继续运行 (仅继电器功能可用)");
	}

	while (1) {
		/* 仅在新数据到达、非脉冲期间、无轨迹执行时处理指令 */
		if (rx_new_data) {
			if (done_pulse == 0 && !trajectory_running) {
				rx_new_data = false;
				process_rx_command();
			} else {
				/* 丢包: 轨迹执行中或完成脉冲期间不处理新指令，但不刷屏 */
				static uint32_t drop_cnt;
				drop_cnt++;
				rx_new_data = false;
				if (drop_cnt <= 5 || drop_cnt % 500 == 0) {
					LOG_WRN("[DROP] rx=%u rsn=%s drop=%u",
						rx_cnt,
						done_pulse > 0 ? "pulse" : "busy",
						drop_cnt);
				}
			}
		}

		tx_data[0] = (done_pulse > 0) ? FLAG_DONE : FLAG_PENDING;
		tx_data[1] = 0.0f; /* reserved */

		if (done_pulse > 0) {
			done_pulse--;
		}

		int flush_ret = uart_tx_flush();
		if (flush_ret != 0) {
			tx_fail++;
			if (tx_fail <= 5 || tx_fail % 500 == 0) {
				LOG_ERR("[TX_FAIL] ret=%d fail=%u", flush_ret, tx_fail);
			}
		}
		tx_cnt++;

		/* 每 5 秒 (500 帧) 输出健康摘要 + 电机角度 + 继电器状态 */
		if (tx_cnt % 500 == 0) {
			LOG_INF("[HEART] TX=%u RX=%u fail=%u | M2=%.1f M3=%.1f M4=%.1f | R1=%d R2=%d R3=%d",
				tx_cnt, rx_cnt, tx_fail,
				(double)motor_get_angle(motor2),
				(double)motor_get_angle(motor3),
				(double)motor_get_angle(dm_motor),
				relay_get(0), relay_get(1), relay_get(2));
		}

		k_msleep(10); /* 100Hz */
	}

	return 0;
}
