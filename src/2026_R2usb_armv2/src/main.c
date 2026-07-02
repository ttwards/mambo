#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/motor.h>
#include <zephyr/drivers/sbus.h>
#include <zephyr/drivers/chassis.h>
#include <ares/board/init.h>
#include <ares/interface/usb/usb_bulk.h>
#include <ares/protocol/dual/dual_protocol.h>
#include <ares/ares_comm.h>
#include <sys/_stdint.h>
#include "devices.h"
#include "zephyr/drivers/gpio.h"
#include <arm_math.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);
#define deg2high(deg)              ((deg) * 3.14159f * 45.0f / 360.0f)
#define high2deg(high)             ((high) * 360.0f / (3.14159f * 45.0f))
#define CHASSIS_CMD_X_IDX          0
#define CHASSIS_CMD_Y_IDX          1
#define CHASSIS_CMD_GYRO_IDX       2
#define CHASSIS_RX_TIMEOUT_MISSES  10U
#define DM_TELEMETRY_LOG_PERIOD_MS 50U

#define SBUS_SRC_SWITCH_CH         5
#define SBUS_HEIGHT_GROUP0_CH      6
#define SBUS_HEIGHT_GROUP1_CH      7
#define SBUS_SRC_SWITCH_THRESHOLD  0.3f
#define SBUS_SWITCH_HIGH_THRESHOLD 0.3f
#define SBUS_SWITCH_LOW_THRESHOLD  -0.3f
#define SBUS_HEIGHT_GROUP_COUNT    2

#define SUSPENSION_HEIGHT_LOW       -20.0f
#define SUSPENSION_HEIGHT_MID       20.0f
#define SUSPENSION_HEIGHT_HIGH      200.0f
#define SUSPENSION_HEIGHT_MAX       400.0f
#define SUSPENSION_HEIGHT_WRAP      1350.0f
#define SUSPENSION_HEIGHT_WRAP_HALF (SUSPENSION_HEIGHT_WRAP / 2.0f)

/* ==================== USB 协议 (原 2026_R2usb_imu) ==================== */

DUAL_PROPOSE_PROTOCOL_DEFINE(dual_protocol);
ARES_BULK_INTERFACE_DEFINE(usb_bulk_interface);

static uint32_t angle_tx_fail;

static uint32_t angle_fail_last_log;

/* ==================== 底盘 / 电机 / 传感器 (原 2026_R2usb_imu, 不变) ==================== */

sync_table_t *angle_tx;

float chassis_cmd[3] = {0};
float height_cmd[4] = {0};
float angle[12] = {0};
uint8_t chassis_cmd_buf[12] = {0};
uint8_t height_cmd_buf[16] = {0};
uint8_t anglebuf[48] = {0};

static struct gpio_callback sensor1_cb_data;
static struct gpio_callback sensor2_cb_data;
static struct gpio_callback sensor3_cb_data;
static struct gpio_callback sensor4_cb_data;
static volatile uint32_t chassis_rx_count;
static volatile uint32_t height_rx_count;
static int8_t sbus_height_level[SBUS_HEIGHT_GROUP_COUNT] = {-2, -2};

void unified_sensor_callback(const struct device *port, struct gpio_callback *cb, uint32_t pins);

static void clear_chassis_cmd(void)
{
	chassis_cmd[CHASSIS_CMD_X_IDX] = 0.0f;
	chassis_cmd[CHASSIS_CMD_Y_IDX] = 0.0f;
	chassis_cmd[CHASSIS_CMD_GYRO_IDX] = 0.0f;
}

static int check_device_ready(const struct device *dev, const char *name)
{
	if (dev == NULL || !device_is_ready(dev)) {
		LOG_ERR("device not ready: %s", name);
		return -ENODEV;
	}

	return 0;
}

static int setup_sensor_input(const struct gpio_dt_spec *sensor, struct gpio_callback *cb_data,
			      const char *name)
{
	int ret;

	if (!gpio_is_ready_dt(sensor)) {
		LOG_ERR("gpio not ready: %s", name);
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(sensor, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("failed to configure %s: %d", name, ret);
		return ret;
	}

	ret = gpio_pin_interrupt_configure_dt(sensor, GPIO_INT_EDGE_BOTH);
	if (ret < 0) {
		LOG_ERR("failed to configure irq for %s: %d", name, ret);
		return ret;
	}

	gpio_init_callback(cb_data, unified_sensor_callback, BIT(sensor->pin));
	ret = gpio_add_callback(sensor->port, cb_data);
	if (ret < 0) {
		LOG_ERR("failed to add callback for %s: %d", name, ret);
		return ret;
	}

	return 0;
}

void unified_sensor_callback(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
	if (port == sensor1.port && (pins & BIT(sensor1.pin))) {
		if (gpio_pin_get_dt(&sensor1) > 0) {
			angle[0] = 1.0f;
		} else {
			angle[0] = 0.0f;
		}
	}

	if (port == sensor2.port && (pins & BIT(sensor2.pin))) {
		if (gpio_pin_get_dt(&sensor2) > 0) {
			angle[1] = 1.0f;
		} else {
			angle[1] = 0.0f;
		}
	}

	if (port == sensor3.port && (pins & BIT(sensor3.pin))) {
		if (gpio_pin_get_dt(&sensor3) > 0) {
			angle[2] = 1.0f;
		} else {
			angle[2] = 0.0f;
		}
	}

	if (port == sensor4.port && (pins & BIT(sensor4.pin))) {
		if (gpio_pin_get_dt(&sensor4) > 0) {
			angle[3] = 1.0f;
		} else {
			angle[3] = 0.0f;
		}
	}
}

int rank[4] = {1, -1, 1, -1};
bool ups[4] = {true, true, true, true};
const struct device *dmmotor[4];

static bool dm_telemetry_log_due(void)
{
	static uint32_t last_log_ms;
	uint32_t now_ms = k_uptime_get_32();

	if ((uint32_t)(now_ms - last_log_ms) < DM_TELEMETRY_LOG_PERIOD_MS) {
		return false;
	}

	last_log_ms = now_ms;
	return true;
}

static void log_dm_telemetry_if_due(void)
{
	motor_status_t status[4] = {0};

	if (!dm_telemetry_log_due()) {
		return;
	}

	for (int i = 0; i < 4; i++) {
		int ret = motor_get(dmmotor[i], &status[i]);

		if (ret < 0) {
			LOG_WRN("[DM] read failed: m%d ret=%d", i + 1, ret);
			return;
		}
	}

	LOG_INF("[DM] pos_deg=[%.1f,%.1f,%.1f,%.1f] speed_rpm=[%.1f,%.1f,%.1f,%.1f] "
		"torque=[%.2f,%.2f,%.2f,%.2f]",
		(double)status[0].angle, (double)status[1].angle, (double)status[2].angle,
		(double)status[3].angle, (double)status[0].rpm, (double)status[1].rpm,
		(double)status[2].rpm, (double)status[3].rpm, (double)status[0].torque,
		(double)status[1].torque, (double)status[2].torque, (double)status[3].torque);
}

void run()
{
	for (int id = 0; id < 4; id++) {
		motor_set_mit(dmmotor[id], 80.0f,
			      -1.0f * rank[id] * (float)high2deg(SUSPENSION_HEIGHT_MID), 0.0f);
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
		motor_set_mit(dmmotor[id], 10.0f, -510.0f * rank[id], 0.0);
		ups[id] = false;
	}
	k_sleep(K_USEC(130));
}
void lifthigh(int id, float high)
{
	if (high > SUSPENSION_HEIGHT_MAX) {
		high = SUSPENSION_HEIGHT_MAX;
	} else if (high < SUSPENSION_HEIGHT_LOW) {
		high = SUSPENSION_HEIGHT_LOW;
	}
	if (dmmotor[id] != NULL) {
		motor_set_mit(dmmotor[id], 250.0f, -1.0f * rank[id] * (float)high2deg(high),
			      -0.5f * rank[id]);
	}

	k_sleep(K_USEC(130));
}

static int8_t sbus_height_level_from_channel(uint8_t channel)
{
	float height_switch = sbus_get_percent(sbus, channel);

	if (height_switch > SBUS_SWITCH_HIGH_THRESHOLD) {
		return 1;
	}
	if (height_switch < SBUS_SWITCH_LOW_THRESHOLD) {
		return -1;
	}
	return 0;
}

static float suspension_height_from_level(int8_t level)
{
	if (level > 0) {
		return SUSPENSION_HEIGHT_HIGH;
	}
	if (level < 0) {
		return SUSPENSION_HEIGHT_LOW;
	}
	return SUSPENSION_HEIGHT_MID;
}

static void apply_suspension_pair_height(int id0, int id1, float high)
{
	lifthigh(id0, high);
	lifthigh(id1, high);
}

static void apply_sbus_suspension_height(void)
{
	int8_t group0_level = sbus_height_level_from_channel(SBUS_HEIGHT_GROUP0_CH);
	int8_t group1_level = sbus_height_level_from_channel(SBUS_HEIGHT_GROUP1_CH);

	angle[11] = sbus_get_percent(sbus, SBUS_HEIGHT_GROUP0_CH);

	if (group0_level != sbus_height_level[0]) {
		sbus_height_level[0] = group0_level;
		apply_suspension_pair_height(0, 1, suspension_height_from_level(group0_level));
	}
	if (group1_level != sbus_height_level[1]) {
		sbus_height_level[1] = group1_level;
		apply_suspension_pair_height(2, 3, suspension_height_from_level(group1_level));
	}
}

static void reset_sbus_height_levels(void)
{
	for (int i = 0; i < SBUS_HEIGHT_GROUP_COUNT; i++) {
		sbus_height_level[i] = -2;
	}
}

static void apply_usb_height_cmd_if_ready(void)
{
	if (height_rx_count == 0U) {
		return;
	}

	for (int id = 0; id < 4; id++) {
		lifthigh(id, height_cmd[id]);
	}
}

static void log_sbus_channels(void)
{
	LOG_INF("[SBUS] ch0=%.2f ch1=%.2f ch2=%.2f ch3=%.2f ch4=%.2f ch5=%.2f ch6=%.2f ch7=%.2f",
		(double)sbus_get_percent(sbus, 0), (double)sbus_get_percent(sbus, 1),
		(double)sbus_get_percent(sbus, 2), (double)sbus_get_percent(sbus, 3),
		(double)sbus_get_percent(sbus, 4), (double)sbus_get_percent(sbus, 5),
		(double)sbus_get_percent(sbus, 6), (double)sbus_get_percent(sbus, 7));
}

int cnt = 0;
void console_feedback(void *arg1, void *arg2, void *arg3)
{

	bool zeroed = false;
	float linear_magnitude;
	bool in_deadzone;
	float X = 0.0f, Y = 0.0f, angvel = 0.0f;
	bool was_sbus_mode = false;
	while (1) {
		k_msleep(2);

		float src_switch = sbus_get_percent(sbus, SBUS_SRC_SWITCH_CH);

		if (src_switch > SBUS_SRC_SWITCH_THRESHOLD) {
			if (!was_sbus_mode) {
				reset_sbus_height_levels();
			}
			was_sbus_mode = true;

			angvel = -sbus_get_percent(sbus, 0);
			X = -sbus_get_percent(sbus, 3);
			Y = -sbus_get_percent(sbus, 1);

			linear_magnitude = sqrtf(X * X + Y * Y);
			in_deadzone = ((linear_magnitude < 0.06f) && (fabsf(angvel) < 0.06f));
			if (linear_magnitude < 0.06f) {
				X = 0;
				Y = 0;
			}
			if (in_deadzone) {
				chassis_set_speed(chassis, 0, 0);
				chassis_set_gyro(chassis, 0);
				angle[8] = 0.0f;
				angle[9] = 0.0f;
				angle[10] = 0.0f;
				zeroed = true;
			} else {
				chassis_set_static(chassis, false);
				chassis_set_speed(chassis, X, Y);
				chassis_set_gyro(chassis, -angvel);
				angle[8] = X;
				angle[9] = Y;
				angle[10] = angvel * 10.0f;
				zeroed = false;
			}
			apply_sbus_suspension_height();
		} else {
			was_sbus_mode = false;

			X = chassis_cmd[CHASSIS_CMD_X_IDX];
			Y = chassis_cmd[CHASSIS_CMD_Y_IDX];
			angvel = chassis_cmd[CHASSIS_CMD_GYRO_IDX];
			chassis_set_static(chassis, false);
			chassis_set_speed(chassis, X, Y);
			chassis_set_gyro(chassis, -angvel);
			angle[8] = X;
			angle[9] = Y;
			angle[10] = angvel;
			zeroed = false;
			apply_usb_height_cmd_if_ready();
		}
		if (cnt++ % 1000 == 0) {
			LOG_INF("[CHASSIS] src=%s sw=%.2f sensor=[%.0f,%.0f,%.0f,%.0f] "
				"high=[%.1f,%.1f,%.1f,%.1f] cmd=(%.1f,%.1f,%.1f) rx(ch=%u,h=%u)",
				was_sbus_mode ? "sbus" : "usb", (double)src_switch,
				(double)angle[0], (double)angle[1], (double)angle[2],
				(double)angle[3], (double)angle[4], (double)angle[5],
				(double)angle[6], (double)angle[7], (double)X, (double)Y,
				(double)angvel, chassis_rx_count, height_rx_count);
			log_sbus_channels();
		}
	}
}

K_THREAD_DEFINE(feedback_thread, 4096, console_feedback, NULL, NULL, NULL, 2, 0, 100);
uint32_t log_cnt;

int chassis_cmd_rx_cb(int status)
{
	if (status == SYNC_PACK_STATUS_DONE) {
		memcpy(chassis_cmd, chassis_cmd_buf, sizeof(chassis_cmd));
		chassis_rx_count++;
	}
	return 0;
}

int height_cmd_rx_cb(int status)
{

	if (status == SYNC_PACK_STATUS_DONE) {
		memcpy(height_cmd, height_cmd_buf, sizeof(height_cmd));
		height_rx_count++;
	}
	return 0;
}
int prepare(void)
{
	int ret;

	ret = check_device_ready(chassis, "chassis");
	if (ret < 0) {
		return ret;
	}

	ret = check_device_ready(dm_motor1, "dm_motor1");
	if (ret < 0) {
		return ret;
	}
	ret = check_device_ready(dm_motor2, "dm_motor2");
	if (ret < 0) {
		return ret;
	}
	ret = check_device_ready(dm_motor3, "dm_motor3");
	if (ret < 0) {
		return ret;
	}
	ret = check_device_ready(dm_motor4, "dm_motor4");
	if (ret < 0) {
		return ret;
	}
	ret = check_device_ready(wheel_motor1, "wheel_motor1");
	if (ret < 0) {
		return ret;
	}
	ret = check_device_ready(wheel_motor2, "wheel_motor2");
	if (ret < 0) {
		return ret;
	}
	ret = check_device_ready(wheel_motor3, "wheel_motor3");
	if (ret < 0) {
		return ret;
	}
	ret = check_device_ready(wheel_motor4, "wheel_motor4");
	if (ret < 0) {
		return ret;
	}
	ret = check_device_ready(steer_motor1, "steer_motor1");
	if (ret < 0) {
		return ret;
	}
	ret = check_device_ready(steer_motor2, "steer_motor2");
	if (ret < 0) {
		return ret;
	}
	ret = check_device_ready(steer_motor3, "steer_motor3");
	if (ret < 0) {
		return ret;
	}
	ret = check_device_ready(steer_motor4, "steer_motor4");
	if (ret < 0) {
		return ret;
	}
	ret = check_device_ready(sbus, "sbus");
	if (ret < 0) {
		return ret;
	}

	dmmotor[0] = dm_motor1;
	dmmotor[1] = dm_motor2;
	dmmotor[2] = dm_motor3;
	dmmotor[3] = dm_motor4;
	motor_control(dm_motor1, ENABLE_MOTOR);
	motor_control(dm_motor2, ENABLE_MOTOR);
	motor_control(dm_motor3, ENABLE_MOTOR);
	motor_control(dm_motor4, ENABLE_MOTOR);
	motor_control(wheel_motor1, ENABLE_MOTOR);
	motor_control(wheel_motor2, ENABLE_MOTOR);
	motor_control(wheel_motor3, ENABLE_MOTOR);
	motor_control(wheel_motor4, ENABLE_MOTOR);
	motor_control(steer_motor1, ENABLE_MOTOR);
	motor_control(steer_motor2, ENABLE_MOTOR);
	motor_control(steer_motor3, ENABLE_MOTOR);
	motor_control(steer_motor4, ENABLE_MOTOR);

	ret = setup_sensor_input(&sensor1, &sensor1_cb_data, "sensor1");
	if (ret < 0) {
		return ret;
	}
	ret = setup_sensor_input(&sensor2, &sensor2_cb_data, "sensor2");
	if (ret < 0) {
		return ret;
	}
	ret = setup_sensor_input(&sensor3, &sensor3_cb_data, "sensor3");
	if (ret < 0) {
		return ret;
	}
	ret = setup_sensor_input(&sensor4, &sensor4_cb_data, "sensor4");
	if (ret < 0) {
		return ret;
	}

	angle[0] = gpio_pin_get_dt(&sensor1) > 0 ? 1.0f : 0.0f;
	angle[1] = gpio_pin_get_dt(&sensor2) > 0 ? 1.0f : 0.0f;
	angle[2] = gpio_pin_get_dt(&sensor3) > 0 ? 1.0f : 0.0f;
	angle[3] = gpio_pin_get_dt(&sensor4) > 0 ? 1.0f : 0.0f;

	return 0;
}
int main(void)
{
	int ret;
	sync_table_t *chassis_rx;
	sync_table_t *height_rx;
	uint32_t last_chassis_rx_count = 0U;
	uint32_t missed_chassis_cycles = 0U;
	// k_msleep(1000);/
	// run();
	k_msleep(100);
	ret = prepare();
	if (ret < 0) {
		LOG_ERR("prepare failed: %d", ret);
		return ret;
	}

	run();
	chassis_set_enabled(chassis, true);
	chassis_set_gyro(chassis, 0);
	LOG_INF("2026_R2usb_armv2 started: 0x0111=chassis, 0x0112=height, 0x0121=telem");

	ret = ares_bind_interface(&usb_bulk_interface, &dual_protocol);
	if (ret < 0) {
		LOG_ERR("failed to bind usb interface: %d", ret);
		return ret;
	}

	chassis_rx = dual_sync_add(&dual_protocol, 0x0111, chassis_cmd_buf, sizeof(chassis_cmd_buf),
				   (dual_trans_cb_t)chassis_cmd_rx_cb);
	height_rx = dual_sync_add(&dual_protocol, 0x0112, height_cmd_buf, sizeof(height_cmd_buf),
				  (dual_trans_cb_t)height_cmd_rx_cb);
	angle_tx = dual_sync_add(&dual_protocol, 0x0121, anglebuf, 48, NULL);
	if (chassis_rx == NULL || height_rx == NULL || angle_tx == NULL) {
		LOG_ERR("failed to register dual sync packs");
		return -ENOMEM;
	}

	bool angle_tx_due = false;

	while (1) {
		k_msleep(5);

		if (chassis_rx_count != last_chassis_rx_count) {
			last_chassis_rx_count = chassis_rx_count;
			missed_chassis_cycles = 0U;
		} else if (missed_chassis_cycles < CHASSIS_RX_TIMEOUT_MISSES) {
			missed_chassis_cycles++;
		}

		if (missed_chassis_cycles >= CHASSIS_RX_TIMEOUT_MISSES) {
			clear_chassis_cmd();
		}

		for (int i = 0; i < 4; i++) {
			float height =
				-1.0f * rank[i] * deg2high((float)motor_get_angle(dmmotor[i]));

			if (height > SUSPENSION_HEIGHT_WRAP_HALF) {
				height -= SUSPENSION_HEIGHT_WRAP;
			} else if (height < -SUSPENSION_HEIGHT_WRAP_HALF) {
				height += SUSPENSION_HEIGHT_WRAP;
			}
			angle[i + 4] = height;
		}
		// log_dm_telemetry_if_due();
		memcpy(anglebuf, angle, sizeof(angle));

		/* Keep telemetry at 100Hz. */
		angle_tx_due = !angle_tx_due;

		if (angle_tx_due) {
			int ret = dual_sync_flush(&dual_protocol, angle_tx);
			if (ret != 0 && ret != -EBUSY) {
				angle_tx_fail++;
				if (angle_fail_last_log == 0 ||
				    angle_tx_fail - angle_fail_last_log >= 100) {
					LOG_ERR("[USB_ANGLE_FAIL] ret=%d (fail=%u)", ret,
						angle_tx_fail);
					angle_fail_last_log = angle_tx_fail;
				}
			}
		}
	}

	return 0;
}
