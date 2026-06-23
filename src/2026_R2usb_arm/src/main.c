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
#include <ares/interface/uart/uart.h>
#include <ares/protocol/dual/dual_protocol.h>
#include <ares/ares_comm.h>
#include <sys/_stdint.h>
#include "devices.h"
#include "zephyr/drivers/gpio.h"
#include <arm_math.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);
#define deg2high(deg)  ((deg)*3.14159f * 45.0f / 360.0f)
#define high2deg(high) ((high)*360.0f / (3.14159f * 45.0f))
#define CHASSIS_CMD_X_IDX      0
#define CHASSIS_CMD_Y_IDX      1
#define CHASSIS_CMD_GYRO_IDX   2
#define CHASSIS_RX_TIMEOUT_MISSES 10U

/* Runtime source select: SBUS channel 5 controls speed source
 * Channel 5 > 0.3 (switch up) -> SBUS, else -> USB
 * When SBUS not receiving, channel returns 0.0 -> falls back to USB */
#define SBUS_SRC_SWITCH_CH 5

/* ==================== USB 协议 (原 2026_R2usb_imu) ==================== */

DUAL_PROPOSE_PROTOCOL_DEFINE(dual_protocol);
ARES_BULK_INTERFACE_DEFINE(usb_bulk_interface);

/* ==================== UART 协议 (对 relay_comm) ==================== */

#define RELAY_UART_NODE DT_NODELABEL(usart6)
static const struct device *relay_uart_dev = DEVICE_DT_GET(RELAY_UART_NODE);

ARES_UART_INTERFACE_DEFINE(relay_uart_interface);
DUAL_PROPOSE_PROTOCOL_DEFINE(relay_uart_protocol);

/* relay_comm UART 侧同步 ID */
#define RELAY_UART_SYNC_TX 0x0200 /* 发给 relay_comm (对应其 RX) */
#define RELAY_UART_SYNC_RX 0x0100 /* 收自 relay_comm (对应其 TX) */

/* relay 数据 USB 侧同步 ID */
#define RELAY_USB_SYNC_TX 0x0202 /* 上传 relay 状态给 PC */
#define RELAY_USB_SYNC_RX 0x0103 /* 接收 PC 的 relay 指令 */

/* 2 个 float = 8 字节 */
#define RELAY_DATA_LEN 8

/* 标志位 magic number (UART 侧, 与 relay_comm 一致) */
typedef union {
	float    f;
	uint32_t u;
} flag_val_t;

#define RELAY_FLAG_DONE_U32    0x4B0CD0F1
#define RELAY_FLAG_PENDING_U32 0x4E0DD0F1

/* USB 侧简化标志 */
#define USB_FLAG_DONE    -1.0f
#define USB_FLAG_PENDING  1.0f

/* UART 侧缓冲区 */
static float relay_uart_tx[2];
static float relay_uart_rx[2];
static uint8_t relay_uart_tx_buf[RELAY_DATA_LEN];
static uint8_t relay_uart_rx_buf[RELAY_DATA_LEN];

/* USB 侧缓冲区 */
static float relay_usb_tx[2];
static float relay_usb_rx[2];
static uint8_t relay_usb_tx_buf[RELAY_DATA_LEN];
static uint8_t relay_usb_rx_buf[RELAY_DATA_LEN];

/* 同步表 */
static sync_table_t *relay_uart_tx_pack;
static sync_table_t *relay_uart_rx_pack;
static sync_table_t *relay_usb_tx_pack;
static sync_table_t *relay_usb_rx_pack;

/* 统计 */
static uint32_t relay_uart_rx_cnt;
static uint32_t relay_uart_tx_cnt;
static uint32_t relay_usb_rx_cnt;
static uint32_t relay_usb_tx_cnt;
static uint32_t relay_usb_tx_fail;
static uint32_t relay_uart_tx_fail;
static uint32_t angle_tx_fail;

/* 优雅退化: 持续失败不刷屏, 每个错误独立计数 */
static uint32_t uart_fwd_fail_last_log;
static uint32_t usb_tx_fail_last_log;
static uint32_t angle_fail_last_log;

/* ==================== Relay 标志位转换 ==================== */

static float convert_relay_flag_to_usb(float relay_flag)
{
	flag_val_t v;
	v.f = relay_flag;
	if (v.u == RELAY_FLAG_DONE_U32) {
		return USB_FLAG_DONE;
	}
	return USB_FLAG_PENDING;
}

/* ==================== Relay UART 接收回调 (从 relay_comm 来) ==================== */

static void relay_uart_rx_cb(int status)
{
	if (status == SYNC_PACK_STATUS_DONE) {
		relay_uart_rx_cnt++;
		memcpy(relay_uart_rx, relay_uart_rx_buf, RELAY_DATA_LEN);
		relay_usb_tx[0] = convert_relay_flag_to_usb(relay_uart_rx[0]);
		relay_usb_tx[1] = relay_uart_rx[1];
		memcpy(relay_usb_tx_buf, relay_usb_tx, RELAY_DATA_LEN);
		/* 每 500 帧 (~5s) 打印 */
		if (relay_uart_rx_cnt % 500 == 0) {
			LOG_INF("[UART_RX] #%u flag_raw=0x%08X done=%.0f",
				relay_uart_rx_cnt,
				((flag_val_t *)relay_uart_rx)[0].u,
				(double)relay_usb_tx[0]);
		}
	}
}

/* ==================== Relay USB 接收回调 (从 PC 来, 转发给 relay_comm) ==================== */

static void relay_usb_rx_cb(int status)
{
	if (status == SYNC_PACK_STATUS_DONE) {
		relay_usb_rx_cnt++;
		memcpy(relay_usb_rx, relay_usb_rx_buf, RELAY_DATA_LEN);
		memcpy(relay_uart_tx, relay_usb_rx, sizeof(relay_usb_rx));
		memcpy(relay_uart_tx_buf, relay_uart_tx, RELAY_DATA_LEN);
		int ret = dual_sync_flush(&relay_uart_protocol, relay_uart_tx_pack);
		if (ret != 0) {
			relay_uart_tx_fail++;
			if (uart_fwd_fail_last_log == 0 ||
			    relay_uart_tx_fail - uart_fwd_fail_last_log >= 100) {
				LOG_ERR("[USB_RX] UART forward fail: %d (fail=%u)",
					ret, relay_uart_tx_fail);
				uart_fwd_fail_last_log = relay_uart_tx_fail;
			}
		}
		relay_uart_tx_cnt++;
		/* 每 500 个 PC 指令打印一次 */
		if (relay_usb_rx_cnt % 500 == 0) {
			LOG_INF("[USB_RX] #%u cmd=%d sub=%.2f", relay_usb_rx_cnt,
				(int)relay_usb_rx[0], (double)relay_usb_rx[1]);
		}
	}
}

/* ==================== Relay UART 初始化 ==================== */

static int relay_uart_init(void)
{
	if (!device_is_ready(relay_uart_dev)) {
		LOG_WRN("Relay UART 设备未就绪, 跳过 relay 桥接");
		return -ENODEV;
	}

	ares_uart_init_dev(&relay_uart_interface, relay_uart_dev);
	int ret = ares_bind_interface(&relay_uart_interface, &relay_uart_protocol);
	if (ret < 0) {
		LOG_ERR("Relay UART 绑定失败: %d", ret);
		return ret;
	}

	/* UART 侧: TX 发给 relay_comm, RX 收自 relay_comm */
	relay_uart_tx_pack = dual_sync_add(&relay_uart_protocol, RELAY_UART_SYNC_TX,
					   relay_uart_tx_buf, RELAY_DATA_LEN, NULL);
	relay_uart_rx_pack = dual_sync_add(&relay_uart_protocol, RELAY_UART_SYNC_RX,
					   relay_uart_rx_buf, RELAY_DATA_LEN, relay_uart_rx_cb);

	/* USB 侧: TX 上传给 PC, RX 收自 PC */
	relay_usb_tx_pack = dual_sync_add(&dual_protocol, RELAY_USB_SYNC_TX,
					  relay_usb_tx_buf, RELAY_DATA_LEN, NULL);
	relay_usb_rx_pack = dual_sync_add(&dual_protocol, RELAY_USB_SYNC_RX,
					  relay_usb_rx_buf, RELAY_DATA_LEN, relay_usb_rx_cb);

	LOG_INF("Relay 桥接初始化完成 (UART TX:0x%04X RX:0x%04X, USB TX:0x%04X RX:0x%04X)",
		RELAY_UART_SYNC_TX, RELAY_UART_SYNC_RX,
		RELAY_USB_SYNC_TX, RELAY_USB_SYNC_RX);
	return 0;
}

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

int rank[4] = {1, 1, -1, -1};
bool ups[4] = {true, true, true, true};
const struct device *dmmotor[4];
void run()
{
	for (int id = 0; id < 4; id++) {
		motor_set_mit(dmmotor[id], 10.0f, -30.0f * rank[id], 0.0);
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
	if (high > 400.0f) {
		high = 400.0f;
	} else if (high < 0.0f) {
		high = 0.0f;
	}
	if (dmmotor[id] != NULL) {
		motor_set_mit(dmmotor[id], 80.0f, -1.0f * rank[id] * (float)high2deg(high), 0.0f);
	}

	k_sleep(K_USEC(130));
}
int cnt = 0;
void console_feedback(void *arg1, void *arg2, void *arg3)
{

	bool zeroed = false;
	float linear_magnitude;
	bool in_deadzone;
	float X = 0.0f, Y = 0.0f, angvel = 0.0f;
	while (1) {
		k_msleep(2);

		lifthigh(0, height_cmd[0]);
		lifthigh(1, height_cmd[1]);
		lifthigh(2, height_cmd[2]);
		lifthigh(3, height_cmd[3]);

		if (sbus_get_percent(sbus, SBUS_SRC_SWITCH_CH) > 0.3f) {
			/* Speed source: SBUS */
			angvel = -sbus_get_percent(sbus, 0);
			X = -sbus_get_percent(sbus, 3);
			Y = -sbus_get_percent(sbus, 1);

			angle[11] = sbus_get_percent(sbus, 4);
			linear_magnitude = sqrtf(X * X + Y * Y);
			in_deadzone = ((linear_magnitude < 0.06f) && (fabsf(angvel) < 0.06f));
			if (linear_magnitude < 0.06f) {
				X = 0;
				Y = 0;
			}
			if (in_deadzone) {
				// chassis_set_static(chassis, true);
				chassis_set_speed(chassis, 0, 0);
				chassis_set_gyro(chassis, 0);
				zeroed = true;
			} else {
				chassis_set_static(chassis, false);
				chassis_set_speed(chassis, Y, X);
				chassis_set_gyro(chassis, angvel);
				angle[8] = X;
				angle[9] = Y;
				angle[10] = angvel * 10.0f;
				zeroed = false;
			}
		} else {
			/* Speed source: USB */
			X = chassis_cmd[CHASSIS_CMD_X_IDX];
			Y = chassis_cmd[CHASSIS_CMD_Y_IDX];
			angvel = chassis_cmd[CHASSIS_CMD_GYRO_IDX];
			chassis_set_static(chassis, false);
			chassis_set_speed(chassis, X, Y);
			chassis_set_gyro(chassis, angvel);
			zeroed = false;
			angle[8] = X;
			angle[9] = Y;
			angle[10] = angvel;
		}
		if (cnt++ % 1000 == 0) {
			LOG_INF("[CHASSIS] sensor=[%.0f,%.0f,%.0f,%.0f] high=[%.1f,%.1f,%.1f,%.1f] cmd=(%.1f,%.1f,%.1f) rx(ch=%u,h=%u)",
				(double)angle[0], (double)angle[1],
				(double)angle[2], (double)angle[3],
				(double)angle[4], (double)angle[5],
				(double)angle[6], (double)angle[7],
				(double)X, (double)Y, (double)angvel,
				chassis_rx_count, height_rx_count);
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
	motor_set_mode(dm_motor1, MIT);
	motor_set_mode(dm_motor2, MIT);
	motor_set_mode(dm_motor3, MIT);
	motor_set_mode(dm_motor4, MIT);
	motor_control(wheel_motor1, ENABLE_MOTOR);
	motor_control(wheel_motor2, ENABLE_MOTOR);
	motor_control(wheel_motor3, ENABLE_MOTOR);
	motor_control(wheel_motor4, ENABLE_MOTOR);
	motor_set_mode(wheel_motor1, ML_SPEED);
	motor_set_mode(wheel_motor2, ML_SPEED);
	motor_set_mode(wheel_motor3, ML_SPEED);
	motor_set_mode(wheel_motor4, ML_SPEED);
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

	k_msleep(100);
	ret = prepare();
	if (ret < 0) {
		LOG_ERR("prepare failed: %d", ret);
		return ret;
	}

	run();
	chassis_set_enabled(chassis, true);
	chassis_set_gyro(chassis, 0);
	LOG_INF("2026_R2usb_arm started: 0x0101=chassis, 0x0102=height, 0x0103=relay_cmd, 0x0201=telem, 0x0202=relay_stat");

	ret = ares_bind_interface(&usb_bulk_interface, &dual_protocol);
	if (ret < 0) {
		LOG_ERR("failed to bind usb interface: %d", ret);
		return ret;
	}

	chassis_rx = dual_sync_add(&dual_protocol, 0x0101, chassis_cmd_buf, sizeof(chassis_cmd_buf),
				     (dual_trans_cb_t)chassis_cmd_rx_cb);
	height_rx = dual_sync_add(&dual_protocol, 0x0102, height_cmd_buf, sizeof(height_cmd_buf),
				    (dual_trans_cb_t)height_cmd_rx_cb);
	angle_tx = dual_sync_add(&dual_protocol, 0x0201, anglebuf, 48, NULL);
	if (chassis_rx == NULL || height_rx == NULL || angle_tx == NULL) {
		LOG_ERR("failed to register dual sync packs");
		return -ENOMEM;
	}

	/* relay 桥接初始化 (失败不阻塞, 仅打印警告) */
	relay_uart_init();

	/* relay_usb_tx[0] 初始化为 PENDING，避免数据通之前 done=0 */
	relay_usb_tx[0] = USB_FLAG_PENDING;

	bool usb_toggle = false;

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
			if(rank[i] == 1) {
				angle[i + 4] = -1.0f * deg2high((float)motor_get_angle(dmmotor[i]));
			} else {
				angle[i + 4] = 900.0f + deg2high((float)motor_get_angle(dmmotor[i]));
			}
		}
		memcpy(anglebuf, angle, sizeof(angle));

		/* 交替发送 angle_tx 和 relay_usb_tx (各 100Hz)，
		 * 避免同一 USB 接口 200Hz 双 flush 导致 EBUSY */
		usb_toggle = !usb_toggle;

		if (usb_toggle) {
			int ret = dual_sync_flush(&dual_protocol, angle_tx);
			if (ret != 0 && ret != -EBUSY) {
				angle_tx_fail++;
				if (angle_fail_last_log == 0 ||
				    angle_tx_fail - angle_fail_last_log >= 100) {
					LOG_ERR("[USB_ANGLE_FAIL] ret=%d (fail=%u)", ret, angle_tx_fail);
					angle_fail_last_log = angle_tx_fail;
				}
			}
		}

		/* relay_usb_tx 数据由 relay_uart_rx_cb 填充，此处无需额外处理 */

		/* 上传 relay 状态给 PC */
		if (!usb_toggle && relay_usb_tx_pack != NULL) {
			int ret = dual_sync_flush(&dual_protocol, relay_usb_tx_pack);
			if (ret != 0) {
				relay_usb_tx_fail++;
				/* EBUSY(-16) 静默; 其他错误每100次报一次 */
				if (ret != -EBUSY &&
				    (usb_tx_fail_last_log == 0 ||
				     relay_usb_tx_fail - usb_tx_fail_last_log >= 100)) {
					LOG_ERR("[USB_TX_FAIL] ret=%d (fail=%u)", ret, relay_usb_tx_fail);
					usb_tx_fail_last_log = relay_usb_tx_fail;
				}
			}
			relay_usb_tx_cnt++;
		}

		/* 每约 1 秒 (100 帧 relay TX) 汇总统计, 任何异常都在这一行体现 */
		if (relay_usb_tx_cnt > 0 && relay_usb_tx_cnt % 100 == 0) {
			LOG_INF("[RELAY] usb_rx=%u usb_tx=%u(fail=%u) uart_rx=%u uart_tx=%u(fail=%u) angle_fail=%u | done=%.0f",
				relay_usb_rx_cnt, relay_usb_tx_cnt, relay_usb_tx_fail,
				relay_uart_rx_cnt, relay_uart_tx_cnt, relay_uart_tx_fail,
				angle_tx_fail,
				(double)relay_usb_tx[0]);
		}
	}

	return 0;
}
