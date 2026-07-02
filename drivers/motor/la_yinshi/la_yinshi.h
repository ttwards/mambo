

/**
 * @file
 * @brief Yinshi LA Series Micro Servo Electric Cylinder Driver
 *
 * Driver for Yinshi Robotics LA/LAS/LAF/LASF/LAXC series micro servo
 * electric cylinders over UART (D-type: 3.3V LVTTL, or 2-type: RS485).
 */

#ifndef LA_YINSHI_H_
#define LA_YINSHI_H_

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/linear_actuator.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#define DT_DRV_COMPAT yinshi_la

#ifdef __cplusplus
extern "C" {
#endif

/*---------------------------------------------------------------------------*
 * Protocol constants
 *---------------------------------------------------------------------------*/

/* Frame markers */
#define LA_FRAME_HEADER1 0x55
#define LA_FRAME_HEADER2 0xAA
#define LA_REPLY_HEADER1 0xAA
#define LA_REPLY_HEADER2 0x55

/* Command types */
#define LA_CMD_RD               0x01 /**< Read control table */
#define LA_CMD_WR               0x02 /**< Write control table */
#define LA_CMD_WR_DRV_POS       0x21 /**< Position mode (with feedback) */
#define LA_CMD_WR_DRV_POS_NF    0x03 /**< Position mode (no feedback) */
#define LA_CMD_WR_DRV_FOLLOW    0x20 /**< Follow mode (with feedback) */
#define LA_CMD_WR_DRV_FOLLOW_NF 0x19 /**< Follow mode (no feedback) */
#define LA_CMD_MC               0x04 /**< Single control command */

/* Control table index for position */
#define LA_INDEX_TARGET_POS  0x37 /**< Target position (16-bit, 0~2000) */
#define LA_INDEX_OVERCURRENT 0x20 /**< Overcurrent protection threshold */
#define LA_INDEX_OVERTEMP    0x62 /**< Overtemperature protection threshold */
#define LA_INDEX_RETURN_TEMP 0x64 /**< Return-to-work temperature threshold */

/* Single control sub-commands (MC data byte) */
#define LA_MC_WORK           0x04 /**< Enable motor power */
#define LA_MC_EMERGENCY_STOP 0x23 /**< Emergency stop */
#define LA_MC_PAUSE          0x14 /**< Pause (soft stop) */
#define LA_MC_SAVE_PARAMS    0x20 /**< Save params to Flash */
#define LA_MC_QUERY_STATUS   0x22 /**< Query status info (BIT) */
#define LA_MC_CLEAR_FAULT    0x1E /**< Clear fault state */

/* Frame size limits */
#define LA_MAX_FRAME_LEN 32
#define LA_MIN_FRAME_LEN 6 /**< Header(2) + Len(1) + ID(1) + CMD(1) + Index(1) + Checksum(1) */

/*---------------------------------------------------------------------------*
 * RX state machine
 *---------------------------------------------------------------------------*/

enum la_rx_state {
	LA_RX_WAIT_HEADER1, /**< Waiting for 0xAA (reply header byte 1) */
	LA_RX_WAIT_HEADER2, /**< Waiting for 0x55 (reply header byte 2) */
	LA_RX_WAIT_LEN,     /**< Waiting for length byte */
	LA_RX_WAIT_DATA,    /**< Collecting ID + payload + checksum */
};

/*---------------------------------------------------------------------------*
 * Per-device configuration (constant, from DT)
 *---------------------------------------------------------------------------*/

struct la_yinshi_config {
	/** UART device for communication */
	const struct device *uart_dev;
	/** Device ID on the bus (1~254) */
	uint8_t id;
	/** RS485 DE pin (optional; .port == NULL if not used) */
	struct gpio_dt_spec de_gpio;
	/** Maximum number of retries for position commands */
	uint8_t max_retries;
	/** Reply timeout in milliseconds */
	uint16_t reply_timeout_ms;
};

/*---------------------------------------------------------------------------*
 * Per-device runtime data
 *---------------------------------------------------------------------------*/

struct la_yinshi_data {
	/** Serialize access to UART and internal state */
	struct k_mutex lock;

	/* ---- RX state machine ---- */
	enum la_rx_state rx_state;
	uint8_t rx_buf[LA_MAX_FRAME_LEN];
	uint8_t rx_pos; /**< Bytes collected in current state */
	uint8_t rx_len; /**< Expected payload length (from length byte) */

	/* ---- Reply synchronization ---- */
	struct k_sem reply_sem;
	uint8_t reply_buf[LA_MAX_FRAME_LEN];
	uint8_t reply_len;

	/* ---- Current status cache ---- */
	struct la_status current_status;

	/* ---- Retry tracking ---- */
	uint8_t retry_count;
};

/*---------------------------------------------------------------------------*
 * API function declarations
 *---------------------------------------------------------------------------*/

int la_yinshi_set_position(const struct device *dev, uint16_t position);
int la_yinshi_get_status(const struct device *dev, struct la_status *status);
int la_yinshi_cmd(const struct device *dev, enum la_cmd cmd);
int la_yinshi_set_param(const struct device *dev, uint8_t index, uint16_t value);
int la_yinshi_init(const struct device *dev);

/*---------------------------------------------------------------------------*
 * Devicetree instantiation macros
 *---------------------------------------------------------------------------*/

/**
 * @brief Allocate per-instance config structure from devicetree.
 */
#define LA_YINSHI_CONFIG_INST(inst)                                                                \
	static const struct la_yinshi_config la_yinshi_config_##inst = {                           \
		.uart_dev = DEVICE_DT_GET(DT_INST_PHANDLE(inst, uart)),                            \
		.id = DT_INST_PROP_OR(inst, id, 1),                                                \
		.de_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, de_gpios, {0}),                          \
		.max_retries = DT_INST_PROP_OR(inst, max_retries, 3),                              \
		.reply_timeout_ms = DT_INST_PROP_OR(inst, reply_timeout_ms, 5),                    \
	}

/**
 * @brief Allocate per-instance data structure.
 */
#define LA_YINSHI_DATA_INST(inst)                                                                  \
	static struct la_yinshi_data la_yinshi_data_##inst = {                                     \
		.rx_state = LA_RX_WAIT_HEADER1,                                                    \
		.current_status = {0},                                                             \
	}

/**
 * @brief Define the driver API function table.
 */
#define LA_YINSHI_API_INST(inst)                                                                   \
	static const struct linear_actuator_driver_api la_yinshi_api_##inst = {                    \
		.set_position = la_yinshi_set_position,                                            \
		.get_status = la_yinshi_get_status,                                                \
		.cmd = la_yinshi_cmd,                                                              \
		.set_param = la_yinshi_set_param,                                                  \
	}

/**
 * @brief Fully define a yinshi,la device instance.
 *
 * Usage:
 *   DT_INST_FOREACH_STATUS_OKAY(LA_YINSHI_DEFINE_INST)
 */
#define LA_YINSHI_DEFINE_INST(inst)                                                                \
	LA_YINSHI_CONFIG_INST(inst);                                                               \
	LA_YINSHI_DATA_INST(inst);                                                                 \
	LA_YINSHI_API_INST(inst);                                                                  \
	DEVICE_DT_INST_DEFINE(inst, la_yinshi_init, NULL, &la_yinshi_data_##inst,                  \
			      &la_yinshi_config_##inst, POST_KERNEL,                               \
			      CONFIG_LA_YINSHI_INIT_PRIORITY, &la_yinshi_api_##inst)

#ifdef __cplusplus
}
#endif

#endif /* LA_YINSHI_H_ */
