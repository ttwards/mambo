/* SPDX-License-Identifier: Apache-2.0
 *
 * Driver for Yinshi Robotics LA Series Micro Servo Electric Cylinder.
 *
 * Protocol: custom binary frames over 3.3V LVTTL UART (D-type) or RS485 (2-type).
 *
 * Command frame:  0x55 0xAA | Len | ID | CMD | Index | Data[N] | Checksum
 * Response frame: 0xAA 0x55 | Len | ID | CMD | Index | Data[N] | Checksum
 *
 *   Len = N + 2  (CMD + Index + N data bytes)
 *   Checksum = low 8 bits of sum of Len + ID + CMD + Index + Data[0..N-1]
 */

#include "la_yinshi.h"

#include <string.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(la_yinshi, CONFIG_LA_YINSHI_LOG_LEVEL);

/*===========================================================================*
 * Protocol codec (pure functions, no side effects)
 *===========================================================================*/

/**
 * @brief Calculate checksum for a frame.
 *
 * Checksum = low 8 bits of sum of all bytes from length through data (excludes
 * the two header bytes).
 *
 * @param buf  Buffer starting at the length byte
 * @param len  Number of bytes (length_byte + ID + payload)
 * @return 8-bit checksum
 */
static uint8_t la_calc_checksum(const uint8_t *buf, uint8_t len)
{
	uint16_t sum = 0;

	for (uint8_t i = 0; i < len; i++) {
		sum += buf[i];
	}
	return (uint8_t)(sum & 0xFF);
}

/**
 * @brief Pack a command frame into the output buffer.
 *
 * Output layout:
 *   out[0..1] = header (0x55, 0xAA)
 *   out[2]    = length (data_len + 2)
 *   out[3]    = id
 *   out[4]    = cmd
 *   out[5]    = index
 *   out[6..6+data_len-1] = data
 *   out[6+data_len] = checksum
 *
 * @return Total frame length (8 + data_len for no-data commands, etc.)
 */
static uint8_t la_pack_command(uint8_t id, uint8_t cmd, uint8_t index, const uint8_t *data,
			       uint8_t data_len, uint8_t *out)
{
	uint8_t payload_len = data_len + 2; /* CMD + Index + Data */
	uint8_t pos = 0;

	/* Header */
	out[pos++] = LA_FRAME_HEADER1; /* 0x55 */
	out[pos++] = LA_FRAME_HEADER2; /* 0xAA */

	/* Length */
	out[pos++] = payload_len;

	/* ID */
	out[pos++] = id;

	/* CMD */
	out[pos++] = cmd;

	/* Index */
	out[pos++] = index;

	/* Data (may be zero-length) */
	if (data_len > 0 && data != NULL) {
		memcpy(&out[pos], data, data_len);
		pos += data_len;
	}

	/* Checksum — over length byte through end of data */
	/* The region for checksum starts at out[2] (length byte) */
	uint8_t chk = la_calc_checksum(&out[2], pos - 2);

	out[pos++] = chk;

	return pos;
}

/**
 * @brief Parse the status query response payload.
 *
 * The payload (after ID byte) for a CMD_MC query-status response:
 *   [0]  = CMD (0x04)
 *   [1]  = param1 (reserved, 0x00)
 *   [2]  = param2 (0x22 = query status)
 *   [3-4]  = target position  (uint16 LE)
 *   [5-6]  = current position (int16 LE)
 *   [7]    = temperature (int8, °C)
 *   [8-9]  = drive current (uint16 LE, mA)
 *   [10-11] = force sensor / internal value (int16 LE)
 *   [12]    = error flags
 *   [13-14] = internal data 1
 *   [15-16] = internal data 2
 */
static void la_parse_status_response(const uint8_t *payload, uint8_t len, struct la_status *status)
{
	if (len < 13) {
		return;
	}

	/* Offset 3: target position (skip CMD + param1 + param2 = 3 bytes) */
	status->target_pos = sys_get_le16(&payload[3]);

	/* Offset 5: current position */
	status->current_pos = (int16_t)sys_get_le16(&payload[5]);

	/* Offset 7: temperature */
	status->temperature = (int8_t)payload[7];

	/* Offset 8: drive current */
	status->current_ma = sys_get_le16(&payload[8]);

	/* Offset 10: force sensor value */
	if (len >= 12) {
		status->force_grams = (int16_t)sys_get_le16(&payload[10]);
	} else {
		status->force_grams = 0;
	}

	/* Offset 12: error flags */
	if (len >= 13) {
		status->faults = payload[12] & 0x0F; /* lower 4 bits only */
	} else {
		status->faults = 0;
	}

	status->online = true;
}

/**
 * @brief Parse a generic response payload for position-set or MC command reply.
 *
 * These responses carry the same status layout as the explicit query-status
 * command. We extract position and fault info to update the cache.
 */
static void la_parse_generic_response(const uint8_t *payload, uint8_t len, struct la_status *status)
{
	/* Most responses from the actuator use the same status layout.
	 * For CMD_WR_DRV (0x21, 0x03, 0x20, 0x19) replies:
	 *   Same as query-status format.
	 * For CMD_MC (0x04) replies:
	 *   Same as query-status format.
	 * For CMD_WR (0x02) / CMD_RD (0x01) replies:
	 *   Payload[0]=CMD, Payload[1]=Index, Payload[2..]=data
	 */
	if (len < 3) {
		return;
	}

	uint8_t cmd = payload[0];

	if (cmd == LA_CMD_MC || cmd == LA_CMD_WR_DRV_POS || cmd == LA_CMD_WR_DRV_POS_NF ||
	    cmd == LA_CMD_WR_DRV_FOLLOW || cmd == LA_CMD_WR_DRV_FOLLOW_NF) {
		/* These all return status info in the same format */
		la_parse_status_response(payload, len, status);
	}
	/* For CMD_WR / CMD_RD, just set online; detailed parsing not needed */
	status->online = true;
}

/*===========================================================================*
 * UART RX interrupt handler
 *===========================================================================*/

/**
 * @brief Feed a single byte into the RX state machine.
 *
 * Called from ISR context. When a complete valid frame targeting our device ID
 * is received, signals reply_sem to wake the waiting thread.
 */
static void la_uart_isr(const struct device *uart_dev, void *user_data)
{
	const struct device *dev = (const struct device *)user_data;
	struct la_yinshi_data *data = dev->data;
	const struct la_yinshi_config *cfg = dev->config;

	uart_irq_update(uart_dev);

	while (uart_irq_rx_ready(uart_dev)) {
		uint8_t byte;

		if (uart_fifo_read(uart_dev, &byte, 1) != 1) {
			break;
		}

		switch (data->rx_state) {

		case LA_RX_WAIT_HEADER1:
			if (byte == LA_REPLY_HEADER1) { /* 0xAA */
				data->rx_state = LA_RX_WAIT_HEADER2;
			}
			break;

		case LA_RX_WAIT_HEADER2:
			if (byte == LA_REPLY_HEADER2) { /* 0x55 */
				data->rx_state = LA_RX_WAIT_LEN;
			} else if (byte != LA_REPLY_HEADER1) {
				/* Not a header, go back to start */
				data->rx_state = LA_RX_WAIT_HEADER1;
			}
			/* else byte == 0xAA: stay in WAIT_HEADER2 (overlapping header) */
			break;

		case LA_RX_WAIT_LEN:
			data->rx_len = byte;
			if (data->rx_len == 0 || data->rx_len > (LA_MAX_FRAME_LEN - 4)) {
				/* Invalid length, reset */
				data->rx_state = LA_RX_WAIT_HEADER1;
			} else {
				data->rx_pos = 0;
				data->rx_state = LA_RX_WAIT_DATA;
			}
			break;

		case LA_RX_WAIT_DATA:
			data->rx_buf[data->rx_pos++] = byte;
			/* Need: ID(1) + payload(rx_len) + checksum(1) = rx_len + 2 */
			if (data->rx_pos >= (uint8_t)(data->rx_len + 2)) {
				/* Frame complete. Verify checksum.
				 * rx_buf layout:
				 *   [0] = ID
				 *   [1..rx_len] = payload (CMD + Index + Data)
				 *   [rx_len] = checksum  (inclusive index, rx_len+1 bytes total)
				 * Wait, rx_pos = rx_len + 2 after storing checksum.
				 * So:
				 *   rx_buf[0] = ID
				 *   rx_buf[1 .. rx_len] = payload (rx_len bytes)
				 *   rx_buf[rx_len+1] = checksum
				 * Total stored: rx_len + 2 bytes
				 *
				 * Checksum covers: length(1) + ID(1) + payload(rx_len) bytes
				 * = 1 + 1 + rx_len = rx_len + 2 bytes
				 *
				 * But the length byte itself is NOT in rx_buf. We stored it
				 * in rx_len. So checksum = (rx_len + sum of rx_buf[0..rx_len]) &
				 * 0xFF
				 */
				uint8_t expected_chk = data->rx_buf[data->rx_pos - 1];
				uint16_t sum = data->rx_len; /* length byte */

				for (uint8_t i = 0; i <= data->rx_len; i++) {
					/* rx_buf[0..rx_len]: ID + payload */
					sum += data->rx_buf[i];
				}

				if ((uint8_t)(sum & 0xFF) == expected_chk) {
					/* Checksum OK. Check if ID matches. */
					uint8_t rx_id = data->rx_buf[0];

					if (rx_id == cfg->id) {
						/* Copy payload (excluding ID and checksum) */
						data->reply_len = data->rx_len;
						memcpy(data->reply_buf, &data->rx_buf[1],
						       data->rx_len);
						/* Parse and update status cache */
						la_parse_generic_response(data->reply_buf,
									  data->reply_len,
									  &data->current_status);
						/* Wake waiting thread */
						k_sem_give(&data->reply_sem);
					}
					/* If ID doesn't match, another device on the bus
					 * responded; we ignore it.
					 */
				}
				/* Reset state machine for next frame */
				data->rx_state = LA_RX_WAIT_HEADER1;
			}
			break;

		} /* switch */
	} /* while */
}

/*===========================================================================*
 * Internal: send frame and wait for reply (with retry)
 *===========================================================================*/

/**
 * @brief Pack, send a command frame, and wait for a reply.
 *
 * @param dev        LA device
 * @param cmd        Command byte
 * @param index      Control table index / param1
 * @param data       Optional data bytes to append after index
 * @param data_len   Length of data
 * @param timeout_ms Timeout per attempt in milliseconds
 * @param max_retries Maximum number of retries (0 = no retry)
 *
 * @return 0 on success, -ETIMEDOUT on all retries exhausted, -EIO on error.
 */
static int la_send_and_wait(const struct device *dev, uint8_t cmd, uint8_t index,
			    const uint8_t *data, uint8_t data_len, uint16_t timeout_ms,
			    uint8_t max_retries)
{
	struct la_yinshi_data *data_d = dev->data;
	const struct la_yinshi_config *cfg = dev->config;
	uint8_t frame[LA_MAX_FRAME_LEN];
	uint8_t frame_len;
	int ret;
	uint8_t attempt;

	frame_len = la_pack_command(cfg->id, cmd, index, data, data_len, frame);

	for (attempt = 0; attempt <= max_retries; attempt++) {
		/* Reset RX state and semaphore before each attempt */
		data_d->rx_state = LA_RX_WAIT_HEADER1;
		k_sem_reset(&data_d->reply_sem);
		data_d->reply_len = 0;

		/* RS485: assert DE (driver enable) for TX */
		if (cfg->de_gpio.port != NULL) {
			gpio_pin_set_dt(&cfg->de_gpio, 1);
			/* Small settle delay */
			k_busy_wait(20);
		}

		/* Send frame via UART */
		for (uint8_t i = 0; i < frame_len; i++) {
			uart_poll_out(cfg->uart_dev, frame[i]);
		}

		/* RS485: wait for TX shift register to empty, then switch to RX */
		if (cfg->de_gpio.port != NULL) {
			/* Wait for transmission: worst case ~11 bits/byte at slowest
			 * baud. Use generous 50us per byte as lower bound.
			 * For 115200 bps, 1 byte ≈ 87us. So 50us/byte × 2x margin.
			 */
			uint32_t tx_us = (uint32_t)frame_len * 120;
			k_busy_wait(tx_us);
			gpio_pin_set_dt(&cfg->de_gpio, 0);
		}

		/* Wait for reply */
		ret = k_sem_take(&data_d->reply_sem, K_MSEC(timeout_ms));
		if (ret == 0) {
			/* Got a valid reply */
			return 0;
		}

		/* Timeout — will retry if attempts remain */
		LOG_DBG("%s: attempt %d/%d timeout (cmd=0x%02x idx=0x%02x)", dev->name, attempt + 1,
			max_retries + 1, cmd, index);

		/* Reset RX state machine after timeout (may be stuck) */
		data_d->rx_state = LA_RX_WAIT_HEADER1;
	}

	/* All retries exhausted */
	data_d->current_status.online = false;
	LOG_WRN("%s: no reply after %d attempts (cmd=0x%02x idx=0x%02x)", dev->name,
		max_retries + 1, cmd, index);
	return -ETIMEDOUT;
}

/*===========================================================================*
 * Driver API implementation
 *===========================================================================*/

int la_yinshi_set_position(const struct device *dev, uint16_t position)
{
	struct la_yinshi_data *data = dev->data;
	const struct la_yinshi_config *cfg = dev->config;
	uint8_t pos_data[2];
	int ret;

	if (position > 2000) {
		return -EINVAL;
	}

	/* Pack position as little-endian */
	sys_put_le16(position, pos_data);

	k_mutex_lock(&data->lock, K_FOREVER);

	/* Position mode with feedback: CMD=0x21, Index=0x37 */
	ret = la_send_and_wait(dev, LA_CMD_WR_DRV_POS, LA_INDEX_TARGET_POS, pos_data,
			       sizeof(pos_data), cfg->reply_timeout_ms, cfg->max_retries);

	k_mutex_unlock(&data->lock);
	return ret;
}

int la_yinshi_get_status(const struct device *dev, struct la_status *status)
{
	struct la_yinshi_data *data = dev->data;
	const struct la_yinshi_config *cfg = dev->config;
	const uint8_t mc_data = LA_MC_QUERY_STATUS;
	int ret;

	if (status == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	/* Query status: CMD=0x04, Index=0x00, Data=0x22 */
	ret = la_send_and_wait(dev, LA_CMD_MC, 0x00, &mc_data, 1, cfg->reply_timeout_ms, 0);

	if (ret == 0) {
		memcpy(status, &data->current_status, sizeof(*status));
	} else {
		data->current_status.online = false;
		memcpy(status, &data->current_status, sizeof(*status));
	}

	k_mutex_unlock(&data->lock);
	return ret;
}

int la_yinshi_cmd(const struct device *dev, enum la_cmd cmd)
{
	struct la_yinshi_data *data = dev->data;
	const struct la_yinshi_config *cfg = dev->config;
	uint8_t mc_data;
	int ret;

	switch (cmd) {
	case LA_CMD_ENABLE:
		mc_data = LA_MC_WORK;
		break;
	case LA_CMD_DISABLE:
		mc_data = LA_MC_EMERGENCY_STOP;
		break;
	case LA_CMD_PAUSE:
		mc_data = LA_MC_PAUSE;
		break;
	case LA_CMD_CLEAR_FAULT:
		mc_data = LA_MC_CLEAR_FAULT;
		break;
	case LA_CMD_SAVE_PARAMS:
		mc_data = LA_MC_SAVE_PARAMS;
		break;
	default:
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	/* All MC commands: CMD=0x04, Index=0x00 */
	ret = la_send_and_wait(dev, LA_CMD_MC, 0x00, &mc_data, 1, cfg->reply_timeout_ms, 0);

	k_mutex_unlock(&data->lock);
	return ret;
}

int la_yinshi_set_param(const struct device *dev, uint8_t index, uint16_t value)
{
	struct la_yinshi_data *data = dev->data;
	const struct la_yinshi_config *cfg = dev->config;
	uint8_t val_data[2];
	int ret;

	sys_put_le16(value, val_data);

	k_mutex_lock(&data->lock, K_FOREVER);

	/* Write control table: CMD=0x02 */
	ret = la_send_and_wait(dev, LA_CMD_WR, index, val_data, sizeof(val_data),
			       cfg->reply_timeout_ms, 0);

	k_mutex_unlock(&data->lock);
	return ret;
}

/*===========================================================================*
 * Driver initialization
 *===========================================================================*/

int la_yinshi_init(const struct device *dev)
{
	const struct la_yinshi_config *cfg = dev->config;
	struct la_yinshi_data *data = dev->data;
	int ret;

	/* Validate UART device */
	if (!device_is_ready(cfg->uart_dev)) {
		LOG_ERR("%s: UART device not ready", dev->name);
		return -ENODEV;
	}

	/* Configure RS485 DE GPIO if present */
	if (cfg->de_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&cfg->de_gpio)) {
			LOG_ERR("%s: DE GPIO not ready", dev->name);
			return -ENODEV;
		}
		ret = gpio_pin_configure_dt(&cfg->de_gpio, GPIO_OUTPUT_INACTIVE);
		if (ret < 0) {
			LOG_ERR("%s: DE GPIO config failed (%d)", dev->name, ret);
			return ret;
		}
		/* Ensure DE is low (RX mode) */
		gpio_pin_set_dt(&cfg->de_gpio, 0);
	}

	/* Initialize mutex for serializing API calls */
	k_mutex_init(&data->lock);

	/* Initialize semaphore (binary, initial count = 0) */
	k_sem_init(&data->reply_sem, 0, 1);

	/* Set up UART RX interrupt */
	uart_irq_callback_user_data_set(cfg->uart_dev, la_uart_isr, (void *)dev);
	uart_irq_rx_enable(cfg->uart_dev);

	LOG_INF("%s: initialized (id=%d, uart=%s, rs485=%s)", dev->name, cfg->id,
		cfg->uart_dev->name, cfg->de_gpio.port ? "yes" : "no");

	return 0;
}

/*===========================================================================*
 * Instantiate devices from devicetree
 *===========================================================================*/

DT_INST_FOREACH_STATUS_OKAY(LA_YINSHI_DEFINE_INST)
