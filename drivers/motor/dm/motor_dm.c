/*
 * Copyright (c) 2024 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <sys/_stdint.h>
#include "../common/common.h"
#include "motor_dm.h"
#include "syscalls/kernel.h"
#include "zephyr/drivers/can.h"
#include "zephyr/drivers/motor.h"
#include "zephyr/drivers/pid.h"
#include "zephyr/kernel.h"

#define DT_DRV_COMPAT dm_motor

LOG_MODULE_REGISTER(motor_dm, CONFIG_MOTOR_LOG_LEVEL);

/**
 * @brief Converts an unsigned integer to a float, given range and number of bits.
 *
 * This function takes an unsigned integer and maps it to a float value within
 * the specified range [x_min, x_max] based on the number of bits.
 *
 * @param x_int The unsigned integer value to be converted.
 * @param x_min The minimum value of the target float range.
 * @param x_max The maximum value of the target float range.
 * @param bits The number of bits representing the unsigned integer.
 * @return The corresponding float value within the range [x_min, x_max].
 */

static inline float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/// converts unsigned int to float, given range and number of bits ///
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/**
 * @brief Converts a float to an unsigned int, given range and number of bits.
 *
 * This function takes a floating-point number and converts it to an unsigned
 * integer representation based on the specified range and number of bits.
 *
 * @param x The floating-point number to convert.
 * @param x_min The minimum value of the range.
 * @param x_max The maximum value of the range.
 * @param bits The number of bits for the unsigned integer representation.
 *
 * @return The unsigned integer representation of the floating-point number.
 */
static inline int float_to_uint(float x, float x_min, float x_max, int bits)
{
	/// Converts a float to an unsigned int, given range and number of bits
	float span = x_max - x_min;
	float offset = x_min;
	return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

int dm_init(const struct device *dev)
{
	const struct dm_motor_config *cfg = dev->config;

	if (!device_is_ready(cfg->common.phy)) {
		return -1;
	}
	if (k_work_busy_get(&dm_init_work) != 0) {
		return 0;
	}
	k_work_queue_init(&dm_work_queue);

	dm_tx_timer.expiry_fn = dm_isr_init_handler;
	k_timer_start(&dm_tx_timer, K_MSEC(500), K_MSEC(7));
	k_timer_user_data_set(&dm_tx_timer, &dm_init_work);
	return 0;
}

void dm_control(const struct device *dev, enum motor_cmd cmd)
{
	struct dm_motor_data *data = dev->data;
	const struct dm_motor_config *cfg = dev->config;

	struct can_frame frame;
	frame.id = cfg->common.tx_id + data->tx_offset;
	frame.flags = 0;
	frame.dlc = 8;

	int err = 0;

	switch (cmd) {
	case ENABLE_MOTOR:
		memcpy(frame.data, enable_frame, 8);
		can_send_queued(cfg->common.phy, &frame);
		data->enable = true;
		break;
	case DISABLE_MOTOR:
		memcpy(frame.data, disable_frame, 8);
		can_send_queued(cfg->common.phy, &frame);
		data->enable = false;
		break;
	case SET_ZERO:
		memcpy(frame.data, set_zero_frame, 8);
		break;
	case CLEAR_PID:
		memset(&data->params, 0, sizeof(data->params));
		break;
	case CLEAR_ERROR:
		memcpy(frame.data, clear_error_frame, 8);
		break;
	}
	if (err != 0) {
		LOG_ERR("Failed to send CAN frame: %d", err);
	}
}

static void dm_motor_pack(const struct device *dev, struct can_frame *frame)
{
	uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
	uint8_t *pbuf, *vbuf;
	// float hybrid_p_tmp;
	struct dm_motor_data *data = dev->data;
	const struct dm_motor_config *cfg = dev->config;

	frame->id = cfg->common.tx_id + data->tx_offset;
	frame->dlc = 8;
	frame->flags = 0;
	switch (data->common.mode) {
	case MIT:
		pos_tmp = float_to_uint(data->target_angle, -cfg->p_max, cfg->p_max, 16);
		vel_tmp = float_to_uint(data->target_radps, -cfg->v_max, cfg->v_max, 12);
		tor_tmp = float_to_uint(data->target_torque, -cfg->t_max, cfg->t_max, 12);
		kp_tmp = float_to_uint(data->params.k_p, 0, 500, 12);
		kd_tmp = float_to_uint(data->params.k_d, 0, 5, 12);

		frame->data[0] = (pos_tmp >> 8);
		frame->data[1] = pos_tmp;
		frame->data[2] = (vel_tmp >> 4);
		frame->data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
		frame->data[4] = kp_tmp;
		frame->data[5] = (kd_tmp >> 4);
		frame->data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
		frame->data[7] = tor_tmp;
		break;
	case PV:
		pbuf = (uint8_t *)&data->target_angle;
		vbuf = (uint8_t *)&data->target_radps;

		memcpy(frame->data, pbuf, 4);
		memcpy(frame->data + 4, vbuf, 4);
		break;
	case VO:
		vbuf = (uint8_t *)&data->target_radps;

		memcpy(frame->data, vbuf, 4);
		break;
	case HYBRID:
		// hybrid_p_tmp = data->target_angle * DEG2RAD;
		// memcpy(frame->data, &hybrid_p_tmp, 4);
		// vel_tmp = data->target_radps * 100;
		// frame->data[4] = (vel_tmp >> 8);
		// frame->data[5] = vel_tmp;
		break;
	default:
		break;
	}
}

int dm_get(const struct device *dev, motor_status_t *status)
{
	struct dm_motor_data *data = dev->data;
	const struct dm_motor_config *cfg = dev->config;

	status->angle = data->common.angle;
	status->rpm = data->common.rpm;
	status->torque = data->common.torque;

	status->mode = data->common.mode;
	status->sum_angle = data->delta_deg_sum;
	status->speed_limit[0] = cfg->v_max;
	status->speed_limit[1] = cfg->v_max;
	status->torque_limit[0] = cfg->t_max;
	status->torque_limit[1] = cfg->t_max;
	return 0;
}

static void dm_rx_handler(const struct device *can_dev, struct can_frame *frame, void *user_data)
{
	const struct device *dev = user_data;
	struct dm_motor_data *data = dev->data;

	data->prev_recv_time = k_uptime_get();

	data->err = frame->data[0] >> 4;
	data->enabled = data->err & 1;
	data->RAWangle = (frame->data[1] << 8) | (frame->data[2]);
	data->RAWrpm = (frame->data[3] << 4) | (frame->data[4] >> 4);
	data->RAWtorque = (frame->data[4] & 0xF) << 8;
	data->update = true;

	if (data->enable && !data->online) {
		// LOG_ERR("motor %s is back online", dev->name);
	}
	data->online = true;

	k_work_submit_to_queue(&dm_work_queue, &dm_rx_data_handle);
}

static void dm_edit_reg_value(const struct device *dev, uint16_t can_id, uint8_t reg_addr,
			      uint32_t reg_value)
{
	struct can_frame frame;
	frame.id = 0x7FF;
	frame.dlc = 8;
	frame.flags = 0;
	frame.data[CANID_L] = can_id & 0xFF;
	frame.data[CANID_H] = can_id >> 8;
	frame.data[2] = 0x55;
	frame.data[RID] = reg_addr;
	frame.data[4] = reg_value & 0xFF;
	frame.data[5] = reg_value >> 8;
	frame.data[6] = reg_value >> 16;
	frame.data[7] = reg_value >> 24;
	can_send_queued(dev, &frame);
}

void dm_motor_set_mode(const struct device *dev, enum motor_mode mode)
{
	struct dm_motor_data *data = dev->data;
	const struct dm_motor_config *cfg = dev->config;
	char mode_str[10];

	data->common.mode = mode;

	switch (mode) {
	case MIT:
		snprintf(mode_str, sizeof(mode_str), "mit");
		data->tx_offset = 0x0;
		dm_edit_reg_value(cfg->common.phy, cfg->common.tx_id, 0x0A, 0x01);
		break;
	case PV:
		snprintf(mode_str, sizeof(mode_str), "pv");
		data->tx_offset = 0x100;
		dm_edit_reg_value(cfg->common.phy, cfg->common.tx_id, 0x0A, 0x02);
		break;
	case VO:
		snprintf(mode_str, sizeof(mode_str), "vo");
		data->tx_offset = 0x200;
		dm_edit_reg_value(cfg->common.phy, cfg->common.tx_id, 0x0A, 0x03);
		break;
	case HYBRID:
		snprintf(mode_str, sizeof(mode_str), "hybrid");
		data->tx_offset = 0x300;
		dm_edit_reg_value(cfg->common.phy, cfg->common.tx_id, 0x0A, 0x04);
		break;
	default:
		data->online = false;
		dm_control(dev, DISABLE_MOTOR);
	}

	bool found = false;
	for (int i = 0; i < SIZE_OF_ARRAY(cfg->common.capabilities); i++) {
		if (cfg->common.pid_datas[i]->pid_dev == NULL) {
			break;
		}
		if (strcmp(cfg->common.capabilities[i], mode_str) == 0) {
			struct pid_config params = {0};
			pid_get_params(cfg->common.pid_datas[i], &params);

			data->common.mode = mode;
			data->params.k_p = params.k_p;
			data->params.k_d = params.k_d;
			found = true;
			break;
		}
	}
	if (!found) {
		LOG_ERR("Mode %s not found", mode_str);
		if (mode != VO && mode != HYBRID) {
			dm_control(dev, DISABLE_MOTOR);
			data->enable = false;
		}
	} else if (mode == VO) {
		union {
			float f;
			uint32_t u;
		} conv;
		conv.f = data->params.k_p;
		dm_edit_reg_value(cfg->common.phy, cfg->common.tx_id, 0x19, conv.u);
		conv.f = data->params.k_i;
		dm_edit_reg_value(cfg->common.phy, cfg->common.tx_id, 0x1A, conv.u);
	} else if (mode == PV) {
		LOG_ERR("PV mode params setting not supported");
	}
}

int dm_set(const struct device *dev, motor_status_t *status)
{
	struct dm_motor_data *data = dev->data;
	const struct dm_motor_config *cfg = dev->config;

	if (status->mode == MIT) {
		data->target_angle = status->angle/(RAD2DEG);
		data->target_radps = RPM2RADPS(status->rpm);
		data->target_torque = status->torque;
		// data->params.k_p = 0;
		// data->params.k_d = 0;
	} else if (status->mode == PV) {
		data->target_angle = status->angle;
		data->target_radps = RPM2RADPS(status->rpm);
	} else if (status->mode == VO) {
		data->target_radps = RPM2RADPS(status->rpm);
		data->target_angle = 0;
		data->target_torque = 0;
		data->params.k_p = 0;
		data->params.k_d = 0;
	} else {
		return -ENOSYS;
	}

	struct can_frame tx_frame;
	dm_motor_pack(dev, &tx_frame);
	can_send_queued(cfg->common.phy, &tx_frame);
	data->last_tx_time = k_uptime_get();

	return 0;
}

void dm_rx_data_handler(struct k_work *work)
{
	for (int i = 0; i < MOTOR_COUNT; i++) {
		struct dm_motor_data *data = (struct dm_motor_data *)(motor_devices[i]->data);
		if (!data->update) {
			continue;
		}
		const struct dm_motor_config *cfg =
			(const struct dm_motor_config *)(motor_devices[i]->config);

		float prev_angle = data->common.angle;
		data->common.angle =
			(uint_to_float(data->RAWangle, -cfg->p_max, cfg->p_max, 16))*RAD2DEG;
		data->common.rpm =
			RADPS2RPM(uint_to_float(data->RAWrpm, -cfg->v_max, cfg->v_max, 12));
		data->common.torque = uint_to_float(data->RAWtorque, -cfg->t_max, cfg->t_max, 12);

		data->delta_deg_sum += data->common.angle - prev_angle;

		data->update = false;
	}
}

void dm_tx_isr_handler(struct k_timer *dummy)
{
	k_work_submit_to_queue(&dm_work_queue, &dm_tx_data_handle);
}

void dm_isr_init_handler(struct k_timer *dummy)
{
	dummy->expiry_fn = dm_tx_isr_handler;
	k_work_queue_start(&dm_work_queue, dm_work_queue_stack, CAN_SEND_STACK_SIZE,
			   CAN_SEND_PRIORITY, NULL);
	k_work_submit_to_queue(&dm_work_queue, &dm_init_work);
}

void dm_tx_data_handler(struct k_work *work)
{
	struct can_frame tx_frame;

	uint64_t now = k_uptime_get();

	for (int i = 0; i < MOTOR_COUNT; i++) {
		struct dm_motor_data *data = motor_devices[i]->data;
		const struct dm_motor_config *cfg = motor_devices[i]->config;

		if ((data->online && data->enable) && data->tx_cnt >= 3) {
			if (!data->enabled) {
				dm_control(motor_devices[i], ENABLE_MOTOR);
			}
			data->tx_cnt = 0;
			if (data->err > 1) {
				dm_control(motor_devices[i], CLEAR_ERROR);
			}
		}
		if (now - data->last_tx_time >= 1000 / cfg->freq) {
			dm_motor_pack(motor_devices[i], &tx_frame);
			can_send_queued(cfg->common.phy, &tx_frame);
			data->last_tx_time = now;
			data->tx_cnt++;
		}
		if (now - data->prev_recv_time > 10000 / cfg->freq && data->online &&
		    data->enable) {
			// LOG_ERR("motor %s is not responding, setting it to offline",
			// 	motor_devices[i]->name);
			data->online = false;
			data->enabled = false;
		}
		k_sleep(K_USEC(130));
	}
}

void dm_init_handler(struct k_work *work)
{
	k_timer_stop(&dm_tx_timer);
	LOG_DBG("DM motor control thread started");

	for (int i = 0; i < MOTOR_COUNT; i++) {
		struct dm_motor_data *data = motor_devices[i]->data;
		const struct dm_motor_config *cfg =
			(const struct dm_motor_config *)(motor_devices[i]->config);

		reg_can_dev(cfg->common.phy);

		data->filter.id = cfg->common.rx_id & 0xFF;
		data->filter.mask = 0x7FF;

		int err = can_add_rx_filter(cfg->common.phy, dm_rx_handler,
					    (void *)motor_devices[i], &data->filter);
		if (err < 0) {
			LOG_ERR("Error adding CAN filter (err %d)", err);
		}
	}

	k_sleep(K_MSEC(500));

	for (int i = 0; i < MOTOR_COUNT; i++) {
		dm_control(motor_devices[i], ENABLE_MOTOR);
		struct dm_motor_data *data = motor_devices[i]->data;
		data->prev_recv_time = k_uptime_get();
	}

	k_timer_start(&dm_tx_timer, K_NO_WAIT, K_MSEC(9));
	k_timer_user_data_set(&dm_tx_timer, &dm_tx_data_handle);
}

DT_INST_FOREACH_STATUS_OKAY(DMMOTOR_INST)
