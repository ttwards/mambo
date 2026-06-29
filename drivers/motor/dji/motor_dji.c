/*
 * Copyright (c) 2024 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "motor_dji.h"
#include "syscalls/can.h"
#include "zephyr/device.h"
#include "zephyr/devicetree.h"
#include "zephyr/drivers/can.h"
#include "zephyr/drivers/motor.h"
#include "zephyr/spinlock.h"
#include "zephyr/sys/util.h"
#include "zephyr/toolchain.h"
#include <string.h>
#include <math.h>
#include <soc.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <sys/_types.h>
#include <sys/types.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/time_units.h>
#include "../common/common.h"

#define DT_DRV_COMPAT dji_motor

LOG_MODULE_REGISTER(motor_dji, CONFIG_MOTOR_LOG_LEVEL);

const struct device *motor_devices[] = {DT_INST_FOREACH_STATUS_OKAY(DJI_DEVICE_POINTER)};

#define CTRL_STRUCT_DATA(i, _)                                                                     \
	{                                                                                          \
		.can_dev = NULL,                                                                   \
		.flags = 0,                                                                        \
		.full = {false},                                                                   \
		.mask = {0},                                                                       \
		.mapping =                                                                         \
			{                                                                          \
				{-1, -1, -1, -1},                                                  \
				{-1, -1, -1, -1},                                                  \
				{-1, -1, -1, -1},                                                  \
				{-1, -1, -1, -1},                                                  \
				{-1, -1, -1, -1},                                                  \
				{-1, -1, -1, -1},                                                  \
				{-1, -1, -1, -1},                                                  \
				{-1, -1, -1, -1},                                                  \
			},                                                                         \
	}

static int frameID_to_index(int tx_id)
{
	if (tx_id == 0x200) {
		return 0;
	} else if (tx_id == 0x1FF) {
		return 1;
	} else if (tx_id == 0x1FE) {
		return 2;
	} else if (tx_id == 0x2FE) {
		return 3;
	} else if (tx_id == 0x2FF) {
		return 4;
	} else if (tx_id == 0x300) {
		return 5;
	} else if (tx_id == 0x3FE) {
		return 6;
	} else if (tx_id == 0x4FE) {
		return 7;
	}
	return -1; // Return a default value if no match is found
}

static int index_to_frameID(int frames_id)
{
	if (frames_id == 0) {
		return 0x200;
	} else if (frames_id == 1) {
		return 0x1FF;
	} else if (frames_id == 2) {
		return 0x1FE;
	} else if (frames_id == 3) {
		return 0x2FE;
	} else if (frames_id == 4) {
		return 0x2FF;
	} else if (frames_id == 5) {
		return 0x300;
	} else if (frames_id == 6) {
		return 0x3FE;
	} else if (frames_id == 7) {
		return 0x4FE;
	}
	return -1;
}

/**
 * @brief 将 float 类型转换为 int16_t，包含溢出处理和四舍五入。
 *
 * @param value 要转换的浮点数。
 * @return 转换后的 int16_t 值。
 */
static int16_t to16t(float value)
{
	// 溢出处理
	if (value > INT16_MAX) {
		return INT16_MAX;
	} else if (value < INT16_MIN) {
		return INT16_MIN;
	} else {
		return (int16_t)value;
	}
}

struct motor_controller ctrl_structs[CONFIG_CAN_COUNT] = {
	LISTIFY(CONFIG_CAN_COUNT, CTRL_STRUCT_DATA, (, ))};

static inline motor_id_t motor_id(const struct device *dev)
{
	const struct dji_motor_config *cfg = dev->config;
	return cfg->common.id - 1;
}

void dji_speed_limit(const struct device *dev, float max_speed, float min_speed)
{
	struct dji_motor_data *data = dev->data;
	data->common.speed_limit[0] = min_speed;
	data->common.speed_limit[1] = max_speed;
}

void dji_torque_limit(const struct device *dev, float max_torque, float min_torque)
{
	struct dji_motor_data *data = dev->data;
	data->common.torque_limit[0] = min_torque;
	data->common.torque_limit[1] = max_torque;
}

static float dji_limit_torque(const struct dji_motor_data *data, float torque)
{
	if (torque > data->common.torque_limit[1]) {
		return data->common.torque_limit[1];
	}
	if (torque < data->common.torque_limit[0]) {
		return data->common.torque_limit[0];
	}
	return torque;
}

int dji_set_speed(const struct device *dev, float speed_rpm)
{
	struct dji_motor_data *data = dev->data;

	if (speed_rpm > data->common.speed_limit[1]) {
		speed_rpm = data->common.speed_limit[1];
	} else if (speed_rpm < data->common.speed_limit[0]) {
		speed_rpm = data->common.speed_limit[0];
	}

	data->target_rpm = speed_rpm;

	return 0;
}

int dji_set_angle(const struct device *dev, float angle)
{
	struct dji_motor_data *data = dev->data;
	data->target_angle = angle;
	return 0;
}

int dji_set_torque(const struct device *dev, float torque)
{
	struct dji_motor_data *data = dev->data;

	data->target_torque = dji_limit_torque(data, torque);

	return 0;
}

int dji_set_mode(const struct device *dev, enum motor_mode mode)
{
	struct dji_motor_data *data = dev->data;
	const struct dji_motor_config *cfg = dev->config;

	char mode_str[10];
	switch (mode) {
	case ML_TORQUE:
		strcpy(mode_str, "torque");
		break;
	case ML_ANGLE:
		strcpy(mode_str, "angle");
		break;
	case ML_SPEED:
		strcpy(mode_str, "speed");
		break;
	case MIT:
		strcpy(mode_str, "mit");
		break;
	default:
		LOG_ERR("Unsupported motor mode: %d", mode);
		return -ENOSYS;
	}

	data->current_mode_index = -1;

	for (int i = 0; i < SIZE_OF_ARRAY(cfg->common.pid_datas); i++) {
		if ((cfg->common.pid_datas[i] == NULL ||
		     cfg->common.pid_datas[i]->pid_dev == NULL) &&
		    mode == ML_TORQUE) {
			data->current_mode_index = i;
			break;
		}
		if (cfg->common.pid_datas[i] == NULL) {
			break;
		}
		if (strcmp(cfg->common.capabilities[i], mode_str) == 0) {
			pid_calc(cfg->common.pid_datas[i]);
			data->current_mode_index = i;
			break;
		}
	}

	if (data->current_mode_index == -1) {
		LOG_ERR("No motor mode found for %s", mode_str);
		return -ENOSYS;
	}

	data->common.mode = mode;

	return 0;
}

int dji_set(const struct device *dev, motor_status_t *status)
{
	struct dji_motor_data *data = dev->data;

	if (status->mode == ML_TORQUE) {
		dji_set_torque(dev, status->torque);
	} else if (status->mode == ML_ANGLE) {
		dji_set_angle(dev, status->angle);
	} else if (status->mode == ML_SPEED) {
		dji_set_speed(dev, status->rpm);
	} else if (status->mode == MIT) {
		dji_set_angle(dev, status->angle);
		dji_set_speed(dev, status->rpm);
		data->target_torque_ff = dji_limit_torque(data, status->torque);
	} else {
		goto limits_set;
	}

	dji_set_mode(dev, status->mode);
	data->common.mode = status->mode;

limits_set:
	if (status->speed_limit[0] < 0 || status->speed_limit[1] > 0) {
		dji_speed_limit(dev, status->speed_limit[1], status->speed_limit[0]);
	}
	if (status->torque_limit[0] < 0 || status->torque_limit[1] > 0) {
		dji_torque_limit(dev, status->torque_limit[1], status->torque_limit[0]);
	}

	return 0;
}

void dji_control(const struct device *dev, enum motor_cmd cmd)
{
	struct dji_motor_data *data = dev->data;
	const struct dji_motor_config *cfg = dev->config;

	struct can_frame frame;
	frame.id = cfg->common.tx_id;

	switch (cmd) {
	case ENABLE_MOTOR:
		data->online = true;
		break;
	case DISABLE_MOTOR:
		data->online = false;
		break;
	case SET_ZERO:
		data->angle_add = 0;
		data->angle_offset = data->common.angle;
		if (cfg->is_dm_motor) {
			frame.id = 0x7FF;
			frame.flags = 0;
			frame.dlc = 4;
			frame.data[0] = (cfg->common.rx_id - 0x200) & 0xFF;
			frame.data[1] = (cfg->common.rx_id - 0x200) >> 8;
			frame.data[2] = 0x55;
			frame.data[3] = 0x3C;
			can_send_queued(cfg->common.phy, &frame);
		}
		break;
	case CLEAR_PID:
		break;
	case CLEAR_ERROR:
		data->missed_times = 0;
		data->online = true;
		if (cfg->is_dm_motor) {
			frame.id = 0x7FF;
			frame.flags = 0;
			frame.dlc = 4;
			frame.data[0] = (cfg->common.rx_id - 0x200) & 0xFF;
			frame.data[1] = (cfg->common.rx_id - 0x200) >> 8;
			frame.data[2] = 0x55;
			frame.data[3] = 0x50;
			can_send_queued(cfg->common.phy, &frame);
		}
		break;
	}
}

int dji_get(const struct device *dev, motor_status_t *status)
{
	struct dji_motor_data *data = dev->data;
	const struct dji_motor_config *cfg = dev->config;

	if (strcmp(cfg->common.capabilities[data->current_mode_index], "torque") == 0) {
		status->mode = ML_TORQUE;
	} else if (strcmp(cfg->common.capabilities[data->current_mode_index], "angle") == 0) {
		status->mode = ML_ANGLE;
	} else if (strcmp(cfg->common.capabilities[data->current_mode_index], "speed") == 0) {
		status->mode = ML_SPEED;
	} else if (strcmp(cfg->common.capabilities[data->current_mode_index], "mit") == 0) {
		status->mode = MIT;
	}

	memcpy(status, &data->common, sizeof(motor_status_t));

	return 0;
}

int dji_init(const struct device *dev)
{
	/*
	    Each motor find their own controller and write their own data,
	    which cannot be done at the first init because the other motors have not
	    been initialized.
	*/
	if (dev) {
		const struct dji_motor_config *cfg = dev->config;
		struct dji_motor_data *data = dev->data;
		data->canbus_id = reg_can_dev(cfg->common.phy);
		data->ctrl_struct = &ctrl_structs[data->canbus_id];
		data->ctrl_struct->can_dev = (struct device *)cfg->common.phy;
		uint8_t frame_id = frameID_to_index(cfg->common.tx_id);
		uint8_t id = motor_id(dev);
		const struct dji_motor_config *follow_cfg =
			cfg->follow ? (const struct dji_motor_config *)cfg->follow->config : NULL;
		if (follow_cfg && cfg->common.phy == follow_cfg->common.phy) {
			const struct device *follow_dev = cfg->follow;
			data->ctrl_struct->mask[frame_id] |= 1 << motor_id(follow_dev);
		} else {
			data->ctrl_struct->mask[frame_id] |= 1 << id;
		}
		if (data->ctrl_struct->rx_ids[id]) {
			LOG_ERR("Conflicting motor id: %d, dev name: %s", id + 1, dev->name);
		}
		data->ctrl_struct->rx_ids[id] = cfg->common.rx_id;

		data->ctrl_struct->full_handle.handler = dji_tx_handler;

		data->online = true;
		data->pid_count = SIZE_OF_ARRAY(cfg->common.pid_datas);
		data->current_mode_index = data->pid_count;
		for (int i = 0;
		     i < sizeof(cfg->common.pid_datas) / sizeof(cfg->common.pid_datas[0]); i++) {
			if (cfg->common.pid_datas[i] == NULL) {
				if (i > 0) {
					pid_reg_output(cfg->common.pid_datas[i - 1],
						       &data->target_torque);
				}
				data->pid_count = i;
				data->current_mode_index = i;
				break;
			}
			if (strcmp(cfg->common.capabilities[i], "speed") == 0) {
				pid_reg_input(cfg->common.pid_datas[i], &data->common.rpm,
					      &data->target_rpm);
				if (i > 0) {
					pid_reg_output(cfg->common.pid_datas[i - 1],
						       &data->target_rpm);
				}
			} else if (strcmp(cfg->common.capabilities[i], "angle") == 0) {
				/* pid_calc_in: 直接传 angle error，无需注册 curr/ref */
				if (i > 0) {
					pid_reg_output(cfg->common.pid_datas[i - 1],
						       &data->target_rpm);
				}
			} else if (strcmp(cfg->common.capabilities[i], "torque") == 0) {
				pid_reg_input(cfg->common.pid_datas[i], &data->common.torque,
					      &data->target_torque);
				if (i > 0) {
					pid_reg_output(cfg->common.pid_datas[i - 1],
						       &data->target_torque);
				}
			} else if (strcmp(cfg->common.capabilities[i], "mit") == 0) {
				/* pid_calc_mit: 直接传 pos/vel error，无需注册 curr/ref/detri */
				if (i > 0) {
					pid_reg_output(cfg->common.pid_datas[i - 1],
						       &data->target_torque);
				}
			} else {
				LOG_ERR("Unsupported motor mode: %s", cfg->common.capabilities[i]);
				return -1;
			}
			pid_reg_time(cfg->common.pid_datas[i], &(data->curr_time),
				     &(data->prev_time));
		}
		data->ctrl_struct->motor_devs[id] = (struct device *)dev;
		data->prev_time = 0;
		data->ctrl_struct->flags = 0;
		data->ctrl_struct->mapping[frame_id][id % 4] = id;
		if (cfg->is_gm6020) {
			data->convert_num = GM6020_CONVERT_NUM;
		} else if (cfg->is_m3508) {
			data->convert_num = M3508_CONVERT_NUM;
		} else if (cfg->is_m2006) {
			data->convert_num = M2006_CONVERT_NUM;
		} else if (cfg->is_dm_motor) {
			data->convert_num = DM_MOTOR_CONVERT_NUM;
		} else {
			LOG_ERR("Unsupported motor type");
		}

		if (!device_is_ready(cfg->common.phy)) {
			return -1;
		}
		if (dji_miss_handle_timer.expiry_fn == NULL) {
			k_work_queue_init(&dji_work_queue);
			k_tid_t thread = k_work_queue_thread_get(&dji_work_queue);
			k_thread_name_set(thread, "dji_motor_ctrl_thread");
			k_timer_init(&dji_miss_handle_timer, dji_init_isr_handler, NULL);
			k_timer_start(&dji_miss_handle_timer, K_MSEC(100), K_MSEC(4));
		}
	}
	return 0;
}

void can_rx_callback(const struct device *can_dev, struct can_frame *frame, void *user_data)
{
	uint32_t curr_time = k_cycle_get_32();
	struct device *dev = (struct device *)user_data;
	struct can_frame rx_frame = *frame;

	struct dji_motor_data *data = dev->data;
	const struct dji_motor_config *cfg = dev->config;
	uint16_t id = motor_id(dev);

	if (!data) {
		return;
	}

	if (data->missed_times > 3) {
		data->missed_times = 0;
		data->online = true;
		const struct dji_motor_config *motor_cfg =
			(const struct dji_motor_config *)dev->config;
		int8_t frame_id = frameID_to_index(motor_cfg->common.tx_id);
		const struct dji_motor_config *follow_cfg =
			motor_cfg->follow
				? (const struct dji_motor_config *)motor_cfg->follow->config
				: NULL;
		if (follow_cfg && motor_cfg->common.phy == follow_cfg->common.phy) {
			data->ctrl_struct->mask[frame_id] |= 1 << motor_id(motor_cfg->follow);
		} else {
			data->ctrl_struct->mask[frame_id] |= 1 << id;
		}
		LOG_ERR("Motor \"%s\" on canbus \"%s\" is responding again.", dev->name,
			motor_cfg->common.phy->name);
	} else if (data->missed_times > 0) {
		data->missed_times--;
	}

	// Store in RAW data. Process when API is called.
	// Using FPU in ISR is not recommended, since it requires actions on registers
	data->RAWprev_angle = data->RAWangle;
	data->RAWangle = COMBINE_HL8(rx_frame.data[0], rx_frame.data[1]);
	int delta = data->RAWangle - data->RAWprev_angle;
	if (data->RAWangle < 2048 && data->RAWprev_angle > 6144) {
		delta += 8192;
	} else if (data->RAWangle > 6144 && data->RAWprev_angle < 2048) {
		delta -= 8192;
	}
	data->angle_add += delta;
	data->RAWrpm = COMBINE_HL8(rx_frame.data[2], rx_frame.data[3]);
	data->RAWcurrent = COMBINE_HL8(rx_frame.data[4], rx_frame.data[5]);
	data->RAWtemp = rx_frame.data[6];
	data->ctrl_struct->flags |= 1 << id;
	data->prev_time = (data->curr_time == 0) ? (curr_time - 1) : data->curr_time;
	data->curr_time = curr_time;
	data->calculated = false;

	struct dji_motor_config *follow_cfg =
		cfg->follow ? (struct dji_motor_config *)cfg->follow->config : NULL;
	if (follow_cfg && cfg->common.phy == follow_cfg->common.phy) {
		goto exit;
	}

	uint8_t clear_flag = 0u;
	for (int i = 0; i < CAN_TX_ID_CNT; i++) {
		uint8_t combined = data->ctrl_struct->mask[i] & data->ctrl_struct->flags;
		if (combined == data->ctrl_struct->mask[i] && data->ctrl_struct->mask[i]) {
			clear_flag |= data->ctrl_struct->mask[i];
			data->ctrl_struct->full[i] = true;
		}
	}

	data->ctrl_struct->flags &= ~clear_flag;
	if (clear_flag && !k_work_is_pending(&data->ctrl_struct->full_handle)) {
		k_work_submit_to_queue(&dji_work_queue, &data->ctrl_struct->full_handle);
	}

exit:
	return;
}

static void proceed_delta_degree(const struct device *dev)
{
	struct dji_motor_data *data = dev->data;
	const struct dji_motor_config *config = dev->config;

	if (fabsf(config->gear_ratio - 1) > 0.001f) {
		data->common.sum_angle = (float)(data->angle_add) *
					 convert[data->convert_num][ANGLE2DEGREE] /
					 config->gear_ratio;

		data->common.angle = fmodf(data->common.sum_angle, 360.0f);
		if (data->common.angle < 0) {
			data->common.angle += 360.0f;
		}
	} else {
		data->common.sum_angle =
			(float)(data->angle_add) * convert[data->convert_num][ANGLE2DEGREE];
		data->common.angle =
			(float)(data->RAWangle) * convert[data->convert_num][ANGLE2DEGREE] -
			data->angle_offset;
	}

	float delta_angle = data->common.sum_angle - data->target_angle;

	if (config->minor_arc) {
		delta_angle = fmodf(delta_angle, 360.0f);
		if (delta_angle > 180) {
			delta_angle -= 360.0f;
		} else if (delta_angle < -180) {
			delta_angle += 360.0f;
		}
	}

	data->pid_angle_input = delta_angle;
}

static void can_pack_add(uint8_t *data, struct device *motor_dev, uint8_t num)
{
	struct dji_motor_data *motor_data = motor_dev->data;
	const struct dji_motor_config *cfg = motor_dev->config;

	int16_t value = to16t(motor_data->target_current);

	if (!cfg->is_dm_motor) {
		data[num * 2] = HIGH_BYTE(value);
		data[num * 2 + 1] = LOW_BYTE(value);
	} else {
		data[num * 2] = LOW_BYTE(value);
		data[num * 2 + 1] = HIGH_BYTE(value);
	}
}

static void dji_timeout_handle(const struct device *dev, uint32_t curr_time)
{
	struct dji_motor_data *data = (struct dji_motor_data *)dev->data;
	const struct dji_motor_config *cfg = (const struct dji_motor_config *)dev->config;

	if (data->online == false) {
		return;
	}
	uint32_t prev_time = data->curr_time;
	if (k_cyc_to_us_near32(curr_time - prev_time) > 2000 || curr_time - prev_time > 100000) {
		data->missed_times++;
		if (data->missed_times > 3) {
			LOG_ERR("Motor \"%s\" on canbus \"%s\" is not responding", dev->name,
				cfg->common.phy->name);
			const struct dji_motor_config *follow_cfg =
				cfg->follow ? (const struct dji_motor_config *)cfg->follow->config
					    : NULL;
			if (follow_cfg && cfg->common.phy == follow_cfg->common.phy) {
				data->ctrl_struct->mask[frameID_to_index(cfg->common.tx_id)] &=
					~(1 << motor_id(cfg->follow));
			} else {
				data->ctrl_struct->mask[frameID_to_index(cfg->common.tx_id)] &=
					~(1 << motor_id(dev));
			}
			data->online = false;
		}
	}
}

static void motor_calc(const struct device *dev)
{
	struct dji_motor_data *data = dev->data;

	k_spinlock_key_t key;
	if (k_spin_trylock(&data->data_input_lock, &key) != 0) {
		return;
	}
	const struct dji_motor_config *config = dev->config;
	// Proceed the RAW data
	// Add up to avoid circular overflow
	proceed_delta_degree(dev);

	data->common.rpm =
		data->RAWrpm * convert[data->convert_num][SPEED2RPM] / config->gear_ratio;
	if (config->inverse) {
		data->common.rpm = -data->common.rpm;
	}
	if (!config->is_dm_motor) {
		data->common.torque = data->RAWcurrent *
				      convert[data->convert_num][CURRENT2TORQUE] *
				      config->gear_ratio;
	} else {
		data->common.torque = ((float)data->RAWcurrent / 16384.0f) * config->dm_i_max *
				      config->dm_torque_ratio * config->gear_ratio;
	}
	if (config->inverse) {
		data->common.torque = -data->common.torque;
	}

	if (config->follow) {
		const struct device *follow_dev = config->follow;
		struct dji_motor_data *follow_data = follow_dev->data;
		if (!follow_data->calculated) {
			motor_calc(follow_dev);
			follow_data->calculated = true;
			data->target_torque = follow_data->target_torque;
		} else if (follow_data->online) {
			data->target_torque = follow_data->target_torque;
		} else {
			data->target_torque = 0;
		}
		if (config->inverse) {
			data->target_torque = -data->target_torque;
		}
	} else {
		float deltaT_us = (float)k_cyc_to_us_near32(data->curr_time - data->prev_time);
		for (int i = data->current_mode_index; i < data->pid_count; i++) {
			if (strcmp(config->common.capabilities[i], "angle") == 0) {
				data->target_rpm = pid_calc_in(config->common.pid_datas[i],
							       -data->pid_angle_input, deltaT_us);
				if (data->target_rpm > data->common.speed_limit[1]) {
					data->target_rpm = data->common.speed_limit[1];
				} else if (data->target_rpm < data->common.speed_limit[0]) {
					data->target_rpm = data->common.speed_limit[0];
				}
			} else if (strcmp(config->common.capabilities[i], "mit") == 0) {
				float vel_error = data->target_rpm - data->common.rpm;
				data->target_torque =
					data->target_torque_ff +
					pid_calc_mit(config->common.pid_datas[i],
						     -data->pid_angle_input, vel_error, deltaT_us);
			} else {
				pid_calc(config->common.pid_datas[i]);
			}
		}
	}

	if (data->target_torque > data->common.torque_limit[1]) {
		data->target_torque = data->common.torque_limit[1];
	} else if (data->target_torque < data->common.torque_limit[0]) {
		data->target_torque = data->common.torque_limit[0];
	}
	if (!config->is_dm_motor) {
		data->target_current = data->target_torque / config->gear_ratio *
				       convert[data->convert_num][TORQUE2CURRENT];
	} else {
		data->target_current =
			data->target_torque * 16384.0f /
			(config->dm_torque_ratio * config->dm_i_max * config->gear_ratio);
	}
	k_spin_unlock(&data->data_input_lock, key);
}

void dji_miss_isr_handler(struct k_timer *dummy)
{
	ARG_UNUSED(dummy);
	k_work_submit_to_queue(&dji_work_queue, &dji_miss_handle);
}

void dji_init_isr_handler(struct k_timer *dummy)
{
	ARG_UNUSED(dummy);
	k_work_queue_start(&dji_work_queue, dji_work_queue_stack, CAN_SEND_STACK_SIZE,
			   CAN_SEND_PRIORITY, NULL);
	k_work_submit_to_queue(&dji_work_queue, &dji_init_handle);
}

void dji_miss_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	int curr_time = k_cycle_get_32();
	for (int i = 0; i < DJI_MOTOR_COUNT; i++) {
		dji_timeout_handle(motor_devices[i], curr_time);
	}
}

void dji_init_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	k_timer_stop(&dji_miss_handle_timer);
	for (int i = 0; i < DJI_MOTOR_COUNT; i++) {
		if (motor_devices[i]) {
			const struct dji_motor_config *cfg = motor_devices[i]->config;
			struct can_filter filter = {
				.id = cfg->common.rx_id,
				.mask = 0x7FF,
				.flags = 0,
			};
			int err = can_add_rx_filter(cfg->common.phy, can_rx_callback,
						    (void *)motor_devices[i], &filter);
			if (err < 0) {
				LOG_ERR("Error attaching CAN RX callback (err %d)", err);
			}
		}
	}
	dji_miss_handle_timer.expiry_fn = dji_miss_isr_handler;
	k_timer_start(&dji_miss_handle_timer, K_NO_WAIT, K_MSEC(4));
}

void dji_tx_handler(struct k_work *work)
{
	struct motor_controller *ctrl_struct =
		CONTAINER_OF(work, struct motor_controller, full_handle);
	if (work == NULL) {
		return;
	}

	for (int i = 0; i < CAN_TX_ID_CNT; i++) { // For each frame
		if (ctrl_struct->full[i]) {
			ctrl_struct->full[i] = false;

			int8_t id_temp = -1;
			bool packed = false;
			struct can_frame txframe = {0};

			for (int j = 0; j < 4; j++) {
				id_temp = ctrl_struct->mapping[i][j];
				if (id_temp < 0) {
					continue;
				}
				const struct device *dev = ctrl_struct->motor_devs[id_temp];
				struct dji_motor_data *data = dev->data;
				if (id_temp < 8 && data->online) {
					if (!data->calculated) {
						motor_calc(ctrl_struct->motor_devs[id_temp]);
						data->calculated = true;
					}
					can_pack_add(txframe.data, ctrl_struct->motor_devs[id_temp],
						     j);
					packed = true;
				}
			}
			if (packed) {
				txframe.id = index_to_frameID(i);
				txframe.dlc = 8;
				txframe.flags = 0;
				const struct device *can_dev = ctrl_struct->can_dev;
				can_send_queued(can_dev, &txframe);
			}
		}
	}
}

DT_INST_FOREACH_STATUS_OKAY(DMOTOR_INST)
