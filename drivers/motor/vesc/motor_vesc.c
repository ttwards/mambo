/**
 * Copyright (c) 2026 EclipseaHime017 <12210226@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * @Description: VESC motor driver implementation for Zephyr RTOS.
 */
#include "motor_vesc.h"
#include "zephyr/device.h"
#include "zephyr/drivers/can.h"
#include "../common/common.h"
#include "zephyr/drivers/motor.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <sys/_stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/time_units.h>

#define DT_DRV_COMPAT vesc_motor

LOG_MODULE_REGISTER(motor_vesc, CONFIG_MOTOR_LOG_LEVEL);

static void vesc_can_rx_handler(const struct device *can_dev, struct can_frame *frame,
				void *user_data);

int vesc_init(const struct device *dev)
{
	LOG_DBG("vesc_init");
	const struct vesc_motor_config *cfg = dev->config;

	if (!device_is_ready(cfg->common.phy)) {
		LOG_ERR("CAN device not ready");
		return -1;
	}
	reg_can_dev(cfg->common.phy);

	struct can_filter filter = {0};
	filter.flags = CAN_FILTER_IDE;
	filter.mask = CAN_FILTER_MASK;

	struct vesc_can_id id = {
		.motor_id = cfg->common.id,
		.msg_type = CAN_PACKET_STATUS,
	};
	filter.id = *((uint32_t *)&id);
	int err = can_add_rx_filter(cfg->common.phy, vesc_can_rx_handler, (void *)dev, &filter);
	if (err < 0) {
		LOG_ERR("Error adding CAN filter (err %d)", err);
		return -1;
	}
	return 0;
}

const struct motor_driver_api vesc_motor_api = {
	.motor_get = vesc_get,
	.motor_set = vesc_set,
	.motor_control = vesc_motor_control,
	.motor_set_mode = vesc_motor_set_mode,
};

void vesc_motor_control(const struct device *dev, enum motor_cmd cmd)
{
	struct vesc_motor_data *data = dev->data;
	switch (cmd) {
	case ENABLE_MOTOR:
		data->enable = true;
		break;
	case DISABLE_MOTOR:
		data->enable = false;
		break;
	case SET_ZERO:
		break;
	case CLEAR_ERROR:
		break;
	case CLEAR_PID:
		break;
	}
}

/**
 * @brief Pack CAN frame for VESC motor control
 * @param dev Pointer to the device
 * @param frame Pointer to the CAN frame to be packed
 */
static void vesc_motor_pack(const struct device *dev, struct can_frame *frame)
{
	int32_t pos_tmp, vel_tmp, cur_tmp;

	struct vesc_motor_data *data = (struct vesc_motor_data *)(dev->data);
	const struct vesc_motor_config *cfg = (const struct vesc_motor_config *)(dev->config);
	struct vesc_can_id *vesc_can_id = (struct vesc_can_id *)&(frame->id);

	vesc_can_id->motor_id = cfg->common.id;

	frame->dlc = 8;
	frame->flags = CAN_FRAME_IDE;
	if (data->enable == true) {
		switch (data->common.mode) {
		case ML_TORQUE:
			vesc_can_id->msg_type = CAN_PACKET_SET_CURRENT;
			if (data->target_current > cfg->i_max) {
				LOG_ERR("vesc_motor_pack: target_current %f exceeds i_max %f",
					(double)data->target_current, (double)cfg->i_max);
				cur_tmp = cfg->i_max * 1000; // mA
			} else if (data->target_current < -cfg->i_max) {
				LOG_ERR("vesc_motor_pack: target_current %f exceeds negative i_max "
					"%f",
					(double)data->target_current, (double)(-cfg->i_max));
				cur_tmp = -cfg->i_max * 1000; // mA
			} else {
				cur_tmp = (int32_t)(data->target_current * 1000); // mA
			}
			frame->data[0] = (cur_tmp >> 24) & 0xFF;
			frame->data[1] = (cur_tmp >> 16) & 0xFF;
			frame->data[2] = (cur_tmp >> 8) & 0xFF;
			frame->data[3] = cur_tmp & 0xFF;
			break;
		case ML_SPEED:
			vesc_can_id->msg_type = CAN_PACKET_SET_RPM;
			if (data->target_radps > cfg->v_max) {
				// LOG_ERR("vesc_motor_pack: target_radps %f exceeds v_max %f",
				// 	data->target_radps, cfg->v_max);
				vel_tmp = cfg->v_max * VESC_RPM_PER_RADPS;
			} else if (data->target_radps < -cfg->v_max) {
				// LOG_ERR("vesc_motor_pack: target_radps %f exceeds negative v_max "
				// 	"%f",
					// data->target_radps, -cfg->v_max);
				vel_tmp = -cfg->v_max * VESC_RPM_PER_RADPS;
			} else {
				vel_tmp = (int32_t)(data->target_radps * VESC_RPM_PER_RADPS);
			}
			vel_tmp = vel_tmp * cfg->pole_pairs * cfg->gear_ratio; // rpm -> erpm
			frame->data[0] = (vel_tmp >> 24) & 0xFF;
			frame->data[1] = (vel_tmp >> 16) & 0xFF;
			frame->data[2] = (vel_tmp >> 8) & 0xFF;
			frame->data[3] = vel_tmp & 0xFF;
			break;
		case ML_ANGLE:
			vesc_can_id->msg_type = CAN_PACKET_SET_POS;
			if (data->target_angle > (cfg->p_max * VESC_RAD_PER_DEG)) {
				LOG_ERR("vesc_motor_pack: target_angle %f exceeds 360 degree",
					(double)(data->target_angle * VESC_DEG_PER_RAD));
				data->target_angle = cfg->p_max * VESC_RAD_PER_DEG;
			} else if (data->target_angle < (-cfg->p_max * VESC_RAD_PER_DEG)) {
				LOG_ERR("vesc_motor_pack: target_angle %f less than 0 degree",
					(double)(data->target_angle * VESC_DEG_PER_RAD));
				data->target_angle = -cfg->p_max * VESC_RAD_PER_DEG;
			}
			pos_tmp = data->target_angle * VESC_DEG_PER_RAD * cfg->gear_ratio;
			frame->data[0] = (pos_tmp >> 24) & 0xFF;
			frame->data[1] = (pos_tmp >> 16) & 0xFF;
			frame->data[2] = (pos_tmp >> 8) & 0xFF;
			frame->data[3] = pos_tmp & 0xFF;
			break;
		default:
			break;
		}
	}
}

/**
 * @brief Set motor mode
 * @param dev Pointer to the device
 * @param mode Desired motor mode
 * @return 0 on success, negative error code on failure
 */
void vesc_motor_set_mode(const struct device *dev, enum motor_mode mode)
{
	struct vesc_motor_data *data = dev->data;
	data->common.mode = mode;
}

/**
 * @brief Set motor torque
 * @param dev Pointer to the device
 * @param torque Desired torque in Nm
 */
int vesc_set_torque(const struct device *dev, float torque)
{
	struct vesc_motor_data *data = dev->data;
	const struct vesc_motor_config *cfg = dev->config;
	data->target_current = torque / cfg->kt; // A
	return 0;
}

/**
 * @brief Set motor speed
 * @param dev Pointer to the device
 * @param speed Desired speed in RPM
 */
int vesc_set_speed(const struct device *dev, float speed)
{
	struct vesc_motor_data *data = dev->data;
	data->target_radps = speed * VESC_RADPS_PER_RPM;
	return 0;
}

/**
 * @brief Set motor angle
 * @param dev Pointer to the device
 * @param angle Desired angle in degrees
 */
int vesc_set_angle(const struct device *dev, float angle)
{
	struct vesc_motor_data *data = dev->data;
	data->target_angle = angle * VESC_RAD_PER_DEG;
	return 0;
}

/**
 * @brief Set motor parameters
 *
 */
int vesc_set(const struct device *dev, motor_status_t *status)
{
	struct vesc_motor_data *data = dev->data;
	const struct vesc_motor_config *cfg = dev->config;
	switch (status->mode) {
	case ML_TORQUE:
		vesc_set_torque(dev, status->torque);
		break;
	case ML_SPEED:
		vesc_set_speed(dev, status->rpm);
		break;
	case ML_ANGLE:
		vesc_set_angle(dev, status->angle);
		break;
	default:
		LOG_ERR("Unsupported motor mode: %d", status->mode);
		return -ENOSYS;
	}

	if (status->mode != data->common.mode) {
		vesc_motor_set_mode(dev, status->mode);
	}

	struct can_frame frame = {0};
	vesc_motor_pack(dev, &frame);
	can_send_queued(cfg->common.phy, &frame);

	return 0;
}

static void vesc_can_rx_handler(const struct device *can_dev, struct can_frame *frame,
				void *user_data)
{
	const struct device *dev = (const struct device *)user_data;
	struct vesc_motor_data *data = (struct vesc_motor_data *)(dev->data);
	struct vesc_can_id *vesc_can_id = (struct vesc_can_id *)&(frame->id);
	switch (vesc_can_id->msg_type) {
	case CAN_PACKET_STATUS:
		data->RAWrpm = (frame->data[0] << 24) | (frame->data[1] << 16) |
			       (frame->data[2] << 8) | (frame->data[3]);
		data->RAWcurrent = (int32_t)((frame->data[4] << 8) | (frame->data[5]));
		break;
	case CAN_PACKET_STATUS_4:
		data->RAWangle = (int32_t)((frame->data[6] << 8) | (frame->data[7]));
		data->RAWtemp = (int32_t)((frame->data[2] << 8) | (frame->data[3]));
		break;
	case CAN_PACKET_STATUS_5:
		break;
	}
}

/**
 * @brief Get motor parameters
 * @param dev Pointer to the device
 * @param status Pointer to motor_status_t structure to be filled
 */
int vesc_get(const struct device *dev, motor_status_t *status)
{
	struct vesc_motor_data *data = dev->data;
	const struct vesc_motor_config *cfg = dev->config;

	data->common.angle = ((float)data->RAWangle) / 50.0f / cfg->gear_ratio;
	data->common.rpm = (float)data->RAWrpm / cfg->pole_pairs / cfg->gear_ratio;
	data->common.torque = (float)data->RAWcurrent / 10.0f / 1000.0f * cfg->kt;
	data->common.temperature = (float)data->RAWtemp / 10.0f;

	status->angle = fmodf(data->common.angle, 360.0f);
	status->rpm = data->common.rpm;
	status->torque = data->common.torque;
	status->temperature = data->common.temperature;
	status->mode = data->common.mode;
	status->sum_angle = data->common.angle;
	status->speed_limit[0] = cfg->v_max;
	status->speed_limit[1] = -cfg->v_max;
	status->torque_limit[0] = cfg->t_max;
	status->torque_limit[1] = -cfg->t_max;

	return 0;
}

DT_INST_FOREACH_STATUS_OKAY(VESC_MOTOR_INST)
