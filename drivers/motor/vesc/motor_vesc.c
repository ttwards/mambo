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
#include "../common/motor_can_sched.h"
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

#define VESC_DEFAULT_PING_FREQ_HZ  10U
#define VESC_MONITOR_TICK_MS       20U
#define VESC_OFFLINE_TIMEOUT_FLOOR 300U

#define VESC_MOTOR_COUNT         DT_NUM_INST_STATUS_OKAY(vesc_motor)
#define VESC_MOTOR_POINTER(inst) DEVICE_DT_GET(DT_DRV_INST(inst)),
static const struct device *vesc_motor_devices[] = {
	DT_INST_FOREACH_STATUS_OKAY(VESC_MOTOR_POINTER)
};

static struct k_work_q vesc_work_queue;
K_THREAD_STACK_DEFINE(vesc_work_queue_stack, VESC_CAN_SEND_STACK_SIZE);

static void vesc_can_rx_handler(const struct device *can_dev, struct can_frame *frame,
				void *user_data);
static void vesc_monitor_handler(struct k_work *work);
static void vesc_monitor_timer_handler(struct k_timer *timer);

K_WORK_DEFINE(vesc_monitor_work, vesc_monitor_handler);
K_TIMER_DEFINE(vesc_monitor_timer, vesc_monitor_timer_handler, NULL);

static bool vesc_monitor_started;

static uint32_t vesc_ping_period_ms(const struct vesc_motor_config *cfg)
{
	uint32_t freq = cfg->freq > 0 ? (uint32_t)cfg->freq : VESC_DEFAULT_PING_FREQ_HZ;
	uint32_t period_ms = 1000U / freq;

	if (period_ms == 0U) {
		period_ms = 1U;
	}

	return period_ms;
}

static uint32_t vesc_offline_timeout_ms(const struct vesc_motor_config *cfg)
{
	uint32_t timeout_ms = vesc_ping_period_ms(cfg) * 3U;

	if (timeout_ms < VESC_OFFLINE_TIMEOUT_FLOOR) {
		timeout_ms = VESC_OFFLINE_TIMEOUT_FLOOR;
	}

	return timeout_ms;
}

static void vesc_send_ping(const struct device *dev)
{
	const struct vesc_motor_config *cfg = dev->config;
	struct can_frame frame = {0};
	struct vesc_can_id *vesc_can_id = (struct vesc_can_id *)&frame.id;

	vesc_can_id->motor_id = cfg->common.id;
	vesc_can_id->msg_type = CAN_PACKET_PING;
	frame.flags = CAN_FRAME_IDE;
	frame.dlc = 0;

	motor_can_sched_send_prio(cfg->common.phy, &frame, true, "vesc-ping");
}

static void vesc_monitor_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	uint64_t now = k_uptime_get();

	for (int i = 0; i < VESC_MOTOR_COUNT; i++) {
		struct vesc_motor_data *data = vesc_motor_devices[i]->data;
		const struct vesc_motor_config *cfg = vesc_motor_devices[i]->config;

		if (!data->enable) {
			continue;
		}

		if (now - data->last_ping_time >= vesc_ping_period_ms(cfg)) {
			vesc_send_ping(vesc_motor_devices[i]);
			data->last_ping_time = now;
		}

		if (data->online && now - data->prev_recv_time > vesc_offline_timeout_ms(cfg)) {
			LOG_ERR("Motor %s is not responding, setting it to offline.",
				vesc_motor_devices[i]->name);
			data->online = false;
		}

		k_sleep(K_USEC(130));
	}
}

static void vesc_monitor_timer_handler(struct k_timer *timer)
{
	ARG_UNUSED(timer);

	k_work_submit_to_queue(&vesc_work_queue, &vesc_monitor_work);
}

int vesc_init(const struct device *dev)
{
	const struct vesc_motor_config *cfg = dev->config;

	if (!device_is_ready(cfg->common.phy)) {
		motor_stats_inc(MOTOR_STAT_CONFIG_ERROR);
		return -1;
	}
	motor_can_sched_register_can(cfg->common.phy);

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
		motor_stats_inc(MOTOR_STAT_CAN_FILTER_ERROR);
		return -1;
	}

	if (!vesc_monitor_started) {
		k_work_queue_start(&vesc_work_queue, vesc_work_queue_stack,
				   VESC_CAN_SEND_STACK_SIZE, VESC_CAN_SEND_PRIORITY, NULL);
		k_timer_start(&vesc_monitor_timer, K_MSEC(VESC_MONITOR_TICK_MS),
			      K_MSEC(VESC_MONITOR_TICK_MS));
		vesc_monitor_started = true;
	}

	return 0;
}

const struct motor_driver_api vesc_motor_api = {
	.motor_get = vesc_get,
	.motor_set = vesc_set,
	.motor_control = vesc_motor_control,
};

void vesc_motor_control(const struct device *dev, enum motor_cmd cmd)
{
	struct vesc_motor_data *data = dev->data;
	switch (cmd) {
	case ENABLE_MOTOR:
		data->enable = true;
		data->online = true;
		data->prev_recv_time = k_uptime_get();
		break;
	case DISABLE_MOTOR:
		data->enable = false;
		break;
	case SET_ZERO:
		break;
	case CLEAR_ERROR:
		break;
	case CLEAR_CONTROLLER:
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
		case VO:
			if (data->common.target == MOTOR_TARGET_TORQUE) {
				vesc_can_id->msg_type = CAN_PACKET_SET_CURRENT;
				if (data->target_current > cfg->i_max) {
					motor_stats_inc(MOTOR_STAT_LIMIT_CLAMP);
					cur_tmp = cfg->i_max * 1000; // mA
				} else if (data->target_current < -cfg->i_max) {
					motor_stats_inc(MOTOR_STAT_LIMIT_CLAMP);
					cur_tmp = -cfg->i_max * 1000; // mA
				} else {
					cur_tmp = (int32_t)(data->target_current * 1000); // mA
				}
				frame->data[0] = (cur_tmp >> 24) & 0xFF;
				frame->data[1] = (cur_tmp >> 16) & 0xFF;
				frame->data[2] = (cur_tmp >> 8) & 0xFF;
				frame->data[3] = cur_tmp & 0xFF;
			} else if (data->common.target == MOTOR_TARGET_SPEED) {
				vesc_can_id->msg_type = CAN_PACKET_SET_RPM;
				if (data->target_radps > cfg->v_max) {
					motor_stats_inc(MOTOR_STAT_LIMIT_CLAMP);
					vel_tmp = cfg->v_max * VESC_RPM_PER_RADPS;
				} else if (data->target_radps < -cfg->v_max) {
					motor_stats_inc(MOTOR_STAT_LIMIT_CLAMP);
					vel_tmp = -cfg->v_max * VESC_RPM_PER_RADPS;
				} else {
					vel_tmp = (int32_t)(data->target_radps * VESC_RPM_PER_RADPS);
				}
				vel_tmp = vel_tmp * cfg->pole_pairs * cfg->gear_ratio; // rpm -> erpm
				frame->data[0] = (vel_tmp >> 24) & 0xFF;
				frame->data[1] = (vel_tmp >> 16) & 0xFF;
				frame->data[2] = (vel_tmp >> 8) & 0xFF;
				frame->data[3] = vel_tmp & 0xFF;
			}
			break;
		case PV:
			if (data->common.target != MOTOR_TARGET_POSITION) {
				break;
			}
				vesc_can_id->msg_type = CAN_PACKET_SET_POS;
				if (data->target_angle > (cfg->p_max * VESC_RAD_PER_DEG)) {
					motor_stats_inc(MOTOR_STAT_LIMIT_CLAMP);
					data->target_angle = cfg->p_max * VESC_RAD_PER_DEG;
				} else if (data->target_angle < (-cfg->p_max * VESC_RAD_PER_DEG)) {
					motor_stats_inc(MOTOR_STAT_LIMIT_CLAMP);
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
int vesc_set(const struct device *dev, motor_setpoint_t *status)
{
	struct vesc_motor_data *data = dev->data;
	const struct vesc_motor_config *cfg = dev->config;
	motor_controller_info_t controller = {0};
	int ret;

	if (status->target == MOTOR_TARGET_NONE) {
		return 0;
	}
	if (status->controller_select == MOTOR_CONTROLLER_BY_ID) {
		ret = motor_resolve_controller(dev, status, &controller);
		if (ret < 0) {
			motor_stats_inc(MOTOR_STAT_UNSUPPORTED_MODE);
			return ret;
		}
	} else {
		status->controller_id = MOTOR_CONTROLLER_ID_AUTO;
	}

	switch (status->mode) {
	case VO:
		if (status->target == MOTOR_TARGET_TORQUE) {
			vesc_set_torque(dev, status->torque);
		} else if (status->target == MOTOR_TARGET_SPEED) {
			vesc_set_speed(dev, status->rpm);
		} else {
			motor_stats_inc(MOTOR_STAT_UNSUPPORTED_MODE);
			return -ENOSYS;
		}
		break;
	case PV:
		if (status->target != MOTOR_TARGET_POSITION) {
			motor_stats_inc(MOTOR_STAT_UNSUPPORTED_MODE);
			return -ENOSYS;
		}
		vesc_set_angle(dev, status->angle);
		break;
	default:
		motor_stats_inc(MOTOR_STAT_UNSUPPORTED_MODE);
		return -ENOSYS;
	}

	data->common.mode = status->mode;
	data->common.target = status->target;
	data->common.controller_id = status->controller_id;

	struct can_frame frame = {0};
	vesc_motor_pack(dev, &frame);
	motor_can_sched_send_prio(cfg->common.phy, &frame, false, "vesc-control");

	return 0;
}

static void vesc_mark_online(const struct device *dev, struct vesc_motor_data *data)
{
	if (!data->online) {
		LOG_INF("Motor %s is online.", dev->name);
	}
	data->online = true;
	data->prev_recv_time = k_uptime_get();
}

static void vesc_can_rx_handler(const struct device *can_dev, struct can_frame *frame,
				void *user_data)
{
	motor_can_sched_report_rx(can_dev, frame);
	const struct device *dev = (const struct device *)user_data;
	struct vesc_motor_data *data = (struct vesc_motor_data *)(dev->data);
	const struct vesc_motor_config *cfg = (const struct vesc_motor_config *)(dev->config);
	struct vesc_can_id *vesc_can_id = (struct vesc_can_id *)&(frame->id);

	if (vesc_can_id->motor_id != cfg->common.id) {
		return;
	}

	switch (vesc_can_id->msg_type) {
	case CAN_PACKET_PONG:
		vesc_mark_online(dev, data);
		break;
	case CAN_PACKET_STATUS:
		vesc_mark_online(dev, data);
		data->RAWrpm = (frame->data[0] << 24) | (frame->data[1] << 16) |
			       (frame->data[2] << 8) | (frame->data[3]);
		data->RAWcurrent = (int32_t)((frame->data[4] << 8) | (frame->data[5]));
		break;
	case CAN_PACKET_STATUS_4:
		vesc_mark_online(dev, data);
		data->RAWangle = (int32_t)((frame->data[6] << 8) | (frame->data[7]));
		data->RAWtemp = (int32_t)((frame->data[2] << 8) | (frame->data[3]));
		break;
	case CAN_PACKET_STATUS_5:
		vesc_mark_online(dev, data);
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
	status->target = data->common.target;
	status->controller_id = data->common.controller_id;
	status->sum_angle = data->common.angle;
	status->speed_limit[0] = cfg->v_max;
	status->speed_limit[1] = -cfg->v_max;
	status->torque_limit[0] = cfg->t_max;
	status->torque_limit[1] = -cfg->t_max;

	return 0;
}

DT_INST_FOREACH_STATUS_OKAY(VESC_MOTOR_INST)
