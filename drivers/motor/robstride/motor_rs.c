#include "motor_rs.h"
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

#define DT_DRV_COMPAT rs_motor

LOG_MODULE_REGISTER(motor_rs, CONFIG_MOTOR_LOG_LEVEL);

static float uint16_to_float(uint16_t x, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	return (span * (float)x / (float)((1 << bits) - 1)) + offset;
}
static int float_to_uint(float x, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	if (x > x_max) {
		x = x_max;
	} else if (x < x_min) {
		x = x_min;
	}
	return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

static void rs_can_rx_handler(const struct device *can_dev, struct can_frame *frame,
			      void *user_data);

static struct can_filter filters[MOTOR_COUNT];

static int get_motor_id(struct can_frame *frame)
{
	for (int i = 0; i < MOTOR_COUNT; i++) {
		const struct device *dev = motor_devices[i];
		const struct rs_motor_cfg *cfg = (const struct rs_motor_cfg *)(dev->config);
		if ((cfg->common.tx_id & 0xFF) == (frame->id & 0xFF00) >> 8) {
			return i;
		}
	}
	return -1;
}

int rs_init(const struct device *dev)
{
	const struct rs_motor_cfg *cfg = dev->config;
	if (!device_is_ready(cfg->common.phy)) {
		motor_stats_inc(MOTOR_STAT_CONFIG_ERROR);
		return -1;
	}

	static bool initialized = false;
	if (initialized) {
		return 0;
	}
	initialized = true;

	k_work_queue_init(&rs_work_queue);
	k_work_queue_start(&rs_work_queue, rs_work_queue_stack, CAN_SEND_STACK_SIZE,
			   CAN_SEND_PRIORITY, NULL);

	for (int i = 0; i < MOTOR_COUNT; i++) {
		const struct rs_motor_cfg *cfg =
			(const struct rs_motor_cfg *)(motor_devices[i]->config);

		motor_can_sched_register_can(cfg->common.phy);
		filters[i].flags = CAN_FILTER_IDE;
		filters[i].mask = CAN_FILTER_MASK;
		filters[i].id = (cfg->common.tx_id & 0xFF) << 8;
		int err = can_add_rx_filter(cfg->common.phy, rs_can_rx_handler, 0, &filters[i]);
		if (err < 0) {
			motor_stats_inc(MOTOR_STAT_CAN_FILTER_ERROR);
			return -1;
		}
	}
	k_sleep(K_MSEC(100));

	for (int i = 0; i < MOTOR_COUNT; i++) {
		const struct rs_motor_cfg *cfg =
			(const struct rs_motor_cfg *)(motor_devices[i]->config);

		if (cfg->auto_report) {
			struct rs_can_id id = {
				.master_id = cfg->common.rx_id,
				.motor_id = cfg->common.tx_id,
				.reserved = 0,
				.msg_type = Communication_Type_MotorReport,
			};
			struct can_frame frame = {
				.data = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x01, 0x00},
				.dlc = 8,
				.flags = CAN_FRAME_IDE,
			};
			frame.id = *(uint32_t *)&id;
			motor_can_sched_send_prio(cfg->common.phy, &frame, true, "rs-report");
			k_sleep(K_MSEC(1));
		}
	}

	k_timer_start(&rs_tx_timer, K_NO_WAIT, K_MSEC(5));
	return 0;
}

void rs_motor_control(const struct device *dev, enum motor_cmd cmd)
{
	struct rs_motor_data *data = dev->data;
	const struct rs_motor_cfg *cfg = dev->config;
	struct rs_can_id id = {
		.master_id = cfg->common.rx_id,
		.motor_id = cfg->common.tx_id,
		.reserved = 0,
	};
	struct can_frame frame = {
		.data = {0},
		.dlc = 8,
		.flags = CAN_FRAME_IDE,
	};
	switch (cmd) {
	case ENABLE_MOTOR:
		id.msg_type = Communication_Type_MotorStop;
		frame.id = *(uint32_t *)&id;
		frame.data[0] = 0x01;
		motor_can_sched_send_prio(cfg->common.phy, &frame, true, "rs-stop-before-enable");
		// clear error before enabling
		id.msg_type = Communication_Type_MotorEnable;
		frame.id = *(uint32_t *)&id;
		frame.data[0] = 0x0;
		motor_can_sched_send_prio(cfg->common.phy, &frame, true, "rs-enable");
		data->enabled = true;
		data->online = true;
		data->missed_times = 0;
		break;
	case DISABLE_MOTOR:
		id.msg_type = Communication_Type_MotorStop;
		frame.id = *(uint32_t *)&id;
		motor_can_sched_send_prio(cfg->common.phy, &frame, true, "rs-disable");
		data->enabled = false;
		break;
	case SET_ZERO:
		id.msg_type = Communication_Type_SetPosZero;
		frame.id = *(uint32_t *)&id;
		frame.data[0] = 0x01;
		data->common.angle = 0;
		motor_can_sched_send_prio(cfg->common.phy, &frame, true, "rs-set-zero");
		break;
	case CLEAR_ERROR:
		id.msg_type = Communication_Type_MotorStop;
		frame.id = *(uint32_t *)&id;
		frame.data[0] = 0x01;
		motor_can_sched_send_prio(cfg->common.phy, &frame, true, "rs-clear-error");
		data->enabled = false;
		break;
	case CLEAR_CONTROLLER:
		break;
	default:
		motor_stats_inc(MOTOR_STAT_UNSUPPORTED_CMD);
		break;
	}
}

static void rs_motor_pack(const struct device *dev, struct can_frame *frame)
{
	uint32_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;

	struct rs_motor_data *data = (struct rs_motor_data *)(dev->data);
	struct rs_motor_cfg *cfg = (struct rs_motor_cfg *)(dev->config);
	struct rs_can_id *rs_can_id = (struct rs_can_id *)&(frame->id);

	rs_can_id->motor_id = cfg->common.tx_id;

	frame->dlc = 8;
	frame->flags = CAN_FRAME_IDE;
	switch (data->common.mode) {
	case MIT:
		rs_can_id->msg_type = Communication_Type_MotionControl_MIT;

		pos_tmp = float_to_uint(data->target_pos, -cfg->p_max, cfg->p_max, 16);
		vel_tmp = float_to_uint(data->target_radps, -cfg->v_max, cfg->v_max, 16);
		kp_tmp = float_to_uint(data->params.k_p, KP_MIN, cfg->kp_max, 16);
		kd_tmp = float_to_uint(data->params.k_d, KD_MIN, cfg->kd_max, 16);
		tor_tmp = float_to_uint(data->target_torque, -cfg->t_max, cfg->t_max, 16);
		rs_can_id->master_id = tor_tmp & 0xFF;
		rs_can_id->reserved = (tor_tmp >> 8) & 0xFF;
		frame->data[0] = (pos_tmp >> 8) & 0xFF;
		frame->data[1] = pos_tmp & 0xFF;
		frame->data[2] = (vel_tmp >> 8) & 0xFF;
		frame->data[3] = vel_tmp & 0xFF;
		frame->data[4] = (kp_tmp >> 8) & 0xFF;
		frame->data[5] = kp_tmp & 0xFF;
		frame->data[6] = (kd_tmp >> 8) & 0xFF;
		frame->data[7] = kd_tmp & 0xFF;
		break;
	default:
		break;
	}
	return;
}

static int rs_apply_controller_mode(const struct device *dev, enum motor_mode mode)
{

	struct rs_motor_data *data = dev->data;
	const struct rs_motor_cfg *cfg = dev->config;
	char mode_str[10] = {0};

	switch (mode) {
	case MIT:
		strcpy(mode_str, "mit");
		break;
	default:
		motor_stats_inc(MOTOR_STAT_UNSUPPORTED_MODE);
		return -ENOSYS;
	}
	struct can_frame frame = {0};
	struct rs_can_id *rs_can_id = (struct rs_can_id *)&(frame.id);
	rs_can_id->msg_type = Communication_Type_SetSingleParameter;
	rs_can_id->motor_id = cfg->common.tx_id;
	rs_can_id->master_id = cfg->common.rx_id;
	frame.flags = CAN_FRAME_IDE;
	frame.dlc = 8;
	uint16_t index = Run_mode;
	memcpy(&frame.data[0], &index, 2);

	frame.data[4] = (uint8_t)mode;
	motor_can_sched_send_prio(cfg->common.phy, &frame, true, "rs-set-mode");

	for (int i = 0; i < motor_get_controller_count(dev); i++) {
		const struct motor_controller_config *ctrl_cfg = &cfg->common.controllers[i];

		if (ctrl_cfg->param_count == 0) {
			motor_stats_inc(MOTOR_STAT_CONFIG_ERROR);
			break;
		}
		if (data->common.controller_id != MOTOR_CONTROLLER_ID_AUTO &&
		    data->common.controller_id != i) {
			continue;
		}
		if (strcmp(ctrl_cfg->info.name, mode_str) == 0 || ctrl_cfg->info.mode == mode) {
			struct motor_controller_params params = {0};

			if (motor_controller_get_params(ctrl_cfg, 0, &params) < 0) {
				continue;
			}

			data->common.mode = mode;
			data->common.target = MOTOR_TARGET_POSITION;
			data->common.controller_id = i;
			data->params.k_p = params.k_p;
			data->params.k_d = params.k_d;
			break;
		}
	}
	return 0;
}

static void rs_can_rx_handler(const struct device *can_dev, struct can_frame *frame,
			      void *user_data)
{
	motor_can_sched_report_rx(can_dev, frame);
	uint32_t id = get_motor_id(frame);
	if (id == -1) {
		motor_stats_inc(MOTOR_STAT_UNKNOWN_RX);
		return;
	}

	const struct device *dev = motor_devices[id];
	struct rs_motor_data *data = (struct rs_motor_data *)(dev->data);
	const struct rs_motor_cfg *cfg = dev->config;
	struct rs_can_id *can_id = (struct rs_can_id *)&(frame->id);

	data->missed_times = 0;

	if (can_id->msg_type == Communication_Type_MotorFeedback ||
	    can_id->msg_type == Communication_Type_MotorReport) {
		data->err = (can_id->reserved) & 0x3f;
		if (data->err) {
			motor_stats_inc(MOTOR_STAT_DRIVER_ERROR);
		}
		data->RAWangle = (frame->data[0] << 8) | (frame->data[1]);
		data->RAWrpm = (frame->data[2] << 8) | (frame->data[3]);
		data->RAWtorque = (frame->data[4] << 8) | (frame->data[5]);
		data->RAWtemp = (frame->data[6] << 8) | (frame->data[7]);
		if (!data->online) {
			data->target_pos = uint16_to_float(data->RAWangle, -cfg->p_max,
							   cfg->p_max, 16);
			data->online = true;
			LOG_INF("Motor %s is online.", dev->name);
		}
	}
}

void rs_tx_isr_handler(struct k_timer *dummy)
{
	k_work_submit_to_queue(&rs_work_queue, &rs_tx_data_handle);
}

void rs_tx_data_handler(struct k_work *work)
{
	struct can_frame tx_frame = {0};

	for (int i = 0; i < MOTOR_COUNT; i++) {
		struct rs_motor_data *data = motor_devices[i]->data;
		const struct rs_motor_cfg *cfg = motor_devices[i]->config;

		if (data->enabled) {
			if (data->missed_times > 100 && data->online) {
				LOG_ERR("Motor %s is not responding, setting it to offline.",
					motor_devices[i]->name);
				data->online = false;
			}
			rs_motor_pack(motor_devices[i], &tx_frame);
			motor_can_sched_send_reply(
				cfg->common.phy, &tx_frame,
				(Communication_Type_MotorFeedback << 24) |
					((cfg->common.tx_id & 0xFF) << 8),
				0x1F00FF00, 5U, "rs-control");
			if (data->missed_times < INT16_MAX) {
				data->missed_times++;
			}
		}
	}
}

int rs_get(const struct device *dev, motor_status_t *status)
{
	struct rs_motor_data *data = dev->data;
	const struct rs_motor_cfg *cfg = dev->config;

	if (!data->online) {
		return -ENODEV;
	}

	data->common.angle = RAD2DEG * uint16_to_float(data->RAWangle, -cfg->p_max,
						       cfg->p_max, 16);
	data->common.rpm = RADPS2RPM(
		uint16_to_float(data->RAWrpm, -cfg->v_max, cfg->v_max, 16));
	data->common.torque =
		uint16_to_float(data->RAWtorque, -cfg->t_max, cfg->t_max, 16);
	data->common.temperature = ((float)(data->RAWtemp)) / 10.0f;

	status->angle = data->common.angle;
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

/**
 * @brief Set motor status: angle, speed, torque, mode
 * @param dev Pointer to the motor device structure: robstride motor device
 * @param status Pointer to the motor status structure
 */
int rs_set(const struct device *dev, motor_setpoint_t *status)
{
	struct rs_motor_data *data = dev->data;
	motor_controller_info_t controller = {0};
	enum motor_mode previous_mode = data->common.mode;
	uint8_t previous_controller_id = data->common.controller_id;
	int ret;

	ret = motor_resolve_controller(dev, status, &controller);
	if (ret < 0) {
		motor_stats_inc(MOTOR_STAT_UNSUPPORTED_MODE);
		return ret;
	}
	if (status->target != MOTOR_TARGET_NONE) {
		data->common.controller_id = status->controller_id;
	} else {
		return 0;
	}

	if (status->mode == MIT) {
		data->common.target = MOTOR_TARGET_POSITION;
		data->target_pos = status->angle / RAD2DEG;
		data->target_radps = RPM2RADPS(status->rpm);
		data->target_torque = status->torque;
	} else {
		return -ENOSYS;
	}
	if (status->mode != previous_mode || status->controller_id != previous_controller_id) {
		if (rs_apply_controller_mode(dev, status->mode) < 0) {
			motor_stats_inc(MOTOR_STAT_UNSUPPORTED_MODE);
			return -EIO;
		}
	} else {
		data->common.mode = status->mode;
	}
	return 0;
}

DT_INST_FOREACH_STATUS_OKAY(RS_MOTOR_INST)
