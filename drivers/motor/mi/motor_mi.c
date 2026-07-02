#include "motor_mi.h"
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

#define DT_DRV_COMPAT mi_motor

LOG_MODULE_REGISTER(motor_mi, CONFIG_MOTOR_LOG_LEVEL);

struct k_sem tx_frame_sem;

static float uint16_to_float(uint16_t x, float x_min, float x_max, int bits)
{
	uint32_t span = (1 << bits) - 1;
	float offset = x_max - x_min;
	return offset * x / span + x_min;
}
int float_to_uint(float x, float x_min, float x_max, int bits)
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

int mi_init(const struct device *dev)
{
	const struct mi_motor_cfg *cfg = dev->config;
	if (!device_is_ready(cfg->common.phy)) {
		motor_stats_inc(MOTOR_STAT_CONFIG_ERROR);
		return -1;
	}
	if (k_work_busy_get(&mi_init_work) != 0) {
		return 0;
	}
	k_work_queue_init(&mi_work_queue);

	mi_tx_timer.expiry_fn = mi_isr_init_handler;
	k_timer_start(&mi_tx_timer, K_MSEC(600), K_MSEC(2));
	k_timer_user_data_set(&mi_tx_timer, &mi_init_work);
	return 0;
}

void mi_motor_control(const struct device *dev, enum motor_cmd cmd)
{
	struct mi_motor_data *data = dev->data;
	const struct mi_motor_cfg *cfg = dev->config;

	struct can_frame frame = {0};
	frame.flags = CAN_FRAME_IDE;

	frame.dlc = 8;
	// uint16_t master_id;

	struct mi_can_id *mi_can_id = (struct mi_can_id *)&(frame.id);
	mi_can_id->id = cfg->common.id;
	switch (cmd) {
	case ENABLE_MOTOR:
		mi_can_id->mi_msg_mode = Communication_Type_MotorEnable;
		motor_can_sched_send_prio(cfg->common.phy, &frame, true, "mi-enable");
		data->online = true;
		data->enabled = true;
		break;
	case DISABLE_MOTOR:
		mi_can_id->mi_msg_mode = Communication_Type_MotorStop;
		motor_can_sched_send_prio(cfg->common.phy, &frame, true, "mi-disable");
		data->online = false;
		data->enabled = false;
		break;
	case SET_ZERO:
		mi_can_id->mi_msg_mode = Communication_Type_SetPosZero;
		frame.data[0] = 0x01;
		data->delta_deg_sum = 0;
		data->common.angle = 0;
		motor_can_sched_send_prio(cfg->common.phy, &frame, true, "mi-set-zero");
		break;

	case CLEAR_CONTROLLER:
		break;
	case CLEAR_ERROR:
		break;
	default:
		motor_stats_inc(MOTOR_STAT_UNSUPPORTED_CMD);
		return;
	}
}

static void mi_motor_pack(const struct device *dev, struct can_frame *frame)
{
	uint32_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp, cur_tep;

	struct mi_motor_data *data = (struct mi_motor_data *)(dev->data);
	struct mi_motor_cfg *cfg = (struct mi_motor_cfg *)(dev->config);
	struct mi_can_id *mi_can_id = (struct mi_can_id *)&(frame->id);

	mi_can_id->id = cfg->common.id;

	frame->dlc = 8;
	frame->flags = 0;

	struct can_frame *frame_follow = &frame[1];
	frame->flags = CAN_FRAME_IDE;
	frame_follow->flags = CAN_FRAME_IDE;
	struct mi_can_id *mi_can_id_fol = (struct mi_can_id *)&(frame_follow->id);
	uint16_t index[2];
	mi_can_id_fol->id = cfg->common.id;
	switch (data->common.mode) {
	case MIT:
		mi_can_id->mi_msg_mode = Communication_Type_MotionControl_MIT;

		pos_tmp = float_to_uint(data->target_pos, P_MIN, P_MAX, 16);
		vel_tmp = float_to_uint(data->target_radps, V_MIN, V_MAX, 16);
		kp_tmp = float_to_uint(data->params.k_p, KP_MIN, KP_MAX, 16);
		kd_tmp = float_to_uint(data->params.k_d, KD_MIN, KD_MAX, 16);
		tor_tmp = float_to_uint(data->target_torque, T_MIN, T_MAX, 16);
		mi_can_id->data = tor_tmp;
		frame->data[0] = (pos_tmp >> 8) & 0xFF;
		frame->data[1] = pos_tmp & 0xFF;
		frame->data[2] = (vel_tmp >> 8) & 0xFF;
		frame->data[3] = vel_tmp & 0xFF;
		frame->data[4] = (kp_tmp >> 8) & 0xFF;
		frame->data[5] = kp_tmp & 0xFF;
		frame->data[6] = (kd_tmp >> 8) & 0xFF;
		frame->data[7] = kd_tmp & 0xFF;
		break;
	case PV:
		mi_can_id->mi_msg_mode = Communication_Type_SetSingleParameter;
		index[0] = Limit_Spd;
		index[1] = Loc_Ref;
		memcpy(&frame->data[0], &index[0], 2);

		frame->data[2] = 0;
		frame->data[3] = 0;
		memcpy(&frame->data[4], &data->target_radps, 4);

		mi_can_id_fol->mi_msg_mode = Communication_Type_SetSingleParameter;
		frame_follow->dlc = 8;

		memcpy(&frame_follow->data[0], &index[1], 2);
		frame_follow->data[2] = 0;
		frame_follow->data[3] = 0;
		memcpy(&frame_follow->data[4], &data->target_pos, 4);

		break;
	case VO:
		mi_can_id->mi_msg_mode = Communication_Type_SetSingleParameter;
		vel_tmp = float_to_uint(data->target_radps, V_MIN, V_MAX, 32);
		cur_tep = float_to_uint(data->limit_cur, 0, CUR_MAX, 32);
		index[0] = Limit_Cur;
		index[1] = Spd_Ref;
		memcpy(&frame->data[0], &index[0], 2);

		frame->data[2] = 0;
		frame->data[3] = 0;
		memcpy(&frame->data[4], &data->limit_cur, 4);

		mi_can_id_fol->mi_msg_mode = Communication_Type_SetSingleParameter;

		frame_follow->dlc = 8;

		memcpy(&frame_follow->data[0], &index[1], 2);
		frame_follow->data[2] = 0;
		frame_follow->data[3] = 0;
		memcpy(&frame_follow->data[4], &data->target_radps, 4);

		break;
	default:
		break;
	}
}

static int mi_apply_controller_mode(const struct device *dev, enum motor_mode mode)
{

	struct mi_motor_data *data = dev->data;
	const struct mi_motor_cfg *cfg = dev->config;
	char mode_str[10] = {0};

	data->common.mode = mode;
	data->common.target = mode == VO ? MOTOR_TARGET_SPEED : MOTOR_TARGET_POSITION;

	switch (mode) {
	case MIT:
		strcpy(mode_str, "mit");

		break;
	case PV:
		strcpy(mode_str, "pv");
		break;
	case VO:
		strcpy(mode_str, "vo");
		data->limit_cur = 23.0f;
		break;
	default:
		motor_stats_inc(MOTOR_STAT_UNSUPPORTED_MODE);
		break;
	}
	struct can_frame frame = {0};
	struct mi_can_id *mi_can_id = (struct mi_can_id *)&(frame.id);
	mi_can_id->mi_msg_mode = Communication_Type_SetSingleParameter;
	mi_can_id->id = cfg->common.id;
	frame.flags = CAN_FRAME_IDE;
	frame.dlc = 8;
	uint16_t index = Run_mode;
	memcpy(&frame.data[0], &index, 2);

	frame.data[4] = (uint8_t)mode;
	motor_can_sched_send_prio(cfg->common.phy, &frame, true, "mi-set-mode");

	for (int i = 0; i < motor_get_controller_count(dev); i++) {
		const struct motor_controller_config *ctrl_cfg = &cfg->common.controllers[i];

		if (ctrl_cfg->param_count == 0) {
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
			data->common.controller_id = i;
			data->params.k_p = params.k_p;
			data->params.k_d = params.k_d;
			break;
		}
	}
	return 0;
}

static int get_motor_id(struct can_frame *frame)
{
	for (int i = 0; i < MOTOR_COUNT; i++) {
		const struct device *dev = motor_devices[i];
		const struct mi_motor_cfg *cfg = (const struct mi_motor_cfg *)(dev->config);
		if ((cfg->common.id & 0xFF) == (frame->id & 0xFF00) >> 8) {
			return i;
		}
	}
	return -1;
}

static struct can_filter filters[MOTOR_COUNT];

static void mi_can_rx_handler(const struct device *can_dev, struct can_frame *frame,
			      void *user_data)
{
	motor_can_sched_report_rx(can_dev, frame);
	int id = get_motor_id(frame);
	if (id == -1) {
		motor_stats_inc(MOTOR_STAT_UNKNOWN_RX);
		return;
	}

	struct mi_motor_data *data = (struct mi_motor_data *)(motor_devices[id]->data);

	if (data->missed_times > 0) {

		data->missed_times = 0;
	}
	struct mi_can_id *can_id = (struct mi_can_id *)&(frame->id);
	if (can_id->mi_msg_mode == Communication_Type_MotorFeedback) {
		data->err = ((can_id->data) >> 8) & 0x3f;
		if (data->err) {
			motor_stats_inc(MOTOR_STAT_DRIVER_ERROR);
		}
		data->update = true;
		data->RAWangle = (frame->data[0] << 8) | (frame->data[1]);
		data->RAWrpm = (frame->data[2] << 8) | (frame->data[3]);
		data->RAWtorque = (frame->data[4] << 8) | (frame->data[5]);
		data->RAWtemp = (frame->data[6] << 8) | (frame->data[7]);
	} else if (can_id->mi_msg_mode == Communication_Type_GetID) {
		data->can_id = can_id->data;
	}

	k_work_submit_to_queue(&mi_work_queue, &mi_rx_data_handle);
}

void mi_rx_data_handler(struct k_work *work)
{
	for (int i = 0; i < MOTOR_COUNT; i++) {
		struct mi_motor_data *data = (struct mi_motor_data *)(motor_devices[i]->data);
		if (!data->update) {
			continue;
		}

		float prev_angle = data->common.angle;
		data->common.angle =
			(uint16_to_float(data->RAWangle, (double)P_MIN, (double)P_MAX, 16))*RAD2DEG;
		data->common.rpm =
			RADPS2RPM(uint16_to_float(data->RAWrpm, (double)V_MIN, (double)V_MAX, 16));
		data->common.torque =
			uint16_to_float(data->RAWtorque, (double)T_MIN, (double)T_MAX, 16);
		data->common.temperature = ((float)(data->RAWtemp)) / 10;
		data->delta_deg_sum += data->common.angle - prev_angle;

		data->update = false;
	}
}

void mi_tx_isr_handler(struct k_timer *dummy)
{
	k_work_submit_to_queue(&mi_work_queue, &mi_tx_data_handle);
}

void mi_isr_init_handler(struct k_timer *dummy)
{
	dummy->expiry_fn = mi_tx_isr_handler;
	k_work_queue_start(&mi_work_queue, mi_work_queue_stack, CAN_SEND_STACK_SIZE,
			   CAN_SEND_PRIORITY, NULL);
	k_work_submit_to_queue(&mi_work_queue, &mi_init_work);
}

void mi_tx_data_handler(struct k_work *work)
{
	struct can_frame tx_frame[2] = {0};

	tx_frame[0].flags = CAN_FRAME_IDE;
	tx_frame[1].flags = CAN_FRAME_IDE;
	for (int i = 0; i < MOTOR_COUNT; i++) {
		struct mi_motor_data *data = motor_devices[i]->data;
		const struct mi_motor_cfg *cfg = motor_devices[i]->config;
		if (data->enabled) {
			if (data->missed_times > 600) {
				LOG_ERR("Motor %s is not responding, setting it to offline.",
					motor_devices[i]->name);
				data->missed_times = 0;
				// struct can_frame frame = {0};
				// frame.flags = CAN_FRAME_IDE;
				// frame.dlc = 8;
				// struct mi_can_id *mi_can_id = (struct mi_can_id *)&(frame.id);
				// mi_can_id->id = cfg->common.id;
				// mi_can_id->mi_msg_mode = Communication_Type_MotorEnable;
				// data->online = true;
				// data->enabled = true;
				// data->missed_times = 0;
			}
			mi_motor_pack(motor_devices[i], tx_frame);

			motor_can_sched_send_reply(
				cfg->common.phy, &tx_frame[0],
				(Communication_Type_MotorFeedback << 24) |
					((cfg->common.id & 0xFF) << 8),
				0x1F00FF00, 5U, "mi-control");

			if ((data->common.mode == PV) || (data->common.mode == VO)) {
				motor_can_sched_send_prio(cfg->common.phy, &tx_frame[1], true,
							  "mi-follow");
			}
			data->missed_times++;
		}
		if (i % 2 == 1) {
			k_usleep(500);
		}
	}
}

void mi_init_handler(struct k_work *work)
{
	k_timer_stop(&mi_tx_timer);

	for (int i = 0; i < MOTOR_COUNT; i++) {
		const struct mi_motor_cfg *cfg =
			(const struct mi_motor_cfg *)(motor_devices[i]->config);

		motor_can_sched_register_can(cfg->common.phy);
		filters[i].flags = CAN_FILTER_IDE;
		filters[i].mask = 0x00000000;
		filters[i].id = 0x00000000;
		int err = can_add_rx_filter(cfg->common.phy, mi_can_rx_handler, 0, &filters[i]);
		if (err < 0) {
			motor_stats_inc(MOTOR_STAT_CAN_FILTER_ERROR);
		}
	}
	k_sleep(K_MSEC(100));

	for (int i = 0; i < MOTOR_COUNT; i++) {
		mi_motor_control(motor_devices[i], SET_ZERO);
		k_sleep(K_MSEC(1));
	}

	k_timer_start(&mi_tx_timer, K_NO_WAIT, K_MSEC(3));
	k_timer_user_data_set(&mi_tx_timer, &mi_tx_data_handle);
}

int mi_get(const struct device *dev, motor_status_t *status)
{
	struct mi_motor_data *data = dev->data;

	status->angle = fmodf(data->common.angle, 360.0f);
	status->rpm = data->common.rpm;
	status->torque = data->common.torque;
	status->temperature = data->common.temperature;
	status->mode = data->common.mode;
	status->target = data->common.target;
	status->controller_id = data->common.controller_id;
	status->sum_angle = data->delta_deg_sum;
	status->speed_limit[0] = V_MAX;
	status->speed_limit[1] = V_MIN;
	status->torque_limit[0] = T_MAX;
	status->torque_limit[1] = T_MIN;

	return 0;
}

int mi_set(const struct device *dev, motor_setpoint_t *status)
{
	struct mi_motor_data *data = dev->data;
	const struct mi_motor_cfg *cfg = dev->config;
	char mode_str[10] = {0};
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
		strcpy(mode_str, "mit");
		data->common.target = MOTOR_TARGET_POSITION;
		data->target_pos = status->angle / RAD2DEG;
		data->target_radps = RPM2RADPS(status->rpm);
		data->target_torque = status->torque;
	} else if (status->mode == PV) {
		if (status->target != MOTOR_TARGET_NONE &&
		    status->target != MOTOR_TARGET_POSITION) {
			return -ENOSYS;
		}
		strcpy(mode_str, "pv");
		data->common.target = MOTOR_TARGET_POSITION;
		data->target_pos = status->angle / RAD2DEG;
		data->target_radps = RPM2RADPS(status->rpm);
	} else if (status->mode == VO) {
		if (status->target != MOTOR_TARGET_NONE && status->target != MOTOR_TARGET_SPEED) {
			return -ENOSYS;
		}
		data->common.target = MOTOR_TARGET_SPEED;
		data->target_radps = RPM2RADPS(status->rpm);
		data->target_pos = 0;
		data->target_torque = 0;
	} else {
		return -ENOSYS;
	}
	if (status->mode == MIT) {
		for (int i = 0; i < motor_get_controller_count(dev); i++) {
			const struct motor_controller_config *ctrl_cfg = &cfg->common.controllers[i];

			if (ctrl_cfg->param_count == 0) {
				break;
			}
			if (data->common.controller_id != MOTOR_CONTROLLER_ID_AUTO &&
			    data->common.controller_id != i) {
				continue;
			}
			if (strcmp(ctrl_cfg->info.name, mode_str) == 0 ||
			    ctrl_cfg->info.mode == status->mode) {
				struct motor_controller_params params = {0};

				if (motor_controller_get_params(ctrl_cfg, 0, &params) < 0) {
					continue;
				}

				data->common.mode = status->mode;
				data->common.controller_id = i;
				data->params.k_p = params.k_p;
				data->params.k_d = params.k_d;
				break;
			}
		}
	}
	if (status->mode != previous_mode || status->controller_id != previous_controller_id) {
		if (mi_apply_controller_mode(dev, status->mode) < 0) {
			motor_stats_inc(MOTOR_STAT_UNSUPPORTED_MODE);
			return -EIO;
		}
	} else {
		data->common.mode = status->mode;
	}

	return 0;
}

DT_INST_FOREACH_STATUS_OKAY(MIMOTOR_INST)
