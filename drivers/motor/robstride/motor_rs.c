#include "motor_rs.h"
#include "zephyr/device.h"
#include "zephyr/drivers/can.h"
#include "../common/common.h"
#include "zephyr/drivers/motor.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <sys/_stdint.h>
#include <zephyr/drivers/pid.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/time_units.h>

#define DT_DRV_COMPAT rs_motor

LOG_MODULE_REGISTER(motor_rs, CONFIG_MOTOR_LOG_LEVEL);

static float uint16_to_float(uint16_t x, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	return (span * (float)x / (float)((1 << bits) - 1)) + offset;
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

static void rs_can_rx_handler(const struct device *can_dev, struct can_frame *frame,
			      void *user_data);

int rs_init(const struct device *dev)
{
	LOG_DBG("rs_init");
	const struct rs_motor_cfg *cfg = dev->config;
	reg_can_dev(cfg->common.phy);
	if (!device_is_ready(cfg->common.phy)) {
		LOG_ERR("CAN device not ready");
		return -1;
	}
	k_work_schedule(&rs_rx_monitor_work, K_MSEC(1000));
	struct can_filter filter = {0};
	filter.flags = CAN_FILTER_IDE;
	filter.mask = CAN_FILTER_MASK;

	struct rs_can_id id = {
		.master_id = cfg->common.rx_id,
		.motor_id = cfg->common.tx_id,
		.msg_type = Communication_Type_MotorFeedback,
		.reserved = 0,
	};
	filter.id = *(uint32_t *)&id;
	int err = can_add_rx_filter(cfg->common.phy, rs_can_rx_handler, (void *)dev, &filter);
	if (err < 0) {
		LOG_ERR("Error adding CAN filter (err %d)", err);
		return -1;
	}
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
		can_send_queued(cfg->common.phy, &frame);
		//clear error before enabling
		id.msg_type = Communication_Type_MotorEnable;
		frame.id = *(uint32_t *)&id;
		frame.data[0] = 0x0;
		can_send_queued(cfg->common.phy, &frame);
		data->online = true;
		data->enabled = true;
		break;
	case DISABLE_MOTOR:
		id.msg_type = Communication_Type_MotorStop;
		frame.id = *(uint32_t *)&id;
		can_send_queued(cfg->common.phy, &frame);
		data->enabled = false;
		break;
	case SET_ZERO:
		id.msg_type = Communication_Type_SetPosZero;
		frame.id = *(uint32_t *)&id;
		frame.data[0] = 0x01;
		data->common.angle = 0;
		can_send_queued(cfg->common.phy, &frame);
		break;
	case CLEAR_ERROR:
		id.msg_type = Communication_Type_MotorStop;
		frame.id = *(uint32_t *)&id;
		frame.data[0] = 0x01;
		can_send_queued(cfg->common.phy, &frame);
		data->enabled = false;
		break;
	case AUTO_REPORT_ENABLE:
		id.msg_type = Communication_Type_MotorReport;
		frame.data[0] = 0x01;
		frame.data[1] = 0x02;
		frame.data[2] = 0x03;
		frame.data[3] = 0x04;
		frame.data[4] = 0x05;
		frame.data[5] = 0x06;
		frame.data[6] = 0x01;
		frame.data[7] = 0x00;
		frame.id = *(uint32_t *)&id;
		can_send_queued(cfg->common.phy, &frame);
		data->reporting = true;
		break;
	case AUTO_REPORT_DISABLE:
		id.msg_type = Communication_Type_MotorReport;
		frame.data[0] = 0x01;
		frame.data[1] = 0x02;
		frame.data[2] = 0x03;
		frame.data[3] = 0x04;
		frame.data[4] = 0x05;
		frame.data[5] = 0x06;
		frame.data[6] = 0x00;
		frame.data[7] = 0x00;
		frame.id = *(uint32_t *)&id;
		can_send_queued(cfg->common.phy, &frame);
		data->reporting = false;
		break;
	}
	data->missed_times++;
}

static void rs_motor_pack(const struct device *dev, struct can_frame *frame)
{
	uint32_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp, cur_tep;

	struct rs_motor_data *data = (struct rs_motor_data *)(dev->data);
	struct rs_motor_cfg *cfg = (struct rs_motor_cfg *)(dev->config);
	struct rs_can_id *rs_can_id = (struct rs_can_id *)&(frame->id);

	rs_can_id->motor_id = cfg->common.tx_id;
	rs_can_id->master_id = cfg->common.rx_id;

	frame->dlc = 8;
	frame->flags = 0;

	struct can_frame *frame_follow = &frame[1];
	frame->flags = CAN_FRAME_IDE;
	frame_follow->flags = CAN_FRAME_IDE;
	struct rs_can_id *rs_can_id_fol = (struct rs_can_id *)&(frame_follow->id);
	uint16_t index[2];
	rs_can_id_fol->motor_id = cfg->common.tx_id;
	rs_can_id_fol->master_id = cfg->common.rx_id;
	switch (data->common.mode) {
	case MIT:
		rs_can_id->msg_type = Communication_Type_MotionControl_MIT;

		pos_tmp = float_to_uint(data->target_pos, -cfg->p_max, cfg->p_max, 16);
		vel_tmp = float_to_uint(data->target_radps, -cfg->v_max, cfg->v_max, 16);
		kp_tmp = float_to_uint(data->params.k_p, KP_MIN, KP_MAX, 16);
		kd_tmp = float_to_uint(data->params.k_d, KD_MIN, KD_MAX, 16);
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
}

int rs_motor_set_mode(const struct device *dev, enum motor_mode mode)
{

	struct rs_motor_data *data = dev->data;
	const struct rs_motor_cfg *cfg = dev->config;
	char mode_str[10] = {0};

	data->common.mode = mode;

	switch (mode) {
	case MIT:
		strcpy(mode_str, "mit");
		break;
	default:
		LOG_DBG("Unsupported motor mode: %d", mode);
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
	can_send_queued(cfg->common.phy, &frame);

	for (int i = 0; i < SIZE_OF_ARRAY(cfg->common.capabilities); i++) {
		if (cfg->common.pid_datas[i]->pid_dev == NULL) {
			LOG_ERR("PID params not found for mode: %d", mode);
			break;
		}
		if (strcmp(cfg->common.capabilities[i], mode_str) == 0) {
			struct pid_config params;
			pid_get_params(cfg->common.pid_datas[i], &params);

			data->common.mode = mode;
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
		const struct rs_motor_cfg *cfg = (const struct rs_motor_cfg *)(dev->config);
		if ((cfg->common.tx_id & 0xFF) == (frame->id & 0xFF00) >> 8) {
			return i;
		}
	}
	return -1;
}

static void rs_can_rx_handler(const struct device *can_dev, struct can_frame *frame,
			      void *user_data)
{
	static uint8_t error_fb_cnt = 0;
	const struct device *dev = (const struct device *)user_data;
	struct rs_motor_data *data = (struct rs_motor_data *)(dev->data);
	struct rs_can_id *can_id = (struct rs_can_id *)&(frame->id);
	if (can_id->msg_type == Communication_Type_MotorFeedback ||
	    can_id->msg_type == Communication_Type_MotorReport) {
		if(!data->online){
			data->online = true;
		}
		data->missed_times = 0;
		data->last_report_time = k_uptime_get_32();
		data->err = (can_id->reserved) & 0x3f;
		if (data->err && error_fb_cnt++ % 100 == 0) {
			if (data->err & 0x01) {
				LOG_ERR("Insufficient voltage on motor %s", dev->name);
			}
			if (data->err & 0x02) {
				LOG_ERR("Three-phase current fault on motor %s", dev->name);
			}
			if (data->err & 0x04) {
				LOG_ERR("Overheat on motor %s", dev->name);
			}
			if (data->err & 0x08) {
				LOG_ERR("Magnetic encoder fault on motor %s", dev->name);
			}
			if (data->err & 0x10) {
				LOG_ERR("Motor blocked on motor %s", dev->name);
			}
			if (data->err & 0x20) {
				LOG_ERR("Motor %s is not calibrated", dev->name);
			}
		}
		data->RAWangle = (frame->data[0] << 8) | (frame->data[1]);
		data->RAWrpm = (frame->data[2] << 8) | (frame->data[3]);
		data->RAWtorque = (frame->data[4] << 8) | (frame->data[5]);
		data->RAWtemp = (frame->data[6] << 8) | (frame->data[7]);
	}
}

void rs_rx_monitor_handler(struct k_work *work)
{
	uint32_t current_time = k_uptime_get_32();

    for (int i = 0; i < MOTOR_COUNT; i++) {
        struct rs_motor_data *data = (struct rs_motor_data *)(motor_devices[i]->data);

        if (data->online && data->reporting) {
            uint32_t time_diff = current_time - data->last_report_time;
            if (time_diff > 500) {
                data->online = false;
                LOG_ERR("Motor with CAN ID %s is offline!", motor_devices[i]->name);
            }
        }
	}
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    k_work_reschedule(dwork, K_MSEC(100));
}

int rs_get(const struct device *dev, motor_status_t *status)
{
	struct rs_motor_data *data = dev->data;
	const struct rs_motor_cfg *cfg = dev->config;

	// LOG_INF("raw angle: %d, raw rpm: %d, raw torque: %d, raw temp: %d",data->RAWangle,
	// data->RAWrpm, data->RAWtorque, data->RAWtemp);
	if (!data->online || !data->reporting) {
		return -ENODEV;
	}

	data->common.angle = RAD2DEG * uint16_to_float(data->RAWangle, (double)-cfg->p_max,
						       (double)cfg->p_max, 16);
	data->common.rpm = RADPS2RPM(
		uint16_to_float(data->RAWrpm, (double)-cfg->v_max, (double)cfg->v_max, 16));
	data->common.torque =
		uint16_to_float(data->RAWtorque, (double)-cfg->t_max, (double)cfg->t_max, 16);
	data->common.temperature = ((float)(data->RAWtemp)) / 10.0f;

	// status->angle = fmodf(data->common.angle, 360.0f);
	status->angle = data->common.angle;
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

int rs_send_mit(const struct device *dev, motor_status_t *status)
{
	struct rs_motor_data *data = dev->data;
	const struct rs_motor_cfg *cfg = dev->config;
	struct rs_can_id id = {
		.msg_type = Communication_Type_MotionControl_MIT,
		.motor_id = cfg->common.tx_id,
		.reserved = 0,
	};
	struct can_frame frame = {
		.data = {0},
		.dlc = 8,
		.flags = CAN_FRAME_IDE,
	};
	uint16_t pos_tmp = float_to_uint(data->target_pos, -cfg->p_max, cfg->p_max, 16);
	uint16_t vel_tmp = float_to_uint(data->target_radps, -cfg->v_max, cfg->v_max, 16);
	uint16_t kp_tmp = float_to_uint(data->params.k_p, KP_MIN, KP_MAX, 16);
	uint16_t kd_tmp = float_to_uint(data->params.k_d, KD_MIN, KD_MAX, 16);
	uint16_t tor_tmp = float_to_uint(data->target_torque, 0, cfg->t_max, 16);
	id.master_id = tor_tmp;
	frame.data[0] = (pos_tmp >> 8) & 0xFF;
	frame.data[1] = pos_tmp & 0xFF;
	frame.data[2] = (vel_tmp >> 8) & 0xFF;
	frame.data[3] = vel_tmp & 0xFF;
	frame.data[4] = (kp_tmp >> 8) & 0xFF;
	frame.data[5] = kp_tmp & 0xFF;
	frame.data[6] = (kd_tmp >> 8) & 0xFF;
	frame.data[7] = kd_tmp & 0xFF;
	can_send_queued(cfg->common.phy, &frame);
}

int rs_send_pid_params(const struct device *dev, motor_status_t *status)
{
	struct rs_motor_data *data = dev->data;
	const struct rs_motor_cfg *cfg = dev->config;
	struct rs_can_id id = {
		.msg_type = Communication_Type_SetSingleParameter,
		.motor_id = cfg->common.tx_id,
		.reserved = 0,
	};
}

/**
 * @brief Set motor status: angle, speed, torque, mode
 * @param dev Pointer to the motor device structure: robstride motor device
 * @param status Pointer to the motor status structure
 */
int rs_set(const struct device *dev, motor_status_t *status)
{
	struct rs_motor_data *data = dev->data;
	const struct rs_motor_cfg *cfg = dev->config;
	char mode_str[10] = {0};
	if (status->mode == MIT) {
		strcpy(mode_str, "mit");
		data->target_pos = status->angle / RAD2DEG;
		data->target_radps = RPM2RADPS(status->rpm);
		data->target_torque = status->torque;
	} else {
		return -ENOSYS;
	}
	if (status->mode == MIT) {
		for (int i = 0; i < SIZE_OF_ARRAY(cfg->common.capabilities); i++) {
			if (cfg->common.pid_datas[i]->pid_dev == NULL) {
				break;
			}
			if (strcmp(cfg->common.capabilities[i], mode_str) == 0) {
				struct pid_config params;
				pid_get_params(cfg->common.pid_datas[i], &params);

				data->common.mode = status->mode;
				data->params.k_p = params.k_p;
				data->params.k_d = params.k_d;
				break;
			}
		}
	}
	struct can_frame frame = {0};
	rs_motor_pack(dev, &frame);
	can_send_queued(cfg->common.phy, &frame);
	data->missed_times++;
	if (data->missed_times > 3) {
		data->online = false;
		LOG_ERR("Motor %s is not responding", dev->name);
	}
	// else{
	// 	LOG_ERR("Unsupported motor mode: %d", status->mode);
	// 	return -ENOSYS;
	// }
	if (status->mode != data->common.mode) {
		// LOG_DBG("rs_set: mode changed from %d to %d", data->common.mode, status->mode);
		data->common.mode = status->mode;
		if (rs_motor_set_mode(dev, status->mode) < 0) {
			LOG_ERR("Failed to set motor mode");
			return -EIO;
		}
	} else {
		return 0; // No mode change, no need to set
	}
}

DT_INST_FOREACH_STATUS_OKAY(RS_MOTOR_INST)
