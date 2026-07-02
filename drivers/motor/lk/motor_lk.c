#include "motor_lk.h"
#include "zephyr/device.h"
#include "zephyr/drivers/can.h"
#include "../common/common.h"
#include "../common/motor_can_sched.h"
#include "zephyr/drivers/motor.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <sys/_stdint.h>

#define DT_DRV_COMPAT lk_motor

LOG_MODULE_REGISTER(motor_lk, CONFIG_MOTOR_LOG_LEVEL);

struct k_sem tx_frame_sem;

int float_to_int(float x, float x_min, float x_max, int bits)
{

	if (x > x_max) {
		x = x_max;
	} else if (x < x_min) {
		x = x_min;
	}
	float x_span = x_max - x_min;
	int target_min = -(1 << (bits - 1));
	int target_max = (1 << (bits - 1)) - 1;
	float target_span = (float)(target_max - target_min);
	return (int)((x - x_min) / x_span * target_span + (float)target_min);
}
static atomic_t global_init_once = ATOMIC_INIT(0);

int lk_init(const struct device *dev)
{
	const struct lk_motor_cfg *cfg = dev->config;
	if (!device_is_ready(cfg->common.phy)) {
		motor_stats_inc(MOTOR_STAT_CONFIG_ERROR);
		return -ENODEV;
	}
	if (atomic_cas(&global_init_once, 0, 1)) {
		k_work_queue_init(&lk_work_queue);
		k_work_queue_start(&lk_work_queue, lk_work_queue_stack, CAN_SEND_STACK_SIZE,
				   CAN_SEND_PRIORITY, NULL);
		k_work_submit_to_queue(&lk_work_queue, &lk_init_work);
	}
	return 0;
}

void lk_motor_control(const struct device *dev, enum motor_cmd cmd)
{
	struct lk_motor_data *data = dev->data;
	const struct lk_motor_cfg *cfg = dev->config;

	struct can_frame frame = {0};
	frame.flags = 0;
	frame.dlc = 8;
	// for (int i = 0; i < 33; i++) {

	frame.id = LK_CMD_ID_BASE + cfg->id; // 0x140 + ID
	// frame.id = LK_CMD_ID_BASE + i;
	switch (cmd) {
	case ENABLE_MOTOR:
		frame.data[0] = LK_CMD_MOTOR_RUN; // 0x88
		motor_can_sched_send_prio(cfg->common.phy, &frame, true, "lk-enable");
		k_msleep(10);
		data->online = true;
		data->enabled = true;
		break;
	case DISABLE_MOTOR:
		frame.data[0] = LK_CMD_MOTOR_OFF; // 0x80
		motor_can_sched_send_prio(cfg->common.phy, &frame, true, "lk-disable");
		data->online = false;
		data->enabled = false;
		break;
	case SET_ZERO:
		// 设置当前位置为零点 (写入ROM)
		// frame.data[0] = LK_CMD_SET_ZERO_ROM;

		// 设置当前位置为零点
		frame.data[0] = LK_CMD_SET_ZERO; // 0x95
		int32_t set_angle = 180;
		frame.data[4] = ((uint32_t)(set_angle * LK_POS_FACTOR)) & 0xFF;
		frame.data[5] = (((uint32_t)(set_angle * LK_POS_FACTOR)) >> 8) & 0xFF;
		frame.data[6] = (((uint32_t)(set_angle * LK_POS_FACTOR)) >> 16) & 0xFF;
		frame.data[7] = (((uint32_t)(set_angle * LK_POS_FACTOR)) >> 24) & 0xFF;
		motor_can_sched_send_prio(cfg->common.phy, &frame, true, "lk-set-zero");
		break;
	case CLEAR_ERROR:
		frame.data[0] = LK_CMD_CLEAR_ERR; // 0x9B
		motor_can_sched_send_prio(cfg->common.phy, &frame, true, "lk-clear-error");
		break;
	case CLEAR_CONTROLLER:
		break;
	default:
		motor_stats_inc(MOTOR_STAT_UNSUPPORTED_CMD);
		return;
	}
	// }
}

static void lk_motor_pack(const struct device *dev, struct can_frame *frame)
{
	struct lk_motor_data *data = (struct lk_motor_data *)(dev->data);
	const struct lk_motor_cfg *cfg = (struct lk_motor_cfg *)(dev->config);

	frame->id = LK_CMD_ID_BASE + cfg->id; // 标准ID: 0x140 + ID
	frame->dlc = 8;
	frame->flags = 0; // 标准帧

	int32_t val_i32;
	int16_t val_i16;

	switch (data->common.mode) {
	case VO:
		if (data->common.target == MOTOR_TARGET_SPEED) {
			frame->data[0] = LK_CMD_SPEED_LOOP;
			frame->data[1] = 0x00;

			val_i16 = (int16_t)float_to_int(data->limit_torque, 0, 12, 12);
			frame->data[2] = val_i16 & 0xFF;
			frame->data[3] = (val_i16 >> 8) & 0xFF;

			val_i32 = (int32_t)(data->target_speed * LK_SPD_FACTOR_FINE);
			frame->data[4] = val_i32 & 0xFF;
			frame->data[5] = (val_i32 >> 8) & 0xFF;
			frame->data[6] = (val_i32 >> 16) & 0xFF;
			frame->data[7] = (val_i32 >> 24) & 0xFF;
			break;
		}
		if (data->common.target != MOTOR_TARGET_TORQUE) {
			frame->data[0] = LK_CMD_READ_STAT;
			break;
		}
		frame->data[0] = LK_CMD_TORQUE_LOOP;
		frame->data[1] = 0x00;
		frame->data[2] = 0x00;
		frame->data[3] = 0x00;

		// iqControl (-2048 ~ 2048)
		val_i16 = (int16_t)float_to_int(data->target_torque, -12, 12, 12);
		frame->data[4] = val_i16 & 0xFF;
		frame->data[5] = (val_i16 >> 8) & 0xFF;
		frame->data[6] = 0x00;
		frame->data[7] = 0x00;
		break;

	case PV:
		if (data->common.target != MOTOR_TARGET_POSITION) {
			frame->data[0] = LK_CMD_READ_STAT;
			break;
		}
		frame->data[0] = LK_CMD_POS_LOOP_MULTI;
		frame->data[1] = 0x00;

		// 速度限制 (1 dps/LSB)
		val_i16 = (int16_t)data->limit_speed;
		frame->data[2] = val_i16 & 0xFF;
		frame->data[3] = (val_i16 >> 8) & 0xFF;

		// 位置控制 (0.01 degree/LSB)
		val_i32 = (int32_t)(data->target_pos * LK_POS_FACTOR);
		frame->data[4] = val_i32 & 0xFF;
		frame->data[5] = (val_i32 >> 8) & 0xFF;
		frame->data[6] = (val_i32 >> 16) & 0xFF;
		frame->data[7] = (val_i32 >> 24) & 0xFF;
		break;

	default:

		frame->data[0] = LK_CMD_READ_STAT;
		break;
	}
}

int lk_set(const struct device *dev, motor_setpoint_t *status)
{
	struct lk_motor_data *data = dev->data;
	const struct lk_motor_cfg *cfg = dev->config;
	motor_controller_info_t controller = {0};
	int ret;

	ret = motor_resolve_controller(dev, status, &controller);
	if (ret < 0) {
		motor_stats_inc(MOTOR_STAT_UNSUPPORTED_MODE);
		return ret;
	}
	if (status->target == MOTOR_TARGET_NONE) {
		return 0;
	}

	switch (status->mode) {
	case PV:
		if (status->target != MOTOR_TARGET_POSITION) {
			return -ENOSYS;
		}
		data->common.mode = PV;
		data->common.target = MOTOR_TARGET_POSITION;
		data->common.controller_id = status->controller_id;
		data->target_pos = status->angle;
		if (status->speed_limit[0] == 0) {
			data->limit_speed = 8000.0f; // 默认800 dps

		} else {
			data->limit_speed = fabsf(status->speed_limit[0] * 6.0f); // RPM -> dps
		}
		break;

	case VO:
		data->common.mode = VO;
		data->common.target = status->target;
		data->common.controller_id = status->controller_id;
		if (status->target == MOTOR_TARGET_SPEED) {
			data->target_speed = status->rpm * 6.0f;
			if (status->torque_limit[0] == 0) {
				data->limit_torque = 3.0f;
			} else {
				data->limit_torque = fabsf(status->torque_limit[0]);
			}
		} else if (status->target == MOTOR_TARGET_TORQUE) {
			data->target_torque = status->torque;
		} else {
			return -ENOSYS;
		}
		break;

	default:
		return -ENOSYS;
	}
	for (int i = 0; i < motor_get_controller_count(dev); i++) {
		const struct motor_controller_config *ctrl_cfg = &cfg->common.controllers[i];
		int controller_param_idx = -1;

		// 0->Angle, 1->Speed, 2->Torque
		if (ctrl_cfg->info.mode == PV &&
		    ctrl_cfg->info.target == MOTOR_TARGET_POSITION) {
			controller_param_idx = 0;
		} else if (ctrl_cfg->info.mode == VO &&
			   ctrl_cfg->info.target == MOTOR_TARGET_SPEED) {
			controller_param_idx = 1;
		} else if (ctrl_cfg->info.mode == VO &&
			   ctrl_cfg->info.target == MOTOR_TARGET_TORQUE) {

			controller_param_idx = 2;
		}

		if (controller_param_idx != -1 && ctrl_cfg->param_count > 0) {
			struct motor_controller_params params = {0};

			if (motor_controller_get_params(ctrl_cfg, 0, &params) < 0) {
				continue;
			}

			if (data->params[controller_param_idx].k_p != params.k_p ||
			    data->params[controller_param_idx].k_d != params.k_d) {

				data->params[controller_param_idx].k_p = params.k_p;
				data->params[controller_param_idx].k_d = params.k_d;

				data->params_update[controller_param_idx] = true;
				k_work_submit_to_queue(&lk_work_queue, &lk_tx_params_data_handle);
			}
		}
	}

	return 0;
}

static int get_motor_id(struct can_frame *frame)
{
	// LK协议回复帧ID: 0x180 + ID
	int id_val = frame->id - LK_CMD_ID_BASE;

	for (int i = 0; i < MOTOR_COUNT; i++) {
		const struct device *dev = motor_devices[i];
		const struct lk_motor_cfg *cfg = (const struct lk_motor_cfg *)(dev->config);
		if (cfg->id == id_val) {
			return i;
		}
	}
	return -1;
}

static struct can_filter filters[MOTOR_COUNT];

static void lk_can_rx_handler(const struct device *can_dev, struct can_frame *frame,
			      void *user_data)
{
	motor_can_sched_report_rx(can_dev, frame);
	int idx = get_motor_id(frame);
	if (idx == -1) {
		return;
	}

	struct lk_motor_data *data = (struct lk_motor_data *)(motor_devices[idx]->data);
	data->missed_times--;
	if (!data->online) {
		data->missed_times = 0;
		data->online = true;
	}
	// Data[0]: 命令字节
	// Data[1]: 温度
	// Data[2-3]: 转矩电流/功率
	// Data[4-5]: 速度
	// Data[6-7]: 编码器位置
	if (frame->data[0] == 0x9A) {
		for (int i = 0; i < 8; i++) {
			if (frame->data[7] >> i & 0x01) {
				motor_stats_inc(MOTOR_STAT_DRIVER_ERROR);
			}
		}
		return;
	}
	data->RAWtemp = (int8_t)frame->data[1];
	data->RAWtorque = (int16_t)(frame->data[2] | (frame->data[3] << 8));
	data->RAWspeed = (int16_t)(frame->data[4] | (frame->data[5] << 8));
	data->RAWencoder = (uint16_t)(frame->data[6] | (frame->data[7] << 8));

	data->update = true;

	k_work_submit_to_queue(&lk_work_queue, &lk_rx_data_handle);
}

void lk_rx_data_handler(struct k_work *work)
{
	for (int i = 0; i < MOTOR_COUNT; i++) {
		struct lk_motor_data *data = (struct lk_motor_data *)(motor_devices[i]->data);
		if (!data->update) {
			continue;
		}
		data->common.temperature = (float)data->RAWtemp; // 1 degC/LSB [cite: 77]
		data->common.torque =
			(float)data->RAWtorque * (16.0f / 4096.0f); // 33Nm满量程，12bit ADC
		// 速度转换: 1 dps/LSB -> RPM
		data->common.rpm = ((float)data->RAWspeed) / 6.0f;
		data->common.angle = (float)data->RAWencoder * (360.0f / 65535.0f);
		data->update = false;
	}
}

void lk_tx_isr_handler(struct k_timer *dummy)
{
	k_work_submit_to_queue(&lk_work_queue, &lk_tx_data_handle);
}

// 发送处理线程
void lk_tx_data_handler(struct k_work *work)
{
	struct can_frame tx_frame = {0};

	for (int i = 0; i < MOTOR_COUNT; i++) {
		struct lk_motor_data *data = motor_devices[i]->data;
		const struct lk_motor_cfg *cfg = motor_devices[i]->config;

		if (data->enabled) {
			if (data->missed_times > 10 && data->online) {
				LOG_ERR("Motor [%d] OFFLINE (No feedback)", cfg->id);

				data->online = false;
			}
			if (data->missed_times < 100) {
				data->missed_times++;
			}
			// if( !data->online) {
			// 	continue;
			// }
			lk_motor_pack(motor_devices[i], &tx_frame);
			motor_can_sched_send_reply(cfg->common.phy, &tx_frame,
						   LK_CMD_ID_BASE + cfg->id, CAN_STD_ID_MASK,
						   5U, "lk-control");
		}
		// if (i % 2 == 1) {
		//     k_usleep(500);
		// }
	}
}
void lk_tx_params_data_handler(struct k_work *work)
{
	struct can_frame tx_frame = {0};
	k_msleep(1000);
	for (int i = 0; i < MOTOR_COUNT; i++) {
		struct lk_motor_data *data = motor_devices[i]->data;
		const struct lk_motor_cfg *cfg = motor_devices[i]->config;
		if (data->params_update[0] || data->params_update[1] ||
		    data->params_update[2]) {
			tx_frame.id = LK_CMD_ID_BASE + cfg->id; // 0x160 + ID
			tx_frame.dlc = 8;
			tx_frame.flags = 0; // 标准帧

			// 优先更新角度、速度、转矩控制器参数
			if (data->params_update[0]) {
				tx_frame.data[0] = LK_SET_PARAM;
				tx_frame.data[1] = LK_PARAM_ANGLE_UPDATE;
				data->params_update[0] = false;

				// Kp
				int16_t kp_val = (int16_t)(data->params[0].k_p);
				tx_frame.data[2] = kp_val & 0xFF;
				tx_frame.data[3] = (kp_val >> 8) & 0xFF;

				// Kd
				int16_t ki_val = (int16_t)(data->params[0].k_i);
				tx_frame.data[4] = ki_val & 0xFF;
				tx_frame.data[5] = (ki_val >> 8) & 0xFF;
				int16_t kd_val = (int16_t)(data->params[0].k_d);
				tx_frame.data[6] = kd_val & 0xFF;
				tx_frame.data[7] = (kd_val >> 8) & 0xFF;
				motor_can_sched_send_prio(cfg->common.phy, &tx_frame, true,
							  "lk-param-angle");
			}
			k_sleep(K_USEC(120));
			if (data->params_update[1]) {
				tx_frame.data[0] = LK_SET_PARAM;
				tx_frame.data[1] = LK_PARAM_SPEED_UPDATE;
				data->params_update[1] = false;

				// Kp
				int16_t kp_val = (int16_t)(data->params[1].k_p);
				tx_frame.data[2] = kp_val & 0xFF;
				tx_frame.data[3] = (kp_val >> 8) & 0xFF;

				// Kd
				int16_t ki_val = (int16_t)(data->params[1].k_i);
				tx_frame.data[4] = ki_val & 0xFF;
				tx_frame.data[5] = (ki_val >> 8) & 0xFF;
				int16_t kd_val = (int16_t)(data->params[1].k_d);
				tx_frame.data[6] = kd_val & 0xFF;
				tx_frame.data[7] = (kd_val >> 8) & 0xFF;
				motor_can_sched_send_prio(cfg->common.phy, &tx_frame, true,
							  "lk-param-speed");
			}
			k_sleep(K_USEC(120));
			if (data->params_update[2]) {
				tx_frame.data[0] = LK_SET_PARAM;
				tx_frame.data[1] = LK_PARAM_TORQUE_UPDATE;
				data->params_update[2] = false;

				// Kp
				int16_t kp_val = (int16_t)(data->params[2].k_p);
				tx_frame.data[2] = kp_val & 0xFF;
				tx_frame.data[3] = (kp_val >> 8) & 0xFF;

				// Kd
				int16_t ki_val = (int16_t)(data->params[2].k_i);
				tx_frame.data[4] = ki_val & 0xFF;
				tx_frame.data[5] = (ki_val >> 8) & 0xFF;
				int16_t kd_val = (int16_t)(data->params[2].k_d);
				tx_frame.data[6] = kd_val & 0xFF;
				tx_frame.data[7] = (kd_val >> 8) & 0xFF;
				motor_can_sched_send_prio(cfg->common.phy, &tx_frame, true,
							  "lk-param-torque");
			}
			k_sleep(K_USEC(120));
		}
	}
}
void lk_init_handler(struct k_work *work)
{
	for (int i = 0; i < MOTOR_COUNT; i++) {

		const struct lk_motor_cfg *cfg =
			(const struct lk_motor_cfg *)(motor_devices[i]->config);
		struct lk_motor_data *data = motor_devices[i]->data;
		motor_can_sched_register_can(cfg->common.phy);
		data->params_update[0] = true;
		data->params_update[1] = true;
		data->params_update[2] = true;
		// 注册 CAN 滤波器
		// 接收 ID = 0x180 + ID
		filters[i].flags = 0; // 标准帧
		filters[i].id = LK_CMD_ID_BASE + cfg->id;
		filters[i].mask = CAN_STD_ID_MASK; // 精确匹配
						   //    filters[i].id = 0x00000000;
						   //    filters[i].mask = 0x00000000;

		int err = can_add_rx_filter(cfg->common.phy, lk_can_rx_handler, 0, &filters[i]);
		if (err < 0) {
			motor_stats_inc(MOTOR_STAT_CAN_FILTER_ERROR);
		}
		for (int ctrl_idx = 0; ctrl_idx < motor_get_controller_count(motor_devices[i]);
		     ctrl_idx++) {
			const struct motor_controller_config *ctrl_cfg =
				&cfg->common.controllers[ctrl_idx];
			int controller_param_idx = -1;

			// 0->Angle, 1->Speed, 2->Torque
			if (ctrl_cfg->info.mode == PV &&
			    ctrl_cfg->info.target == MOTOR_TARGET_POSITION) {
				controller_param_idx = 0;
			} else if (ctrl_cfg->info.mode == VO &&
				   ctrl_cfg->info.target == MOTOR_TARGET_SPEED) {
				controller_param_idx = 1;
			} else if (ctrl_cfg->info.mode == VO &&
				   ctrl_cfg->info.target == MOTOR_TARGET_TORQUE) {
				controller_param_idx = 2;
			}

			if (controller_param_idx != -1 && ctrl_cfg->param_count > 0) {
				struct motor_controller_params params = {0};

				if (motor_controller_get_params(ctrl_cfg, 0, &params) < 0) {
					continue;
				}

				if (data->params[controller_param_idx].k_p != params.k_p ||
				    data->params[controller_param_idx].k_d != params.k_d) {
					data->params[controller_param_idx].k_p = params.k_p;
					data->params[controller_param_idx].k_d = params.k_d;
					data->params_update[controller_param_idx] = true;
				}
			}
		}
		lk_motor_control(motor_devices[i], ENABLE_MOTOR);
		k_sleep(K_USEC(120));
		// lk_motor_control(motor_devices[i], SET_ZERO);
	}
	k_work_submit_to_queue(&lk_work_queue, &lk_tx_params_data_handle);
	lk_tx_timer.expiry_fn = lk_tx_isr_handler;
	k_timer_start(&lk_tx_timer, K_MSEC(600), K_MSEC(4));
	k_timer_user_data_set(&lk_tx_timer, &lk_tx_data_handle);
}

int lk_get(const struct device *dev, motor_status_t *status)
{
	struct lk_motor_data *data = dev->data;
	status->angle = data->common.angle;
	status->rpm = data->common.rpm;
	status->torque = data->common.torque;
	status->temperature = data->common.temperature;
	status->mode = data->common.mode;
	status->target = data->common.target;
	status->controller_id = data->common.controller_id;
	return 0;
}

DT_INST_FOREACH_STATUS_OKAY(LKMOTOR_INST)
