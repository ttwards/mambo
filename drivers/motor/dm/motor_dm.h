/*
 * motor_dm.h
 *
 * Header file for motor_dm.c
 */

#ifndef MOTOR_DM_H
#define MOTOR_DM_H

#include "zephyr/kernel.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/motor.h>
#include <zephyr/drivers/pid.h>
#include <zephyr/drivers/can.h>

#define DT_DRV_COMPAT dm_motor

#define INT12_MAX 0x7FF
#define INT12_MIN -0x800

#define PI 3.14159265f

#define RAD2ROUND 1.0f / (2 * PI)
#define RAD2DEG   (180.0f / PI)
#define DEG2RAD   PI / 180.0f

#define CANID_L 0u
#define CANID_H 1u
#define RID     3u

static const uint8_t enable_frame[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
static const uint8_t disable_frame[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
static const uint8_t set_zero_frame[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
static const uint8_t clear_error_frame[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};

#define CAN_SEND_STACK_SIZE 1536
#define CAN_SEND_PRIORITY   -1

#define SIZE_OF_ARRAY(x) (sizeof(x) / sizeof(x[0]))

struct dm_motor_data {
	struct motor_driver_data common;

	int tx_offset;
	struct can_filter filter;

	// Control status
	bool online;
	bool enable;
	bool enabled;
	bool update;
	uint8_t tx_cnt;

	uint64_t prev_recv_time;
	int8_t err;

	// Process round
	float delta_deg_sum; // Will be % by 360

	// Target
	float target_angle;
	float target_radps;
	float target_torque;

	int16_t RAWangle;
	int16_t RAWrpm;
	int16_t RAWtorque;

	struct pid_config params;

	uint64_t last_tx_time;
};

struct dm_motor_config {
	struct motor_driver_config common;

	float gear_ratio;
	int freq;

	float v_max;
	float p_max;
	float t_max;
};

struct k_work_q dm_work_queue;

// 函数声明
int dm_set(const struct device *dev, motor_status_t *status);
void dm_control(const struct device *dev, enum motor_cmd cmd);
int dm_get(const struct device *dev, motor_status_t *status);
void dm_motor_set_mode(const struct device *dev, enum motor_mode mode);

void dm_rx_data_handler(struct k_work *work);

void dm_tx_isr_handler(struct k_timer *dummy);
void dm_tx_data_handler(struct k_work *work);

void dm_isr_init_handler(struct k_timer *dummy);
void dm_init_handler(struct k_work *work);

static const struct motor_driver_api motor_api_funcs = {
	.motor_get = dm_get,
	.motor_set = dm_set,
	.motor_set_mode = dm_motor_set_mode,
	.motor_control = dm_control,
};

#define MOTOR_COUNT            DT_NUM_INST_STATUS_OKAY(dm_motor)
#define DM_MOTOR_POINTER(inst) DEVICE_DT_GET(DT_DRV_INST(inst)),
static const struct device *motor_devices[] = {DT_INST_FOREACH_STATUS_OKAY(DM_MOTOR_POINTER)};

K_THREAD_STACK_DEFINE(dm_work_queue_stack, CAN_SEND_STACK_SIZE);

CAN_MSGQ_DEFINE(dm_can_rx_msgq, 12);
K_MSGQ_DEFINE(dm_thread_proc_msgq, sizeof(bool), MOTOR_COUNT * 2, 4);

K_WORK_DEFINE(dm_rx_data_handle, dm_rx_data_handler);
K_WORK_DEFINE(dm_tx_data_handle, dm_tx_data_handler);

K_WORK_DEFINE(dm_init_work, dm_init_handler);

K_TIMER_DEFINE(dm_tx_timer, dm_tx_isr_handler, NULL);

#define DMMOTOR_DATA_INST(inst)                                                                    \
	static struct dm_motor_data dm_motor_data_##inst = {                                       \
		.common = MOTOR_DT_DRIVER_DATA_INST_GET(inst),                                     \
		.tx_offset = 0,                                                                    \
		.online = true,                                                                    \
		.enable = true,                                                                    \
		.enabled = true,                                                                   \
		.prev_recv_time = 0,                                                               \
		.err = 0,                                                                          \
		.delta_deg_sum = 0,                                                                \
		.target_angle = 0,                                                                 \
		.target_radps = 0,                                                                 \
		.target_torque = 0,                                                                \
		.params = {0, 0},                                                                  \
		.update = false,                                                                   \
	};

#define DMMOTOR_CONFIG_INST(inst)                                                                  \
	static const struct dm_motor_config dm_motor_cfg_##inst = {                                \
		.common = MOTOR_DT_DRIVER_CONFIG_INST_GET(inst),                                   \
		.v_max = (float)DT_STRING_UNQUOTED_OR(DT_DRV_INST(inst), v_max, 12.5),             \
		.p_max = (float)DT_STRING_UNQUOTED_OR(DT_DRV_INST(inst), p_max, 20),               \
		.t_max = (float)DT_STRING_UNQUOTED_OR(DT_DRV_INST(inst), t_max, 200),              \
		.freq = (float)DT_PROP_OR(DT_DRV_INST(inst), freq, 500),                           \
	};

#define MOTOR_DEVICE_DT_DEFINE(node_id, init_fn, pm, data, config, level, prio, api, ...)          \
	DEVICE_DT_DEFINE(node_id, init_fn, pm, data, config, level, prio, api, __VA_ARGS__)

#define MOTOR_DEVICE_DT_INST_DEFINE(inst, ...)                                                     \
	MOTOR_DEVICE_DT_DEFINE(DT_DRV_INST(inst), __VA_ARGS__)

#define DMMOTOR_DEFINE_INST(inst)                                                                  \
	MOTOR_DEVICE_DT_INST_DEFINE(inst, dm_init, NULL, &dm_motor_data_##inst,                    \
				    &dm_motor_cfg_##inst, POST_KERNEL, CONFIG_MOTOR_INIT_PRIORITY, \
				    &motor_api_funcs);

#define DMMOTOR_INST(inst)                                                                         \
	MOTOR_DT_DRIVER_PID_DEFINE(DT_DRV_INST(inst))                                              \
	DMMOTOR_CONFIG_INST(inst)                                                                  \
	DMMOTOR_DATA_INST(inst)                                                                    \
	DMMOTOR_DEFINE_INST(inst)

#endif // MOTOR_DM_H
