/*
 * motor_dji.h
 *
 * Header file for motor_dji.c
 */

#ifndef MOTOR_DJI_H
#define MOTOR_DJI_H

#include <stdbool.h>
#include <stdint.h>
#include "dji_ratios.h"
#include "dji_macros.h"
#include "zephyr/kernel.h"
#include "zephyr/spinlock.h"
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/motor.h>

#define CAN_TX_ID_CNT 8

/*  canbus_id_t specifies the ID of the CAN bus the motor is on
	which is defined in motor_devices[] */
typedef uint8_t canbus_id_t;

/*  allmotor_id_t specifies the ID of the motor
	which is in the flags
	use flags |= 1 << all_motor_id*/
typedef uint16_t allmotor_id_t;

/*  motor_id_t specifies the ID of the motor in motor_controller struct
	find the rpm in motor_cans->target_rpm[canbus_id] */
typedef uint16_t motor_id_t;

struct k_work_q dji_work_queue;

struct motor_controller {
	struct device *can_dev;

	/*
	  There are 4 tx addresses
	  0x1FF, 0x200 for M3508/M2006
	  0x1FE, 0x2FE for GM6020 Current control.
	  The 5 nums in full[] are: 0x200, 0x1FF, 0x1FE, 0x2FE, 0x2FF, 0x300
      */
	int rx_ids[8];
	bool full[CAN_TX_ID_CNT];
	int8_t mapping[CAN_TX_ID_CNT][4];
	uint8_t flags;
	uint8_t mask[CAN_TX_ID_CNT];
	struct device *motor_devs[8];

	struct k_work full_handle;
};

struct dji_motor_data {
	struct motor_driver_data common;
	canbus_id_t canbus_id;
	struct motor_controller *ctrl_struct;

	// Control status
	bool online;
	uint8_t convert_num;
	int8_t current_mode_index;

	// RAW DATA
	uint16_t RAWangle;
	uint16_t RAWprev_angle;
	int16_t RAWcurrent;
	int16_t RAWrpm;
	int8_t RAWtemp;
	int32_t angle_add;

		uint32_t curr_time;
		uint32_t prev_time;
	int8_t missed_times;

	float angle_offset;

	float position_error;

	struct k_spinlock data_input_lock;

	// Target
	float target_angle;
	float target_rpm;
	float target_torque;
	float target_torque_ff;
	float target_current;
	bool calculated;
	bool new_data;
};

struct dji_motor_config {
	struct motor_driver_config common;

	float gear_ratio;
	bool is_gm6020;
	bool is_m3508;
	bool is_m2006;
	bool is_dm_motor;
	float dm_i_max;
	float dm_torque_ratio;
	const struct device *follow;

	bool minor_arc;
	bool inverse;
};

// 全局变量声明
extern struct motor_controller ctrl_structs[];

int dji_get(const struct device *dev, motor_status_t *status);
int dji_set(const struct device *dev, motor_setpoint_t *status);
int dji_init(const struct device *dev);

void dji_control(const struct device *dev, enum motor_cmd cmd);

void dji_tx_handler(struct k_work *work);
void dji_miss_handler(struct k_work *work);

void dji_miss_isr_handler(struct k_timer *dummy);
void dji_init_isr_handler(struct k_timer *dummy);

void dji_init_handler(struct k_work *work);

K_THREAD_STACK_DEFINE(dji_work_queue_stack, CAN_SEND_STACK_SIZE);

K_WORK_DEFINE(dji_miss_handle, dji_miss_handler);
K_WORK_DEFINE(dji_init_handle, dji_init_handler);

K_TIMER_DEFINE(dji_miss_handle_timer, NULL, NULL);

static const struct motor_driver_api motor_api_funcs = {
	.motor_get = dji_get,
	.motor_set = dji_set,
	.motor_control = dji_control,
};

extern const struct device *can_devices[];
extern const struct device *motor_devices[];

#endif // MOTOR_DJI_H
