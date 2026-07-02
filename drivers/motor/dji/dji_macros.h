/*
 * Copyright (c) 2024 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/motor.h>

#define CAN_SEND_STACK_SIZE 2048
#define CAN_SEND_PRIORITY   -1

#define HIGH_BYTE(x)           ((x) >> 8)
#define LOW_BYTE(x)            ((x) & 0xFF)
#define COMBINE_HL8(HIGH, LOW) ((HIGH << 8) + LOW)

#define SIZE_OF_ARRAY(arr) (sizeof(arr) / sizeof(arr[0]))

/** @brief Macros for each in dts */
#define MOTOR_NODE   DT_NODELABEL(motor)
#define CAN_BUS_NODE DT_NODELABEL(canbus)

#define CAN_BUS_PATH DT_PATH(canbus)

#define IS_DJI_COMPAT(node_id)                                                                     \
	(DT_NODE_HAS_COMPAT(node_id, dji_m3508) || DT_NODE_HAS_COMPAT(node_id, dji_m2006) ||       \
	 DT_NODE_HAS_COMPAT(node_id, dji_m6020) ||                                                 \
	 DT_NODE_HAS_COMPAT(node_id, dji_others)) // Add other DJI compatible strings as needed

#define DJI_DEVICE_POINTER(inst)    DEVICE_DT_GET(DT_DRV_INST(inst)),
#define CAN_DEVICE_POINTER(node_id) DEVICE_DT_GET(DT_PROP(node_id, can_device))

#define GET_CANPHY_POINTER_BY_IDX(node_id, idx)                                                    \
	DEVICE_DT_GET(DT_PHANDLE(DT_CHILD_BY_IDX(node_id, idx)))
#define CANPHY_BY_IDX(idx) GET_CANPHY_POINTER_BY_IDX(CAN_BUS_PATH, idx)

#define DJI_MOTOR_COUNT sizeof(motor_devices) / sizeof(motor_devices[0])

#define GET_CAN_CHANNEL_IDT(node_id) DT_PHANDLE(node_id, can_channel)
#define GET_CAN_DEV(node_id)         DEVICE_DT_GET(DT_PHANDLE(node_id, can_device))
#define GET_CTRL_ID(node_id)         DT_PHANDLE(node_id, id)

/**
 * @brief Static initializer for @p motor_driver_config struct
 *
 * @param node_id Devicetree node identifier
 */

#define DT_DRIVER_GET_CANPHY(inst) DT_GET_CANPHY(DT_DRIVER_GET_CANBUS_IDT(inst))

#define DT_DRIVER_INST_GET_MOTOR_IDT(inst) DT_DRV_INST(inst)

#define DT_DRIVER_INST_GET_CANBUS_IDT(inst)                                                        \
	DT_PHANDLE(DT_DRIVER_INST_GET_MOTOR_IDT(inst), can_channel)

#define DT_DRIVER_GET_CANBUS_ID(inst) DT_NODE_CHILD_IDX(DT_DRIVER_INST_GET_CANBUS_IDT(inst))

#define DT_MOTOR_NAME(node)      DT_NODE_FULL_NAME_UNQUOTED(node)
#define DT_MOTOR_NAME_INST(inst) DT_MOTOR_NAME(DT_DRV_INST(inst))

#define DMOTOR_DATA(inst, node, name)                                                              \
	static struct dji_motor_data DT_CAT(dji_motor_data_, name) = {                             \
		.common = MOTOR_DT_DRIVER_DATA_INST_GET(inst),                                     \
		.canbus_id = 0,                                                                    \
		.ctrl_struct = NULL,                                                               \
		.online = false,                                                                   \
		.enabled = false,                                                                  \
		.convert_num = 0,                                                                  \
		.current_mode_index = -1,                                                          \
		.RAWangle = 0,                                                                     \
		.RAWprev_angle = 0,                                                                \
		.RAWcurrent = 0,                                                                   \
		.RAWrpm = 0,                                                                       \
		.RAWtemp = 0,                                                                      \
		.angle_add = 0,                                                                    \
		.curr_time = 0,                                                                    \
		.prev_time = 0,                                                                    \
		.missed_times = 0,                                                                 \
		.angle_offset = 0,                                                                 \
		.position_error = 0,                                                               \
		.target_torque_ff = 0,                                                            \
	};

#define CONFIG_GET_FOLLOW(node) DT_PHANDLE(node, follow)

#define DMOTOR_CONFIG(inst, node, name)                                                            \
	static const struct dji_motor_config DT_CAT(dji_motor_cfg_, name) = {                      \
		.common = MOTOR_DT_DRIVER_CONFIG_INST_GET(inst),                                   \
		.gear_ratio = (float)DT_STRING_UNQUOTED(node, gear_ratio),                         \
		.is_gm6020 = DT_PROP(node, is_gm6020),                                             \
		.is_m3508 = DT_PROP(node, is_m3508),                                               \
		.is_m2006 = DT_PROP(node, is_m2006),                                               \
		.is_dm_motor = DT_PROP(node, is_dm_motor),                                         \
		.dm_i_max = DT_STRING_UNQUOTED_OR(node, dm_i_max, 0.0f),                           \
		.dm_torque_ratio = DT_STRING_UNQUOTED_OR(node, dm_torque_ratio, 0.0f),             \
		.minor_arc = DT_PROP(node, minor_arc),                                             \
		.inverse = DT_PROP(node, inverse),                                                 \
		.follow = DEVICE_DT_GET_OR_NULL(CONFIG_GET_FOLLOW(node)),                          \
	};

#define DMOTOR_DATA_INST(inst)   DMOTOR_DATA(inst, DT_DRV_INST(inst), DT_MOTOR_NAME_INST(inst))
#define DMOTOR_CONFIG_INST(inst) DMOTOR_CONFIG(inst, DT_DRV_INST(inst), DT_MOTOR_NAME_INST(inst))

#define MOTOR_DEVICE_DT_DEFINE(node_id, init_fn, pm, data, config, level, prio, api, ...)          \
	DEVICE_DT_DEFINE(node_id, init_fn, pm, data, config, level, prio, api, __VA_ARGS__)

#define MOTOR_DEVICE_DT_INST_DEFINE(inst, ...)                                                     \
	MOTOR_DEVICE_DT_DEFINE(DT_DRV_INST(inst), __VA_ARGS__)

#define DMOTOR_DEFINE(inst, name)                                                                  \
	MOTOR_DEVICE_DT_INST_DEFINE(inst, dji_init, NULL, &DT_CAT(dji_motor_data_, name),          \
				    &DT_CAT(dji_motor_cfg_, name), POST_KERNEL,                    \
				    CONFIG_MOTOR_INIT_PRIORITY, &motor_api_funcs)

#define DMOTOR_DEFINE_INST(inst) DMOTOR_DEFINE(inst, DT_MOTOR_NAME_INST(inst))

#define DMOTOR_INST(inst)                                                                          \
	DMOTOR_CONFIG_INST(inst)                                                                   \
	DMOTOR_DATA_INST(inst)                                                                     \
	DMOTOR_DEFINE_INST(inst)

typedef uint8_t motor_mode_t;
