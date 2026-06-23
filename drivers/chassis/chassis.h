#include "zephyr/drivers/pid.h"
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/chassis.h>

#define DT_DRV_COMPAT ares_chassis

#define CHASSIS_STACK_SIZE 2048

#define CHASSIS_DATA_INST(inst)                                                                    \
	static chassis_data_t chassis_data_##inst = {                                              \
		.chassis_status = {0.0f},                                                          \
		.set_status = {0.0f},                                                              \
		.currTime = 0ULL,                                                                  \
		.prevTime = 0ULL,                                                                  \
		.angleControl = true,                                                              \
		.angle_to_center = {0.0f},                                                         \
		.distance_to_center = {0.0f},                                                      \
		.last_wheel_angle = {0.0f},                                                        \
		.chassis_sensor_data = {0},                                                        \
		.enabled = true,                                                                   \
		.track_angle = DT_PROP(DT_DRV_INST(inst), track_angle),                            \
		.static_angle = false,                                                             \
	};

#define WHEELS_FOREACH(inst, fn) {DT_FOREACH_PROP_ELEM_SEP(DT_DRV_INST(inst), wheels, fn, (,)) }

#define GET_WHEEL_DEVICE(node_id, prop, idx) DEVICE_DT_GET(DT_PHANDLE_BY_IDX(node_id, prop, idx))

#define GET_WHEEL_X_OFFSET(node_id, prop, idx)                                                     \
	(float)((int)DT_PHA_BY_IDX(node_id, prop, idx, offset_x)) / 10000.0f

#define GET_WHEEL_Y_OFFSET(node_id, prop, idx)                                                     \
	(float)((int)DT_PHA_BY_IDX(node_id, prop, idx, offset_y)) / 10000.0f

#define CHASSIS_CONFIG_INST(inst)                                                                  \
	static const chassis_cfg_t chassis_cfg_##inst = {                                          \
		.angle_pid = &PID_INS_NAME(DT_PROP(DT_DRV_INST(inst), angle_pid),                  \
					   DT_NODE_FULL_NAME_UNQUOTED(DT_DRV_INST(inst))),         \
		.wheels = WHEELS_FOREACH(inst, GET_WHEEL_DEVICE),                                  \
		.pos_X_offset = WHEELS_FOREACH(inst, GET_WHEEL_X_OFFSET),                          \
		.pos_Y_offset = WHEELS_FOREACH(inst, GET_WHEEL_Y_OFFSET),                          \
		.max_gyro = DT_STRING_UNQUOTED_OR(DT_DRV_INST(inst), max_gyro, 10),                \
		.max_lin_accel = DT_STRING_UNQUOTED_OR(DT_DRV_INST(inst), max_linear_accel, 10),   \
	};

#define CHASSIS_DEFINE_INST(inst)                                                                  \
	DEVICE_DT_DEFINE(DT_DRV_INST(inst), cchassis_init, NULL, &chassis_data_##inst,             \
			 &chassis_cfg_##inst, POST_KERNEL, CONFIG_CHASSIS_INIT_PRIORITY,           \
			 &chassis_driver_api);

#define CHASSIS_PID_DEFINE(inst)                                                                   \
	PID_NEW_INSTANCE(DT_PROP(DT_DRV_INST(inst), angle_pid),                                    \
			 DT_NODE_FULL_NAME_UNQUOTED(DT_DRV_INST(inst)))

#define CHASSIS_INIT(inst)                                                                         \
	CHASSIS_PID_DEFINE(inst)                                                                   \
	CHASSIS_CONFIG_INST(inst)                                                                  \
	CHASSIS_DATA_INST(inst)                                                                    \
	CHASSIS_DEFINE_INST(inst)
