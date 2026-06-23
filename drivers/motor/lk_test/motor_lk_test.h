#ifndef MOTOR_LK_TEST_H_
#define MOTOR_LK_TEST_H_

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/motor.h>

#define DT_DRV_COMPAT lk_test_motor

#include "motor_lk_test_internal.h"

int lk_test_init(const struct device *dev);
int lk_test_set(const struct device *dev, motor_status_t *status);
int lk_test_get(const struct device *dev, motor_status_t *status);
void lk_test_control(const struct device *dev, enum motor_cmd cmd);

extern const struct motor_driver_api lk_test_motor_api;

#define LK_TEST_MOTOR_DATA_INST(inst)                                                              \
	static struct lk_test_motor_data lk_test_motor_data_##inst = {                             \
		.common = MOTOR_DT_DRIVER_DATA_INST_GET(inst),                                     \
		.target =                                                                          \
			{                                                                          \
				.mode = ML_ANGLE,                                                  \
				.limit_speed_dps = 8000.0f,                                        \
				.limit_torque = 3.0f,                                              \
			},                                                                         \
	};

#define LK_TEST_MOTOR_CONFIG_INST(inst)                                                            \
	static const struct lk_test_motor_cfg lk_test_motor_cfg_##inst = {                         \
		.common = MOTOR_DT_DRIVER_CONFIG_INST_GET(inst),                                   \
		.id = (uint8_t)DT_PROP(DT_DRV_INST(inst), id),                                     \
		.control_rate_hz = DT_PROP_OR(DT_DRV_INST(inst), freq, LK_TEST_DEFAULT_RATE_HZ),   \
		.trace_lifecycle = DT_PROP(DT_DRV_INST(inst), trace_lifecycle),                    \
	};


	#define LK_TEST_MOTOR_DEVICE_DT_DEFINE(node_id, init_fn, pm, data, config, level, prio, api, ...) \
		DEVICE_DT_DEFINE(node_id, init_fn, pm, data, config, level, prio, api, __VA_ARGS__)

	#define LK_TEST_MOTOR_DEVICE_DT_INST_DEFINE(inst, ...)                                           \
		LK_TEST_MOTOR_DEVICE_DT_DEFINE(DT_DRV_INST(inst), __VA_ARGS__)

	#define LK_TEST_MOTOR_DEFINE_INST(inst)                                                          \
		LK_TEST_MOTOR_DEVICE_DT_INST_DEFINE(inst, lk_test_init, NULL, &lk_test_motor_data_##inst, \
						    &lk_test_motor_cfg_##inst, POST_KERNEL,               \
						    CONFIG_MOTOR_INIT_PRIORITY, &lk_test_motor_api);

#define LK_TEST_MOTOR_INST(inst)                                                                   \
	MOTOR_DT_DRIVER_PID_DEFINE(DT_DRV_INST(inst))                                              \
	LK_TEST_MOTOR_CONFIG_INST(inst)                                                            \
	LK_TEST_MOTOR_DATA_INST(inst)                                                              \
	LK_TEST_MOTOR_DEFINE_INST(inst)

#endif /* MOTOR_LK_TEST_H_ */
