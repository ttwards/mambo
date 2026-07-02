/*
 * Copyright (c) 2024 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief General Servo Motor Interface
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MOTOR_H_
#define ZEPHYR_INCLUDE_DRIVERS_MOTOR_H_

/**
 * @brief Servo Motor Interface
 * @defgroup servo_motor_interface Servo Motor Interface
 * @since 3.7.99
 * @version 1.0.0
 * @ingroup io_interfaces
 * @{
 */

#include <zephyr/toolchain.h>
#include <sys/_intsup.h>
#include <stdbool.h>
#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/kernel/thread.h>
#include <zephyr/sys/util.h>

#include <stdint.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef RPM2RADPS
#define RPM2RADPS(rpm) ((rpm) * 0.104719755f)
#endif

#ifndef RADPS2RPM
#define RADPS2RPM(radps) ((radps) * 9.54929659f)
#endif

#ifndef RAD2DEG
#define RAD2DEG(x) 57.2957795131f * x
#endif

#define MOTOR_CONTROLLER_ID_AUTO UINT8_MAX
#define MOTOR_CONTROLLER_MAX     4
#define MOTOR_CONTROLLER_PARAM_MAX 2

/**
 * @brief 电机工作模式枚举
 *
 * MIT: MIT模式
 * PV: 位置-速度控制
 * VO: 单控制量控制
 */
enum motor_mode {
	MIT = 0,
	PV = 1,
	VO = 2,
};

enum motor_target {
	MOTOR_TARGET_NONE = 0,
	MOTOR_TARGET_TORQUE = 1,
	MOTOR_TARGET_SPEED = 2,
	MOTOR_TARGET_POSITION = 3,
};

enum motor_controller_select {
	MOTOR_CONTROLLER_DEFAULT = 0,
	MOTOR_CONTROLLER_BY_ID = 1,
};

enum motor_state_flags {
	MOTOR_STATE_POSITION = BIT(0),
	MOTOR_STATE_SPEED = BIT(1),
	MOTOR_STATE_TORQUE = BIT(2),
	MOTOR_STATE_CURRENT = BIT(3),
	MOTOR_STATE_TEMPERATURE = BIT(4),
};

enum motor_output_type {
	MOTOR_OUTPUT_NONE = 0,
	MOTOR_OUTPUT_TORQUE = 1,
	MOTOR_OUTPUT_CURRENT = 2,
	MOTOR_OUTPUT_SPEED = 3,
	MOTOR_OUTPUT_POSITION = 4,
	MOTOR_OUTPUT_NATIVE = 5,
};

/**
 * @brief 电机控制命令枚举
 */
enum motor_cmd {
	ENABLE_MOTOR,
	DISABLE_MOTOR,
	SET_ZERO,
	CLEAR_CONTROLLER,
	CLEAR_ERROR,
};

typedef float (*motor_slip_cb_t)(const struct device *dev);

struct motor_controller_config;
struct motor_controller_data;
struct motor_controller_input;
struct motor_controller_output;

struct motor_controller_info {
	uint8_t id;
	enum motor_mode mode;
	enum motor_target target;
	enum motor_output_type output;
	uint32_t required_states;
	char name[32];
};

typedef struct motor_controller_info motor_controller_info_t;

struct motor_controller_params {
	float k_p;
	float k_i;
	float k_d;
	float integral_limit;
	float output_limit;
	float output_offset;
	float detri_lpf;
};

typedef int (*motor_controller_update_t)(struct motor_controller_data *data,
					 const struct motor_controller_config *cfg,
					 const struct motor_controller_input *input,
					 struct motor_controller_output *output);
typedef int (*motor_controller_get_params_t)(const struct motor_controller_config *cfg,
					     uint8_t index,
					     struct motor_controller_params *params);
typedef void (*motor_controller_reset_t)(struct motor_controller_data *data);

struct motor_controller_api {
	motor_controller_update_t update;
	motor_controller_get_params_t get_params;
	motor_controller_reset_t reset;
};

struct motor_controller_config {
	motor_controller_info_t info;
	const struct motor_controller_api *api;
	uint8_t param_count;
	struct motor_controller_params params[MOTOR_CONTROLLER_PARAM_MAX];
};

struct motor_controller_stage_data {
	float err_integral;
	float err_prev;
	float err_derivate;
	uint32_t prev_time;
};

struct motor_controller_data {
	struct motor_controller_stage_data stages[MOTOR_CONTROLLER_PARAM_MAX];
};

struct motor_driver_config {
	/** Physical device */
	const struct device *phy;
	/** motor ID  */
	uint8_t id;
	/** CAN TX ID */
	int tx_id;
	/** CAN RX ID */
	int rx_id;
	/** Configured controllers */
	struct motor_controller_config controllers[MOTOR_CONTROLLER_MAX];
};

struct motor_setpoint {
	float angle;
	float rpm;
	float torque;

	float speed_limit[2];
	float torque_limit[2];

	enum motor_mode mode;
	enum motor_target target;
	enum motor_controller_select controller_select;
	uint8_t controller_id;
};

typedef struct motor_setpoint motor_setpoint_t;

struct motor_status {
	float angle;
	float rpm;
	float torque;
	float temperature;
	float sum_angle;

	float speed_limit[2];
	float torque_limit[2];

	enum motor_mode mode;
	enum motor_target target;
	uint8_t controller_id;
	bool online;
	bool enabled;
	int error;
};

typedef struct motor_status motor_status_t;

struct motor_controller_input {
	motor_status_t status;
	motor_setpoint_t setpoint;
	float position_error;
	bool has_position_error;
	uint32_t timestamp;
};

struct motor_controller_output {
	enum motor_output_type type;
	float value;
	float speed;
};

struct motor_driver_data {
	float angle;
	float rpm;
	float torque;
	float temperature; /* Cannot be set in target */
	float sum_angle;

	float speed_limit[2];
	float torque_limit[2];

	enum motor_mode mode;
	enum motor_target target;
	uint8_t controller_id;
	struct motor_controller_data controllers[MOTOR_CONTROLLER_MAX];
};

static inline float motor_controller_clamp(float value, float min, float max)
{
	if (value > max) {
		return max;
	}
	if (value < min) {
		return min;
	}
	return value;
}

static inline float motor_controller_stage_update(struct motor_controller_stage_data *data,
						  const struct motor_controller_params *params,
						  float error, uint32_t timestamp)
{
	float output;

	if (data == NULL || params == NULL) {
		return 0;
	}
	if (data->prev_time == 0 || timestamp == 0) {
		data->prev_time = timestamp;
		data->err_prev = error;
		output = isnan(params->k_p) ? params->output_offset :
					      params->k_p * error + params->output_offset;
		if (params->output_limit != 0) {
			output = motor_controller_clamp(output, -params->output_limit,
							params->output_limit);
		}
		return output;
	}

	float delta_us = k_cyc_to_us_near32(timestamp - data->prev_time);
	if (delta_us < 1.0f) {
		return 0;
	}

	if (!isnan(params->k_i)) {
		data->err_integral += error * delta_us / 1000000.0f;
		if (params->integral_limit != 0) {
			data->err_integral = motor_controller_clamp(
				data->err_integral, -params->integral_limit,
				params->integral_limit);
		}
	}

	float raw_derivate = (error - data->err_prev) / delta_us * 1000000.0f;
	if (isnan(params->detri_lpf)) {
		data->err_derivate = raw_derivate;
	} else {
		data->err_derivate =
			params->detri_lpf * data->err_derivate +
			(1.0f - params->detri_lpf) * raw_derivate;
	}

	output = params->output_offset;
	if (!isnan(params->k_p)) {
		output += params->k_p * error;
	}
	if (!isnan(params->k_i)) {
		output += params->k_i * data->err_integral;
	}
	if (!isnan(params->k_d)) {
		output += params->k_d * data->err_derivate;
	}
	if (params->output_limit != 0) {
		output = motor_controller_clamp(output, -params->output_limit,
						params->output_limit);
	}

	data->err_prev = error;
	data->prev_time = timestamp;

	return output;
}

static inline int motor_builtin_controller_update(struct motor_controller_data *data,
						  const struct motor_controller_config *cfg,
						  const struct motor_controller_input *input,
						  struct motor_controller_output *output)
{
	float target_speed;
	float target_torque;

	if (data == NULL || cfg == NULL || input == NULL || output == NULL) {
		return -EINVAL;
	}

	*output = (struct motor_controller_output){
		.type = cfg->info.output,
		.value = 0,
		.speed = 0,
	};

	if (cfg->info.mode == PV && cfg->info.target == MOTOR_TARGET_POSITION) {
		if (cfg->param_count < 2) {
			return -EINVAL;
		}
		float position_error = input->has_position_error ?
					       input->position_error :
					       input->setpoint.angle - input->status.sum_angle;

		target_speed = motor_controller_stage_update(&data->stages[0],
							     &cfg->params[0],
							     position_error,
							     input->timestamp);
		if (input->setpoint.speed_limit[0] < input->setpoint.speed_limit[1]) {
			target_speed = motor_controller_clamp(target_speed,
							     input->setpoint.speed_limit[0],
							     input->setpoint.speed_limit[1]);
		}
		target_torque = motor_controller_stage_update(&data->stages[1],
							      &cfg->params[1],
							      target_speed - input->status.rpm,
							      input->timestamp);
		target_torque += input->setpoint.torque;
		output->type = MOTOR_OUTPUT_TORQUE;
		output->value = target_torque;
		output->speed = target_speed;
		return 0;
	}

	if (cfg->info.mode == VO && cfg->info.target == MOTOR_TARGET_SPEED) {
		if (cfg->param_count < 1) {
			return -EINVAL;
		}
		target_torque = motor_controller_stage_update(&data->stages[0],
							      &cfg->params[0],
							      input->setpoint.rpm -
								      input->status.rpm,
							      input->timestamp);
		target_torque += input->setpoint.torque;
		output->type = MOTOR_OUTPUT_TORQUE;
		output->value = target_torque;
		return 0;
	}

	if (cfg->info.mode == VO && cfg->info.target == MOTOR_TARGET_TORQUE) {
		output->type = MOTOR_OUTPUT_TORQUE;
		output->value = input->setpoint.torque;
		return 0;
	}

	return -ENOTSUP;
}

static inline int motor_builtin_controller_get_params(const struct motor_controller_config *cfg,
						      uint8_t index,
						      struct motor_controller_params *params)
{
	if (cfg == NULL || params == NULL || index >= cfg->param_count ||
	    index >= MOTOR_CONTROLLER_PARAM_MAX) {
		return -EINVAL;
	}

	*params = cfg->params[index];
	return 0;
}

static inline void motor_builtin_controller_reset(struct motor_controller_data *data)
{
	if (data != NULL) {
		memset(data, 0, sizeof(*data));
	}
}

static const struct motor_controller_api motor_builtin_controller_api = {
	.update = motor_builtin_controller_update,
	.get_params = motor_builtin_controller_get_params,
	.reset = motor_builtin_controller_reset,
};

static inline int motor_controller_update(struct motor_controller_data *data,
					  const struct motor_controller_config *cfg,
					  const struct motor_controller_input *input,
					  struct motor_controller_output *output)
{
	if (cfg == NULL || cfg->api == NULL || cfg->api->update == NULL) {
		return -ENOSYS;
	}

	return cfg->api->update(data, cfg, input, output);
}

static inline int motor_controller_get_params(const struct motor_controller_config *cfg,
					      uint8_t index,
					      struct motor_controller_params *params)
{
	if (cfg == NULL || cfg->api == NULL || cfg->api->get_params == NULL) {
		return -ENOSYS;
	}

	return cfg->api->get_params(cfg, index, params);
}

static inline void motor_controller_reset(struct motor_controller_data *data,
					  const struct motor_controller_config *cfg)
{
	if (cfg != NULL && cfg->api != NULL && cfg->api->reset != NULL) {
		cfg->api->reset(data);
	}
}

#define motor_set_angle(dev, _angle)                                                               \
	motor_set(dev, &(motor_setpoint_t){                                                        \
			       .angle = _angle, .mode = PV, .target = MOTOR_TARGET_POSITION})
#define motor_set_rpm(dev, _rpm)                                                                   \
	motor_set(dev, &(motor_setpoint_t){.rpm = _rpm, .mode = VO, .target = MOTOR_TARGET_SPEED})
#define motor_set_torque(dev, _torque)                                                             \
	motor_set(dev, &(motor_setpoint_t){                                                        \
			       .torque = _torque, .mode = VO, .target = MOTOR_TARGET_TORQUE})
#define motor_set_speed(dev, _speed)                                                               \
	motor_set(dev, &(motor_setpoint_t){                                                          \
			       .rpm = _speed, .mode = VO, .target = MOTOR_TARGET_SPEED})
#define motor_set_mit(dev, _speed, _angle, _torque)                                                \
	motor_set(dev, &(motor_setpoint_t){                                                        \
			       .rpm = _speed, .angle = _angle, .torque = _torque, .mode = MIT,       \
			       .target = MOTOR_TARGET_POSITION})

#define motor_get_angle(dev)                                                                       \
	({                                                                                         \
		motor_status_t status;                                                             \
		motor_get(dev, &status);                                                           \
		status.angle;                                                                      \
	})
#define motor_get_rpm(dev)                                                                         \
	({                                                                                         \
		motor_status_t status;                                                             \
		motor_get(dev, &status);                                                           \
		status.rpm;                                                                        \
	})
#define motor_get_torque(dev)                                                                      \
	({                                                                                         \
		motor_status_t status;                                                             \
		motor_get(dev, &status);                                                           \
		status.torque;                                                                     \
	})
#define motor_get_speed(dev)                                                                       \
	({                                                                                         \
		motor_status_t status;                                                             \
		motor_get(dev, &status);                                                           \
		status.rpm;                                                                        \
	})
#define motor_get_mode(dev)                                                                        \
	({                                                                                         \
		motor_status_t status;                                                             \
		motor_get(dev, &status);                                                           \
		status.mode;                                                                       \
	})

/**
 * @typedef motor_api_stat_t
 * @brief 获取电机当前状态
 *
 * @param dev 指向电机设备的指针
 * @return int 成功: 0, 失败: 负值
 */
typedef int (*motor_api_stat_t)(const struct device *dev, motor_status_t *status);

/**
 * @typedef motor_api_set_t
 * @brief 设置电机目标状态的回调函数
 *
 * @param dev 指向电机设备的指针
 * @param setpoint 目标设定
 * @return int 成功: 0, 失败: 负值
 */
typedef int (*motor_api_set_t)(const struct device *dev, motor_setpoint_t *setpoint);

/**
 * @typedef motor_api_ctrl_t
 * @brief 电机控制命令
 *
 * @param dev 指向电机设备的指针
 * @param cmd 控制命令
 * @return void
 */
typedef void (*motor_api_ctrl_t)(const struct device *dev, enum motor_cmd cmd);

/**
 * @brief Motor driver API
 */
__subsystem struct motor_driver_api {
	motor_api_ctrl_t motor_control;
	motor_api_set_t motor_set;
	motor_api_stat_t motor_get;
};

/**
 * @brief 获取电机当前状态
 *
 * @param dev 电机设备指针
 * @return int 成功: 0, 失败: 负值
 */
__syscall int motor_get(const struct device *dev, motor_status_t *status);

static inline int z_impl_motor_get(const struct device *dev, motor_status_t *status)
{
	const struct motor_driver_api *api = (const struct motor_driver_api *)dev->api;
	if (api->motor_get == NULL) {
		return -ENOSYS;
	}
	return api->motor_get(dev, status);
}

/**
 * @brief 设置电机目标状态
 *
 * @param dev 电机设备指针
 * @param status 目标状态
 * @return int 成功: 0, 失败: 负值
 */
__syscall int motor_set(const struct device *dev, motor_setpoint_t *setpoint);

static inline int z_impl_motor_set(const struct device *dev, motor_setpoint_t *setpoint)
{
	const struct motor_driver_api *api = (const struct motor_driver_api *)dev->api;
	if (api->motor_set == NULL) {
		return -ENOSYS;
	}
	return api->motor_set(dev, setpoint);
}

/**
 * @brief 执行电机控制命令
 *
 * @param dev 电机设备指针
 * @param cmd 控制命令
 * @return int 0:成功，负值:错误码
 */
__syscall void motor_control(const struct device *dev, enum motor_cmd cmd);

static inline void z_impl_motor_control(const struct device *dev, enum motor_cmd cmd)
{
	const struct motor_driver_api *api = (const struct motor_driver_api *)dev->api;
	if (api->motor_control == NULL) {
		return;
	}
	api->motor_control(dev, cmd);
}

static inline bool motor_controller_info_valid(const motor_controller_info_t *info)
{
	return info != NULL && info->name[0] != '\0';
}

static inline int motor_get_controller_count(const struct device *dev)
{
	const struct motor_driver_config *cfg = dev->config;
	int count = 0;

	for (int i = 0; i < ARRAY_SIZE(cfg->controllers); i++) {
		if (!motor_controller_info_valid(&cfg->controllers[i].info)) {
			break;
		}
		count++;
	}

	return count;
}

static inline int motor_get_controller_info(const struct device *dev, uint8_t index,
					    motor_controller_info_t *info)
{
	const struct motor_driver_config *cfg = dev->config;

	if (info == NULL || index >= ARRAY_SIZE(cfg->controllers) ||
	    !motor_controller_info_valid(&cfg->controllers[index].info)) {
		return -EINVAL;
	}

	*info = cfg->controllers[index].info;
	return 0;
}

static inline int motor_resolve_controller(const struct device *dev, motor_setpoint_t *setpoint,
					   motor_controller_info_t *info)
{
	if (setpoint == NULL) {
		return -EINVAL;
	}
	if (setpoint->target == MOTOR_TARGET_NONE) {
		return 0;
	}

	if (setpoint->controller_select == MOTOR_CONTROLLER_BY_ID) {
		motor_controller_info_t selected = {0};
		motor_controller_info_t *selected_info = info != NULL ? info : &selected;
		int ret = motor_get_controller_info(dev, setpoint->controller_id, selected_info);

		if (ret < 0) {
			return ret;
		}
		if (selected_info->mode != setpoint->mode ||
		    selected_info->target != setpoint->target) {
			return -EINVAL;
		}
		return 0;
	}

	for (uint8_t i = 0; i < motor_get_controller_count(dev); i++) {
		motor_controller_info_t candidate = {0};

		if (motor_get_controller_info(dev, i, &candidate) < 0) {
			continue;
		}
		if (candidate.mode == setpoint->mode && candidate.target == setpoint->target) {
			setpoint->controller_id = i;
			if (info != NULL) {
				*info = candidate;
			}
			return 0;
		}
	}

	return -ENOTSUP;
}

#define DT_GET_CANPHY(node_id) DEVICE_DT_GET(DT_PHANDLE(node_id, can_channel))

#define MOTOR_DT_CONTROLLER_PARAM_VALUE(node_id, prop, fallback_prop, default_value)                \
	COND_CODE_1(DT_NODE_HAS_PROP(node_id, prop), (DT_STRING_UNQUOTED(node_id, prop)),          \
		    (DT_STRING_UNQUOTED_OR(node_id, fallback_prop, default_value)))

#define MOTOR_DT_CONTROLLER_PARAMS(node_id, kp_prop, ki_prop, kd_prop, i_prop, out_prop, lpf_prop, \
				   offset_prop, i_fallback, out_fallback, lpf_fallback,            \
				   offset_fallback)                                                \
	{                                                                                          \
		.k_p = DT_STRING_UNQUOTED_OR(node_id, kp_prop, 0),                                \
		.integral_limit = MOTOR_DT_CONTROLLER_PARAM_VALUE(node_id, i_prop, i_fallback, 0), \
		.output_limit = MOTOR_DT_CONTROLLER_PARAM_VALUE(node_id, out_prop, out_fallback, 0), \
		.detri_lpf = MOTOR_DT_CONTROLLER_PARAM_VALUE(node_id, lpf_prop, lpf_fallback, NAN), \
		.k_i = DT_STRING_UNQUOTED_OR(node_id, ki_prop, NAN),                              \
		.k_d = DT_STRING_UNQUOTED_OR(node_id, kd_prop, NAN),                              \
		.output_offset = MOTOR_DT_CONTROLLER_PARAM_VALUE(node_id, offset_prop,             \
								 offset_fallback, 0),                 \
	}

#define MOTOR_DT_SINGLE_CONTROLLER_PARAMS(node_id)                                                  \
	MOTOR_DT_CONTROLLER_PARAMS(node_id, k_p, k_i, k_d, i_max, out_max, detri_lpf, offset,       \
				   i_max, out_max, detri_lpf, offset)

#define MOTOR_DT_PV_CONTROLLER_PARAMS(node_id)                                                      \
	MOTOR_DT_CONTROLLER_PARAMS(node_id, pos_k_p, pos_k_i, pos_k_d, pos_i_max, pos_out_max,      \
				   pos_detri_lpf, pos_offset, i_max, out_max, detri_lpf, offset),   \
		MOTOR_DT_CONTROLLER_PARAMS(node_id, vel_k_p, vel_k_i, vel_k_d, vel_i_max,           \
					   vel_out_max, vel_detri_lpf, vel_offset, i_max, out_max,      \
					   detri_lpf, offset)

#define MOTOR_DT_MIT_CONTROLLER_PARAMS(node_id)                                                     \
	{                                                                                          \
		.k_p = DT_STRING_UNQUOTED_OR(node_id, k_p, 0),                                   \
		.integral_limit = DT_STRING_UNQUOTED_OR(node_id, i_max, 0),                       \
		.output_limit = DT_STRING_UNQUOTED_OR(node_id, out_max, 0),                       \
		.detri_lpf = DT_STRING_UNQUOTED_OR(node_id, detri_lpf, NAN),                      \
		.k_i = DT_STRING_UNQUOTED_OR(node_id, k_i, NAN),                                  \
		.k_d = DT_STRING_UNQUOTED_OR(node_id, k_d, NAN),                                  \
		.output_offset = DT_STRING_UNQUOTED_OR(node_id, offset, 0),                       \
	}

#define MOTOR_DT_CONTROLLER_MODE(node_id)                                                          \
	COND_CODE_1(DT_NODE_HAS_COMPAT(node_id, motor_controller_mit), (MIT),                     \
		    (COND_CODE_1(DT_NODE_HAS_COMPAT(node_id, motor_controller_pv), (PV), (VO))))

#define MOTOR_DT_CONTROLLER_TARGET(node_id)                                                        \
	COND_CODE_1(DT_NODE_HAS_COMPAT(node_id, motor_controller_vo),                            \
		    ((enum motor_target)(DT_ENUM_IDX_OR(node_id, target, 1) + 1)),                  \
		    (MOTOR_TARGET_POSITION))

#define MOTOR_DT_CONTROLLER_OUTPUT(node_id)                                                        \
	COND_CODE_1(DT_NODE_HAS_COMPAT(node_id, motor_controller_mit), (MOTOR_OUTPUT_NATIVE),    \
		    (MOTOR_OUTPUT_TORQUE))

#define MOTOR_DT_CONTROLLER_REQUIRED_STATES(node_id)                                               \
	COND_CODE_1(DT_NODE_HAS_COMPAT(node_id, motor_controller_pv),                             \
		    (MOTOR_STATE_POSITION | MOTOR_STATE_SPEED),                                      \
		    (COND_CODE_1(DT_NODE_HAS_COMPAT(node_id, motor_controller_vo),                  \
				 (COND_CODE_1(DT_ENUM_IDX_OR(node_id, target, 1),                   \
					      (MOTOR_STATE_SPEED), (MOTOR_STATE_TORQUE))),              \
				 (0))))

#define MOTOR_DT_CONTROLLER_PARAM_COUNT(node_id)                                                    \
	COND_CODE_1(DT_NODE_HAS_COMPAT(node_id, motor_controller_pv), (2),                        \
		    (COND_CODE_1(DT_NODE_HAS_COMPAT(node_id, motor_controller_mit), (1), (1))))

#define MOTOR_DT_CONTROLLER_PARAM_CONFIGS(node_id)                                                  \
	COND_CODE_1(DT_NODE_HAS_COMPAT(node_id, motor_controller_pv),                             \
		    (MOTOR_DT_PV_CONTROLLER_PARAMS(node_id)),                                       \
		    (COND_CODE_1(DT_NODE_HAS_COMPAT(node_id, motor_controller_mit),                  \
				 (MOTOR_DT_MIT_CONTROLLER_PARAMS(node_id)),                         \
				 (MOTOR_DT_SINGLE_CONTROLLER_PARAMS(node_id)))))

#define MOTOR_DT_CONTROLLER_CONFIG_BY_IDX(node_id, prop, idx)                                      \
	{                                                                                          \
		.info =                                                                            \
			{                                                                          \
				.id = idx,                                                        \
				.mode = MOTOR_DT_CONTROLLER_MODE(DT_PROP_BY_IDX(node_id, prop, idx)), \
				.target = MOTOR_DT_CONTROLLER_TARGET(                            \
					DT_PROP_BY_IDX(node_id, prop, idx)),                     \
				.output = MOTOR_DT_CONTROLLER_OUTPUT(                            \
					DT_PROP_BY_IDX(node_id, prop, idx)),                     \
				.required_states = MOTOR_DT_CONTROLLER_REQUIRED_STATES(           \
					DT_PROP_BY_IDX(node_id, prop, idx)),                     \
				.name = DT_NODE_FULL_NAME(DT_PROP_BY_IDX(node_id, prop, idx)),    \
			},                                                                         \
		.api = &motor_builtin_controller_api,                                             \
		.param_count = MOTOR_DT_CONTROLLER_PARAM_COUNT(DT_PROP_BY_IDX(node_id, prop, idx)), \
		.params = {MOTOR_DT_CONTROLLER_PARAM_CONFIGS(DT_PROP_BY_IDX(node_id, prop, idx))}, \
	}

#define MOTOR_DT_CONTROLLER_CONFIGS(node_id)                                                       \
	COND_CODE_1(DT_NODE_HAS_PROP(node_id, controllers),                                      \
		    (DT_FOREACH_PROP_ELEM_SEP(node_id, controllers,                              \
					      MOTOR_DT_CONTROLLER_CONFIG_BY_IDX, (, ))),              \
		    ())

#define MOTOR_DT_DRIVER_CONFIG_GET(node_id)                                                        \
	{                                                                                          \
		.phy = (const struct device *)DT_GET_CANPHY(node_id),                              \
		.id = DT_PROP(node_id, id),                                                        \
		.tx_id = DT_PROP_OR(node_id, tx_id, 0x00),                                         \
		.rx_id = DT_PROP_OR(node_id, rx_id, 0x00),                                         \
		.controllers = {MOTOR_DT_CONTROLLER_CONFIGS(node_id)},                             \
		}

#define MOTOR_DT_DRIVER_DATA_GET(node_id)                                                          \
	{                                                                                          \
		.angle = 0,                                                                        \
		.rpm = 0,                                                                          \
		.torque = 0,                                                                       \
		.temperature = 0,                                                                  \
		.sum_angle = 0,                                                                    \
		.speed_limit = {-99999, 99999},                                                    \
		.torque_limit = {-99999, 99999},                                                   \
		.mode = MIT,                                                                       \
		.target = MOTOR_TARGET_POSITION,                                                   \
		.controller_id = MOTOR_CONTROLLER_ID_AUTO,                                         \
	}

#define MOTOR_DT_DRIVER_CONFIG_INST_GET(inst) MOTOR_DT_DRIVER_CONFIG_GET(DT_DRV_INST(inst))
#define MOTOR_DT_DRIVER_DATA_INST_GET(inst)   MOTOR_DT_DRIVER_DATA_GET(DT_DRV_INST(inst))
/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#include <zephyr/syscalls/motor.h>

#endif /* ZEPHYR_INCLUDE_DRIVERS_MOTOR_H_ */
