/*
 * Copyright (c) 2025 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef STEERWHEEL_C
#define STEERWHEEL_C

#include <math.h>
#include <stdint.h>
#include <string.h>
#include <sys/types.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/wheel.h>
#include <zephyr/drivers/motor.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef PI
#define PI 3.14159265358f
#endif

typedef struct {
	wheel_cfg_t common;

	bool inverse_steer;
	bool inverse_wheel;
	 bool multi_turn;
	const struct device *steer_motor;
	const struct device *wheel_motor;
} steerwheel_cfg_t;

typedef struct {
	wheel_status_t status;
	wheel_status_t target;
	float set_rpm;

	bool negative;
} steerwheel_data_t;

static inline void steerwheel_set_speed(const struct device *dev, float speed, float angle)
{
    steerwheel_data_t *data = dev->data;
    const steerwheel_cfg_t *cfg = dev->config;


    float curr_speed = cfg->inverse_wheel ? -motor_get_speed(cfg->wheel_motor)
                                          : motor_get_speed(cfg->wheel_motor);
    float curr_angle = cfg->inverse_steer ? -motor_get_angle(cfg->steer_motor)
                                          : motor_get_angle(cfg->steer_motor);


    data->status.angle =
        !data->negative ? fmodf(curr_angle - cfg->common.angle_offset, 360.0f)
                        : fmodf(curr_angle + 180.0f - cfg->common.angle_offset, 360.0f);
    data->status.speed = RPM2RADPS(fabsf(curr_speed)) * cfg->common.wheel_radius;


    if (cfg->inverse_wheel) {
        angle = -angle;
    }
    if (cfg->inverse_steer) {
        speed = -speed;
    }
    data->target.speed = speed;


    if (cfg->multi_turn) {
		// if (1) {
        float desired_st = fmodf(fmodf(angle + cfg->common.angle_offset, 360.0f) + 360.0f, 360.0f);

        float current_target_st = fmodf(fmodf(data->target.angle, 360.0f) + 360.0f, 360.0f);


        float diff = desired_st - current_target_st;
        if (diff > 180.0f) {
            diff -= 360.0f;
        } else if (diff < -180.0f) {
            diff += 360.0f;
        }


        data->negative = false;
        if (diff > 90.0f) {
            diff -= 180.0f;
            data->negative = true;
        } else if (diff < -90.0f) {
            diff += 180.0f;
            data->negative = true;
        }


        data->target.angle += diff;


        float base_rpm = RADPS_TO_RPM * speed / cfg->common.wheel_radius;
        float sign = cfg->inverse_wheel ? -1.0f : 1.0f;
        data->set_rpm = data->negative ? (-sign * base_rpm) : (sign * base_rpm);

    } else {

        float target_angle = fmodf(angle + cfg->common.angle_offset, 360.0f);
        float delta_angle = curr_angle - target_angle;

        if (delta_angle >= 0) {
            if (delta_angle < 90) {
                data->target.angle = target_angle;
                data->set_rpm = (cfg->inverse_wheel ? -1 : 1) * RADPS_TO_RPM * speed / cfg->common.wheel_radius;
                data->negative = false;
            } else if (delta_angle < 180) {
                data->target.angle = target_angle - 180.0f;
                data->set_rpm = -(cfg->inverse_wheel ? -1 : 1) * RADPS_TO_RPM * speed / cfg->common.wheel_radius;
                data->negative = true;
            } else if (delta_angle < 270) {
                data->target.angle = target_angle - 180.0f;
                data->set_rpm = -(cfg->inverse_wheel ? -1 : 1) * RADPS_TO_RPM * speed / cfg->common.wheel_radius;
                data->negative = true;
            } else {
                data->target.angle = target_angle;
                data->set_rpm = (cfg->inverse_wheel ? -1 : 1) * RADPS_TO_RPM * speed / cfg->common.wheel_radius;
                data->negative = false;
            }
        } else if (delta_angle < 0) {
            if (delta_angle > -90) {
                data->target.angle = target_angle;
                data->set_rpm = (cfg->inverse_wheel ? -1 : 1) * RADPS_TO_RPM * speed / cfg->common.wheel_radius;
                data->negative = false;
            } else if (delta_angle > -180) {
                data->target.angle = target_angle - 180.0f;
                data->set_rpm = -(cfg->inverse_wheel ? -1 : 1) * RADPS_TO_RPM * speed / cfg->common.wheel_radius;
                data->negative = true;
            } else if (delta_angle > -270) {
                data->target.angle = target_angle - 180.0f;
                data->set_rpm = -(cfg->inverse_wheel ? -1 : 1) * RADPS_TO_RPM * speed / cfg->common.wheel_radius;
                data->negative = true;
            } else {
                data->target.angle = target_angle;
                data->set_rpm = (cfg->inverse_wheel ? -1 : 1) * RADPS_TO_RPM * speed / cfg->common.wheel_radius;
                data->negative = false;
            }
        }
    }


    motor_set_angle(cfg->steer_motor, data->target.angle);
    motor_set_speed(cfg->wheel_motor, data->set_rpm);
}

static inline int steerwheel_set_static(const struct device *dev, float angle)
{
    steerwheel_data_t *data = dev->data;
    const steerwheel_cfg_t *cfg = dev->config;


    data->set_rpm = 0;
    data->target.speed = 0;


    float curr_speed = cfg->inverse_wheel ? -motor_get_speed(cfg->wheel_motor)
                                          : motor_get_speed(cfg->wheel_motor);
    float curr_angle = cfg->inverse_steer ? -motor_get_angle(cfg->steer_motor)
                                          : motor_get_angle(cfg->steer_motor);


    float status_angle = !data->negative ? (curr_angle - cfg->common.angle_offset)
                                         : (curr_angle + 180.0f - cfg->common.angle_offset);
    data->status.angle = fmodf(fmodf(status_angle, 360.0f) + 360.0f, 360.0f);
    data->status.speed = RPM2RADPS(fabsf(curr_speed)) * cfg->common.wheel_radius;


    if (cfg->inverse_wheel) {
        angle = -angle;
    }

    if (cfg->multi_turn) {

        float desired_st = fmodf(fmodf(angle + cfg->common.angle_offset, 360.0f) + 360.0f, 360.0f);

        float current_target_st = fmodf(fmodf(data->target.angle, 360.0f) + 360.0f, 360.0f);

        float diff = desired_st - current_target_st;
        if (diff > 180.0f) {
            diff -= 360.0f;
        } else if (diff < -180.0f) {
            diff += 360.0f;
        }

        data->negative = false;
        if (diff > 90.0f) {
            diff -= 180.0f;
            data->negative = true;
        } else if (diff < -90.0f) {
            diff += 180.0f;
            data->negative = true;
        }
        data->target.angle += diff;

    } else {

        float target_angle = fmodf(angle + cfg->common.angle_offset, 360.0f);
        float delta_angle = curr_angle - target_angle;

        data->negative = false;

        if (delta_angle >= 0) {
            if (delta_angle >= 90 && delta_angle < 270) {
                data->target.angle = target_angle - 180.0f;
                data->negative = true;
            } else {
                data->target.angle = target_angle;
                data->negative = false;
            }
        } else {
            if (delta_angle <= -90 && delta_angle > -270) {
                data->target.angle = target_angle - 180.0f;
                data->negative = true;
            } else {
                data->target.angle = target_angle;
                data->negative = false;
            }
        }
    }
    motor_control(cfg->wheel_motor, SET_ZERO);
    motor_set_angle(cfg->steer_motor, data->target.angle);
    return motor_set_angle(cfg->wheel_motor, 0);
}

static inline wheel_status_t *steerwheel_get_speed(const struct device *dev)
{
	steerwheel_data_t *data = dev->data;
	const steerwheel_cfg_t *cfg = dev->config;

	float curr_speed = cfg->inverse_wheel ? -motor_get_speed(cfg->wheel_motor)
					      : motor_get_speed(cfg->wheel_motor);
	float curr_angle = cfg->inverse_steer ? -motor_get_angle(cfg->steer_motor)
					      : motor_get_angle(cfg->steer_motor);
	data->status.angle =
		curr_speed > 0 ? curr_angle
			       : fmodf(curr_angle + 180.0f - cfg->common.angle_offset, 360.0f);
	data->status.speed = RPM2RADPS(fabsf(curr_speed)) * cfg->common.wheel_radius;

	return &data->status;
}

static inline wheel_status_t *steerwheel_get_target(const struct device *dev)
{
	steerwheel_data_t *data = dev->data;
	return &data->target;
}

static inline void steerwheel_disable(const struct device *dev)
{
	const steerwheel_cfg_t *cfg = dev->config;
	motor_set_torque(cfg->steer_motor, 0);
	motor_set_torque(cfg->wheel_motor, 0);
}

struct wheel_driver_api steerwheel_driver_api = {
	.wheel_set_speed = steerwheel_set_speed,
	.wheel_set_static = steerwheel_set_static,
	.wheel_get_speed = steerwheel_get_speed,
	.wheel_get_target = steerwheel_get_target,
	.wheel_disable = steerwheel_disable,
};

#define STEERWHEEL_DEVICE_DT_DEFINE(node_id, init_fn, pm, data, config, level, prio, api, ...)     \
	DEVICE_DT_DEFINE(node_id, init_fn, pm, data, config, level, prio, api, __VA_ARGS__)

#define DT_DRV_COMPAT ares_steerwheel

#define STEERWHEEL_DEVICE_DEFINE(inst)                                                             \
	static steerwheel_data_t steerwheel_data_##inst = {                                        \
		.status = {.speed = 0.0f, .angle = 0.0f, .restricted = true},                      \
		.target = {.speed = 0.0f, .angle = 0.0f, .restricted = true},                      \
		.negative = false,                                                                 \
		.set_rpm = 0.0f,                                                                   \
	};                                                                                         \
	static const steerwheel_cfg_t steerwheel_cfg_##inst = {                                    \
		.common = DT_WHEEL_CONFIG_GET(inst),                                               \
		.steer_motor = DEVICE_DT_GET(DT_PROP(DT_DRV_INST(inst), steer_motor)),             \
		.wheel_motor = DEVICE_DT_GET(DT_PROP(DT_DRV_INST(inst), wheel_motor)),             \
		.inverse_steer = DT_PROP(DT_DRV_INST(inst), inverse_steer),                        \
		.inverse_wheel = DT_PROP(DT_DRV_INST(inst), inverse_wheel),                        \
		.multi_turn = DT_PROP(DT_DRV_INST(inst), multi_turn),                        \
	};                                                                                         \
	STEERWHEEL_DEVICE_DT_DEFINE(DT_DRV_INST(inst), NULL, NULL, &steerwheel_data_##inst,        \
				    &steerwheel_cfg_##inst, POST_KERNEL, 90,                       \
				    &steerwheel_driver_api);

DT_INST_FOREACH_STATUS_OKAY(STEERWHEEL_DEVICE_DEFINE)

#ifdef __cplusplus
}
#endif

#endif // STEERWHEEL_C
