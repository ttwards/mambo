/*
 * Copyright (c) 2024 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef PID_H
#define PID_H

#include <math.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <sys/types.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

// TODO: PID, ADRC, FSF, LQR, MPC, etc.

#ifdef __cplusplus
extern "C" {
#endif

#define PID_DEVICE_DT_DEFINE(node_id, init_fn, pm, data, config, level, prio, api, ...)            \
	DEVICE_DT_DEFINE(node_id, init_fn, pm, data, config, level, prio, api, __VA_ARGS__)

#define NORMAL_PID 0U
#define MIT_PID    1U

#define STATIC_VOID __attribute__((unused)) static void
#define STATIC      __attribute__((unused)) static

#define PID_INS_CAT_NAME(node_name, name) DT_CAT3(node_name, _, name)
#define PID_INS_NAME(node_id, name)       PID_INS_CAT_NAME(DT_NODE_FULL_NAME_UNQUOTED(node_id), name)

#define PID_NEW_INSTANCE(node_id, name)                                                            \
	struct pid_data PID_INS_NAME(node_id, name) = {                                            \
		.pid_dev = DEVICE_DT_GET(node_id),                                                 \
	};

struct pid_config {
	float k_p;
	float k_i;
	float k_d;
	float integral_limit;
	float output_limit;
	float output_offset;
	float detri_lpf;
	bool mit;
};

struct pid_data {
	float *ref;
	float *curr;

	float *detri_ref;
	float *detri_curr;

	float err_integral;
	float err_derivate;
	float ratio;
	const struct device *pid_dev;
	uint32_t *curr_time;
	uint32_t *prev_time;
	float *output;
};

static bool float_equal(float a, float b)
{
	return fabsf(a - b) < 0.0001f;
}

STATIC_VOID pid_calc(struct pid_data *data)
{
	const struct device *dev = data->pid_dev;
	if (dev == NULL) {
		return;
	}
	const struct pid_config *pid_para = dev->config;
	if (data->curr == NULL) {
		return;
	}
	float kp = pid_para->k_p;
	float ki = pid_para->k_i;
	float kd = pid_para->k_d;
	float err = *(data->ref) - *(data->curr);
	float deltaT = k_cyc_to_us_near32(*(data->curr_time) - *(data->prev_time));
	if (deltaT == 0) {
		return;
	}
	if ((ki == ki) && !float_equal(ki, 0)) {
		data->err_integral += (err * deltaT) / ki;
		if (pid_para->integral_limit != 0) {
			if (fabsf(data->err_integral) > pid_para->integral_limit) {
				data->err_integral = data->err_integral > 0
							     ? pid_para->integral_limit
							     : -pid_para->integral_limit;
			}
		}
	}
	if ((kd == kd)) {
		if ((pid_para->detri_lpf != pid_para->detri_lpf)) {
			data->err_derivate =
				kd * (*(data->detri_ref) - *(data->detri_curr)) / deltaT;
		} else {
			data->err_derivate =
				pid_para->detri_lpf * data->err_derivate +
				(1 - pid_para->detri_lpf) *
					(kd * (*(data->detri_ref) - *(data->detri_curr)) / deltaT);
		}
	}
	*(data->output) =
		kp * (err + data->err_integral + data->err_derivate) + pid_para->output_offset;
	if (pid_para->output_limit != 0 && fabsf(*(data->output)) > pid_para->output_limit) {
		*(data->output) =
			*(data->output) > 0 ? pid_para->output_limit : -pid_para->output_limit;
	}
}

STATIC float pid_calc_in(struct pid_data *data, float error, float deltaT_us)
{
	const struct device *dev = data->pid_dev;
	if (dev == NULL) {
		return 0;
	}
	const struct pid_config *pid_para = dev->config;

	float kp = pid_para->k_p;
	float ki = pid_para->k_i;
	float kd = pid_para->k_d;

	if (deltaT_us == 0) {
		return 0;
	}
	float out;
	if ((ki == ki) && !float_equal(ki, 0)) {
		data->err_integral += (error * deltaT_us) / ki;
		if (pid_para->integral_limit != 0) {
			if (fabsf(data->err_integral) > pid_para->integral_limit) {
				data->err_integral = data->err_integral > 0
							     ? pid_para->integral_limit
							     : -pid_para->integral_limit;
			}
		}
	}
	if ((kd == kd)) {
		if ((pid_para->detri_lpf != pid_para->detri_lpf)) {
			data->err_derivate =
				kd * (*(data->detri_ref) - *(data->detri_curr)) / deltaT_us;
		} else {
			data->err_derivate =
				pid_para->detri_lpf * data->err_derivate +
				(1 - pid_para->detri_lpf) *
					(kd * (*(data->detri_ref) - *(data->detri_curr)) /
					 deltaT_us);
		}
	}
	out = kp * (error + data->err_integral + data->err_derivate) + pid_para->output_offset;
	if (pid_para->output_limit != 0 && fabsf(out) > pid_para->output_limit) {
		out = out > 0 ? pid_para->output_limit : -pid_para->output_limit;
	}
	if (data->output != NULL) {
		*(data->output) = out;
	}
	return out;
}

/**
 * @brief MIT 模式 PID 计算，直接传入位置误差和速度误差。
 *
 * @param data       PID 实例
 * @param pos_error  位置误差 (target - actual)
 * @param vel_error  速度误差 (target_vel - actual_vel)
 * @param deltaT_us  时间步长，单位微秒
 * @return 控制输出
 */
STATIC float pid_calc_mit(struct pid_data *data, float pos_error, float vel_error, float deltaT_us)
{
	const struct device *dev = data->pid_dev;
	if (dev == NULL) {
		return 0;
	}
	const struct pid_config *pid_para = dev->config;

	float kp = pid_para->k_p;
	float ki = pid_para->k_i;
	float kd = pid_para->k_d;

	if (deltaT_us == 0) {
		return 0;
	}
	if ((ki == ki) && !float_equal(ki, 0)) {
		data->err_integral += (pos_error * deltaT_us) / ki;
		if (pid_para->integral_limit != 0) {
			if (fabsf(data->err_integral) > pid_para->integral_limit) {
				data->err_integral = data->err_integral > 0
							     ? pid_para->integral_limit
							     : -pid_para->integral_limit;
			}
		}
	}
	if ((kd == kd)) {
		if ((pid_para->detri_lpf != pid_para->detri_lpf)) {
			data->err_derivate = kd * vel_error / deltaT_us;
		} else {
			data->err_derivate =
				pid_para->detri_lpf * data->err_derivate +
				(1 - pid_para->detri_lpf) * (kd * vel_error / deltaT_us);
		}
	}
	float out = kp * (pos_error + data->err_integral + data->err_derivate) +
		    pid_para->output_offset;
	if (pid_para->output_limit != 0 && fabsf(out) > pid_para->output_limit) {
		out = out > 0 ? pid_para->output_limit : -pid_para->output_limit;
	}
	if (data->output != NULL) {
		*(data->output) = out;
	}
	return out;
}

STATIC_VOID pid_reg_input(struct pid_data *data, float *curr, float *ref)
{
	if (data == NULL) {
		return;
	}
	data->curr = curr;
	data->ref = ref;
}

STATIC_VOID pid_reg_output(struct pid_data *data, float *output)
{
	if (data == NULL) {
		return;
	}
	data->output = output;
}

STATIC_VOID pid_reg_time(struct pid_data *data, uint32_t *curr_cyc, uint32_t *prev_cyc)
{
	if (data == NULL) {
		return;
	}
	data->curr_time = curr_cyc;
	data->prev_time = prev_cyc;
	data->err_integral = 0;
	data->err_derivate = 0;
}

STATIC_VOID mit_reg_detri_input(struct pid_data *data, float *detri_curr, float *detri_ref)
{
	if (data == NULL) {
		return;
	}
	data->detri_curr = detri_curr;
	data->detri_ref = detri_ref;
}

STATIC int pid_get_params(struct pid_data *data, struct pid_config *config)
{
	const struct device *dev = data->pid_dev;
	struct pid_config *pid_para = (struct pid_config *)dev->config;
	if (dev == NULL || pid_para == NULL) {
		return -ENOSYS;
	}
	if (config != NULL) {
		*config = *pid_para;
	}
	return 0;
}

STATIC int pid_set_params(const struct device *dev, struct pid_config *config)
{
	struct pid_config *pid_para = (struct pid_config *)dev->config;
	if (dev == NULL || pid_para == NULL) {
		return -ENOSYS;
	}
	*pid_para = *config;
	return 0;
}

#ifdef __cplusplus
}
#endif

#endif // PID_H
