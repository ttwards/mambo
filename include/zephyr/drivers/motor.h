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

#include <zephyr/drivers/pid.h>
#include <zephyr/toolchain.h>
#include <sys/_intsup.h>
#include <zephyr/kernel.h>
#include <zephyr/kernel/thread.h>
#include <zephyr/sys/util.h>

#include <stdint.h>
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

#define motor_set_angle(dev, _angle)                                                               \
	motor_set(dev, &(motor_status_t){.angle = _angle, .mode = ML_ANGLE})
#define motor_set_rpm(dev, _rpm) motor_set(dev, &(motor_status_t){.rpm = _rpm, .mode = ML_SPEED})
#define motor_set_torque(dev, _torque)                                                             \
	motor_set(dev, &(motor_status_t){.torque = _torque, .mode = ML_TORQUE})
#define motor_set_speed(dev, _speed)                                                               \
	motor_set(dev, &(motor_status_t){.rpm = _speed, .mode = ML_SPEED})
#define motor_set_mit(dev, _speed, _angle, _torque)                                                \
	motor_set(dev, &(motor_status_t){                                                          \
			       .rpm = _speed, .angle = _angle, .torque = _torque, .mode = MIT})
#define motor_set_vo(dev, _speed) motor_set(dev, &(motor_status_t){.rpm = _speed, .mode = VO})
#define motor_set_speed_limit(dev, _min, _max)                                                     \
	motor_set(dev, &(motor_status_t){.speed_limit = {_min, _max}})
#define motor_set_torque_limit(dev, _min, _max)                                                    \
	motor_set(dev, &(motor_status_t){.torque_limit = {_min, _max}})

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
 * @brief 电机工作模式枚举
 *
 * MIT: MIT模式
 * PV: 位置-速度控制
 * VO: 速度控制
 * MULTILOOP: 多环串联控制
 */
enum motor_mode {
	MIT = 0,
	PV = 1,
	VO = 2,
	ML_TORQUE = 3,
	ML_ANGLE = 4,
	ML_SPEED = 5,
	HYBRID = 6,
};

/**
 * @brief 电机控制命令枚举
 */
enum motor_cmd {
	ENABLE_MOTOR,
	DISABLE_MOTOR,
	SET_ZERO,
	CLEAR_PID,
	CLEAR_ERROR,
	AUTO_REPORT_ENABLE,	//rs motor only
	AUTO_REPORT_DISABLE //rs motor only
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
	/** Motor capabilities */
	char capabilities[4][12];
	struct pid_data *pid_datas[4];
};

typedef float (*motor_slip_cb_t)(const struct device *dev);

struct motor_driver_data {
	float angle;
	float rpm;
	float torque;
	float temperature; /* Cannot be set in target */
	float sum_angle;

	float speed_limit[2];
	float torque_limit[2];

	enum motor_mode mode;
};

typedef struct motor_driver_data motor_status_t;

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
 * @param status 目标状态
 * @return int 成功: 0, 失败: 负值
 */
typedef int (*motor_api_set_t)(const struct device *dev, motor_status_t *status);

/**
 * @typedef motor_api_set_mode_t
 * @brief 设置电机模式
 *
 * @param dev 指向电机设备的指针
 * @param mode 电机模式
 * @return void
 */
typedef void (*motor_api_set_mode_t)(const struct device *dev, enum motor_mode mode);

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
	motor_api_set_mode_t motor_set_mode;
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
__syscall int motor_set(const struct device *dev, motor_status_t *status);

static inline int z_impl_motor_set(const struct device *dev, motor_status_t *status)
{
	const struct motor_driver_api *api = (const struct motor_driver_api *)dev->api;
	if (api->motor_set == NULL) {
		return -ENOSYS;
	}
	return api->motor_set(dev, status);
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

/**
 * @brief 设置电机模式
 *
 * @param dev 电机设备指针
 * @param mode 电机模式
 * @return void
 */
__syscall void motor_set_mode(const struct device *dev, enum motor_mode mode);

static inline void z_impl_motor_set_mode(const struct device *dev, enum motor_mode mode)
{
	const struct motor_driver_api *api = (const struct motor_driver_api *)dev->api;
	if (api->motor_set_mode == NULL) {
		return;
	}
	api->motor_set_mode(dev, mode);
}

#define DT_GET_CANPHY(node_id) DEVICE_DT_GET(DT_PHANDLE(node_id, can_channel))

#define NEW_PID_INSTANCE_STRUCT(node_id, prop, idx)                                                \
	PID_NEW_INSTANCE(DT_PROP_BY_IDX(node_id, prop, idx), DT_NODE_FULL_NAME_UNQUOTED(node_id))

#define GET_PID_INSTANCE_PTR(node_id, prop, idx)                                                   \
	&PID_INS_NAME(DT_PROP_BY_IDX(node_id, prop, idx), DT_NODE_FULL_NAME_UNQUOTED(node_id))

#define MOTOR_DT_DRIVER_PID_DEFINE(node_id)                                                        \
	DT_FOREACH_PROP_ELEM(node_id, controllers, NEW_PID_INSTANCE_STRUCT)

#define MOTOR_DT_DRIVER_CONFIG_GET(node_id)                                                        \
	{                                                                                          \
		.phy = (const struct device *)DT_GET_CANPHY(node_id),                              \
		.id = DT_PROP(node_id, id),                                                        \
		.tx_id = DT_PROP_OR(node_id, tx_id, 0x00),                                         \
		.rx_id = DT_PROP_OR(node_id, rx_id, 0x00),                                         \
		.capabilities = DT_PROP(node_id, capabilities),                                    \
		.pid_datas = {DT_FOREACH_PROP_ELEM_SEP(node_id, controllers, GET_PID_INSTANCE_PTR,   \
                                                  (, ))},                                      \
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
