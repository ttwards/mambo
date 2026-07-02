/*
 * Copyright (c) 2025 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Linear Actuator Interface
 *
 * Generic API for linear servo actuators (micro electric cylinders, grippers, etc.).
 * This API is designed for position-controlled linear actuators using UART/RS485
 * communication, distinct from the CAN-based rotary motor API (motor.h).
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_LINEAR_ACTUATOR_H_
#define ZEPHYR_INCLUDE_DRIVERS_LINEAR_ACTUATOR_H_

#include <stdint.h>
#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Fault bit definitions for linear actuator status
 * @{
 */
#define LA_FAULT_STALL       BIT(0) /**< Stall / blocked rotor */
#define LA_FAULT_OVERTEMP    BIT(1) /**< Over-temperature protection */
#define LA_FAULT_OVERCURRENT BIT(2) /**< Over-current protection */
#define LA_FAULT_MOTOR_ERROR BIT(3) /**< Motor error / end of life */
/** @} */

/**
 * @brief Linear actuator status structure
 */
struct la_status {
	uint16_t target_pos; /**< Target position (0~2000) */
	int16_t current_pos; /**< Actual current position */
	int8_t temperature;  /**< Internal temperature in degrees Celsius */
	uint16_t current_ma; /**< Motor drive current in milliamps */
	int16_t force_grams; /**< Force sensor value in grams (0 if no sensor) */
	uint8_t faults;      /**< Fault bitmask (LA_FAULT_*) */
	bool online;         /**< Device is online and communicating */
};

/**
 * @brief Linear actuator command codes
 */
enum la_cmd {
	LA_CMD_ENABLE,      /**< Enable motor power output (work) */
	LA_CMD_DISABLE,     /**< Emergency stop (disable motor output) */
	LA_CMD_PAUSE,       /**< Pause (resume by sending position) */
	LA_CMD_CLEAR_FAULT, /**< Clear fault state */
	LA_CMD_SAVE_PARAMS, /**< Save current params to Flash (persist) */
};

/**
 * @typedef la_api_set_position_t
 * @brief Set target position callback
 */
typedef int (*la_api_set_position_t)(const struct device *dev, uint16_t position);

/**
 * @typedef la_api_get_status_t
 * @brief Get actuator status callback
 */
typedef int (*la_api_get_status_t)(const struct device *dev, struct la_status *status);

/**
 * @typedef la_api_cmd_t
 * @brief Execute a control command callback
 */
typedef int (*la_api_cmd_t)(const struct device *dev, enum la_cmd cmd);

/**
 * @typedef la_api_set_param_t
 * @brief Write parameter to control table callback
 */
typedef int (*la_api_set_param_t)(const struct device *dev, uint8_t index, uint16_t value);

/**
 * @brief Linear actuator driver API
 *
 * Each driver implementation fills in the callbacks it supports.
 * Unsupported callbacks should be NULL; the inline wrappers check
 * and return -ENOSYS.
 */
__subsystem struct linear_actuator_driver_api {
	la_api_set_position_t set_position;
	la_api_get_status_t get_status;
	la_api_cmd_t cmd;
	la_api_set_param_t set_param;
};

/**
 * @brief Set target position with feedback and automatic retry.
 *
 * Sends a position command in positioning mode. The driver waits for a
 * reply frame. If no reply is received within the configured timeout,
 * the frame is retried up to max_retries times.
 *
 * @param dev Linear actuator device pointer
 * @param position Target position (valid range: 0~2000)
 * @return 0 on success, negative errno on failure
 * @retval -ETIMEDOUT No reply after all retries
 * @retval -EIO       Protocol or communication error
 */
static inline int la_set_position(const struct device *dev, uint16_t position)
{
	const struct linear_actuator_driver_api *api =
		(const struct linear_actuator_driver_api *)dev->api;

	if (api == NULL || api->set_position == NULL) {
		return -ENOSYS;
	}
	return api->set_position(dev, position);
}

/**
 * @brief Get current actuator status.
 *
 * Sends a status query command and parses the response into @p status.
 * Does NOT retry on failure (use la_set_position for reliable feedback).
 *
 * @param dev Linear actuator device pointer
 * @param status Pointer to status structure to fill
 * @return 0 on success, negative errno on failure
 */
static inline int la_get_status(const struct device *dev, struct la_status *status)
{
	const struct linear_actuator_driver_api *api =
		(const struct linear_actuator_driver_api *)dev->api;

	if (api == NULL || api->get_status == NULL) {
		return -ENOSYS;
	}
	return api->get_status(dev, status);
}

/**
 * @brief Enable the actuator (start motor power output).
 *
 * The actuator must be enabled before position commands take effect.
 *
 * @param dev Linear actuator device pointer
 * @return 0 on success, negative errno on failure
 */
static inline int la_enable(const struct device *dev)
{
	const struct linear_actuator_driver_api *api =
		(const struct linear_actuator_driver_api *)dev->api;

	if (api == NULL || api->cmd == NULL) {
		return -ENOSYS;
	}
	return api->cmd(dev, LA_CMD_ENABLE);
}

/**
 * @brief Disable (emergency stop) the actuator.
 *
 * Cuts motor power output immediately. A subsequent position command
 * requires re-enabling first.
 *
 * @param dev Linear actuator device pointer
 * @return 0 on success, negative errno on failure
 */
static inline int la_disable(const struct device *dev)
{
	const struct linear_actuator_driver_api *api =
		(const struct linear_actuator_driver_api *)dev->api;

	if (api == NULL || api->cmd == NULL) {
		return -ENOSYS;
	}
	return api->cmd(dev, LA_CMD_DISABLE);
}

/**
 * @brief Clear fault state on the actuator.
 *
 * After an overcurrent or stall fault, the actuator must be explicitly
 * cleared before it can operate again.
 *
 * @param dev Linear actuator device pointer
 * @return 0 on success, negative errno on failure
 */
static inline int la_clear_fault(const struct device *dev)
{
	const struct linear_actuator_driver_api *api =
		(const struct linear_actuator_driver_api *)dev->api;

	if (api == NULL || api->cmd == NULL) {
		return -ENOSYS;
	}
	return api->cmd(dev, LA_CMD_CLEAR_FAULT);
}

/**
 * @brief Save current parameters to Flash (parameter binding).
 *
 * Persists the current RAM parameter table to internal Flash so
 * settings survive power cycles.
 *
 * @param dev Linear actuator device pointer
 * @return 0 on success, negative errno on failure
 */
static inline int la_save_params(const struct device *dev)
{
	const struct linear_actuator_driver_api *api =
		(const struct linear_actuator_driver_api *)dev->api;

	if (api == NULL || api->cmd == NULL) {
		return -ENOSYS;
	}
	return api->cmd(dev, LA_CMD_SAVE_PARAMS);
}

/**
 * @brief Pause the actuator (soft stop).
 *
 * Suspends motor output. Can be resumed by sending a new position
 * command without needing to re-enable.
 *
 * @param dev Linear actuator device pointer
 * @return 0 on success, negative errno on failure
 */
static inline int la_pause(const struct device *dev)
{
	const struct linear_actuator_driver_api *api =
		(const struct linear_actuator_driver_api *)dev->api;

	if (api == NULL || api->cmd == NULL) {
		return -ENOSYS;
	}
	return api->cmd(dev, LA_CMD_PAUSE);
}

/**
 * @brief Set a parameter in the actuator's control table.
 *
 * Low-level API to write arbitrary parameters by control table index.
 * Most users should use the higher-level la_* functions instead.
 *
 * @param dev Linear actuator device pointer
 * @param index Control table byte offset
 * @param value 16-bit value to write (little-endian)
 * @return 0 on success, negative errno on failure
 */
static inline int la_set_param(const struct device *dev, uint8_t index, uint16_t value)
{
	const struct linear_actuator_driver_api *api =
		(const struct linear_actuator_driver_api *)dev->api;

	if (api == NULL || api->set_param == NULL) {
		return -ENOSYS;
	}
	return api->set_param(dev, index, value);
}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_LINEAR_ACTUATOR_H_ */
