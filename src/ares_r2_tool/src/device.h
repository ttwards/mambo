/*
 * Copyright (c) 2026 EclipseaHime017 <12210226@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef APP_ARES_R2_DEVICE_H_
#define APP_ARES_R2_DEVICE_H_

#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#define ARM_YAW_MOTOR_NODE   DT_NODELABEL(arm_motor1)
#define ARM_ZED_MOTOR_NODE   DT_NODELABEL(arm_motor2)
#define ARM_PITCH_MOTOR_NODE DT_NODELABEL(arm_motor3)

#define CONNECTOR_PITCH_MOTOR_NODE DT_NODELABEL(connector_motor1)
#define CONNECTOR_WYE_MOTOR_NODE   DT_NODELABEL(connector_motor2)
#define CONNECTOR_ROLL_MOTOR_NODE  DT_NODELABEL(connector_motor3)
#define CONNECTOR_GRIPPER_NODE     DT_NODELABEL(connector_gripper)

static const struct device *const arm_yaw_motor = DEVICE_DT_GET(ARM_YAW_MOTOR_NODE);
static const struct device *const arm_zed_motor = DEVICE_DT_GET(ARM_ZED_MOTOR_NODE);
static const struct device *const arm_pitch_motor = DEVICE_DT_GET(ARM_PITCH_MOTOR_NODE);

static const struct device *const connector_pitch_motor = DEVICE_DT_GET(CONNECTOR_PITCH_MOTOR_NODE);
static const struct device *const connector_wye_motor = DEVICE_DT_GET(CONNECTOR_WYE_MOTOR_NODE);
static const struct device *const connector_roll_motor = DEVICE_DT_GET(CONNECTOR_ROLL_MOTOR_NODE);
static const struct device *const connector_gripper = DEVICE_DT_GET(CONNECTOR_GRIPPER_NODE);

static const struct device *const arm_motors[] = {
	arm_yaw_motor,
	arm_zed_motor,
	arm_pitch_motor,
};

static const struct device *const connector_motors[] = {
	connector_pitch_motor,
	connector_wye_motor,
	connector_roll_motor,
};

#endif
