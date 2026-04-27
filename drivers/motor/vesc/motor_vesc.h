/**
 * motor_vesc.h
 *
 * Header file for VESC motor driver.
 */
#ifndef MOTOR_VESC_H
#define MOTOR_VESC_H

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/motor.h>
#include <zephyr/drivers/pid.h>

#define DT_DRV_COMPAT vesc_motor

#define VESC_PI             3.14159265f
#define VESC_DEG_PER_RAD    (180.0f / VESC_PI)
#define VESC_RAD_PER_DEG    (VESC_PI / 180.0f)
#define VESC_RPM_PER_RADPS  (60.0f / (2.0f * VESC_PI))
#define VESC_RADPS_PER_RPM  ((2.0f * VESC_PI) / 60.0f)

// 控制指令
#define CAN_PACKET_SET_DUTY          0 // 设置占空比
#define CAN_PACKET_SET_CURRENT       1 // 设置电流
#define CAN_PACKET_SET_CURRENT_BRAKE 2 // 设置制动电流
#define CAN_PACKET_SET_RPM           3 // 设置转速
#define CAN_PACKET_SET_POS           4 // 设置位置

#define CAN_PACKET_PING                17 // 测试连接
#define CAN_PACKET_PONG                18 // 测试响应
#define CAN_PACKET_CONF_CURRENT_LIMITS 21 // 配置电流限制

// 常用状态
#define CAN_PACKET_STATUS   9  // 返回RPM（int32），十倍总电流（int16）和1000倍占空比（int16）
#define CAN_PACKET_STATUS_4 16 // 10倍MOSFET温度、10倍电机温度、10倍输入电流、50倍电机位置（int16）
#define CAN_PACKET_STATUS_5 27 // 转速计（int32），十倍输入电压（int16）和一个保留位（int16）

#define CAN_FILTER_MASK 0x0

struct vesc_can_id {
	uint32_t motor_id: 8; // 电机ID
	uint32_t msg_type: 8; // 消息类型
};

struct vesc_motor_data {
	struct motor_driver_data common;
	int8_t err;

	bool online; // 电机在线状态
	bool enable; // 电机使能状态
	// bool update;
	float delta_deg_sum;
	float target_angle;   // 目标位置，单位度
	float target_radps;   // 目标速度，单位弧度每秒
	float target_current; // 目标电流，单位安培

	int32_t RAWangle;   // 原始位置数据
	int32_t RAWrpm;     // 原始速度数据
	int32_t RAWcurrent; // 原始电流数据
	int32_t RAWtemp;    // 原始温度数据
};

struct vesc_motor_config {
	struct motor_driver_config common;
	float gear_ratio; // 减速比
	int freq;

	float kv; // 电机速度常数
	float kt; // 电机力矩常数

	float pole_pairs; // 极对数

	float v_max; // 最大电压
	float p_max; // 最大位置
	float t_max; // 最大扭矩
	float i_max; // 最大电流
};

int vesc_set(const struct device *dev, motor_status_t *status);
int vesc_get(const struct device *dev, motor_status_t *status);
void vesc_motor_control(const struct device *dev, enum motor_cmd cmd);
void vesc_motor_set_mode(const struct device *dev, enum motor_mode mode);

extern const struct motor_driver_api vesc_motor_api;

#define VESC_MOTOR_DATA_INST(inst)                                                                 \
	static struct vesc_motor_data vesc_motor_data_##inst = {                                   \
		.common = MOTOR_DT_DRIVER_DATA_INST_GET(inst),                                     \
		.online = false,                                                                   \
		.enable = false,                                                                   \
		.err = 0,                                                                          \
		.delta_deg_sum = 0,                                                                \
		.target_angle = 0,                                                                 \
		.target_radps = 0,                                                                 \
		.target_current = 0,                                                               \
	};

#define VESC_MOTOR_CONFIG_INST(inst)                                                               \
	static const struct vesc_motor_config vesc_motor_config_##inst = {                         \
		.common =                                                                          \
			{                                                                          \
				.phy = DEVICE_DT_GET(DT_INST_PHANDLE(inst, can_channel)),          \
				.id = DT_INST_PROP(inst, id),                                      \
			},                                                                         \
		.kv = DT_STRING_UNQUOTED_OR(DT_DRV_INST(inst), kv, 150.0f),                        \
		.kt = 60.0f /                                                                      \
		      (2.f * VESC_PI * (float)DT_STRING_UNQUOTED_OR(DT_DRV_INST(inst), kv, 150)),  \
		.pole_pairs = DT_STRING_UNQUOTED_OR(DT_DRV_INST(inst), pole_pairs, 14),            \
		.gear_ratio = DT_STRING_UNQUOTED_OR(DT_DRV_INST(inst), gear_ratio, 1.0f),          \
		.p_max = DT_STRING_UNQUOTED_OR(DT_DRV_INST(inst), p_max, 12.57f),                  \
		.v_max = DT_STRING_UNQUOTED_OR(DT_DRV_INST(inst), v_max, 314.0f),                  \
		.t_max = DT_STRING_UNQUOTED_OR(DT_DRV_INST(inst), t_max, 1.5f),                    \
		.i_max = DT_STRING_UNQUOTED_OR(DT_DRV_INST(inst), i_max, 22.2f),                   \
	};

#define MOTOR_DEVICE_DT_DEFINE(node_id, init_fn, pm, data, config, level, prio, api, ...)          \
	DEVICE_DT_DEFINE(node_id, init_fn, pm, data, config, level, prio, api, __VA_ARGS__)

#define MOTOR_DEVICE_DT_INST_DEFINE(inst, ...)                                                     \
	MOTOR_DEVICE_DT_DEFINE(DT_DRV_INST(inst), __VA_ARGS__)

#define VESC_MOTOR_DEFINE_INST(inst)                                                               \
	MOTOR_DEVICE_DT_INST_DEFINE(inst, vesc_init, NULL, &vesc_motor_data_##inst,                \
				    &vesc_motor_config_##inst, POST_KERNEL,                        \
				    CONFIG_MOTOR_INIT_PRIORITY, &vesc_motor_api);

#define VESC_MOTOR_INST(inst)                                                                      \
	VESC_MOTOR_CONFIG_INST(inst)                                                               \
	VESC_MOTOR_DATA_INST(inst)                                                                 \
	VESC_MOTOR_DEFINE_INST(inst)

#endif // MOTOR_VESC_H
