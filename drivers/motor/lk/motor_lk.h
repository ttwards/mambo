#ifndef MOTOR_LK_H
#define MOTOR_LK_H

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/motor.h>

#define DT_DRV_COMPAT lk_motor

// 基础ID定义
#define LK_CMD_ID_BASE   0x140
#define LK_REPLY_ID_BASE 0x180

// 命令字节 (Data[0])
#define LK_CMD_READ_STAT    0x9C
#define LK_CMD_CLEAR_ERR    0x9B
#define LK_CMD_MOTOR_OFF    0x80
#define LK_CMD_MOTOR_RUN    0x88
#define LK_CMD_MOTOR_STOP   0x81
#define LK_CMD_SET_ZERO_ROM 0x19
#define LK_CMD_SET_ZERO     0x95
#define LK_SET_PARAM        0xC1

// 控制命令字节
#define LK_CMD_TORQUE_LOOP    0xA1 // 转矩闭环
#define LK_CMD_SPEED_LOOP     0xA2 // 速度闭环
#define LK_CMD_POS_LOOP_MULTI 0xA4 // 多圈位置闭环2 (带限速)
#define LK_PARAM_ANGLE_UPDATE  0x0A // 角度控制器参数更新
#define LK_PARAM_SPEED_UPDATE  0x0B // 速度控制器参数更新
#define LK_PARAM_TORQUE_UPDATE 0x0C // 转矩控制器参数更新
// 单位转换因子
#define LK_POS_FACTOR         100.0f  // 0.01 degree/LSB -> float * 100 = int
#define LK_SPD_FACTOR_FINE    100.0f  // 0.01 dps/LSB (速度闭环控制值)
#define LK_SPD_FACTOR_COARSE  1.0f    // 1 dps/LSB (位置模式下的限速值)
#define LK_TORQUE_RAW_MAX     2048.0f // 转矩控制范围 -2048~2048

#define SIZE_OF_ARRAY(x) (sizeof(x) / sizeof(x[0]))

#define CAN_SEND_STACK_SIZE 4096
#define CAN_SEND_PRIORITY   -1
#define PI                  3.14159265f
#ifdef RAD2DEG
#undef RAD2DEG
#endif
#define RAD2DEG             (180.0f / PI)
#define RPM2DPS             6.0f // RPM 转 degree per second

enum CONTROL_MODE {
	TORQUE_MODE = 0, // 对应 0xA1
	SPEED_MODE,      // 对应 0xA2
	POSITION_MODE    // 对应 0xA4
};

struct lk_can_id {

	uint16_t id;
};

struct lk_motor_data {
	struct motor_driver_data common;
	uint8_t can_id; // 电机物理ID (1~32)

	int16_t missed_times;
	int8_t err;

	// 目标值缓存
	float target_pos;    // 度
	float target_speed;  // dps (degree per second)
	float target_torque; // 原始单位或映射后的单位

	// 限制值缓存
	float limit_speed;  // dps
	float limit_torque; // raw (-2048~2048)

	// 反馈原始数据
	int16_t RAWtorque;
	int16_t RAWspeed;
	uint16_t RAWencoder;
	int8_t RAWtemp;

	bool online;
	bool update;
	bool enabled;
	struct motor_controller_params params[3];
	bool params_update[3];
};

struct lk_motor_cfg {
	struct motor_driver_config common;
	uint8_t id; // 配置文件中定义的ID
};

struct k_work_q lk_work_queue;
int lk_set(const struct device *dev, motor_setpoint_t *status);
int lk_get(const struct device *dev, motor_status_t *status);
void lk_motor_control(const struct device *dev, enum motor_cmd cmd);
void lk_rx_data_handler(struct k_work *work);
void lk_tx_data_handler(struct k_work *work);
void lk_tx_params_data_handler(struct k_work *work);
void lk_init_handler(struct k_work *work);
void lk_tx_isr_handler(struct k_timer *dummy);

static const struct motor_driver_api motor_api_funcs = {
	.motor_get = lk_get,
	.motor_set = lk_set,
	.motor_control = lk_motor_control,
};
#define MOTOR_COUNT            DT_NUM_INST_STATUS_OKAY(lk_motor)
#define LK_MOTOR_POINTER(inst) DEVICE_DT_GET(DT_DRV_INST(inst)),
static const struct device *motor_devices[] = {DT_INST_FOREACH_STATUS_OKAY(LK_MOTOR_POINTER)};

K_THREAD_STACK_DEFINE(lk_work_queue_stack, CAN_SEND_STACK_SIZE);

K_WORK_DEFINE(lk_rx_data_handle, lk_rx_data_handler);
K_WORK_DEFINE(lk_tx_data_handle, lk_tx_data_handler);
K_WORK_DEFINE(lk_tx_params_data_handle, lk_tx_params_data_handler);
K_WORK_DEFINE(lk_init_work, lk_init_handler);

K_TIMER_DEFINE(lk_tx_timer, lk_tx_isr_handler, NULL);

#define LKMOTOR_DATA_INST(inst)                                                                    \
	static struct lk_motor_data lk_motor_data_##inst = {                                       \
		.common = MOTOR_DT_DRIVER_DATA_INST_GET(inst),                                     \
		.online = false,                                                                   \
		.missed_times = 0,                                                                 \
		.err = 0,                                                                          \
		.limit_speed = 800,                                                                \
		.limit_torque = 12,                                                                \
		.target_pos = 0,                                                                   \
		.target_speed = 0,                                                                 \
		.target_torque = 0,                                                                \
		.update = false,                                                                   \
	};

#define LKMOTOR_CONFIG_INST(inst)                                                                  \
	static const struct lk_motor_cfg lk_motor_cfg_##inst = {                                   \
		.common = MOTOR_DT_DRIVER_CONFIG_INST_GET(inst),                                   \
		.id = (uint8_t)DT_PROP(DT_DRV_INST(inst), id),                                     \
	};

#define MOTOR_DEVICE_DT_DEFINE(node_id, init_fn, pm, data, config, level, prio, api, ...)          \
	DEVICE_DT_DEFINE(node_id, init_fn, pm, data, config, level, prio, api, __VA_ARGS__)

#define MOTOR_DEVICE_DT_INST_DEFINE(inst, ...)                                                     \
	MOTOR_DEVICE_DT_DEFINE(DT_DRV_INST(inst), __VA_ARGS__)

int lk_init(const struct device *dev);

#define LKMOTOR_DEFINE_INST(inst)                                                                  \
	MOTOR_DEVICE_DT_INST_DEFINE(inst, lk_init, NULL, &lk_motor_data_##inst,                    \
				    &lk_motor_cfg_##inst, POST_KERNEL, CONFIG_MOTOR_INIT_PRIORITY, \
				    &motor_api_funcs);

#define LKMOTOR_INST(inst)                                                                         \
	LKMOTOR_CONFIG_INST(inst)                                                                  \
	LKMOTOR_DATA_INST(inst)                                                                    \
	LKMOTOR_DEFINE_INST(inst)

#endif // MOTOR_LK_H
