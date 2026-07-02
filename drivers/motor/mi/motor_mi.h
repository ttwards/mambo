#ifndef MOTOR_MI_H
#define MOTOR_MI_H

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/motor.h>

#define DT_DRV_COMPAT mi_motor

// 控制参数最值
#define P_MIN         -12.5f
#define P_MAX         12.5f
#define V_MIN         -30.0f
#define V_MAX         30.0f
#define KP_MIN        0.0f
#define KP_MAX        500.0f
#define KD_MIN        0.0f
#define KD_MAX        5.0f
#define T_MIN         -12.0f
#define T_MAX         12.0f
// #define MAX_P  720
// #define MIN_P  -720
#define CUR_MAX       23.0f
// 主机CAN ID设置
#define Master_CAN_ID 0x00

// 控制命令宏定义
#define Communication_Type_GetID              0x00 // 获取设备ID和64位MCU唯一标识符
#define Communication_Type_MotionControl_MIT  0x01 // 向主机发送控制指令
#define Communication_Type_MotorFeedback      0x02 // 向主机反馈电机运行状态
#define Communication_Type_MotorEnable        0x03 // 电机使能运行
#define Communication_Type_MotorStop          0x04 // 电机停止运行
#define Communication_Type_SetPosZero         0x06 // 设置电机机械零位
#define Communication_Type_SetID              0x07 // 更改当前电机CAN_ID
#define Communication_Type_Control_Mode       0x12
#define Communication_Type_GetSingleParameter 0x11 // 读取单个参数
#define Communication_Type_SetSingleParameter 0x12 // 设定单个参数
#define Communication_Type_ErrorFeedback      0x15 // 故障反馈帧

// 操作宏定义
#define HIGH_BYTE(x)           ((x) >> 8)
#define LOW_BYTE(x)            ((x) & 0xFF)
#define COMBINE_HL8(HIGH, LOW) ((HIGH << 8) + LOW)
#define PI                     3.14159265f
#define SIZE_OF_ARRAY(x)       (sizeof(x) / sizeof(x[0]))
#define RAD2ROUND              1.0f / (2 * PI)
#ifdef RAD2DEG
#undef RAD2DEG
#endif
#define RAD2DEG                (180.0f / PI)
// 参数读取宏定义
#define Run_mode               0x7005
#define Iq_Ref                 0x7006
#define Spd_Ref                0x700A
#define Limit_Torque           0x700B
#define Cur_Kp                 0x7010
#define Cur_Ki                 0x7011
#define Cur_Filt_Gain          0x7014
#define Loc_Ref                0x7016
#define Limit_Spd              0x7017
#define Limit_Cur              0x7018

#define Gain_Angle  720 / 32767.0
#define Bias_Angle  0x8000
#define Gain_Speed  30 / 32767.0
#define Bias_Speed  0x8000
#define Gain_Torque 12 / 32767.0
#define Bias_Torque 0x8000
#define Temp_Gain   0.1

#define Motor_Error 0x00
#define Motor_OK    0x01

#define CAN_SEND_STACK_SIZE 4096
#define CAN_SEND_PRIORITY   -1

enum CONTROL_MODE // 控制模式定义
{
	Motion_mode = 0, // 运控模式
	Position_mode,   // 位置模式
	Speed_mode,      // 速度模式
	Current_mode     // 电流模式
};

enum ERROR_TAG // 错误回传对照
{
	OK = 0,                // 无故障
	BAT_LOW_ERR = 1,       // 欠压故障
	OVER_CURRENT_ERR = 2,  // 过流
	OVER_TEMP_ERR = 3,     // 过温
	MAGNETIC_ERR = 4,      // 磁编码故障
	HALL_ERR_ERR = 5,      // HALL编码故障
	NO_CALIBRATION_ERR = 6 // 未标定
};

struct mi_can_id {
	uint32_t id: 8;          // 目标ID
	uint32_t data: 16;       // 数据区
	uint32_t mi_msg_mode: 5; // 通信类型
	uint32_t res: 3;         // 保留位src/yunqiu
};
struct mi_motor_data {
	struct motor_driver_data common;
	uint8_t can_id;    // CAN ID
	uint8_t master_id; // MCU唯一标识符[后8位，共64位]

	int16_t missed_times;
	int8_t err;

	float limit_cur;
	float delta_deg_sum;
	// Target
	float target_pos;
	float target_radps;
	float target_torque;

	uint16_t RAWangle;
	uint16_t RAWrpm;
	uint16_t RAWtorque;
	uint16_t RAWtemp;

	uint8_t error_code;
	bool online;
	bool update;
	bool enabled;
	struct motor_controller_params params;
};

struct mi_motor_cfg {
	struct motor_driver_config common;
	float gear_ratio;
};

struct k_work_q mi_work_queue;
void mi_rx_handler(const struct device *can_dev, struct can_frame *frame, void *user_data);
int mi_set(const struct device *dev, motor_setpoint_t *status);
int mi_get(const struct device *dev, motor_status_t *status);
void mi_motor_control(const struct device *dev, enum motor_cmd cmd);

void mi_rx_data_handler(struct k_work *work);

void mi_tx_isr_handler(struct k_timer *dummy);
void mi_tx_data_handler(struct k_work *work);

void mi_isr_init_handler(struct k_timer *dummy);
void mi_init_handler(struct k_work *work);

static const struct motor_driver_api motor_api_funcs = {
	.motor_get = mi_get,
	.motor_set = mi_set,
	.motor_control = mi_motor_control,
};

#define MOTOR_COUNT            DT_NUM_INST_STATUS_OKAY(mi_motor)
#define MI_MOTOR_POINTER(inst) DEVICE_DT_GET(DT_DRV_INST(inst)),
static const struct device *motor_devices[] = {DT_INST_FOREACH_STATUS_OKAY(MI_MOTOR_POINTER)};

K_THREAD_STACK_DEFINE(mi_work_queue_stack, CAN_SEND_STACK_SIZE);

CAN_MSGQ_DEFINE(mi_can_rx_msgq, 12);
K_MSGQ_DEFINE(mi_thread_proc_msgq, sizeof(bool), MOTOR_COUNT * 2, 4);

K_WORK_DEFINE(mi_rx_data_handle, mi_rx_data_handler);
K_WORK_DEFINE(mi_tx_data_handle, mi_tx_data_handler);

K_WORK_DEFINE(mi_init_work, mi_init_handler);

K_TIMER_DEFINE(mi_tx_timer, mi_tx_isr_handler, NULL);

#define MIMOTOR_DATA_INST(inst)                                                                    \
	static struct mi_motor_data mi_motor_data_##inst = {                                       \
		.common = MOTOR_DT_DRIVER_DATA_INST_GET(inst),                                     \
		.online = false,                                                                   \
		.missed_times = 0,                                                                 \
		.err = 0,                                                                          \
		.delta_deg_sum = 0,                                                                \
		.target_pos = 0,                                                                   \
		.target_radps = 0,                                                                 \
		.target_torque = 0,                                                                \
		.params = {0, 0},                                                                  \
		.update = false,                                                                   \
	};

#define MIMOTOR_CONFIG_INST(inst)                                                                  \
	static const struct mi_motor_cfg mi_motor_cfg_##inst = {                                   \
		.common = MOTOR_DT_DRIVER_CONFIG_INST_GET(inst),                                   \
		.gear_ratio = (float)DT_STRING_UNQUOTED_OR(DT_DRV_INST(inst), gear_ratio, 1),      \
	};

#define MOTOR_DEVICE_DT_DEFINE(node_id, init_fn, pm, data, config, level, prio, api, ...)          \
	DEVICE_DT_DEFINE(node_id, init_fn, pm, data, config, level, prio, api, __VA_ARGS__)

#define MOTOR_DEVICE_DT_INST_DEFINE(inst, ...)                                                     \
	MOTOR_DEVICE_DT_DEFINE(DT_DRV_INST(inst), __VA_ARGS__)

#define MIMOTOR_DEFINE_INST(inst)                                                                  \
	MOTOR_DEVICE_DT_INST_DEFINE(inst, mi_init, NULL, &mi_motor_data_##inst,                    \
				    &mi_motor_cfg_##inst, POST_KERNEL, CONFIG_MOTOR_INIT_PRIORITY, \
				    &motor_api_funcs);

#define MIMOTOR_INST(inst)                                                                         \
	MIMOTOR_CONFIG_INST(inst)                                                                  \
	MIMOTOR_DATA_INST(inst)                                                                    \
	MIMOTOR_DEFINE_INST(inst)

#endif // MOTOR_MI_H
