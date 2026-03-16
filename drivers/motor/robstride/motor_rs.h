#ifndef MOTOR_RS_H
#define MOTOR_RS_H

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/motor.h>
#include <zephyr/drivers/pid.h>

#define DT_DRV_COMPAT rs_motor

#define KP_MAX 500.0f
#define KP_MIN 0.0f
#define KD_MAX 5.0f
#define KD_MIN 0.0f

// 控制命令宏定义
#define Communication_Type_GetID             0x00 // 获取设备ID和64位MCU唯一标识符
#define Communication_Type_MotionControl_MIT 0x01 // 向主机发送控制指令
#define Communication_Type_MotorFeedback     0x02 // 向主机反馈电机运行状态
#define Communication_Type_MotorEnable       0x03 // 电机使能运行
#define Communication_Type_MotorStop         0x04 // 电机停止运行
#define Communication_Type_SetPosZero        0x06 // 设置电机机械零位
#define Communication_Type_SetID             0x07 // 更改当前电机CAN_ID

#define Communication_Type_GetSingleParameter 0x11 // 读取单个参数
#define Communication_Type_SetSingleParameter 0x12 // 设定单个参数

#define Communication_Type_ErrorFeedback  0x15 // 故障反馈帧
#define Communication_Type_SaveAllData    0x16 // 保存所有数据
#define Communication_Type_ModifyBitrate  0x17 // 修改波特率
#define Communication_Type_MotorReport    0x18 // 电机运行状态主动报告
#define Communication_Type_SwitchProtocol 0x19 // 切换协议

#define PI               3.14159265f
#define SIZE_OF_ARRAY(x) (sizeof(x) / sizeof(x[0]))
#define RAD2ROUND        1.0f / (2 * PI)
#define RAD2DEG          (180.0f / PI)
// 参数读取宏定义
#define Run_mode         0x7005
#define Iq_Ref           0x7006
#define Spd_Ref          0x700A
#define Limit_Torque     0x700B
#define Cur_Kp           0x7010
#define Cur_Ki           0x7011
#define Cur_Filt_Gain    0x7014
#define Loc_Ref          0x7016
#define Limit_Spd        0x7017
#define Limit_Cur        0x7018
#define Loc_Kp           0x701E
#define Spd_Kp           0x701F
#define Spd_Ki           0x7020
#define EPScan_time      0x7026

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

#define CAN_FILTER_MASK 0x00000000

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

struct rs_can_id {
	uint8_t motor_id: 8;  // 目标ID
	uint8_t master_id: 8; // 主机ID
	uint8_t reserved: 8;  // 数据区
	uint32_t msg_type: 5; // 通信类型
};

enum MOTOR_TYPE {
	RS00 = 0x00,
	RS01 = 0x01,
	RS02 = 0x02,
	RS03 = 0x03,
	RS04 = 0x04,
	RS05 = 0x05,
	RS06 = 0x06,

	EL05 = 0x15,
};

struct rs_motor_data {
	struct motor_driver_data common;
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
	uint8_t missed_times;
	bool online;
	bool enabled;
	bool reporting;
	struct pid_config params;
	uint32_t last_report_time;
};

struct rs_motor_cfg {
	struct motor_driver_config common;
	enum MOTOR_TYPE motor_type;

	float p_max;
	float v_max;
	float t_max;
};

struct k_work_q rs_work_queue;
int rs_set(const struct device *dev, motor_status_t *status);
int rs_get(const struct device *dev, motor_status_t *status);
void rs_motor_control(const struct device *dev, enum motor_cmd cmd);
int rs_motor_set_mode(const struct device *dev, enum motor_mode mode);

void rs_rx_monitor_handler(struct k_work *work);

static const struct motor_driver_api rs_motor_api = {
	.motor_get = rs_get,
	.motor_set = rs_set,
	.motor_control = rs_motor_control,
	.motor_set_mode = rs_motor_set_mode,
};

#define MOTOR_COUNT            DT_NUM_INST_STATUS_OKAY(rs_motor)
#define RS_MOTOR_POINTER(inst) DEVICE_DT_GET(DT_DRV_INST(inst)),
static const struct device *motor_devices[] = {DT_INST_FOREACH_STATUS_OKAY(RS_MOTOR_POINTER)};

K_THREAD_STACK_DEFINE(rs_work_queue_stack, CAN_SEND_STACK_SIZE);

K_MSGQ_DEFINE(rs_thread_proc_msgq, sizeof(bool), MOTOR_COUNT * 2, 4);

K_WORK_DELAYABLE_DEFINE(rs_rx_monitor_work, rs_rx_monitor_handler);

#define RS_MOTOR_DATA_INST(inst)                                                                   \
	static struct rs_motor_data rs_motor_data_##inst = {                                       \
		.common = MOTOR_DT_DRIVER_DATA_INST_GET(inst),                                     \
		.online = false,                                                                   \
		.err = 0,                                                                          \
		.delta_deg_sum = 0,                                                                \
		.target_pos = 0,                                                                   \
		.target_radps = 0,                                                                 \
		.target_torque = 0,                                                                \
		.params = {0, 0},                                                                  \
	};

#define RS_MOTOR_CONFIG_INST(inst)                                                                 \
	static const struct rs_motor_cfg rs_motor_cfg_##inst = {                                   \
		.common = MOTOR_DT_DRIVER_CONFIG_INST_GET(inst),                                   \
		.motor_type = DT_STRING_UNQUOTED_OR(DT_DRV_INST(inst), motor_type, RS02),          \
		.p_max = DT_STRING_UNQUOTED_OR(DT_DRV_INST(inst), p_max, 12.57f),                  \
		.v_max = DT_STRING_UNQUOTED_OR(DT_DRV_INST(inst), v_max, 44.0f),                   \
		.t_max = DT_STRING_UNQUOTED_OR(DT_DRV_INST(inst), t_max, 17.0f),                   \
	};

#define MOTOR_DEVICE_DT_DEFINE(node_id, init_fn, pm, data, config, level, prio, api, ...)          \
	DEVICE_DT_DEFINE(node_id, init_fn, pm, data, config, level, prio, api, __VA_ARGS__)

#define MOTOR_DEVICE_DT_INST_DEFINE(inst, ...)                                                     \
	MOTOR_DEVICE_DT_DEFINE(DT_DRV_INST(inst), __VA_ARGS__)

#define RS_MOTOR_DEFINE_INST(inst)                                                                 \
	MOTOR_DEVICE_DT_INST_DEFINE(inst, rs_init, NULL, &rs_motor_data_##inst,                    \
				    &rs_motor_cfg_##inst, POST_KERNEL, CONFIG_MOTOR_INIT_PRIORITY, \
				    &rs_motor_api);

#define RS_MOTOR_INST(inst)                                                                        \
	MOTOR_DT_DRIVER_PID_DEFINE(DT_DRV_INST(inst))                                              \
	RS_MOTOR_CONFIG_INST(inst)                                                                 \
	RS_MOTOR_DATA_INST(inst)                                                                   \
	RS_MOTOR_DEFINE_INST(inst)

#endif // MOTOR_RS_H
