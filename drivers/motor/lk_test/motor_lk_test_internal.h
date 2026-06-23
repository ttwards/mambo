#ifndef MOTOR_LK_TEST_INTERNAL_H_
#define MOTOR_LK_TEST_INTERNAL_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/motor.h>
#include <zephyr/drivers/pid.h>

#include "../common/motor_can_sched.h"

#define LK_TEST_CMD_ID_BASE   0x140U
#define LK_TEST_REPLY_ID_BASE 0x140U

#define LK_TEST_CMD_READ_STAT1     0x9A
#define LK_TEST_CMD_READ_STAT      0x9C
#define LK_TEST_CMD_CLEAR_ERR      0x9B
#define LK_TEST_CMD_MOTOR_OFF      0x80
#define LK_TEST_CMD_MOTOR_RUN      0x88
#define LK_TEST_CMD_SET_ZERO       0x95
#define LK_TEST_CMD_TORQUE_LOOP    0xA1
#define LK_TEST_CMD_SPEED_LOOP     0xA2
#define LK_TEST_CMD_POS_LOOP_MULTI 0xA4
#define LK_TEST_SET_PARAM          0xC1
#define LK_TEST_PID_ANGLE_UPDATE   0x0A
#define LK_TEST_PID_SPEED_UPDATE   0x0B
#define LK_TEST_PID_TORQUE_UPDATE  0x0C

#define LK_TEST_POS_FACTOR       100.0f
#define LK_TEST_SPEED_FACTOR     100.0f
#define LK_TEST_DEFAULT_RATE_HZ  200U
#define LK_TEST_REPLY_TIMEOUT_MS 2U
#define LK_TEST_ONLINE_TIMEOUT_MS    80U
#define LK_TEST_WATCHDOG_PERIOD_MS   20U
#define LK_TEST_RECOVERY_ENABLE_MS   50U

enum lk_test_pid_slot {
	LK_TEST_PID_ANGLE = 0,
	LK_TEST_PID_SPEED,
	LK_TEST_PID_TORQUE,
	LK_TEST_PID_COUNT,
};

struct lk_test_target_state {
	enum motor_mode mode;
	float angle_deg;
	float speed_dps;
	float torque;
	float limit_speed_dps;
	float limit_torque;
};

struct lk_test_feedback_state {
	float angle_deg;
	float rpm;
	float torque;
	float temperature;
	uint32_t last_rx_ms;
	int16_t raw_torque;
	int16_t raw_speed;
	uint16_t raw_encoder;
	int8_t raw_temp;
	int8_t err;
};

struct lk_test_comm_state {
	const struct device *dev;
	bool online;
	bool enabled;
	bool target_valid;
	bool feedback_seen;
	bool reconnect_pending;
	uint8_t missed_count;
	uint32_t last_recovery_tx_ms;
	motor_can_sched_handle_t periodic;
	struct can_filter filter;
	struct k_work_delayable watchdog_work;
};

struct lk_test_motor_data {
	struct motor_driver_data common;
	struct lk_test_target_state target;
	struct lk_test_feedback_state feedback;
	struct lk_test_comm_state comm;
	struct pid_config params[LK_TEST_PID_COUNT];
	bool pid_dirty[LK_TEST_PID_COUNT];
};

struct lk_test_motor_cfg {
	struct motor_driver_config common;
	uint8_t id;
	uint16_t control_rate_hz;
	bool trace_lifecycle;
};

/* Codec layer — called from motor_lk_test.c */
uint32_t lk_test_codec_tx_id(const struct lk_test_motor_cfg *cfg);
uint32_t lk_test_codec_rx_id(const struct lk_test_motor_cfg *cfg);
int lk_test_codec_pack_control(const struct lk_test_motor_cfg *cfg,
			       const struct lk_test_target_state *target,
			       struct can_frame *frame);
int lk_test_codec_pack_command(const struct lk_test_motor_cfg *cfg,
			       enum motor_cmd cmd,
			       struct can_frame *frame);
int lk_test_codec_pack_read_status(const struct lk_test_motor_cfg *cfg,
				   struct can_frame *frame);
int lk_test_codec_pack_pid(const struct lk_test_motor_cfg *cfg,
			   enum lk_test_pid_slot slot,
			   const struct pid_config *params,
			   struct can_frame *frame);
bool lk_test_codec_is_feedback_frame(const struct can_frame *frame);
bool lk_test_codec_is_status1_frame(const struct can_frame *frame);
bool lk_test_codec_is_echo_frame(const struct can_frame *frame);
bool lk_test_codec_reply_matches(const struct can_frame *tx_frame,
				 const struct can_frame *rx_frame,
				 const void *user_data);
void lk_test_codec_parse_feedback(const struct can_frame *frame,
				  struct lk_test_feedback_state *feedback);

/* 异步发送策略：所有 control 命令都立即返回 */
bool lk_test_need_reply_wait(enum motor_cmd cmd);

#endif /* MOTOR_LK_TEST_INTERNAL_H_ */
