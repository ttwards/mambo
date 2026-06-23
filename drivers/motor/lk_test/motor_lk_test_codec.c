#include "motor_lk_test_internal.h"

#include <errno.h>
#include <string.h>

#include <zephyr/sys/util.h>

uint32_t lk_test_codec_tx_id(const struct lk_test_motor_cfg *cfg)
{
	return LK_TEST_CMD_ID_BASE + cfg->id;
}

uint32_t lk_test_codec_rx_id(const struct lk_test_motor_cfg *cfg)
{
	return LK_TEST_REPLY_ID_BASE + cfg->id;
}

static int16_t torque_to_raw(float torque)
{
	int32_t raw = (int32_t)((torque / 12.0f) * 2048.0f);

	return (int16_t)CLAMP(raw, -2048, 2047);
}

static void frame_base(const struct lk_test_motor_cfg *cfg, struct can_frame *frame)
{
	memset(frame, 0, sizeof(*frame));
	frame->id = lk_test_codec_tx_id(cfg);
	frame->dlc = 8;
	frame->flags = 0;
}

int lk_test_codec_pack_control(const struct lk_test_motor_cfg *cfg,
			       const struct lk_test_target_state *target, struct can_frame *frame)
{
	int16_t val_i16;
	int32_t val_i32;

	frame_base(cfg, frame);

	switch (target->mode) {
	case ML_TORQUE:
		frame->data[0] = LK_TEST_CMD_TORQUE_LOOP;
		val_i16 = torque_to_raw(target->torque);
		frame->data[4] = val_i16 & 0xFF;
		frame->data[5] = (val_i16 >> 8) & 0xFF;
		break;
	case ML_SPEED:
		frame->data[0] = LK_TEST_CMD_SPEED_LOOP;
		val_i16 = torque_to_raw(target->limit_torque);
		frame->data[2] = val_i16 & 0xFF;
		frame->data[3] = (val_i16 >> 8) & 0xFF;
		val_i32 = (int32_t)(target->speed_dps * LK_TEST_SPEED_FACTOR);
		frame->data[4] = val_i32 & 0xFF;
		frame->data[5] = (val_i32 >> 8) & 0xFF;
		frame->data[6] = (val_i32 >> 16) & 0xFF;
		frame->data[7] = (val_i32 >> 24) & 0xFF;
		break;
	case ML_ANGLE:
		frame->data[0] = LK_TEST_CMD_POS_LOOP_MULTI;
		val_i16 = (int16_t)CLAMP((int32_t)target->limit_speed_dps, 0, 32767);
		frame->data[2] = val_i16 & 0xFF;
		frame->data[3] = (val_i16 >> 8) & 0xFF;
		val_i32 = (int32_t)(target->angle_deg * LK_TEST_POS_FACTOR);
		frame->data[4] = val_i32 & 0xFF;
		frame->data[5] = (val_i32 >> 8) & 0xFF;
		frame->data[6] = (val_i32 >> 16) & 0xFF;
		frame->data[7] = (val_i32 >> 24) & 0xFF;
		break;
	default:
		frame->data[0] = LK_TEST_CMD_READ_STAT;
		return -ENOTSUP;
	}

	return 0;
}

bool lk_test_codec_is_feedback_frame(const struct can_frame *frame)
{
	if ((frame == NULL) || (frame->dlc < 8U)) {
		return false;
	}

	switch (frame->data[0]) {
	case LK_TEST_CMD_READ_STAT:
	case LK_TEST_CMD_TORQUE_LOOP:
	case LK_TEST_CMD_SPEED_LOOP:
	case LK_TEST_CMD_POS_LOOP_MULTI:
		return true;
	default:
		return false;
	}
}

bool lk_test_codec_is_status1_frame(const struct can_frame *frame)
{
	if ((frame == NULL) || (frame->dlc < 8U)) {
		return false;
	}

	switch (frame->data[0]) {
	case LK_TEST_CMD_READ_STAT1:
	case LK_TEST_CMD_CLEAR_ERR:
		return true;
	default:
		return false;
	}
}

bool lk_test_codec_is_echo_frame(const struct can_frame *frame)
{
	if ((frame == NULL) || (frame->dlc < 1U)) {
		return false;
	}

	switch (frame->data[0]) {
	case LK_TEST_CMD_MOTOR_OFF:
	case LK_TEST_CMD_MOTOR_RUN:
	case LK_TEST_CMD_SET_ZERO:
	case LK_TEST_SET_PARAM:
		return true;
	default:
		return false;
	}
}

bool lk_test_codec_reply_matches(const struct can_frame *tx_frame, const struct can_frame *rx_frame,
				 const void *user_data)
{
	ARG_UNUSED(user_data);

	if ((tx_frame == NULL) || (rx_frame == NULL) || (rx_frame->dlc < 1U)) {
		return false;
	}

	if (rx_frame->data[0] != tx_frame->data[0]) {
		return false;
	}

	if ((tx_frame->data[0] == LK_TEST_SET_PARAM) &&
	    ((rx_frame->dlc < 2U) || (rx_frame->data[1] != tx_frame->data[1]))) {
		return false;
	}

	return true;
}

int lk_test_codec_pack_command(const struct lk_test_motor_cfg *cfg, enum motor_cmd cmd,
			       struct can_frame *frame)
{
	frame_base(cfg, frame);

	switch (cmd) {
	case ENABLE_MOTOR:
		frame->data[0] = LK_TEST_CMD_MOTOR_RUN;
		break;
	case DISABLE_MOTOR:
		frame->data[0] = LK_TEST_CMD_MOTOR_OFF;
		break;
	case SET_ZERO:
		/* 0x95 carries the zero position in 0.01 degree units. */
		frame->data[0] = LK_TEST_CMD_SET_ZERO;
		frame->data[4] = 0x00;
		frame->data[5] = 0x00;
		frame->data[6] = 0x00;
		frame->data[7] = 0x00;
		break;
	case CLEAR_ERROR:
		frame->data[0] = LK_TEST_CMD_CLEAR_ERR;
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

int lk_test_codec_pack_read_status(const struct lk_test_motor_cfg *cfg,
				   struct can_frame *frame)
{
	frame_base(cfg, frame);
	frame->data[0] = LK_TEST_CMD_READ_STAT;
	return 0;
}

int lk_test_codec_pack_pid(const struct lk_test_motor_cfg *cfg, enum lk_test_pid_slot slot,
			   const struct pid_config *params, struct can_frame *frame)
{
	static const uint8_t pid_cmds[LK_TEST_PID_COUNT] = {
		[LK_TEST_PID_ANGLE] = LK_TEST_PID_ANGLE_UPDATE,
		[LK_TEST_PID_SPEED] = LK_TEST_PID_SPEED_UPDATE,
		[LK_TEST_PID_TORQUE] = LK_TEST_PID_TORQUE_UPDATE,
	};
	int16_t kp = (int16_t)params->k_p;
	int16_t ki = (int16_t)params->k_i;
	int16_t kd = (int16_t)params->k_d;

	if ((unsigned int)slot >= LK_TEST_PID_COUNT) {
		return -EINVAL;
	}

	frame_base(cfg, frame);
	frame->data[0] = LK_TEST_SET_PARAM;
	frame->data[1] = pid_cmds[slot];
	frame->data[2] = kp & 0xFF;
	frame->data[3] = (kp >> 8) & 0xFF;
	frame->data[4] = ki & 0xFF;
	frame->data[5] = (ki >> 8) & 0xFF;
	frame->data[6] = kd & 0xFF;
	frame->data[7] = (kd >> 8) & 0xFF;
	return 0;
}

bool lk_test_need_reply_wait(enum motor_cmd cmd)
{
	/*
	 * 统一异步策略：所有 control 命令立即返回
	 * - 电机层会自动重试失败命令
	 * - 状态更新通过 RX 帧异步通知
	 * - 应用层无需关心发送细节
	 */
	return false;  /* 所有命令都立即返回 */
}

void lk_test_codec_parse_feedback(const struct can_frame *frame,
				  struct lk_test_feedback_state *feedback)
{
	feedback->raw_temp = (int8_t)frame->data[1];
	feedback->raw_torque = (int16_t)(frame->data[2] | (frame->data[3] << 8));
	feedback->raw_speed = (int16_t)(frame->data[4] | (frame->data[5] << 8));
	feedback->raw_encoder = (uint16_t)(frame->data[6] | (frame->data[7] << 8));

	feedback->temperature = (float)feedback->raw_temp;
	feedback->torque = (float)feedback->raw_torque * (16.0f / 4096.0f);
	feedback->rpm = ((float)feedback->raw_speed) / 6.0f;
	feedback->angle_deg = (float)feedback->raw_encoder * (360.0f / 65535.0f);
}
