#include <ares/board/init.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "motor_can_sched.h"

LOG_MODULE_REGISTER(dm_can_sched_demo, LOG_LEVEL_INF);

#define DMOTOR_NODE0 DT_NODELABEL(dm_motor0)
#define DMOTOR_NODE1 DT_NODELABEL(dm_motor1)
#define DMOTOR_NODE2 DT_NODELABEL(dm_motor2)
#define DMOTOR_NODE3 DT_NODELABEL(dm_motor3)

#define DM_UINT_MAX(bits) ((1U << (bits)) - 1U)
#define DM_STATUS_PERIOD_HZ 250U
#define DM_PROBE_PERIOD_MS  1000U
#define DM_SWEEP_STEP_DEG   1.5f

struct dm_demo_motor {
	const char *name;
	const char *probe_tag;
	const struct device *can_dev;
	uint32_t tx_id;
	uint32_t rx_id;
	float p_max;
	float v_max;
	float t_max;
	float target_deg;
	float direction;
	float angle_deg;
	float rpm;
	float torque;
	struct can_filter filter;
	motor_can_sched_handle_t periodic;
};

static struct dm_demo_motor motors[] = {
	{
		.name = "dm0",
		.probe_tag = "dm0-probe",
		.can_dev = DEVICE_DT_GET(DT_PHANDLE(DMOTOR_NODE0, can_channel)),
		.tx_id = DT_PROP(DMOTOR_NODE0, tx_id),
		.rx_id = DT_PROP(DMOTOR_NODE0, rx_id),
		.p_max = (float)DT_STRING_UNQUOTED_OR(DMOTOR_NODE0, p_max, 20.0),
		.v_max = (float)DT_STRING_UNQUOTED_OR(DMOTOR_NODE0, v_max, 785.0),
		.t_max = (float)DT_STRING_UNQUOTED_OR(DMOTOR_NODE0, t_max, 5.0),
		.direction = 1.0f,
	},
	{
		.name = "dm1",
		.probe_tag = "dm1-probe",
		.can_dev = DEVICE_DT_GET(DT_PHANDLE(DMOTOR_NODE1, can_channel)),
		.tx_id = DT_PROP(DMOTOR_NODE1, tx_id),
		.rx_id = DT_PROP(DMOTOR_NODE1, rx_id),
		.p_max = (float)DT_STRING_UNQUOTED_OR(DMOTOR_NODE1, p_max, 20.0),
		.v_max = (float)DT_STRING_UNQUOTED_OR(DMOTOR_NODE1, v_max, 785.0),
		.t_max = (float)DT_STRING_UNQUOTED_OR(DMOTOR_NODE1, t_max, 5.0),
		.direction = -1.0f,
	},
	{
		.name = "dm2",
		.probe_tag = "dm2-probe",
		.can_dev = DEVICE_DT_GET(DT_PHANDLE(DMOTOR_NODE2, can_channel)),
		.tx_id = DT_PROP(DMOTOR_NODE2, tx_id),
		.rx_id = DT_PROP(DMOTOR_NODE2, rx_id),
		.p_max = (float)DT_STRING_UNQUOTED_OR(DMOTOR_NODE2, p_max, 20.0),
		.v_max = (float)DT_STRING_UNQUOTED_OR(DMOTOR_NODE2, v_max, 785.0),
		.t_max = (float)DT_STRING_UNQUOTED_OR(DMOTOR_NODE2, t_max, 5.0),
		.direction = 1.0f,
	},
	{
		.name = "dm3",
		.probe_tag = "dm3-probe",
		.can_dev = DEVICE_DT_GET(DT_PHANDLE(DMOTOR_NODE3, can_channel)),
		.tx_id = DT_PROP(DMOTOR_NODE3, tx_id),
		.rx_id = DT_PROP(DMOTOR_NODE3, rx_id),
		.p_max = (float)DT_STRING_UNQUOTED_OR(DMOTOR_NODE3, p_max, 20.0),
		.v_max = (float)DT_STRING_UNQUOTED_OR(DMOTOR_NODE3, v_max, 785.0),
		.t_max = (float)DT_STRING_UNQUOTED_OR(DMOTOR_NODE3, t_max, 5.0),
		.direction = -1.0f,
	},
};

static inline uint32_t dm_float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
	const float span = x_max - x_min;
	const float offset = x_min;

	if (x > x_max) {
		x = x_max;
	} else if (x < x_min) {
		x = x_min;
	}

	return (uint32_t)((x - offset) * (float)DM_UINT_MAX(bits) / span);
}

static inline float dm_uint_to_float(uint32_t x, float x_min, float x_max, uint8_t bits)
{
	const float span = x_max - x_min;
	const float offset = x_min;

	return ((float)x) * span / (float)DM_UINT_MAX(bits) + offset;
}

static void dm_pack_mit_frame(const struct dm_demo_motor *motor, struct can_frame *frame,
			      float pos_deg, float vel_radps, float kp, float kd, float torque)
{
	uint16_t pos_raw = (uint16_t)dm_float_to_uint(pos_deg, -motor->p_max, motor->p_max, 16);
	uint16_t vel_raw = (uint16_t)dm_float_to_uint(vel_radps, -motor->v_max, motor->v_max, 12);
	uint16_t kp_raw = (uint16_t)dm_float_to_uint(kp, 0.0f, 500.0f, 12);
	uint16_t kd_raw = (uint16_t)dm_float_to_uint(kd, 0.0f, 5.0f, 12);
	uint16_t tor_raw = (uint16_t)dm_float_to_uint(torque, -motor->t_max, motor->t_max, 12);

	memset(frame, 0, sizeof(*frame));
	frame->id = motor->tx_id;
	frame->dlc = 8;
	frame->data[0] = (uint8_t)(pos_raw >> 8);
	frame->data[1] = (uint8_t)pos_raw;
	frame->data[2] = (uint8_t)(vel_raw >> 4);
	frame->data[3] = (uint8_t)(((vel_raw & 0x0F) << 4) | (kp_raw >> 8));
	frame->data[4] = (uint8_t)kp_raw;
	frame->data[5] = (uint8_t)(kd_raw >> 4);
	frame->data[6] = (uint8_t)(((kd_raw & 0x0F) << 4) | (tor_raw >> 8));
	frame->data[7] = (uint8_t)tor_raw;
}

static void dm_pack_cmd_frame(const struct dm_demo_motor *motor, struct can_frame *frame,
			      const uint8_t payload[8])
{
	memset(frame, 0, sizeof(*frame));
	frame->id = motor->tx_id;
	frame->dlc = 8;
	memcpy(frame->data, payload, 8);
}

static void dm_pack_mode_frame(const struct dm_demo_motor *motor, struct can_frame *frame)
{
	memset(frame, 0, sizeof(*frame));
	frame->id = 0x7FFU;
	frame->dlc = 8;
	frame->data[0] = motor->tx_id & 0xFF;
	frame->data[1] = motor->tx_id >> 8;
	frame->data[2] = 0x55;
	frame->data[3] = 0x0A;
	frame->data[4] = 0x01;
}

static void dm_rx_callback(const struct device *can_dev, struct can_frame *frame, void *user_data)
{
	struct dm_demo_motor *motor = user_data;
	uint32_t raw_angle = ((uint32_t)frame->data[1] << 8) | frame->data[2];
	uint32_t raw_rpm = ((uint32_t)frame->data[3] << 4) | (frame->data[4] >> 4);
	uint32_t raw_torque = (((uint32_t)frame->data[4] & 0x0F) << 8) | frame->data[5];

	motor_can_sched_report_rx(can_dev, frame);

	motor->angle_deg = dm_uint_to_float(raw_angle, -motor->p_max, motor->p_max, 16);
	motor->rpm = dm_uint_to_float(raw_rpm, -motor->v_max, motor->v_max, 12);
	motor->torque = dm_uint_to_float(raw_torque, -motor->t_max, motor->t_max, 12);
	LOG_DBG("%s rx state angle=%.1f rpm=%.1f torque=%.2f raw_id=0x%03x",
		motor->name, (double)motor->angle_deg, (double)motor->rpm, (double)motor->torque,
		frame->id);
}

static int dm_install_rx_filters(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(motors); i++) {
		int ret;

		ret = motor_can_sched_register_can(motors[i].can_dev);
		if (ret != 0) {
			return ret;
		}

		motors[i].filter.id = motors[i].rx_id;
		motors[i].filter.mask = CAN_STD_ID_MASK;
		motors[i].filter.flags = 0;
		ret = can_add_rx_filter(motors[i].can_dev, dm_rx_callback, &motors[i], &motors[i].filter);
		if (ret < 0) {
			return ret;
		}
		LOG_INF("%s rx filter added on %s rx=0x%03x", motors[i].name, motors[i].can_dev->name,
			motors[i].rx_id);
	}

	return 0;
}

static int dm_setup_scheduler(void)
{
	static const uint8_t enable_frame[8] = {0xFF, 0xFF, 0xFF, 0xFF,
						0xFF, 0xFF, 0xFF, 0xFC};
	struct motor_can_sched_tx_param mode_param = {
		.high_priority = true,
		.trace_lifecycle = true,
		.tag = "dm-mode",
	};
	struct motor_can_sched_tx_param enable_param = {
		.track_reply = true,
		.high_priority = true,
		.immediate_reply = true,
		.trace_lifecycle = true,
		.ack_timeout_ms = 6U,
		.max_retries = 2U,
		.reply_mask = CAN_STD_ID_MASK,
		.tag = "dm-enable",
	};
	struct motor_can_sched_tx_param periodic_param = {
		.loop = true,
		.track_reply = true,
		.immediate_reply = true,
		.loop_hz = DM_STATUS_PERIOD_HZ,
		.ack_timeout_ms = 4U,
		.max_retries = 2U,
		.reply_mask = CAN_STD_ID_MASK,
	};
	int ret;

	ret = motor_can_sched_init();
	if (ret != 0) {
		return ret;
	}

	for (size_t i = 0; i < ARRAY_SIZE(motors); i++) {
		struct can_frame frame;

		ret = motor_can_sched_register_can(motors[i].can_dev);
		if (ret != 0) {
			return ret;
		}

		dm_pack_mode_frame(&motors[i], &frame);
		ret = motor_can_sched_send(motors[i].can_dev, &frame, &mode_param, NULL);
		if (ret != 0) {
			return ret;
		}

		dm_pack_cmd_frame(&motors[i], &frame, enable_frame);
		enable_param.reply_id = motors[i].rx_id;
		ret = motor_can_sched_send(motors[i].can_dev, &frame, &enable_param, NULL);
		if (ret != 0) {
			return ret;
		}

		dm_pack_mit_frame(&motors[i], &frame, 0.0f, 0.0f, 10.0f, 1.0f, 0.0f);
		periodic_param.reply_id = motors[i].rx_id;
		periodic_param.tag = motors[i].name;
		ret = motor_can_sched_send(motors[i].can_dev, &frame, &periodic_param,
					   &motors[i].periodic);
		if (ret != 0) {
			return ret;
		}

		LOG_INF("%s attached to %s tx=0x%03x rx=0x%03x periodic=%uHz",
			motors[i].name, motors[i].can_dev->name, motors[i].tx_id, motors[i].rx_id,
			DM_STATUS_PERIOD_HZ);
	}

	return 0;
}

static void dm_send_reply_probe(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(motors); i++) {
		struct can_frame frame;
		struct motor_can_sched_tx_param probe_param = {
			.track_reply = true,
			.high_priority = true,
			.immediate_reply = true,
			.trace_lifecycle = true,
			.ack_timeout_ms = 6U,
			.max_retries = 2U,
			.reply_mask = CAN_STD_ID_MASK,
		};
		int ret;

		dm_pack_mit_frame(&motors[i], &frame, motors[i].target_deg, 0.0f, 10.0f, 1.0f, 0.0f);
		probe_param.reply_id = motors[i].rx_id;
		probe_param.tag = motors[i].probe_tag;
		ret = motor_can_sched_send(motors[i].can_dev, &frame, &probe_param, NULL);
		if (ret != 0) {
			LOG_ERR("%s probe submit failed: %d", motors[i].name, ret);
		} else {
			LOG_INF("%s probe submitted target=%.1f expect_reply=0x%03x", motors[i].name,
				(double)motors[i].target_deg, motors[i].rx_id);
		}
	}
}

static void dm_update_targets(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(motors); i++) {
		struct can_frame frame;

		motors[i].target_deg += motors[i].direction * DM_SWEEP_STEP_DEG;
		if (motors[i].target_deg > 35.0f) {
			motors[i].target_deg = 35.0f;
			motors[i].direction = -1.0f;
		} else if (motors[i].target_deg < -35.0f) {
			motors[i].target_deg = -35.0f;
			motors[i].direction = 1.0f;
		}

		dm_pack_mit_frame(&motors[i], &frame, motors[i].target_deg, 0.0f, 10.0f, 1.0f, 0.0f);
		motor_can_sched_update(motors[i].periodic, &frame);
	}
}

int main(void)
{
	int ret;
	uint32_t log_count = 0U;
	uint32_t probe_count = 0U;

	ret = dm_install_rx_filters();
	if (ret != 0) {
		LOG_ERR("install dm rx filters failed: %d", ret);
		return ret;
	}

	ret = dm_setup_scheduler();
	if (ret != 0) {
		LOG_ERR("setup scheduler failed: %d", ret);
		return ret;
	}

	while (1) {
		k_msleep(20);
		dm_update_targets();

		if ((probe_count++ % (DM_PROBE_PERIOD_MS / 20U)) == 0U) {
			dm_send_reply_probe();
		}

		if ((log_count++ % 25U) == 0U) {
			for (size_t i = 0; i < ARRAY_SIZE(motors); i++) {
				LOG_INF("%s target=%.1f angle=%.1f rpm=%.1f torque=%.2f",
					motors[i].name, (double)motors[i].target_deg,
					(double)motors[i].angle_deg, (double)motors[i].rpm,
					(double)motors[i].torque);
			}
		}
	}

	return 0;
}
