#ifndef MOTOR_CAN_SCHED_H_
#define MOTOR_CAN_SCHED_H_

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MOTOR_CAN_SCHED_MAX_BUS
#define MOTOR_CAN_SCHED_MAX_BUS 4
#endif

#ifndef MOTOR_CAN_SCHED_MAX_PERIODIC
#define MOTOR_CAN_SCHED_MAX_PERIODIC 24
#endif

#ifndef MOTOR_CAN_SCHED_MAX_PENDING
#define MOTOR_CAN_SCHED_MAX_PENDING 16
#endif

#ifndef MOTOR_CAN_SCHED_QUEUE_DEPTH
#define MOTOR_CAN_SCHED_QUEUE_DEPTH 24
#endif

enum motor_can_sched_prio {
	MOTOR_CAN_SCHED_PRIO_CRITICAL = 0,
	MOTOR_CAN_SCHED_PRIO_HIGH,
	MOTOR_CAN_SCHED_PRIO_NORMAL,
	MOTOR_CAN_SCHED_PRIO_LOW,
	MOTOR_CAN_SCHED_PRIO_COUNT,
};

struct motor_can_sched_meta {
	enum motor_can_sched_prio priority;
	bool expect_reply;
	bool has_periodic_reply;
	bool trace_lifecycle;
	uint16_t reply_reserve_us;
	uint16_t ack_timeout_ms;
	uint8_t max_retries;
	uint32_t reply_id;
	uint32_t reply_mask;
	const char *tag;
};

struct motor_can_sched_periodic {
	const struct device *can_dev;
	struct can_frame frame;
	struct motor_can_sched_meta meta;
	uint16_t rate_hz;
	bool enabled;

	uint16_t phase_tick;
	uint16_t period_ticks;
	uint32_t next_release_tick;
};

struct motor_can_sched_stats {
	uint32_t tx_frames;
	uint32_t rx_frames;
	uint32_t ack_matches;
	uint32_t dropped_frames;
	uint32_t retry_frames;
	uint32_t ack_timeouts;
	uint32_t queue_peak[MOTOR_CAN_SCHED_PRIO_COUNT];
	uint32_t window_tx_busy_us;
	uint32_t window_rx_busy_us;
	uint32_t window_reserved_us;
};

struct motor_can_sched_handle;
typedef struct motor_can_sched_handle *motor_can_sched_handle_t;

struct motor_can_sched_tx_param {
	bool loop;
	bool track_reply;
	bool high_priority;
	bool immediate_reply;
	bool trace_lifecycle;
	uint16_t loop_hz;
	uint16_t ack_timeout_ms;
	uint8_t max_retries;
	uint32_t reply_id;
	uint32_t reply_mask;
	const char *tag;
};

int motor_can_sched_init(void);
int motor_can_sched_register_can(const struct device *can_dev);

int motor_can_sched_send(const struct device *can_dev, const struct can_frame *frame,
			 const struct motor_can_sched_tx_param *param,
			 motor_can_sched_handle_t *handle_out);
int motor_can_sched_update(motor_can_sched_handle_t handle, const struct can_frame *frame);
int motor_can_sched_remove(motor_can_sched_handle_t handle);
void motor_can_sched_report_rx(const struct device *can_dev, const struct can_frame *frame);
int motor_can_sched_get_stats(const struct device *can_dev, struct motor_can_sched_stats *stats);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CAN_SCHED_H_ */
