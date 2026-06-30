#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/motor.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

LOG_MODULE_REGISTER(motor_common, CONFIG_MOTOR_LOG_LEVEL);

#include "common.h"
#include "motor_can_sched.h"

static struct device *can_devices[CONFIG_CAN_COUNT];
static atomic_t runtime_stats[MOTOR_STAT_COUNT];

static bool initialized = false;
int can_work_init(void)
{
	motor_can_sched_init();
	initialized = true;
	return 0;
}

int8_t reg_can_dev(const struct device *dev)
{
	if (!initialized) {
		can_work_init();
		initialized = true;
	}
	int8_t can_id = get_can_id(dev);
	if (can_id != -1) {
		return can_id;
	}
	for (int i = 0; i < CONFIG_CAN_COUNT; i++) {
		if (can_devices[i] == NULL) {
			can_devices[i] = (struct device *)dev;
			motor_can_sched_register_can(dev);
			return i;
		}
	}
	return -1;
}

int8_t get_can_id(const struct device *dev)
{
	for (int i = 0; i < CONFIG_CAN_COUNT; i++) {
		if (can_devices[i] == dev) {
			return i;
		}
	}
	return -1;
}

int can_send_queued(const struct device *can_dev, struct can_frame *frame)
{
	if (!initialized) {
		can_work_init();
	}

	if (get_can_id(can_dev) < 0) {
		reg_can_dev(can_dev);
	}

	return motor_can_sched_send_prio(can_dev, frame, false, "legacy-motor");
}

void motor_stats_inc(enum motor_runtime_stat stat)
{
	if ((unsigned int)stat < MOTOR_STAT_COUNT) {
		atomic_inc(&runtime_stats[stat]);
	}
}

void motor_stats_get(struct motor_runtime_stats *stats)
{
	if (stats == NULL) {
		return;
	}

	for (size_t i = 0; i < MOTOR_STAT_COUNT; i++) {
		stats->counters[i] = (uint32_t)atomic_get(&runtime_stats[i]);
	}
}

SYS_INIT(can_work_init, APPLICATION, CONFIG_MOTOR_INIT_PRIORITY);
