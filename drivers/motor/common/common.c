#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/motor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(motor_common, CONFIG_MOTOR_LOG_LEVEL);

#include "common.h"

static struct k_sem tx_queue_sem[CONFIG_CAN_COUNT];
static struct device *can_devices[CONFIG_CAN_COUNT];

K_MSGQ_DEFINE(can_tx_msgq, sizeof(struct tx_frame), 4 * CONFIG_CAN_COUNT, 4);

static void can_tx_entry(void *arg1, void *arg2, void *arg3);
K_THREAD_DEFINE(can_tx_thread, 1024, can_tx_entry, NULL, NULL, NULL, 0, 0, 0);

static bool initialized = false;
int can_work_init(void)
{
	k_thread_start(can_tx_thread);
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
			k_sem_init(&tx_queue_sem[i], 3, 3);
			can_start(can_devices[i]);
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

static void can_tx_callback(const struct device *can_dev, int error, void *user_data)
{
	struct k_sem *queue_sem = user_data;
	k_sem_give(queue_sem);
}

int can_send_queued(const struct device *can_dev, struct can_frame *frame)
{
	if (!initialized) {
		return -ENOSYS;
	}

	int err = k_sem_take(&tx_queue_sem[get_can_id(can_dev)], K_NO_WAIT);
	if (err == 0) {
		err = can_send(can_dev, frame, K_NO_WAIT, can_tx_callback,
			       &tx_queue_sem[get_can_id(can_dev)]);
		// LOG_ERR("Send CAN frame: %d", err);
		if (err) {
			// LOG_ERR("TX queue full, will be put into msgq: %d", err);
		}
	} else if (err < 0) {
		// LOG_ERR("CAN hardware TX queue is full. (err %d)", err);
		struct tx_frame q_frame = {
			.can_dev = can_dev,
			.sem = &tx_queue_sem[get_can_id(can_dev)],
			.frame = *frame,
		};
		err = k_msgq_put(&can_tx_msgq, &q_frame, K_NO_WAIT);
	}
	// if (err) {
	// 	LOG_ERR("Failed to send CAN frame: %d", err);
	// }
	return err;
}

void can_tx_entry(void *arg1, void *arg2, void *arg3)
{
	struct tx_frame frame;
	int err = 0;
	uint32_t last_time = 0;
	uint16_t failed_times = 0;
	while (1) {
		k_msgq_get(&can_tx_msgq, &frame, K_FOREVER);
		err = k_sem_take(frame.sem, K_MSEC(2));
		if (err == 0) {
			err = can_send(frame.can_dev, &(frame.frame), K_USEC(100), can_tx_callback,
				       frame.sem);
			if (err && k_uptime_get() - last_time > 400) {
				// LOG_ERR("Failed to send CAN frame: %d", err);
				last_time = k_uptime_get();
			}
		} else {
			if (failed_times > 127) {
				k_msgq_purge(&can_tx_msgq);
				// LOG_ERR("Failed too many times, purge msgq");
				k_sem_give(frame.sem);
				failed_times = 0;
				continue;
			}
			k_sleep(K_USEC(50));
			failed_times++;
		}
	}
}

SYS_INIT(can_work_init, APPLICATION, CONFIG_MOTOR_INIT_PRIORITY);
