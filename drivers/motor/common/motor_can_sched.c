#include "motor_can_sched.h"

#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(motor_can_sched, LOG_LEVEL_INF);

#define MOTOR_CAN_SCHED_TICK_HZ       1000U
#define MOTOR_CAN_SCHED_TICK_US       1000U
#define MOTOR_CAN_SCHED_LOG_WINDOW_MS 2000U
#define MOTOR_CAN_SCHED_SUPERFRAME    1000U
#define MOTOR_CAN_SCHED_STACK_SIZE    2048
#define MOTOR_CAN_SCHED_THREAD_PRIO   0

struct motor_can_sched_entry {
	struct can_frame frame;
	struct motor_can_sched_meta meta;
	uint32_t trace_id;
	uint8_t retries;
};

struct motor_can_sched_ring {
	uint16_t head;
	uint16_t tail;
	uint16_t count;
	struct motor_can_sched_entry items[MOTOR_CAN_SCHED_QUEUE_DEPTH];
};

struct motor_can_sched_pending {
	bool used;
	struct motor_can_sched_entry entry;
	uint32_t reply_id;
	uint32_t reply_mask;
	uint32_t deadline_ms;
	uint32_t sent_at_ms;
};

struct motor_can_sched_bus {
	const struct device *can_dev;
	struct motor_can_sched_ring rings[MOTOR_CAN_SCHED_PRIO_COUNT];
	struct motor_can_sched_pending pending[MOTOR_CAN_SCHED_MAX_PENDING];
	struct motor_can_sched_stats stats;
	uint16_t phase_load[MOTOR_CAN_SCHED_SUPERFRAME];
	uint32_t last_log_ms;
	bool started;
};

static struct motor_can_sched_bus sched_buses[MOTOR_CAN_SCHED_MAX_BUS];
static struct motor_can_sched_periodic *periodic_jobs[MOTOR_CAN_SCHED_MAX_PERIODIC];
struct motor_can_sched_handle {
	bool in_use;
	struct motor_can_sched_periodic job;
};
static struct motor_can_sched_handle sched_handles[MOTOR_CAN_SCHED_MAX_PERIODIC];
static struct k_spinlock sched_lock;
static struct k_thread sched_thread;
static K_THREAD_STACK_DEFINE(sched_stack, MOTOR_CAN_SCHED_STACK_SIZE);
static bool sched_initialized;
static uint32_t sched_tick;
static uint32_t sched_trace_id;

static uint16_t frame_time_us(uint8_t flags)
{
	return (flags & CAN_FRAME_IDE) ? 150U : 130U;
}

static struct motor_can_sched_meta meta_from_param(const struct can_frame *frame,
						   const struct motor_can_sched_tx_param *param)
{
	struct motor_can_sched_meta meta = {
		.priority = MOTOR_CAN_SCHED_PRIO_LOW,
		.expect_reply = false,
		.has_periodic_reply = false,
		.trace_lifecycle = false,
		.reply_reserve_us = 0U,
		.ack_timeout_ms = 5U,
		.max_retries = 1U,
		.reply_id = 0U,
		.reply_mask = CAN_STD_ID_MASK,
		.tag = "frame",
	};

	if (param == NULL) {
		return meta;
	}

	meta.priority = param->high_priority ? MOTOR_CAN_SCHED_PRIO_HIGH : MOTOR_CAN_SCHED_PRIO_LOW;
	meta.expect_reply = param->track_reply;
	meta.trace_lifecycle = param->trace_lifecycle;
	meta.reply_reserve_us = param->immediate_reply ? frame_time_us(frame->flags) : 0U;
	meta.ack_timeout_ms = param->ack_timeout_ms != 0U ? param->ack_timeout_ms : 5U;
	meta.max_retries = param->max_retries != 0U ? param->max_retries : 1U;
	meta.reply_id = param->reply_id;
	meta.reply_mask = param->reply_mask != 0U ? param->reply_mask : CAN_STD_ID_MASK;
	meta.tag = param->tag != NULL ? param->tag : "frame";
	return meta;
}

static struct motor_can_sched_handle *alloc_handle_locked(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(sched_handles); i++) {
		if (!sched_handles[i].in_use) {
			memset(&sched_handles[i], 0, sizeof(sched_handles[i]));
			sched_handles[i].in_use = true;
			return &sched_handles[i];
		}
	}

	return NULL;
}

static void free_handle_locked(struct motor_can_sched_handle *handle)
{
	if (handle != NULL) {
		memset(handle, 0, sizeof(*handle));
	}
}

static bool time_reached(uint32_t now, uint32_t deadline)
{
	return (int32_t)(now - deadline) >= 0;
}

static uint16_t frame_cost_us(const struct can_frame *frame, const struct motor_can_sched_meta *meta)
{
	uint16_t cost = frame_time_us(frame->flags);

	if (meta->reply_reserve_us != 0U) {
		cost += meta->reply_reserve_us;
	} else if (meta->expect_reply) {
		cost += frame_time_us(frame->flags);
	}

	if (meta->has_periodic_reply) {
		cost += frame_time_us(frame->flags);
	}

	return cost;
}

static bool ring_push(struct motor_can_sched_ring *ring, const struct motor_can_sched_entry *entry)
{
	if (ring->count >= MOTOR_CAN_SCHED_QUEUE_DEPTH) {
		return false;
	}

	ring->items[ring->tail] = *entry;
	ring->tail = (ring->tail + 1U) % MOTOR_CAN_SCHED_QUEUE_DEPTH;
	ring->count++;
	return true;
}

static bool ring_pop(struct motor_can_sched_ring *ring, struct motor_can_sched_entry *entry)
{
	if (ring->count == 0U) {
		return false;
	}

	*entry = ring->items[ring->head];
	ring->head = (ring->head + 1U) % MOTOR_CAN_SCHED_QUEUE_DEPTH;
	ring->count--;
	return true;
}

static struct motor_can_sched_bus *find_bus(const struct device *can_dev)
{
	for (size_t i = 0; i < ARRAY_SIZE(sched_buses); i++) {
		if (sched_buses[i].can_dev == can_dev) {
			return &sched_buses[i];
		}
	}

	return NULL;
}

static struct motor_can_sched_bus *register_bus_locked(const struct device *can_dev)
{
	struct motor_can_sched_bus *bus = find_bus(can_dev);

	if (bus != NULL) {
		return bus;
	}

	for (size_t i = 0; i < ARRAY_SIZE(sched_buses); i++) {
		if (sched_buses[i].can_dev == NULL) {
			sched_buses[i].can_dev = can_dev;
			return &sched_buses[i];
		}
	}

	return NULL;
}

static int queue_entry_locked(const struct device *can_dev, const struct can_frame *frame,
			      const struct motor_can_sched_meta *meta, uint8_t retries)
{
	struct motor_can_sched_bus *bus = register_bus_locked(can_dev);
	struct motor_can_sched_entry entry;
	struct motor_can_sched_ring *ring;

	if (bus == NULL) {
		return -ENOMEM;
	}

	if ((unsigned int)meta->priority >= MOTOR_CAN_SCHED_PRIO_COUNT) {
		return -EINVAL;
	}

	entry.frame = *frame;
	entry.meta = *meta;
	entry.trace_id = ++sched_trace_id;
	entry.retries = retries;

	ring = &bus->rings[meta->priority];
	if (!ring_push(ring, &entry)) {
		bus->stats.dropped_frames++;
		return -ENOSPC;
	}

	bus->stats.queue_peak[meta->priority] = MAX(bus->stats.queue_peak[meta->priority], ring->count);
	return 0;
}

static int requeue_entry_locked(struct motor_can_sched_bus *bus,
				const struct motor_can_sched_entry *entry)
{
	struct motor_can_sched_ring *ring;

	if ((bus == NULL) || ((unsigned int)entry->meta.priority >= MOTOR_CAN_SCHED_PRIO_COUNT)) {
		return -EINVAL;
	}

	ring = &bus->rings[entry->meta.priority];
	if (!ring_push(ring, entry)) {
		bus->stats.dropped_frames++;
		return -ENOSPC;
	}

	bus->stats.queue_peak[entry->meta.priority] =
		MAX(bus->stats.queue_peak[entry->meta.priority], ring->count);

	return 0;
}

static int reserve_pending_locked(struct motor_can_sched_bus *bus,
				  const struct motor_can_sched_entry *entry,
				  struct motor_can_sched_pending **pending_out)
{
	if (!entry->meta.expect_reply) {
		*pending_out = NULL;
		return 0;
	}

	for (size_t i = 0; i < ARRAY_SIZE(bus->pending); i++) {
		if (!bus->pending[i].used) {
			bus->pending[i].used = true;
			bus->pending[i].entry = *entry;
			bus->pending[i].reply_id = entry->meta.reply_id;
			bus->pending[i].reply_mask = entry->meta.reply_mask;
			bus->pending[i].deadline_ms =
				k_uptime_get_32() + MAX(entry->meta.ack_timeout_ms, 1U);
			bus->pending[i].sent_at_ms = k_uptime_get_32();
			*pending_out = &bus->pending[i];
			return 0;
		}
	}

	return -ENOSPC;
}

static void clear_pending_locked(struct motor_can_sched_pending *pending)
{
	if (pending != NULL) {
		memset(pending, 0, sizeof(*pending));
	}
}

static void tx_done_callback(const struct device *dev, int error, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(error);
	ARG_UNUSED(user_data);
}

static int send_one(struct motor_can_sched_bus *bus, struct motor_can_sched_entry *entry)
{
	struct motor_can_sched_pending *pending = NULL;
	k_spinlock_key_t key = k_spin_lock(&sched_lock);
	int ret = reserve_pending_locked(bus, entry, &pending);

	if (ret != 0) {
		bus->stats.dropped_frames++;
		bus->stats.pending_full++;
		k_spin_unlock(&sched_lock, key);
		return ret;
	}
	k_spin_unlock(&sched_lock, key);

	ret = can_send(bus->can_dev, &entry->frame, K_NO_WAIT, tx_done_callback, NULL);
	if (ret != 0) {
		key = k_spin_lock(&sched_lock);
		clear_pending_locked(pending);
		bus->stats.tx_busy++;
		requeue_entry_locked(bus, entry);
		k_spin_unlock(&sched_lock, key);
		return ret;
	}

	key = k_spin_lock(&sched_lock);
	if (pending != NULL) {
		pending->sent_at_ms = k_uptime_get_32();
	}
	bus->stats.tx_frames++;
	bus->stats.window_tx_busy_us += frame_time_us(entry->frame.flags);
	if (entry->meta.reply_reserve_us != 0U) {
		bus->stats.window_reserved_us += entry->meta.reply_reserve_us;
	} else if (entry->meta.expect_reply) {
		bus->stats.window_reserved_us += frame_time_us(entry->frame.flags);
	}
	if (entry->meta.has_periodic_reply) {
		bus->stats.window_reserved_us += frame_time_us(entry->frame.flags);
	}
	k_spin_unlock(&sched_lock, key);

	return 0;
}

static void check_timeouts_locked(void)
{
	uint32_t now = k_uptime_get_32();

	for (size_t i = 0; i < ARRAY_SIZE(sched_buses); i++) {
		struct motor_can_sched_bus *bus = &sched_buses[i];

		if (bus->can_dev == NULL) {
			continue;
		}

		for (size_t j = 0; j < ARRAY_SIZE(bus->pending); j++) {
			struct motor_can_sched_pending *pending = &bus->pending[j];

			if (!pending->used || !time_reached(now, pending->deadline_ms)) {
				continue;
			}

			bus->stats.ack_timeouts++;
			if (pending->entry.retries < pending->entry.meta.max_retries) {
				struct motor_can_sched_entry retry_entry = pending->entry;

				retry_entry.retries++;
				retry_entry.meta.priority = MOTOR_CAN_SCHED_PRIO_HIGH;
				requeue_entry_locked(bus, &retry_entry);
				bus->stats.retry_frames++;
			} else {
				bus->stats.giveups++;
			}

			memset(pending, 0, sizeof(*pending));
		}
	}
}

static void release_periodic_locked(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(periodic_jobs); i++) {
		struct motor_can_sched_periodic *job = periodic_jobs[i];

		if ((job == NULL) || !job->enabled || (job->can_dev == NULL) ||
		    (job->period_ticks == 0U)) {
			continue;
		}

		while ((int32_t)(sched_tick - job->next_release_tick) >= 0) {
			queue_entry_locked(job->can_dev, &job->frame, &job->meta, 0U);
			job->next_release_tick += job->period_ticks;
		}
	}
}

static void reset_window_if_needed(struct motor_can_sched_bus *bus)
{
	uint32_t now = k_uptime_get_32();

	if ((bus->last_log_ms != 0U) && (now - bus->last_log_ms < MOTOR_CAN_SCHED_LOG_WINDOW_MS)) {
		return;
	}

	bus->last_log_ms = now;
	bus->stats.window_tx_busy_us = 0U;
	bus->stats.window_rx_busy_us = 0U;
	bus->stats.window_reserved_us = 0U;
}

static void motor_can_sched_thread(void *arg1, void *arg2, void *arg3)
{
	uint32_t next_wake = k_uptime_get_32();

	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	while (1) {
		struct motor_can_sched_entry entry;

		next_wake += 1U;
		sched_tick++;

		k_spinlock_key_t key = k_spin_lock(&sched_lock);
		release_periodic_locked();
		check_timeouts_locked();
		k_spin_unlock(&sched_lock, key);

		for (size_t bus_idx = 0; bus_idx < ARRAY_SIZE(sched_buses); bus_idx++) {
			struct motor_can_sched_bus *bus = &sched_buses[bus_idx];
			uint32_t budget_us = MOTOR_CAN_SCHED_TICK_US;

			if (bus->can_dev == NULL) {
				continue;
			}

			while (budget_us > 0U) {
				bool found = false;
				uint16_t cost_us = 0U;

				key = k_spin_lock(&sched_lock);
				for (size_t prio = 0; prio < MOTOR_CAN_SCHED_PRIO_COUNT; prio++) {
					if (ring_pop(&bus->rings[prio], &entry)) {
						found = true;
						break;
					}
				}
				k_spin_unlock(&sched_lock, key);

				if (!found) {
					break;
				}

				cost_us = frame_cost_us(&entry.frame, &entry.meta);
				if (cost_us > budget_us) {
					key = k_spin_lock(&sched_lock);
					requeue_entry_locked(bus, &entry);
					k_spin_unlock(&sched_lock, key);
					break;
				}

				if (send_one(bus, &entry) != 0) {
					break;
				}

				budget_us -= cost_us;
			}

			key = k_spin_lock(&sched_lock);
			reset_window_if_needed(bus);
			k_spin_unlock(&sched_lock, key);
		}

		uint32_t now = k_uptime_get_32();

		if ((int32_t)(next_wake - now) > 0) {
			k_msleep(next_wake - now);
		} else {
			next_wake = now;
			k_yield();
		}
	}
}

static uint16_t choose_phase_locked(struct motor_can_sched_bus *bus,
				    const struct motor_can_sched_periodic *job)
{
	uint16_t best_phase = 0U;
	uint32_t best_score = UINT32_MAX;
	uint16_t period_ticks = job->period_ticks;
	uint16_t cost = frame_cost_us(&job->frame, &job->meta);

	for (uint16_t phase = 0U; phase < period_ticks; phase++) {
		uint32_t peak = 0U;
		uint32_t total = 0U;

		for (uint16_t slot = phase; slot < MOTOR_CAN_SCHED_SUPERFRAME; slot += period_ticks) {
			uint32_t loaded = bus->phase_load[slot] + cost;

			total += loaded;
			peak = MAX(peak, loaded);
		}

		if ((peak * 1024U + total) < best_score) {
			best_score = peak * 1024U + total;
			best_phase = phase;
		}
	}

	for (uint16_t slot = best_phase; slot < MOTOR_CAN_SCHED_SUPERFRAME; slot += period_ticks) {
		bus->phase_load[slot] += cost;
	}

	return best_phase;
}

int motor_can_sched_init(void)
{
	if (sched_initialized) {
		return 0;
	}

	k_thread_create(&sched_thread, sched_stack, K_THREAD_STACK_SIZEOF(sched_stack),
			motor_can_sched_thread, NULL, NULL, NULL, MOTOR_CAN_SCHED_THREAD_PRIO, 0,
			K_NO_WAIT);
	k_thread_name_set(&sched_thread, "motor_can_sched");
	sched_initialized = true;
	return 0;
}

int motor_can_sched_register_can(const struct device *can_dev)
{
	k_spinlock_key_t key;
	struct motor_can_sched_bus *bus;
	int ret;

	if ((can_dev == NULL) || !device_is_ready(can_dev)) {
		return -ENODEV;
	}

	motor_can_sched_init();

	key = k_spin_lock(&sched_lock);
	bus = register_bus_locked(can_dev);
	k_spin_unlock(&sched_lock, key);
	if (bus == NULL) {
		return -ENOMEM;
	}

	if (!bus->started) {
		ret = can_start(can_dev);
		if ((ret != 0) && (ret != -EALREADY)) {
			return ret;
		}
		bus->started = true;
	}

	return 0;
}

static int motor_can_sched_submit(const struct device *can_dev, const struct can_frame *frame,
				  const struct motor_can_sched_meta *meta)
{
	k_spinlock_key_t key;
	int ret;

	if ((can_dev == NULL) || (frame == NULL) || (meta == NULL)) {
		return -EINVAL;
	}

	ret = motor_can_sched_register_can(can_dev);
	if (ret != 0) {
		return ret;
	}

	key = k_spin_lock(&sched_lock);
	ret = queue_entry_locked(can_dev, frame, meta, 0U);
	k_spin_unlock(&sched_lock, key);
	return ret;
}

static int motor_can_sched_add_periodic(struct motor_can_sched_periodic *job)
{
	k_spinlock_key_t key;
	struct motor_can_sched_bus *bus;

	if ((job == NULL) || (job->can_dev == NULL) || (job->rate_hz == 0U)) {
		return -EINVAL;
	}

	if (motor_can_sched_register_can(job->can_dev) != 0) {
		return -ENODEV;
	}

	job->period_ticks = MAX(1U, DIV_ROUND_CLOSEST(MOTOR_CAN_SCHED_TICK_HZ, job->rate_hz));

	key = k_spin_lock(&sched_lock);
	bus = register_bus_locked(job->can_dev);
	if (bus == NULL) {
		k_spin_unlock(&sched_lock, key);
		return -ENOMEM;
	}

	for (size_t i = 0; i < ARRAY_SIZE(periodic_jobs); i++) {
		if (periodic_jobs[i] == job) {
			job->enabled = true;
			k_spin_unlock(&sched_lock, key);
			return 0;
		}
	}

	for (size_t i = 0; i < ARRAY_SIZE(periodic_jobs); i++) {
		if (periodic_jobs[i] == NULL) {
			periodic_jobs[i] = job;
			job->phase_tick = choose_phase_locked(bus, job);
			job->next_release_tick = sched_tick + job->phase_tick;
			job->enabled = true;
			k_spin_unlock(&sched_lock, key);
			return 0;
		}
	}

	k_spin_unlock(&sched_lock, key);
	return -ENOSPC;
}

static int motor_can_sched_remove_periodic(struct motor_can_sched_periodic *job)
{
	k_spinlock_key_t key;

	if (job == NULL) {
		return -EINVAL;
	}

	key = k_spin_lock(&sched_lock);
	for (size_t i = 0; i < ARRAY_SIZE(periodic_jobs); i++) {
		if (periodic_jobs[i] == job) {
			periodic_jobs[i] = NULL;
			job->enabled = false;
			k_spin_unlock(&sched_lock, key);
			return 0;
		}
	}
	k_spin_unlock(&sched_lock, key);

	return -ENOENT;
}

static int motor_can_sched_update_periodic_frame(struct motor_can_sched_periodic *job,
						 const struct can_frame *frame)
{
	k_spinlock_key_t key;

	if ((job == NULL) || (frame == NULL)) {
		return -EINVAL;
	}

	key = k_spin_lock(&sched_lock);
	job->frame = *frame;
	k_spin_unlock(&sched_lock, key);
	return 0;
}

void motor_can_sched_report_rx(const struct device *can_dev, const struct can_frame *frame)
{
	k_spinlock_key_t key;
	struct motor_can_sched_bus *bus;

	if ((can_dev == NULL) || (frame == NULL)) {
		return;
	}

	key = k_spin_lock(&sched_lock);
	bus = find_bus(can_dev);
	if (bus == NULL) {
		k_spin_unlock(&sched_lock, key);
		return;
	}

	bus->stats.rx_frames++;
	bus->stats.window_rx_busy_us += frame_time_us(frame->flags);

	for (size_t i = 0; i < ARRAY_SIZE(bus->pending); i++) {
		struct motor_can_sched_pending *pending = &bus->pending[i];

		if (!pending->used) {
			continue;
		}
		if ((frame->id & pending->reply_mask) == (pending->reply_id & pending->reply_mask)) {
			uint32_t rtt_ms = k_uptime_get_32() - pending->sent_at_ms;

			bus->stats.ack_matches++;
			ARG_UNUSED(rtt_ms);
			memset(pending, 0, sizeof(*pending));
			break;
		}
	}
	k_spin_unlock(&sched_lock, key);
}

int motor_can_sched_get_stats(const struct device *can_dev, struct motor_can_sched_stats *stats)
{
	k_spinlock_key_t key;
	struct motor_can_sched_bus *bus;

	if ((can_dev == NULL) || (stats == NULL)) {
		return -EINVAL;
	}

	key = k_spin_lock(&sched_lock);
	bus = find_bus(can_dev);
	if (bus == NULL) {
		k_spin_unlock(&sched_lock, key);
		return -ENOENT;
	}

	*stats = bus->stats;
	k_spin_unlock(&sched_lock, key);
	return 0;
}

int motor_can_sched_send(const struct device *can_dev, const struct can_frame *frame,
			 const struct motor_can_sched_tx_param *param,
			 motor_can_sched_handle_t *handle_out)
{
	struct motor_can_sched_meta meta;
	k_spinlock_key_t key;
	struct motor_can_sched_handle *handle;
	int ret;

	if ((can_dev == NULL) || (frame == NULL)) {
		return -EINVAL;
	}

	if (handle_out != NULL) {
		*handle_out = NULL;
	}

	meta = meta_from_param(frame, param);
	if ((param == NULL) || !param->loop) {
		return motor_can_sched_submit(can_dev, frame, &meta);
	}

	if ((param->loop_hz == 0U) || (handle_out == NULL)) {
		return -EINVAL;
	}

	ret = motor_can_sched_register_can(can_dev);
	if (ret != 0) {
		return ret;
	}

	key = k_spin_lock(&sched_lock);
	handle = alloc_handle_locked();
	k_spin_unlock(&sched_lock, key);
	if (handle == NULL) {
		return -ENOSPC;
	}

	handle->job.can_dev = can_dev;
	handle->job.frame = *frame;
	handle->job.meta = meta;
	handle->job.rate_hz = param->loop_hz;
	ret = motor_can_sched_add_periodic(&handle->job);
	if (ret != 0) {
		key = k_spin_lock(&sched_lock);
		free_handle_locked(handle);
		k_spin_unlock(&sched_lock, key);
		return ret;
	}

	*handle_out = handle;
	return 0;
}

int motor_can_sched_send_prio(const struct device *can_dev, const struct can_frame *frame,
			      bool high_priority, const char *tag)
{
	const struct motor_can_sched_tx_param param = {
		.high_priority = high_priority,
		.tag = tag,
	};

	return motor_can_sched_send(can_dev, frame, &param, NULL);
}

int motor_can_sched_send_reply(const struct device *can_dev, const struct can_frame *frame,
			       uint32_t reply_id, uint32_t reply_mask, uint16_t timeout_ms,
			       const char *tag)
{
	const struct motor_can_sched_tx_param param = {
		.track_reply = true,
		.high_priority = true,
		.immediate_reply = true,
		.ack_timeout_ms = timeout_ms,
		.max_retries = 1U,
		.reply_id = reply_id,
		.reply_mask = reply_mask,
		.tag = tag,
	};

	return motor_can_sched_send(can_dev, frame, &param, NULL);
}

int motor_can_sched_update(motor_can_sched_handle_t handle, const struct can_frame *frame)
{
	if ((handle == NULL) || !handle->in_use) {
		return -EINVAL;
	}

	return motor_can_sched_update_periodic_frame(&handle->job, frame);
}

int motor_can_sched_remove(motor_can_sched_handle_t handle)
{
	k_spinlock_key_t key;
	int ret;

	if ((handle == NULL) || !handle->in_use) {
		return -EINVAL;
	}

	ret = motor_can_sched_remove_periodic(&handle->job);
	if (ret != 0) {
		return ret;
	}

	key = k_spin_lock(&sched_lock);
	free_handle_locked(handle);
	k_spin_unlock(&sched_lock, key);
	return 0;
}
