/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT custom_spi_can_mfd

#include "spi_can_mfd.h"

#include <string.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(spi_can_mfd, CONFIG_CAN_LOG_LEVEL);

#define SPI_CAN_LOG_INTERVAL_MS 5000U

#define SPI_CAN_SYNC_MAGIC 0x5643414eUL
#define SPI_CAN_SYNC_VERSION 1U
#define SPI_CAN_SYNC_FRAME_SIZE 192U
#define SPI_CAN_SYNC_MAX_FRAMES 4U
#define SPI_CAN_SYNC_POLL_US 500U
#define SPI_CAN_KICK_COALESCE_US 50U
#define SPI_CAN_SYNC_TX_QUEUE_DEPTH CONFIG_VCAN_TX_QUEUE_DEPTH
#define SPI_CAN_REARM_DELAY_US 200U
#define SPI_CAN_STATE_FLAG_STARTED BIT(0)
#define SPI_CAN_STATE_FLAG_RX_PENDING BIT(1)
#define SPI_CAN_SPI_OPERATION                                                                  \
	(SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPHA | SPI_OP_MODE_MASTER)

struct spi_can_sync_can_entry {
	uint32_t id;
	uint8_t dlc;
	uint8_t flags;
	uint8_t channel;
	uint8_t reserved;
	uint8_t data[CAN_MAX_DLC];
};

struct spi_can_sync_state_entry {
	uint8_t state;
	uint8_t tx_err;
	uint8_t rx_err;
	uint8_t flags;
};

struct spi_can_sync_bitrate_entry {
	uint8_t bitrate[4];
};

struct spi_can_host_sync_frame {
	uint8_t magic[4];
	uint8_t version;
	uint8_t seq;
	uint8_t tx_count;
	uint8_t bitrate_mask;
	uint8_t reserved0[8];
	struct spi_can_sync_bitrate_entry bitrates[SPI_CAN_MFD_MAX_CHANNELS];
	struct spi_can_sync_can_entry tx_entries[SPI_CAN_SYNC_MAX_FRAMES];
	uint8_t reserved1[104];
};

struct spi_can_slave_sync_frame {
	uint8_t magic[4];
	uint8_t version;
	uint8_t seq;
	uint8_t rx_count;
	uint8_t more_rx_mask;
	uint8_t state_mask;
	uint8_t reserved0[7];
	struct spi_can_sync_state_entry states[SPI_CAN_MFD_MAX_CHANNELS];
	struct spi_can_sync_can_entry rx_entries[SPI_CAN_SYNC_MAX_FRAMES];
	uint8_t reserved1[104];
};

BUILD_ASSERT(sizeof(struct spi_can_host_sync_frame) == SPI_CAN_SYNC_FRAME_SIZE,
	     "unexpected host sync frame size");
BUILD_ASSERT(sizeof(struct spi_can_slave_sync_frame) == SPI_CAN_SYNC_FRAME_SIZE,
	     "unexpected slave sync frame size");

struct spi_can_mfd_config {
	struct spi_dt_spec bus;
	struct gpio_dt_spec int_gpio;
	uint32_t can_core_clock;
	const struct device *channels[SPI_CAN_MFD_MAX_CHANNELS];
};

struct spi_can_mfd_channel_data {
	struct k_msgq tx_queue;
	uint8_t tx_storage[SPI_CAN_SYNC_TX_QUEUE_DEPTH * sizeof(struct can_frame)];
	uint32_t pending_bitrate;
	enum can_state cached_state;
	struct can_bus_err_cnt cached_err_cnt;
	bool bitrate_dirty;
	bool state_valid;
};

struct spi_can_mfd_data {
	const struct device *dev;
	struct spi_config spi_cfg;
	uint8_t tx_frame[SPI_CAN_SYNC_FRAME_SIZE] __aligned(4);
	uint8_t rx_frame[SPI_CAN_SYNC_FRAME_SIZE] __aligned(4);
	struct k_mutex bus_lock;
	struct k_sem service_sem;
	struct k_timer service_timer;
	struct spi_can_mfd_channel_data channel_data[SPI_CAN_MFD_MAX_CHANNELS];
	struct k_thread poll_thread;
	K_KERNEL_STACK_MEMBER(poll_stack, CONFIG_VCAN_WORKQ_STACK_SIZE);
	atomic_t service_timer_armed;
	uint8_t tx_seq;
	uint8_t rr_channel;
};

struct spi_can_service_plan {
	uint8_t tx_count_per_channel[SPI_CAN_MFD_MAX_CHANNELS];
	uint8_t bitrate_mask;
	uint8_t rr_channel_after;
};

static int spi_can_validate_channel(uint8_t channel)
{
	return channel < SPI_CAN_MFD_MAX_CHANNELS ? 0 : -EINVAL;
}

static int spi_can_transceive_frame_locked(const struct spi_can_mfd_config *cfg,
					   struct spi_can_mfd_data *data)
{
	struct spi_buf tx_buf = {
		.buf = data->tx_frame,
		.len = sizeof(data->tx_frame),
	};
	struct spi_buf rx_buf = {
		.buf = data->rx_frame,
		.len = sizeof(data->rx_frame),
	};
	struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1U,
	};
	struct spi_buf_set rx = {
		.buffers = &rx_buf,
		.count = 1U,
	};

	return spi_transceive(cfg->bus.bus, &data->spi_cfg, &tx, &rx);
}

static void spi_can_pack_bitrate(uint32_t bitrate, struct spi_can_sync_bitrate_entry *entry)
{
	memset(entry, 0, sizeof(*entry));
	sys_put_le32(bitrate, entry->bitrate);
}

static void spi_can_pack_frame_entry(struct spi_can_sync_can_entry *entry,
				     const struct can_frame *frame, uint8_t channel)
{
	memset(entry, 0, sizeof(*entry));
	entry->id = frame->id;
	entry->dlc = frame->dlc;
	entry->flags = frame->flags;
	entry->channel = channel;
	memcpy(entry->data, frame->data, CAN_MAX_DLC);
}

static int spi_can_unpack_frame_entry(const struct spi_can_sync_can_entry *entry,
				      struct can_frame *frame, uint8_t *channel)
{
	memset(frame, 0, sizeof(*frame));
	frame->id = entry->id;
	frame->dlc = entry->dlc;
	frame->flags = entry->flags;

	if (frame->dlc > CAN_MAX_DLC) {
		return -EINVAL;
	}

	memcpy(frame->data, entry->data, CAN_MAX_DLC);
	*channel = entry->channel;

	return 0;
}

static bool spi_can_try_align_sync_frame(uint8_t *frame, size_t len)
{
	static const uint8_t magic_le[] = { 0x4e, 0x41, 0x43, 0x56 };

	for (size_t offset = 1U; offset <= 3U; offset++) {
		if ((offset + sizeof(magic_le)) > len) {
			break;
		}

		if (memcmp(&frame[offset], magic_le, sizeof(magic_le)) != 0) {
			continue;
		}

		memmove(frame, &frame[offset], len - offset);
		memset(&frame[len - offset], 0, offset);
		LOG_WRN_RATELIMIT_RATE(SPI_CAN_LOG_INTERVAL_MS,
				       "realigned slave sync frame by %u byte(s)",
				       (unsigned int)offset);
		return true;
	}

	return false;
}

static bool spi_can_should_log_again(uint32_t *last_ms, uint32_t period_ms)
{
	uint32_t now = k_uptime_get_32();

	if ((now - *last_ms) < period_ms) {
		return false;
	}

	*last_ms = now;

	return true;
}

static void spi_can_log_bad_sync_frame(const uint8_t *frame)
{
	static uint32_t last_ms;
	static uint32_t last_magic;
	uint32_t magic = sys_get_le32(frame);

	if (magic == last_magic &&
	    !spi_can_should_log_again(&last_ms, SPI_CAN_LOG_INTERVAL_MS)) {
		return;
	}

	last_magic = magic;
	last_ms = k_uptime_get_32();

	LOG_ERR_RATELIMIT_RATE(SPI_CAN_LOG_INTERVAL_MS,
			       "bad sync rsp magic=0x%08x raw=%02x %02x %02x %02x", magic,
			       frame[0], frame[1], frame[2], frame[3]);
}

static bool spi_can_channel_has_pending_locked(const struct spi_can_mfd_channel_data *channel)
{
	return channel->bitrate_dirty ||
	       k_msgq_num_used_get((struct k_msgq *)&channel->tx_queue) > 0U;
}

static void spi_can_service_timer_handler(struct k_timer *timer)
{
	struct spi_can_mfd_data *data = k_timer_user_data_get(timer);

	atomic_clear(&data->service_timer_armed);
	k_sem_give(&data->service_sem);
}

static void spi_can_schedule_service(struct spi_can_mfd_data *data)
{
	if (!atomic_cas(&data->service_timer_armed, 0, 1)) {
		return;
	}

	k_timer_start(&data->service_timer, K_USEC(SPI_CAN_KICK_COALESCE_US), K_NO_WAIT);
}

static bool spi_can_should_transfer_locked(const struct spi_can_mfd_config *cfg,
					   const struct spi_can_mfd_data *data, bool force)
{
	if (force) {
		return true;
	}

	for (uint8_t channel = 0U; channel < SPI_CAN_MFD_MAX_CHANNELS; channel++) {
		if (spi_can_channel_has_pending_locked(&data->channel_data[channel])) {
			return true;
		}
	}

	/*
	 * The slave exposes INT as a level signal: as long as remote RX/state data
	 * is pending, the pin stays asserted and the next 500 us poll slot will
	 * pick it up.
	 */
	return gpio_pin_get_dt(&cfg->int_gpio) > 0;
}

static size_t spi_can_plan_tx_entries_locked(struct spi_can_mfd_data *data,
					     struct spi_can_host_sync_frame *frame,
					     struct spi_can_service_plan *plan)
{
	size_t count = 0U;
	uint8_t rr_channel = data->rr_channel;

	while (count < SPI_CAN_SYNC_MAX_FRAMES) {
		bool found = false;

		for (uint8_t offset = 0U; offset < SPI_CAN_MFD_MAX_CHANNELS; offset++) {
			uint8_t channel = (rr_channel + offset) % SPI_CAN_MFD_MAX_CHANNELS;
			struct can_frame can_frame;
			uint32_t queued_idx = plan->tx_count_per_channel[channel];

			if (k_msgq_peek_at(&data->channel_data[channel].tx_queue, &can_frame, queued_idx) <
			    0) {
				continue;
			}

			spi_can_pack_frame_entry(&frame->tx_entries[count], &can_frame, channel);
			plan->tx_count_per_channel[channel]++;
			frame->tx_count++;
			count++;
			rr_channel = (channel + 1U) % SPI_CAN_MFD_MAX_CHANNELS;
			found = true;
			break;
		}

		if (!found) {
			break;
		}
	}

	plan->rr_channel_after = rr_channel;

	return count;
}

static void spi_can_build_host_frame_locked(struct spi_can_mfd_data *data,
					    struct spi_can_service_plan *plan)
{
	struct spi_can_host_sync_frame *frame = (struct spi_can_host_sync_frame *)data->tx_frame;

	memset(plan, 0, sizeof(*plan));
	memset(frame, 0, sizeof(*frame));
	sys_put_le32(SPI_CAN_SYNC_MAGIC, frame->magic);
	frame->version = SPI_CAN_SYNC_VERSION;
	frame->seq = ++data->tx_seq;

	(void)spi_can_plan_tx_entries_locked(data, frame, plan);

	for (uint8_t channel = 0U; channel < SPI_CAN_MFD_MAX_CHANNELS; channel++) {
		struct spi_can_mfd_channel_data *channel_data = &data->channel_data[channel];

		if (channel_data->pending_bitrate == 0U) {
			continue;
		}

		if (!channel_data->bitrate_dirty && plan->tx_count_per_channel[channel] == 0U) {
			continue;
		}

		frame->bitrate_mask |= BIT(channel);
		plan->bitrate_mask |= BIT(channel);
		spi_can_pack_bitrate(channel_data->pending_bitrate, &frame->bitrates[channel]);
	}
}

static void spi_can_commit_service_locked(struct spi_can_mfd_data *data,
					  const struct spi_can_service_plan *plan)
{
	for (uint8_t channel = 0U; channel < SPI_CAN_MFD_MAX_CHANNELS; channel++) {
		struct spi_can_mfd_channel_data *channel_data = &data->channel_data[channel];
		struct can_frame dropped;

		if ((plan->bitrate_mask & BIT(channel)) != 0U) {
			channel_data->bitrate_dirty = false;
		}

		for (uint8_t i = 0U; i < plan->tx_count_per_channel[channel]; i++) {
			int ret = k_msgq_get(&channel_data->tx_queue, &dropped, K_NO_WAIT);

			__ASSERT_NO_MSG(ret == 0);
		}
	}

	data->rr_channel = plan->rr_channel_after;
}

static void spi_can_update_state_locked(struct spi_can_mfd_channel_data *channel_data,
					const struct spi_can_sync_state_entry *entry)
{
	channel_data->cached_state = (enum can_state)entry->state;
	channel_data->cached_err_cnt.tx_err_cnt = entry->tx_err;
	channel_data->cached_err_cnt.rx_err_cnt = entry->rx_err;
	channel_data->state_valid = true;
}

static int spi_can_parse_slave_frame_locked(const struct spi_can_mfd_config *cfg,
					    struct spi_can_mfd_data *data)
{
	const struct spi_can_slave_sync_frame *frame;

	if (sys_get_le32(data->rx_frame) != SPI_CAN_SYNC_MAGIC) {
		(void)spi_can_try_align_sync_frame(data->rx_frame, sizeof(data->rx_frame));
	}

	frame = (const struct spi_can_slave_sync_frame *)data->rx_frame;

	if (sys_get_le32(frame->magic) != SPI_CAN_SYNC_MAGIC ||
	    frame->version != SPI_CAN_SYNC_VERSION) {
		spi_can_log_bad_sync_frame(data->rx_frame);
		return -EIO;
	}

	for (uint8_t channel = 0U; channel < SPI_CAN_MFD_MAX_CHANNELS; channel++) {
		if ((frame->state_mask & BIT(channel)) == 0U) {
			continue;
		}

		spi_can_update_state_locked(&data->channel_data[channel], &frame->states[channel]);

		if (cfg->channels[channel] != NULL && device_is_ready(cfg->channels[channel])) {
			spi_can_node_report_state(cfg->channels[channel],
					       (enum can_state)frame->states[channel].state,
					       &(struct can_bus_err_cnt){
						       .tx_err_cnt = frame->states[channel].tx_err,
						       .rx_err_cnt = frame->states[channel].rx_err,
					       });
		}
	}

	for (size_t i = 0U; i < MIN((size_t)frame->rx_count, (size_t)SPI_CAN_SYNC_MAX_FRAMES); i++) {
		struct can_frame can_frame;
		uint8_t channel;
		int ret;

		ret = spi_can_unpack_frame_entry(&frame->rx_entries[i], &can_frame, &channel);
		if (ret < 0) {
			LOG_WRN_RATELIMIT_RATE(SPI_CAN_LOG_INTERVAL_MS,
					       "drop invalid remote frame idx=%u (%d)",
					       (unsigned int)i, ret);
			continue;
		}

		if (channel >= SPI_CAN_MFD_MAX_CHANNELS || cfg->channels[channel] == NULL ||
		    !device_is_ready(cfg->channels[channel])) {
			LOG_WRN_RATELIMIT_RATE(SPI_CAN_LOG_INTERVAL_MS,
					       "drop remote frame for invalid channel %u",
					       channel);
			continue;
		}

		(void)spi_can_node_handle_rx_frame(cfg->channels[channel], &can_frame);
	}

	return 0;
}

static int spi_can_service_locked(const struct spi_can_mfd_config *cfg,
				  struct spi_can_mfd_data *data, bool force)
{
	struct spi_can_service_plan plan;
	int ret;

	if (!spi_can_should_transfer_locked(cfg, data, force)) {
		return 0;
	}

	spi_can_build_host_frame_locked(data, &plan);
	memset(data->rx_frame, 0, sizeof(data->rx_frame));

	ret = spi_can_transceive_frame_locked(cfg, data);
	if (ret < 0) {
		return ret;
	}

	spi_can_commit_service_locked(data, &plan);

	k_busy_wait(SPI_CAN_REARM_DELAY_US);

	ret = spi_can_parse_slave_frame_locked(cfg, data);
	if (ret < 0) {
		return ret;
	}

	return 1;
}

static int spi_can_service_once(const struct device *parent, bool force)
{
	const struct spi_can_mfd_config *cfg = parent->config;
	struct spi_can_mfd_data *data = parent->data;
	int ret;

	ret = k_mutex_lock(&data->bus_lock, K_MSEC(20));
	if (ret < 0) {
		return ret;
	}

	ret = spi_can_service_locked(cfg, data, force);
	k_mutex_unlock(&data->bus_lock);

	return ret;
}

static void spi_can_poll_thread(void *p1, void *p2, void *p3)
{
	const struct device *parent = p1;
	struct spi_can_mfd_data *data = parent->data;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		int ret;
		uint8_t serviced = 0U;

		(void)k_sem_take(&data->service_sem, K_USEC(SPI_CAN_SYNC_POLL_US));

		while (1) {
			ret = spi_can_service_once(parent, false);
			if (ret <= 0) {
				if (ret < 0) {
					LOG_WRN_RATELIMIT_RATE(SPI_CAN_LOG_INTERVAL_MS,
							       "service loop failed (%d)",
							       ret);
				}

				break;
			}

			serviced++;
			if (serviced >= CONFIG_VCAN_MAX_SERVICE_BATCH) {
				k_sem_give(&data->service_sem);
				k_yield();
				break;
			}
		}
	}
}

int spi_can_mfd_set_bitrate(const struct device *parent, uint8_t channel, uint32_t bitrate)
{
	struct spi_can_mfd_data *data = parent->data;
	int ret;

	if (bitrate == 0U) {
		return -EINVAL;
	}

	ret = spi_can_validate_channel(channel);
	if (ret < 0) {
		return ret;
	}

	ret = k_mutex_lock(&data->bus_lock, K_FOREVER);
	if (ret < 0) {
		return ret;
	}

	data->channel_data[channel].pending_bitrate = bitrate;
	data->channel_data[channel].bitrate_dirty = true;
	k_mutex_unlock(&data->bus_lock);

	spi_can_schedule_service(data);

	return 0;
}

int spi_can_mfd_send(const struct device *parent, uint8_t channel,
		     const struct can_frame *frame, k_timeout_t timeout)
{
	struct spi_can_mfd_data *data = parent->data;
	int ret;

	if (frame == NULL) {
		return -EINVAL;
	}

	ret = spi_can_validate_channel(channel);
	if (ret < 0) {
		return ret;
	}

	/*
	 * If the caller is willing to wait, kick the service thread first so a
	 * full queue can start draining immediately instead of waiting for the
	 * next poll slot.
	 */
	if (!K_TIMEOUT_EQ(timeout, K_NO_WAIT)) {
		spi_can_schedule_service(data);
	}

	ret = k_msgq_put(&data->channel_data[channel].tx_queue, frame, timeout);
	if (ret < 0) {
		return ret;
	}

	spi_can_schedule_service(data);

	return 0;
}

int spi_can_mfd_get_core_clock(const struct device *parent, uint32_t *rate)
{
	const struct spi_can_mfd_config *cfg = parent->config;

	if (rate == NULL || cfg->can_core_clock == 0U) {
		return -EINVAL;
	}

	*rate = cfg->can_core_clock;

	return 0;
}

int spi_can_mfd_get_state(const struct device *parent, uint8_t channel,
			  enum can_state *state, struct can_bus_err_cnt *err_cnt)
{
	struct spi_can_mfd_data *data = parent->data;
	int ret;

	ret = spi_can_validate_channel(channel);
	if (ret < 0) {
		return ret;
	}

	ret = spi_can_service_once(parent, true);
	if (ret < 0) {
		return ret;
	}

	ret = k_mutex_lock(&data->bus_lock, K_FOREVER);
	if (ret < 0) {
		return ret;
	}

	if (!data->channel_data[channel].state_valid) {
		k_mutex_unlock(&data->bus_lock);
		return -EIO;
	}

	if (state != NULL) {
		*state = data->channel_data[channel].cached_state;
	}

	if (err_cnt != NULL) {
		*err_cnt = data->channel_data[channel].cached_err_cnt;
	}

	k_mutex_unlock(&data->bus_lock);

	return 0;
}

static int spi_can_mfd_init(const struct device *dev)
{
	const struct spi_can_mfd_config *cfg = dev->config;
	struct spi_can_mfd_data *data = dev->data;
	int ret;

	if (!spi_is_ready_dt(&cfg->bus)) {
		return -ENODEV;
	}

	if (!gpio_is_ready_dt(&cfg->int_gpio)) {
		return -ENODEV;
	}

	k_mutex_init(&data->bus_lock);
	k_sem_init(&data->service_sem, 0, K_SEM_MAX_LIMIT);
	k_timer_init(&data->service_timer, spi_can_service_timer_handler, NULL);
	k_timer_user_data_set(&data->service_timer, data);
	atomic_clear(&data->service_timer_armed);
	data->dev = dev;
	data->spi_cfg = cfg->bus.config;
	data->tx_seq = 0U;
	data->rr_channel = 0U;

	for (uint8_t channel = 0U; channel < SPI_CAN_MFD_MAX_CHANNELS; channel++) {
		struct spi_can_mfd_channel_data *channel_data = &data->channel_data[channel];

		k_msgq_init(&channel_data->tx_queue, channel_data->tx_storage,
			    sizeof(struct can_frame), SPI_CAN_SYNC_TX_QUEUE_DEPTH);
		channel_data->cached_state = CAN_STATE_STOPPED;
		channel_data->cached_err_cnt = (struct can_bus_err_cnt){0};
	}

	ret = gpio_pin_configure_dt(&cfg->int_gpio, GPIO_INPUT);
	if (ret < 0) {
		return ret;
	}

	k_thread_create(&data->poll_thread, data->poll_stack,
			K_KERNEL_STACK_SIZEOF(data->poll_stack), spi_can_poll_thread, (void *)dev,
			NULL, NULL, CONFIG_VCAN_WORKQ_PRIORITY, 0, K_NO_WAIT);

	return 0;
}

#define VCAN_CHILD_DEV_ENTRY(child) [DT_PROP(child, can_channel)] = DEVICE_DT_GET(child)

#define SPI_CAN_MFD_INIT(inst)                                                                    \
	BUILD_ASSERT(DT_INST_CHILD_NUM_STATUS_OKAY(inst) <= SPI_CAN_MFD_MAX_CHANNELS,           \
		     "custom,spi-can-mfd supports up to two CAN child nodes");                \
	static const struct spi_can_mfd_config spi_can_mfd_config_##inst = {                     \
		.bus = SPI_DT_SPEC_INST_GET(inst, SPI_CAN_SPI_OPERATION),                        \
		.int_gpio = GPIO_DT_SPEC_INST_GET(inst, int_gpios),                              \
		.can_core_clock = DT_INST_PROP(inst, can_core_clock),                           \
		.channels = { DT_INST_FOREACH_CHILD_STATUS_OKAY_SEP(inst,                        \
								    VCAN_CHILD_DEV_ENTRY, (,)) }, \
	};                                                                                         \
	static struct spi_can_mfd_data spi_can_mfd_data_##inst;                                  \
	DEVICE_DT_INST_DEFINE(inst, spi_can_mfd_init, NULL, &spi_can_mfd_data_##inst,            \
			      &spi_can_mfd_config_##inst, POST_KERNEL,                         \
			      CONFIG_VCAN_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(SPI_CAN_MFD_INIT)
