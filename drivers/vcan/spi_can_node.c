/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT custom_spi_can_node

#include "spi_can_mfd.h"

#include <string.h>

#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(spi_can_node, CONFIG_CAN_LOG_LEVEL);

#define VCAN_MAX_BITRATE 1000000U

struct spi_can_filter_entry {
	can_rx_callback_t callback;
	void *user_data;
	struct can_filter filter;
};

struct spi_can_node_config {
	struct can_driver_config common;
	const struct device *parent;
	uint8_t channel;
};

struct spi_can_node_data {
	struct can_driver_data common;
	struct k_mutex lock;
	struct spi_can_filter_entry filters[CONFIG_VCAN_MAX_RX_FILTER];
	enum can_state cached_state;
	struct can_bus_err_cnt cached_err_cnt;
	uint32_t bitrate;
};

struct spi_can_filter_match {
	can_rx_callback_t callback;
	void *user_data;
};

static int spi_can_node_get_free_filter(struct spi_can_node_data *data)
{
	for (int i = 0; i < ARRAY_SIZE(data->filters); i++) {
		if (data->filters[i].callback == NULL) {
			return i;
		}
	}

	return -ENOSPC;
}

static void spi_can_node_update_state(const struct device *dev, enum can_state state,
				      const struct can_bus_err_cnt *err_cnt, bool notify)
{
	struct spi_can_node_data *data = dev->data;
	can_state_change_callback_t callback = NULL;
	void *user_data = NULL;
	bool changed;

	k_mutex_lock(&data->lock, K_FOREVER);
	changed = (data->cached_state != state) ||
		  (data->cached_err_cnt.tx_err_cnt != err_cnt->tx_err_cnt) ||
		  (data->cached_err_cnt.rx_err_cnt != err_cnt->rx_err_cnt);
	data->cached_state = state;
	data->cached_err_cnt = *err_cnt;

	if (notify && changed) {
		callback = data->common.state_change_cb;
		user_data = data->common.state_change_cb_user_data;
	}
	k_mutex_unlock(&data->lock);

	if (callback != NULL) {
		callback(dev, state, *err_cnt, user_data);
	}
}

static int spi_can_node_timing_to_bitrate(const struct device *dev, const struct can_timing *timing,
					  uint32_t *bitrate)
{
	uint32_t core_clock;
	uint32_t total_tq;
	uint64_t denom;
	int ret;

	if (timing == NULL || bitrate == NULL || timing->prescaler == 0U) {
		return -EINVAL;
	}

	ret = spi_can_mfd_get_core_clock(((const struct spi_can_node_config *)dev->config)->parent,
					 &core_clock);
	if (ret < 0) {
		return ret;
	}

	total_tq = 1U + timing->prop_seg + timing->phase_seg1 + timing->phase_seg2;
	if (total_tq <= 1U) {
		return -EINVAL;
	}

	denom = (uint64_t)timing->prescaler * total_tq;
	*bitrate = (uint32_t)(((uint64_t)core_clock + (denom / 2U)) / denom);

	return (*bitrate == 0U) ? -EINVAL : 0;
}

int spi_can_node_handle_rx_frame(const struct device *dev, const struct can_frame *frame)
{
	struct spi_can_node_data *data = dev->data;
	struct spi_can_filter_match matches[CONFIG_VCAN_MAX_RX_FILTER];
	struct can_frame frame_copy = *frame;
	size_t match_count = 0U;

	k_mutex_lock(&data->lock, K_FOREVER);
	for (int i = 0; i < ARRAY_SIZE(data->filters); i++) {
		if (data->filters[i].callback == NULL) {
			continue;
		}

		if (!can_frame_matches_filter(frame, &data->filters[i].filter)) {
			continue;
		}

		matches[match_count].callback = data->filters[i].callback;
		matches[match_count].user_data = data->filters[i].user_data;
		match_count++;
	}
	k_mutex_unlock(&data->lock);

	for (size_t i = 0U; i < match_count; i++) {
		struct can_frame delivered = frame_copy;

		matches[i].callback(dev, &delivered, matches[i].user_data);
	}

	return 0;
}

void spi_can_node_report_state(const struct device *dev, enum can_state state,
			       const struct can_bus_err_cnt *err_cnt)
{
	spi_can_node_update_state(dev, state, err_cnt, true);
}

static int spi_can_node_get_capabilities(const struct device *dev, can_mode_t *cap)
{
	ARG_UNUSED(dev);

	*cap = CAN_MODE_NORMAL;

	return 0;
}

static int spi_can_node_start(const struct device *dev)
{
	const struct spi_can_node_config *cfg = dev->config;
	struct spi_can_node_data *data = dev->data;
	struct can_bus_err_cnt err_cnt = {0};
	enum can_state state = CAN_STATE_ERROR_ACTIVE;
	int ret;

	if (data->common.started) {
		return -EALREADY;
	}

	ret = spi_can_mfd_set_bitrate(cfg->parent, cfg->channel, data->bitrate);
	if (ret < 0) {
		return ret;
	}

	data->common.started = true;
	CAN_STATS_RESET(dev);

	ret = spi_can_mfd_get_state(cfg->parent, cfg->channel, &state, &err_cnt);
	if (ret < 0) {
		state = CAN_STATE_ERROR_ACTIVE;
		err_cnt.tx_err_cnt = 0U;
		err_cnt.rx_err_cnt = 0U;
	}

	spi_can_node_update_state(dev, state, &err_cnt, false);

	return 0;
}

static int spi_can_node_stop(const struct device *dev)
{
	struct spi_can_node_data *data = dev->data;
	struct can_bus_err_cnt err_cnt = {0};

	if (!data->common.started) {
		return -EALREADY;
	}

	data->common.started = false;
	spi_can_node_update_state(dev, CAN_STATE_STOPPED, &err_cnt, false);

	return 0;
}

static int spi_can_node_set_mode(const struct device *dev, can_mode_t mode)
{
	struct spi_can_node_data *data = dev->data;

	if (data->common.started) {
		return -EBUSY;
	}

	if (mode != CAN_MODE_NORMAL) {
		return -ENOTSUP;
	}

	data->common.mode = mode;

	return 0;
}

static int spi_can_node_set_timing(const struct device *dev, const struct can_timing *timing)
{
	struct spi_can_node_data *data = dev->data;
	uint32_t bitrate;
	int ret;

	if (data->common.started) {
		return -EBUSY;
	}

	ret = spi_can_node_timing_to_bitrate(dev, timing, &bitrate);
	if (ret < 0) {
		return ret;
	}

	data->bitrate = bitrate;

	return 0;
}

static int spi_can_node_send(const struct device *dev, const struct can_frame *frame,
			     k_timeout_t timeout, can_tx_callback_t callback, void *user_data)
{
	const struct spi_can_node_config *cfg = dev->config;
	struct spi_can_node_data *data = dev->data;
	int ret;

	if ((frame->flags & ~(CAN_FRAME_IDE | CAN_FRAME_RTR)) != 0U) {
		return -ENOTSUP;
	}

	if (frame->dlc > CAN_MAX_DLC) {
		return -EINVAL;
	}

	if (!data->common.started) {
		ret = spi_can_node_start(dev);
		if (ret < 0 && ret != -EALREADY) {
			return ret;
		}
	}

	ret = spi_can_mfd_send(cfg->parent, cfg->channel, frame, timeout);
	if (ret < 0) {
		return ret;
	}

	if (callback != NULL) {
		callback(dev, 0, user_data);
	}

	return 0;
}

static int spi_can_node_add_rx_filter(const struct device *dev, can_rx_callback_t callback,
				      void *user_data, const struct can_filter *filter)
{
	struct spi_can_node_data *data = dev->data;
	int filter_id;

	if ((filter->flags & ~CAN_FILTER_IDE) != 0U) {
		return -ENOTSUP;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	filter_id = spi_can_node_get_free_filter(data);
	if (filter_id >= 0) {
		data->filters[filter_id].callback = callback;
		data->filters[filter_id].user_data = user_data;
		data->filters[filter_id].filter = *filter;
	}
	k_mutex_unlock(&data->lock);

	return filter_id;
}

static void spi_can_node_remove_rx_filter(const struct device *dev, int filter_id)
{
	struct spi_can_node_data *data = dev->data;

	if (filter_id < 0 || filter_id >= ARRAY_SIZE(data->filters)) {
		return;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	memset(&data->filters[filter_id], 0, sizeof(data->filters[filter_id]));
	k_mutex_unlock(&data->lock);
}

static int spi_can_node_get_state(const struct device *dev, enum can_state *state,
				  struct can_bus_err_cnt *err_cnt)
{
	const struct spi_can_node_config *cfg = dev->config;
	struct spi_can_node_data *data = dev->data;
	struct can_bus_err_cnt latest_err = {0};
	enum can_state latest_state = CAN_STATE_STOPPED;
	int ret;

	if (!data->common.started) {
		if (state != NULL) {
			*state = CAN_STATE_STOPPED;
		}

		if (err_cnt != NULL) {
			*err_cnt = (struct can_bus_err_cnt){0};
		}

		spi_can_node_update_state(dev, CAN_STATE_STOPPED,
					  &(struct can_bus_err_cnt){0}, false);
		return 0;
	}

	ret = spi_can_mfd_get_state(cfg->parent, cfg->channel, &latest_state, &latest_err);
	if (ret < 0) {
		return ret;
	}

	spi_can_node_update_state(dev, latest_state, &latest_err, false);

	if (state != NULL) {
		*state = latest_state;
	}

	if (err_cnt != NULL) {
		*err_cnt = latest_err;
	}

	return 0;
}

static void spi_can_node_set_state_change_callback(const struct device *dev,
						   can_state_change_callback_t callback,
						   void *user_data)
{
	struct spi_can_node_data *data = dev->data;

	k_mutex_lock(&data->lock, K_FOREVER);
	data->common.state_change_cb = callback;
	data->common.state_change_cb_user_data = user_data;
	k_mutex_unlock(&data->lock);
}

static int spi_can_node_get_core_clock(const struct device *dev, uint32_t *rate)
{
	const struct spi_can_node_config *cfg = dev->config;

	return spi_can_mfd_get_core_clock(cfg->parent, rate);
}

static int spi_can_node_get_max_filters(const struct device *dev, bool ide)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(ide);

	return CONFIG_VCAN_MAX_RX_FILTER;
}

static DEVICE_API(can, spi_can_node_driver_api) = {
	.get_capabilities = spi_can_node_get_capabilities,
	.start = spi_can_node_start,
	.stop = spi_can_node_stop,
	.set_mode = spi_can_node_set_mode,
	.set_timing = spi_can_node_set_timing,
	.send = spi_can_node_send,
	.add_rx_filter = spi_can_node_add_rx_filter,
	.remove_rx_filter = spi_can_node_remove_rx_filter,
	.get_state = spi_can_node_get_state,
	.set_state_change_callback = spi_can_node_set_state_change_callback,
	.get_core_clock = spi_can_node_get_core_clock,
	.get_max_filters = spi_can_node_get_max_filters,
	.timing_min = {
		.sjw = 1,
		.prop_seg = 0,
		.phase_seg1 = 2,
		.phase_seg2 = 2,
		.prescaler = 1,
	},
	.timing_max = {
		.sjw = 128,
		.prop_seg = 0,
		.phase_seg1 = 256,
		.phase_seg2 = 128,
		.prescaler = 32,
	},
};

static int spi_can_node_init(const struct device *dev)
{
	const struct spi_can_node_config *cfg = dev->config;
	struct spi_can_node_data *data = dev->data;
	struct can_bus_err_cnt err_cnt = {0};

	if (!device_is_ready(cfg->parent)) {
		return -ENODEV;
	}

	k_mutex_init(&data->lock);
	data->common.mode = CAN_MODE_NORMAL;
	data->common.started = false;
	data->cached_state = CAN_STATE_STOPPED;
	data->cached_err_cnt = err_cnt;
	data->bitrate = cfg->common.bitrate;
	memset(data->filters, 0, sizeof(data->filters));

	return 0;
}

#define SPI_CAN_NODE_INIT(inst)                                                                   \
	BUILD_ASSERT(DT_INST_PROP(inst, can_channel) < SPI_CAN_MFD_MAX_CHANNELS,                 \
		     "custom,spi-can-node channel must be 0 or 1");                           \
	static const struct spi_can_node_config spi_can_node_config_##inst = {                   \
		.common = CAN_DT_DRIVER_CONFIG_INST_GET(inst, 0, VCAN_MAX_BITRATE),            \
		.parent = DEVICE_DT_GET(DT_INST_PARENT(inst)),                                  \
		.channel = DT_INST_PROP(inst, can_channel),                                     \
	};                                                                                         \
	static struct spi_can_node_data spi_can_node_data_##inst;                                 \
	CAN_DEVICE_DT_INST_DEFINE(inst, spi_can_node_init, NULL, &spi_can_node_data_##inst,       \
				  &spi_can_node_config_##inst, POST_KERNEL,                     \
				  CONFIG_VCAN_NODE_INIT_PRIORITY, &spi_can_node_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SPI_CAN_NODE_INIT)
