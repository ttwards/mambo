#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(vcan_slave_demo, LOG_LEVEL_INF);

#define SLAVE_DEMO_LOG_INTERVAL_MS 5000U

#define SPI_NODE DT_NODELABEL(spi2)
#define CAN0_NODE DT_NODELABEL(can1)
#define CAN1_NODE DT_NODELABEL(can2)
#define GPIOB_NODE DT_NODELABEL(gpiob)

#define SLAVE_INT_PIN 8U
#define SLAVE_INT_REFRESH_US 500U

#define SPI_CAN_SYNC_MAGIC 0x5643414eUL
#define SPI_CAN_SYNC_VERSION 1U
#define SPI_CAN_SYNC_FRAME_SIZE 192U
#define SPI_CAN_SYNC_MAX_FRAMES 4U
#define SPI_CAN_SYNC_RX_QUEUE_DEPTH 32U
#define SPI_CAN_SYNC_TX_QUEUE_DEPTH 128U
#define CAN_TX_TARGET_FPS_PER_CHANNEL 3000U
#define CAN_TX_THREAD_STACK_SIZE 1024U
#define CAN_TX_THREAD_PRIORITY 1
#define CAN_TX_RETRY_DELAY_US 50U
#define CAN_TX_PACE_SPIN_MARGIN_US 30U
#define SPI_CAN_STATE_FLAG_STARTED BIT(0)
#define SPI_CAN_STATE_FLAG_RX_PENDING BIT(1)

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
	struct spi_can_sync_bitrate_entry bitrates[2];
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
	struct spi_can_sync_state_entry states[2];
	struct spi_can_sync_can_entry rx_entries[SPI_CAN_SYNC_MAX_FRAMES];
	uint8_t reserved1[104];
};

BUILD_ASSERT(sizeof(struct spi_can_host_sync_frame) == SPI_CAN_SYNC_FRAME_SIZE,
	     "unexpected host sync frame size");
BUILD_ASSERT(sizeof(struct spi_can_slave_sync_frame) == SPI_CAN_SYNC_FRAME_SIZE,
	     "unexpected slave sync frame size");

struct slave_channel {
	uint8_t id;
	const struct device *can_dev;
	struct k_msgq rx_queue;
	struct k_msgq tx_queue;
	struct k_sem tx_done_sem;
	struct can_timing timing;
	struct can_bus_err_cnt err_cnt;
	enum can_state state;
	uint32_t bitrate;
	uint32_t tx_pace_error_us;
	int64_t next_tx_deadline_us;
	bool bitrate_valid;
	bool started;
	bool state_dirty;
	bool tx_pace_ready;
	uint8_t rx_storage[SPI_CAN_SYNC_RX_QUEUE_DEPTH * sizeof(struct can_frame)];
	uint8_t tx_storage[SPI_CAN_SYNC_TX_QUEUE_DEPTH * sizeof(struct can_frame)];
};

static const struct device *const spi_dev = DEVICE_DT_GET(SPI_NODE);
static const struct device *const can0_dev = DEVICE_DT_GET(CAN0_NODE);
static const struct device *const can1_dev = DEVICE_DT_GET(CAN1_NODE);
static const struct device *const gpio_b = DEVICE_DT_GET(GPIOB_NODE);

static struct spi_config spi_cfg = {
	.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPHA | SPI_OP_MODE_SLAVE,
	.frequency = 10000000,
};

static struct slave_channel channels[2] = {
	[0] = {
		.id = 0U,
		.can_dev = DEVICE_DT_GET(CAN0_NODE),
		.state = CAN_STATE_STOPPED,
	},
	[1] = {
		.id = 1U,
		.can_dev = can1_dev,
		.state = CAN_STATE_STOPPED,
	},
};

static uint8_t spi_tx_buf[SPI_CAN_SYNC_FRAME_SIZE] __aligned(4);
static uint8_t spi_rx_buf[SPI_CAN_SYNC_FRAME_SIZE] __aligned(4);
static uint8_t host_seq;
static uint8_t rr_channel;
static struct k_thread int_thread;
static struct k_thread can_tx_threads[ARRAY_SIZE(channels)];
K_KERNEL_STACK_DEFINE(int_stack, 1024);
K_THREAD_STACK_ARRAY_DEFINE(can_tx_stacks, ARRAY_SIZE(channels), CAN_TX_THREAD_STACK_SIZE);

BUILD_ASSERT(CAN_TX_TARGET_FPS_PER_CHANNEL > 0U, "CAN TX target fps must be non-zero");

static int64_t now_us(void)
{
	return (int64_t)k_cyc_to_us_floor64(k_cycle_get_64());
}

static void sleep_until_us(int64_t deadline_us)
{
	while (1) {
		int64_t remaining_us = deadline_us - now_us();

		if (remaining_us <= 0) {
			return;
		}

		if (remaining_us > CAN_TX_PACE_SPIN_MARGIN_US) {
			(void)k_usleep((int32_t)(remaining_us - CAN_TX_PACE_SPIN_MARGIN_US));
			continue;
		}

		k_busy_wait((uint32_t)remaining_us);
		return;
	}
}

static void channel_reset_tx_pacing(struct slave_channel *channel)
{
	channel->next_tx_deadline_us = now_us();
	channel->tx_pace_error_us = 0U;
	channel->tx_pace_ready = true;
}

static void channel_wait_tx_slot(struct slave_channel *channel)
{
	if (!channel->tx_pace_ready) {
		channel_reset_tx_pacing(channel);
	}

	sleep_until_us(channel->next_tx_deadline_us);
}

static void channel_commit_tx_slot(struct slave_channel *channel)
{
	channel->next_tx_deadline_us += (1000000U / CAN_TX_TARGET_FPS_PER_CHANNEL);
	channel->tx_pace_error_us += (1000000U % CAN_TX_TARGET_FPS_PER_CHANNEL);

	if (channel->tx_pace_error_us >= CAN_TX_TARGET_FPS_PER_CHANNEL) {
		channel->next_tx_deadline_us += 1;
		channel->tx_pace_error_us -= CAN_TX_TARGET_FPS_PER_CHANNEL;
	}

	if (channel->next_tx_deadline_us < now_us()) {
		channel->next_tx_deadline_us = now_us();
	}
}

static uint32_t unpack_bitrate_payload(const struct spi_can_sync_bitrate_entry *entry)
{
	return sys_get_le32(entry->bitrate);
}

static void pack_frame_entry(struct spi_can_sync_can_entry *entry, const struct can_frame *frame,
			     uint8_t channel)
{
	memset(entry, 0, sizeof(*entry));
	entry->id = frame->id;
	entry->dlc = frame->dlc;
	entry->flags = frame->flags;
	entry->channel = channel;
	memcpy(entry->data, frame->data, CAN_MAX_DLC);
}

static int unpack_frame_entry(const struct spi_can_sync_can_entry *entry, struct can_frame *frame,
			      uint8_t *channel)
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

static bool try_align_sync_frame(uint8_t *frame, size_t len)
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
			LOG_WRN_RATELIMIT_RATE(SLAVE_DEMO_LOG_INTERVAL_MS,
					       "realigned host sync frame by %u byte(s)",
					       (unsigned int)offset);
		return true;
	}

	return false;
}

static bool channel_available(const struct slave_channel *channel)
{
	return channel->can_dev != NULL && device_is_ready(channel->can_dev);
}

static uint16_t frame_seq_get(const struct can_frame *frame)
{
	if (frame->dlc < 4U) {
		return 0U;
	}

	return sys_get_le16(&frame->data[2]);
}

static void update_int_line(void)
{
	bool asserted = false;
	static bool last_asserted;

	for (uint8_t channel = 0U; channel < ARRAY_SIZE(channels); channel++) {
		if (k_msgq_num_used_get(&channels[channel].rx_queue) > 0U || channels[channel].state_dirty) {
			asserted = true;
			break;
		}
	}

	gpio_pin_set_raw(gpio_b, SLAVE_INT_PIN, asserted ? 1 : 0);

	if (asserted != last_asserted) {
		LOG_DBG("slave INT %s", asserted ? "assert" : "clear");
		last_asserted = asserted;
	}
}

static void int_refresh_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		update_int_line();
		k_sleep(K_USEC(SLAVE_INT_REFRESH_US));
	}
}

static void can_state_cb(const struct device *dev, enum can_state state,
			 struct can_bus_err_cnt err_cnt, void *user_data)
{
	struct slave_channel *channel = user_data;

	channel->state = state;
	channel->err_cnt = err_cnt;
	channel->state_dirty = true;
	update_int_line();

	LOG_INF_RATELIMIT_RATE(SLAVE_DEMO_LOG_INTERVAL_MS,
			       "CAN state dev=%s state=%d tx_err=%u rx_err=%u", dev->name,
			       state, err_cnt.tx_err_cnt, err_cnt.rx_err_cnt);
}

static void can_rx_cb(const struct device *dev, struct can_frame *frame, void *user_data)
{
	struct slave_channel *channel = user_data;
	int ret;

	ret = k_msgq_put(&channel->rx_queue, frame, K_NO_WAIT);
	if (ret < 0) {
		LOG_WRN_RATELIMIT_RATE(SLAVE_DEMO_LOG_INTERVAL_MS,
				       "CAN RX queue full, dropping id=0x%03x", frame->id);
		return;
	}

	update_int_line();
	LOG_DBG("CAN RX dev=%s id=0x%03x dlc=%u seq=%u queued=%u", dev->name, frame->id,
		frame->dlc, frame_seq_get(frame), k_msgq_num_used_get(&channel->rx_queue));
}

static void can_tx_cb(const struct device *dev, int error, void *user_data)
{
	struct slave_channel *channel = user_data;

	if (error < 0) {
		LOG_WRN_RATELIMIT_RATE(SLAVE_DEMO_LOG_INTERVAL_MS,
				       "CAN TX complete dev=%s channel=%u err=%d", dev->name,
				       channel->id, error);
	}

	k_sem_give(&channel->tx_done_sem);
}

static int channel_start_with_bitrate(struct slave_channel *channel, uint8_t channel_id,
				      uint32_t bitrate)
{
	struct can_timing timing;
	int ret;

	if (!channel_available(channel)) {
		return -ENODEV;
	}

	if (bitrate == 0U) {
		return -EINVAL;
	}

	if (channel->started && channel->bitrate_valid && channel->bitrate == bitrate) {
		return 0;
	}

	if (channel->started) {
		ret = can_stop(channel->can_dev);
		if (ret < 0 && ret != -EALREADY) {
			LOG_WRN_RATELIMIT_RATE(SLAVE_DEMO_LOG_INTERVAL_MS,
					       "channel=%u stop before reconfig failed ret=%d",
					       channel_id, ret);
			return ret;
		}

		channel->started = false;
	}

	ret = can_set_mode(channel->can_dev, CAN_MODE_NORMAL);
	if (ret < 0) {
		LOG_WRN_RATELIMIT_RATE(SLAVE_DEMO_LOG_INTERVAL_MS,
				       "channel=%u set mode failed ret=%d", channel_id, ret);
		return ret;
	}

	ret = can_calc_timing(channel->can_dev, &timing, bitrate, 0U);
	if (ret < 0) {
		LOG_WRN_RATELIMIT_RATE(SLAVE_DEMO_LOG_INTERVAL_MS,
				       "channel=%u timing calc failed bitrate=%u err=%d",
				       channel_id, bitrate, ret);
		return ret;
	}

	channel->timing = timing;
	ret = can_set_timing(channel->can_dev, &channel->timing);
	if (ret < 0) {
		LOG_WRN_RATELIMIT_RATE(SLAVE_DEMO_LOG_INTERVAL_MS,
				       "channel=%u set timing failed ret=%d", channel_id, ret);
		return ret;
	}

	ret = can_start(channel->can_dev);
	if (ret < 0 && ret != -EALREADY) {
		LOG_WRN_RATELIMIT_RATE(SLAVE_DEMO_LOG_INTERVAL_MS,
				       "channel=%u start failed ret=%d", channel_id, ret);
		return ret;
	}

	channel->bitrate = bitrate;
	channel->bitrate_valid = true;
	channel->started = true;
	channel->state_dirty = true;
	k_sem_reset(&channel->tx_done_sem);
	k_sem_give(&channel->tx_done_sem);
	channel_reset_tx_pacing(channel);

	LOG_INF("channel=%u ready dev=%s bitrate=%u sjw=%u prop=%u ph1=%u ph2=%u prescaler=%u",
		channel_id, channel->can_dev->name, bitrate, channel->timing.sjw,
		channel->timing.prop_seg, channel->timing.phase_seg1,
		channel->timing.phase_seg2, channel->timing.prescaler);

	return 0;
}

static int channel_apply_bitrate(struct slave_channel *channel, uint8_t channel_id,
				 const struct spi_can_sync_bitrate_entry *entry)
{
	uint32_t bitrate;

	bitrate = unpack_bitrate_payload(entry);
	if (bitrate == 0U) {
		return -EINVAL;
	}

	channel->bitrate = bitrate;
	channel->bitrate_valid = true;

	return channel_start_with_bitrate(channel, channel_id, bitrate);
}

static int channel_ensure_started(struct slave_channel *channel, uint8_t channel_id)
{
	if (channel->started) {
		return 0;
	}

	if (!channel->bitrate_valid) {
		LOG_WRN_RATELIMIT_RATE(SLAVE_DEMO_LOG_INTERVAL_MS,
				       "channel=%u has no bitrate yet", channel_id);
		return -ENETDOWN;
	}

	return channel_start_with_bitrate(channel, channel_id, channel->bitrate);
}

static int channel_send_can(struct slave_channel *channel, const struct can_frame *frame,
			    uint8_t channel_id)
{
	int ret;

	if (!channel_available(channel)) {
		return -ENODEV;
	}

	ret = channel_ensure_started(channel, channel_id);
	if (ret < 0) {
		return ret;
	}

	channel_wait_tx_slot(channel);

	ret = can_send(channel->can_dev, frame, K_NO_WAIT, can_tx_cb, channel);
	if (ret < 0) {
		LOG_WRN_RATELIMIT_RATE(SLAVE_DEMO_LOG_INTERVAL_MS,
				       "CAN TX enqueue failed channel=%u id=0x%03x seq=%u ret=%d",
				       channel_id, frame->id, frame_seq_get(frame), ret);
	} else {
		LOG_DBG("CAN TX enqueue channel=%u id=0x%03x dlc=%u seq=%u", channel_id, frame->id,
			frame->dlc, frame_seq_get(frame));
	}

	return ret;
}

static int channel_queue_can_tx(struct slave_channel *channel, const struct can_frame *frame,
				uint8_t channel_id)
{
	int ret;

	ret = k_msgq_put(&channel->tx_queue, frame, K_NO_WAIT);
	if (ret < 0) {
		LOG_WRN_RATELIMIT_RATE(SLAVE_DEMO_LOG_INTERVAL_MS,
				       "CAN TX queue full, dropping channel=%u id=0x%03x seq=%u",
				       channel_id, frame->id, frame_seq_get(frame));
		return ret;
	}

	LOG_DBG("CAN TX queued channel=%u id=0x%03x dlc=%u seq=%u queued=%u", channel_id,
		frame->id, frame->dlc, frame_seq_get(frame),
		k_msgq_num_used_get(&channel->tx_queue));

	return 0;
}

static void can_tx_thread(void *p1, void *p2, void *p3)
{
	struct slave_channel *channel = p1;
	uint8_t channel_id = POINTER_TO_UINT(p2);

	ARG_UNUSED(p3);

	while (1) {
		struct can_frame frame;
		int ret;

		k_msgq_get(&channel->tx_queue, &frame, K_FOREVER);

		while (1) {
			k_sem_take(&channel->tx_done_sem, K_FOREVER);
			ret = channel_send_can(channel, &frame, channel_id);
			if (ret == 0) {
				channel_commit_tx_slot(channel);
				break;
			}

			k_sem_give(&channel->tx_done_sem);

			if (ret == -EAGAIN || ret == -EBUSY) {
				k_sleep(K_USEC(CAN_TX_RETRY_DELAY_US));
				continue;
			}

			channel_reset_tx_pacing(channel);
			k_sleep(K_MSEC(1));
		}
	}
}

static void snapshot_state(struct slave_channel *channel, struct spi_can_sync_state_entry *entry)
{
	enum can_state state = CAN_STATE_STOPPED;
	struct can_bus_err_cnt err_cnt = {0};
	int ret;

	memset(entry, 0, sizeof(*entry));

	if (!channel_available(channel)) {
		entry->state = CAN_STATE_STOPPED;
		return;
	}

	state = channel->state;
	err_cnt = channel->err_cnt;

	ret = can_get_state(channel->can_dev, &state, &err_cnt);
	if (ret < 0) {
		LOG_WRN_RATELIMIT_RATE(SLAVE_DEMO_LOG_INTERVAL_MS,
				       "can_get_state failed (%d), using cached state", ret);
	}

	channel->state = state;
	channel->err_cnt = err_cnt;

	entry->state = (uint8_t)state;
	entry->tx_err = err_cnt.tx_err_cnt;
	entry->rx_err = err_cnt.rx_err_cnt;
	if (channel->started) {
		entry->flags |= SPI_CAN_STATE_FLAG_STARTED;
	}
	if (k_msgq_num_used_get(&channel->rx_queue) > 0U) {
		entry->flags |= SPI_CAN_STATE_FLAG_RX_PENDING;
	}

	channel->state_dirty = false;
}

static void build_slave_response(struct spi_can_slave_sync_frame *frame)
{
	memset(frame, 0, sizeof(*frame));
	sys_put_le32(SPI_CAN_SYNC_MAGIC, frame->magic);
	frame->version = SPI_CAN_SYNC_VERSION;
	frame->seq = host_seq;

	for (uint8_t channel = 0U; channel < ARRAY_SIZE(channels); channel++) {
		frame->state_mask |= BIT(channel);
		snapshot_state(&channels[channel], &frame->states[channel]);
	}

	while (frame->rx_count < SPI_CAN_SYNC_MAX_FRAMES) {
		bool found = false;

		for (uint8_t offset = 0U; offset < ARRAY_SIZE(channels); offset++) {
			uint8_t channel = (rr_channel + offset) % ARRAY_SIZE(channels);
			struct can_frame can_frame;

			if (k_msgq_get(&channels[channel].rx_queue, &can_frame, K_NO_WAIT) < 0) {
				continue;
			}

			pack_frame_entry(&frame->rx_entries[frame->rx_count], &can_frame, channel);
			frame->rx_count++;
			rr_channel = (channel + 1U) % ARRAY_SIZE(channels);
			found = true;
			break;
		}

		if (!found) {
			break;
		}
	}

	for (uint8_t channel = 0U; channel < ARRAY_SIZE(channels); channel++) {
		if (k_msgq_num_used_get(&channels[channel].rx_queue) > 0U) {
			frame->more_rx_mask |= BIT(channel);
		}
	}
}

static bool frame_is_empty(const uint8_t *buf, size_t len)
{
	for (size_t i = 0U; i < len; i++) {
		if (buf[i] != 0U) {
			return false;
		}
	}

	return true;
}

static void process_host_frame(uint8_t *raw_frame)
{
	const struct spi_can_host_sync_frame *frame =
		(const struct spi_can_host_sync_frame *)raw_frame;

	if (sys_get_le32(raw_frame) != SPI_CAN_SYNC_MAGIC) {
		(void)try_align_sync_frame(raw_frame, SPI_CAN_SYNC_FRAME_SIZE);
		frame = (const struct spi_can_host_sync_frame *)raw_frame;
	}

	if (frame_is_empty((const uint8_t *)frame, sizeof(*frame))) {
		return;
	}

	if (sys_get_le32(frame->magic) != SPI_CAN_SYNC_MAGIC || frame->version != SPI_CAN_SYNC_VERSION) {
			LOG_WRN_RATELIMIT_RATE(SLAVE_DEMO_LOG_INTERVAL_MS,
					       "drop bad sync req raw=%02x %02x %02x %02x",
					       frame->magic[0], frame->magic[1],
					       frame->magic[2], frame->magic[3]);
		return;
	}

	host_seq = frame->seq;

	for (uint8_t channel = 0U; channel < ARRAY_SIZE(channels); channel++) {
		if ((frame->bitrate_mask & BIT(channel)) != 0U) {
			(void)channel_apply_bitrate(&channels[channel], channel,
						    &frame->bitrates[channel]);
		}
	}

	for (size_t i = 0U; i < MIN((size_t)frame->tx_count, (size_t)SPI_CAN_SYNC_MAX_FRAMES); i++) {
		struct can_frame can_frame;
		uint8_t channel_id;
		int ret;

		ret = unpack_frame_entry(&frame->tx_entries[i], &can_frame, &channel_id);
		if (ret < 0) {
			LOG_WRN_RATELIMIT_RATE(SLAVE_DEMO_LOG_INTERVAL_MS,
					       "drop invalid host frame idx=%u (%d)",
					       (unsigned int)i, ret);
			continue;
		}

		if (channel_id >= ARRAY_SIZE(channels)) {
			LOG_WRN_RATELIMIT_RATE(SLAVE_DEMO_LOG_INTERVAL_MS,
					       "drop host frame for invalid channel %u",
					       channel_id);
			continue;
		}

		(void)channel_queue_can_tx(&channels[channel_id], &can_frame, channel_id);
	}

	update_int_line();
}

static int spi_bridge_init(void)
{
	int ret;

	if (!device_is_ready(spi_dev) || !device_is_ready(can0_dev) || !device_is_ready(can1_dev) ||
	    !device_is_ready(gpio_b)) {
		return -ENODEV;
	}

	for (uint8_t channel = 0U; channel < ARRAY_SIZE(channels); channel++) {
		k_msgq_init(&channels[channel].rx_queue, channels[channel].rx_storage,
			    sizeof(struct can_frame), SPI_CAN_SYNC_RX_QUEUE_DEPTH);
		k_msgq_init(&channels[channel].tx_queue, channels[channel].tx_storage,
			    sizeof(struct can_frame), SPI_CAN_SYNC_TX_QUEUE_DEPTH);
		k_sem_init(&channels[channel].tx_done_sem, 1, 1);
		channel_reset_tx_pacing(&channels[channel]);
	}

	ret = gpio_pin_configure(gpio_b, SLAVE_INT_PIN, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		return ret;
	}

	for (uint8_t channel = 0U; channel < ARRAY_SIZE(channels); channel++) {
		if (!channel_available(&channels[channel])) {
			continue;
		}

		ret = can_add_rx_filter(channels[channel].can_dev, can_rx_cb, &channels[channel],
					&(struct can_filter){ .id = 0U, .mask = 0U });
		if (ret < 0) {
			return ret;
		}

		can_set_state_change_callback(channels[channel].can_dev, can_state_cb, &channels[channel]);
	}

	k_thread_create(&int_thread, int_stack, K_KERNEL_STACK_SIZEOF(int_stack),
			int_refresh_thread, NULL, NULL, NULL, 0, 0, K_NO_WAIT);

	for (uint8_t channel = 0U; channel < ARRAY_SIZE(channels); channel++) {
		k_thread_create(&can_tx_threads[channel], can_tx_stacks[channel],
				K_THREAD_STACK_SIZEOF(can_tx_stacks[channel]), can_tx_thread,
				&channels[channel], UINT_TO_POINTER(channel), NULL,
				CAN_TX_THREAD_PRIORITY, 0, K_NO_WAIT);
		k_thread_name_set(&can_tx_threads[channel], channel == 0U ? "can1-tx" : "can2-tx");
	}

	LOG_INF("slave init done spi=%s can0=%s can1=%s int_out=PB%u refresh=%uus tx_pace=%ufps/ch",
		spi_dev->name, channels[0].can_dev->name, channels[1].can_dev->name,
		SLAVE_INT_PIN, SLAVE_INT_REFRESH_US, CAN_TX_TARGET_FPS_PER_CHANNEL);

	return 0;
}

int main(void)
{
	struct spi_buf tx_buf = {
		.buf = spi_tx_buf,
		.len = sizeof(spi_tx_buf),
	};
	struct spi_buf rx_buf = {
		.buf = spi_rx_buf,
		.len = sizeof(spi_rx_buf),
	};
	struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1U,
	};
	struct spi_buf_set rx = {
		.buffers = &rx_buf,
		.count = 1U,
	};
	int ret;

	ret = spi_bridge_init();
	if (ret < 0) {
		LOG_ERR("slave init failed (%d)", ret);
		return ret;
	}

	while (1) {
		build_slave_response((struct spi_can_slave_sync_frame *)spi_tx_buf);
		update_int_line();
		memset(spi_rx_buf, 0, sizeof(spi_rx_buf));

		ret = spi_transceive(spi_dev, &spi_cfg, &tx, &rx);
		if (ret == -ENOTSUP) {
			LOG_ERR("SPI slave mode not supported by driver");
			return ret;
		}

		if (ret < 0) {
			LOG_ERR_RATELIMIT_RATE(SLAVE_DEMO_LOG_INTERVAL_MS,
					       "SPI transceive failed (%d)", ret);
			k_sleep(K_MSEC(10));
			continue;
		}

		process_host_frame(spi_rx_buf);
	}
}
