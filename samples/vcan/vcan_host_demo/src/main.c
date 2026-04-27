#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(vcan_host_demo, LOG_LEVEL_INF);

#define HOST_DEMO_LOG_INTERVAL_MS 5000U

#define HOST_CAN0_NODE DT_NODELABEL(can1)
#define HOST_CAN1_NODE DT_NODELABEL(can2)
#define REMOTE_CAN0_NODE DT_NODELABEL(can_v0)
#define REMOTE_CAN1_NODE DT_NODELABEL(can_v1)

#define VCAN_CHANNEL_COUNT 2U
#define TEST_MAGIC 0xA5U
#define CAN_BITRATE_PER_CHANNEL 1000000U
#define CAN_STD_8B_FRAME_BITS 111U
#define CAN_SHARED_DIRECTIONS_PER_CHANNEL 2U
#define SPI_SYNC_MAX_FRAMES_PER_TRANSFER 4U
#define SPI_SYNC_POLL_US 500U
#define TARGET_FPS_OVERRIDE 3000U
#define RAW_TARGET_FPS_PER_DIRECTION                                                        \
	((((CAN_BITRATE_PER_CHANNEL / CAN_STD_8B_FRAME_BITS) /                              \
	   CAN_SHARED_DIRECTIONS_PER_CHANNEL) <                                            \
	  (((1000000U / SPI_SYNC_POLL_US) * SPI_SYNC_MAX_FRAMES_PER_TRANSFER) /            \
	   VCAN_CHANNEL_COUNT)) ?                                                          \
		 ((CAN_BITRATE_PER_CHANNEL / CAN_STD_8B_FRAME_BITS) /                        \
		  CAN_SHARED_DIRECTIONS_PER_CHANNEL) :                                       \
		 (((1000000U / SPI_SYNC_POLL_US) * SPI_SYNC_MAX_FRAMES_PER_TRANSFER) /       \
		  VCAN_CHANNEL_COUNT))
#define TARGET_FPS_PER_DIRECTION TARGET_FPS_OVERRIDE
#define TEST_PERIOD_MS 2U
#define TEST_GAP_MS 1U
#define TEST_TIMEOUT_MS 100U
#define SUMMARY_INTERVAL_MS 5000U
#define STRESS_FRAMES_PER_PERIOD_NUMERATOR (TARGET_FPS_PER_DIRECTION * TEST_PERIOD_MS)
#define STRESS_BATCH_BASE (STRESS_FRAMES_PER_PERIOD_NUMERATOR / 1000U)
#define STRESS_BATCH_PEAK DIV_ROUND_UP(STRESS_FRAMES_PER_PERIOD_NUMERATOR, 1000U)
#define PENDING_DEPTH 512U
#define TIMEOUT_LOG_BURST 8U
#define HOUSEKEEPING_PERIOD_MS 10U
#define WORKER_COUNT (VCAN_CHANNEL_COUNT * 2U)
#define WORKER_STACK_SIZE 1536U
#define WORKER_PRIORITY 4

#define CAN_IDEAL_MAX_FPS_PER_DIRECTION                                                     \
	((CAN_BITRATE_PER_CHANNEL / CAN_STD_8B_FRAME_BITS) /                                \
	 CAN_SHARED_DIRECTIONS_PER_CHANNEL)
#define CODE_MAX_FPS_PER_DIRECTION                                                           \
	(((1000000U / SPI_SYNC_POLL_US) * SPI_SYNC_MAX_FRAMES_PER_TRANSFER) /              \
	 VCAN_CHANNEL_COUNT)

BUILD_ASSERT(TEST_GAP_MS < TEST_PERIOD_MS, "TEST_GAP_MS must be smaller than TEST_PERIOD_MS");
BUILD_ASSERT(TARGET_FPS_PER_DIRECTION > 0U, "target fps must be non-zero");
BUILD_ASSERT(TARGET_FPS_PER_DIRECTION <= RAW_TARGET_FPS_PER_DIRECTION,
	     "target fps exceeds current stress capacity");

struct timeout_report {
	const char *name;
	uint16_t seq[TIMEOUT_LOG_BURST];
	size_t seq_count;
	size_t total_count;
};

struct pending_entry {
	bool active;
	uint16_t seq;
	int64_t sent_at_ms;
};

struct pending_case {
	const char *name;
	struct pending_entry entries[PENDING_DEPTH];
};

struct channel_stats {
	const struct device *host_can;
	const struct device *remote_can;
	const char *name;
	uint32_t host_to_remote_id;
	uint32_t remote_to_host_id;
	uint8_t host_to_remote_tag;
	uint8_t remote_to_host_tag;
	struct pending_case host_to_remote;
	struct pending_case remote_to_host;
	uint32_t host_tx_count;
	uint32_t remote_tx_count;
	uint32_t host_to_remote_pass;
	uint32_t remote_to_host_pass;
	uint32_t host_to_remote_timeout;
	uint32_t remote_to_host_timeout;
	uint32_t host_send_err;
	uint32_t remote_send_err;
	uint32_t host_backpressure;
	uint32_t remote_backpressure;
	uint32_t unexpected_host_rx;
	uint32_t unexpected_remote_rx;
};

struct endpoint_ctx {
	uint8_t channel;
	const char *name;
};

struct sender_worker {
	uint8_t channel;
	bool remote_to_host;
	const char *name;
	uint16_t next_seq;
	int64_t start_delay_ms;
};

struct demo_stats {
	struct k_spinlock lock;
	struct channel_stats channels[VCAN_CHANNEL_COUNT];
};

struct summary_snapshot {
	uint32_t host_tx_count;
	uint32_t remote_tx_count;
	uint32_t host_to_remote_pass;
	uint32_t remote_to_host_pass;
	uint32_t host_to_remote_timeout;
	uint32_t remote_to_host_timeout;
	uint32_t host_send_err;
	uint32_t remote_send_err;
	uint32_t host_backpressure;
	uint32_t remote_backpressure;
	uint32_t unexpected_host_rx;
	uint32_t unexpected_remote_rx;
};

static struct demo_stats stats = {
	.channels =
		{
			[0] =
				{
					.host_can = DEVICE_DT_GET(HOST_CAN0_NODE),
					.remote_can = DEVICE_DT_GET(REMOTE_CAN0_NODE),
					.name = "channel0(can1<->vcan0)",
					.host_to_remote_id = 0x120,
					.remote_to_host_id = 0x220,
					.host_to_remote_tag = 0x10,
					.remote_to_host_tag = 0x20,
					.host_to_remote = { .name = "can1->vcan0" },
					.remote_to_host = { .name = "vcan0->can1" },
				},
			[1] =
				{
					.host_can = DEVICE_DT_GET(HOST_CAN1_NODE),
					.remote_can = DEVICE_DT_GET(REMOTE_CAN1_NODE),
					.name = "channel1(can2<->vcan1)",
					.host_to_remote_id = 0x121,
					.remote_to_host_id = 0x221,
					.host_to_remote_tag = 0x11,
					.remote_to_host_tag = 0x21,
					.host_to_remote = { .name = "can2->vcan1" },
					.remote_to_host = { .name = "vcan1->can2" },
				},
		},
};

static const struct endpoint_ctx host_ctx[VCAN_CHANNEL_COUNT] = {
	[0] = {
		.channel = 0U,
		.name = "host_can1",
	},
	[1] = {
		.channel = 1U,
		.name = "host_can2",
	},
};

static const struct endpoint_ctx remote_ctx[VCAN_CHANNEL_COUNT] = {
	[0] = {
		.channel = 0U,
		.name = "vcan0",
	},
	[1] = {
		.channel = 1U,
		.name = "vcan1",
	},
};

static struct sender_worker workers[WORKER_COUNT] = {
	[0] = {
		.channel = 0U,
		.remote_to_host = false,
		.name = "can1->vcan0",
		.next_seq = 1U,
		.start_delay_ms = 0,
	},
	[1] = {
		.channel = 1U,
		.remote_to_host = false,
		.name = "can2->vcan1",
		.next_seq = 1U,
		.start_delay_ms = 0,
	},
	[2] = {
		.channel = 0U,
		.remote_to_host = true,
		.name = "vcan0->can1",
		.next_seq = 1U,
		.start_delay_ms = TEST_GAP_MS,
	},
	[3] = {
		.channel = 1U,
		.remote_to_host = true,
		.name = "vcan1->can2",
		.next_seq = 1U,
		.start_delay_ms = TEST_GAP_MS,
	},
};

K_THREAD_STACK_ARRAY_DEFINE(worker_stacks, WORKER_COUNT, WORKER_STACK_SIZE);
static struct k_thread worker_threads[WORKER_COUNT];

static void sleep_until_ms(int64_t deadline_ms)
{
	int64_t now = k_uptime_get();

	if (deadline_ms <= now) {
		return;
	}

	k_sleep(K_MSEC((uint32_t)(deadline_ms - now)));
}

static const char *can_state_name(enum can_state state)
{
	switch (state) {
	case CAN_STATE_ERROR_ACTIVE:
		return "error-active";
	case CAN_STATE_ERROR_WARNING:
		return "error-warning";
	case CAN_STATE_ERROR_PASSIVE:
		return "error-passive";
	case CAN_STATE_BUS_OFF:
		return "bus-off";
	case CAN_STATE_STOPPED:
		return "stopped";
	default:
		return "unknown";
	}
}

static uint16_t frame_seq_get(const struct can_frame *frame)
{
	if (frame->dlc < 4U) {
		return 0U;
	}

	return sys_get_le16(&frame->data[2]);
}

static void frame_fill(struct can_frame *frame, uint32_t id, uint8_t tag, uint8_t channel,
		       uint16_t seq)
{
	memset(frame, 0, sizeof(*frame));
	frame->id = id;
	frame->dlc = 8U;
	frame->data[0] = TEST_MAGIC;
	frame->data[1] = tag;
	sys_put_le16(seq, &frame->data[2]);
	frame->data[4] = channel;
	frame->data[5] = 0x11U + channel;
	frame->data[6] = 0x22U + channel;
	frame->data[7] = 0x33U + channel;
}

static bool frame_has_signature(const struct can_frame *frame, uint32_t id, uint8_t tag,
				uint8_t channel)
{
	return frame->id == id && frame->dlc == 8U && frame->data[0] == TEST_MAGIC &&
	       frame->data[1] == tag && frame->data[4] == channel;
}

static const char *can_err_name(int err)
{
	switch (err) {
	case 0:
		return "OK";
	case -EAGAIN:
		return "EAGAIN";
	case -EIO:
		return "EIO";
	case -ENETDOWN:
		return "ENETDOWN";
	case -ENODEV:
		return "ENODEV";
	case -ENOSPC:
		return "ENOSPC";
	case -EINVAL:
		return "EINVAL";
	case -EBUSY:
		return "EBUSY";
	default:
		return "UNKNOWN";
	}
}

static int pending_add(struct pending_case *pending, uint16_t seq, int64_t sent_at_ms)
{
	for (size_t i = 0U; i < ARRAY_SIZE(pending->entries); i++) {
		if (pending->entries[i].active) {
			continue;
		}

		pending->entries[i].active = true;
		pending->entries[i].seq = seq;
		pending->entries[i].sent_at_ms = sent_at_ms;
		return 0;
	}

	return -ENOSPC;
}

static bool pending_complete(struct pending_case *pending, uint16_t seq)
{
	for (size_t i = 0U; i < ARRAY_SIZE(pending->entries); i++) {
		if (!pending->entries[i].active || pending->entries[i].seq != seq) {
			continue;
		}

		pending->entries[i].active = false;
		return true;
	}

	return false;
}

static bool pending_remove(struct pending_case *pending, uint16_t seq)
{
	for (size_t i = 0U; i < ARRAY_SIZE(pending->entries); i++) {
		if (!pending->entries[i].active || pending->entries[i].seq != seq) {
			continue;
		}

		pending->entries[i].active = false;
		return true;
	}

	return false;
}

static uint8_t pending_count(const struct pending_case *pending)
{
	uint8_t used = 0U;

	for (size_t i = 0U; i < ARRAY_SIZE(pending->entries); i++) {
		if (pending->entries[i].active) {
			used++;
		}
	}

	return used;
}

static size_t pending_collect_timeouts(struct pending_case *pending, int64_t now_ms,
				       uint16_t *expired_seq, size_t expired_seq_len)
{
	size_t expired = 0U;

	for (size_t i = 0U; i < ARRAY_SIZE(pending->entries); i++) {
		if (!pending->entries[i].active) {
			continue;
		}

		if ((now_ms - pending->entries[i].sent_at_ms) < TEST_TIMEOUT_MS) {
			continue;
		}

		if (expired < expired_seq_len) {
			expired_seq[expired] = pending->entries[i].seq;
		}

		pending->entries[i].active = false;
		expired++;
	}

	return expired;
}

static void tx_callback(const struct device *dev, int error, void *user_data)
{
	const char *name = user_data;

	if (error < 0) {
		LOG_WRN_RATELIMIT_RATE(HOST_DEMO_LOG_INTERVAL_MS,
				       "%s TX callback dev=%s err=%d(%s)", name, dev->name,
				       error, can_err_name(error));
	}
}

static void state_callback(const struct device *dev, enum can_state state,
			   struct can_bus_err_cnt err_cnt, void *user_data)
{
	const struct endpoint_ctx *ctx = user_data;

	if (state == CAN_STATE_ERROR_ACTIVE && err_cnt.tx_err_cnt == 0U && err_cnt.rx_err_cnt == 0U) {
		return;
	}

	LOG_WRN_RATELIMIT_RATE(HOST_DEMO_LOG_INTERVAL_MS,
			       "%s state dev=%s state=%s(%d) tx_err=%u rx_err=%u", ctx->name,
			       dev->name, can_state_name(state), state, err_cnt.tx_err_cnt,
			       err_cnt.rx_err_cnt);
}

static void check_timeout_case_locked(struct pending_case *pending, uint32_t *timeout_counter,
				      struct timeout_report *report)
{
	int64_t now = k_uptime_get();
	size_t expired_count;

	report->name = pending->name;
	report->seq_count = 0U;
	report->total_count = 0U;

	expired_count = pending_collect_timeouts(pending, now, report->seq, ARRAY_SIZE(report->seq));
	if (expired_count == 0U) {
		return;
	}

	*timeout_counter += expired_count;
	report->total_count = expired_count;
	report->seq_count = MIN(expired_count, ARRAY_SIZE(report->seq));
}

static void check_timeouts(void)
{
	struct timeout_report reports[VCAN_CHANNEL_COUNT * 2U];
	size_t report_count = 0U;
	k_spinlock_key_t key = k_spin_lock(&stats.lock);

	for (uint8_t channel = 0U; channel < VCAN_CHANNEL_COUNT; channel++) {
		struct channel_stats *channel_stats = &stats.channels[channel];

		check_timeout_case_locked(&channel_stats->host_to_remote,
					  &channel_stats->host_to_remote_timeout,
					  &reports[report_count++]);
		check_timeout_case_locked(&channel_stats->remote_to_host,
					  &channel_stats->remote_to_host_timeout,
					  &reports[report_count++]);
	}

	k_spin_unlock(&stats.lock, key);

	for (size_t i = 0U; i < report_count; i++) {
		if (reports[i].total_count == 0U) {
			continue;
		}

		LOG_ERR_RATELIMIT_RATE(HOST_DEMO_LOG_INTERVAL_MS,
				       "%s timeout count=%u first_seq=%u sample_count=%u timeout=%u ms",
				       reports[i].name, (unsigned int)reports[i].total_count,
				       reports[i].seq_count > 0U ? reports[i].seq[0] : 0U,
				       (unsigned int)reports[i].seq_count, TEST_TIMEOUT_MS);
	}
}

static void summary_log(void)
{
	static struct summary_snapshot previous[VCAN_CHANNEL_COUNT];

	for (uint8_t channel = 0U; channel < VCAN_CHANNEL_COUNT; channel++) {
		const char *name;
		struct summary_snapshot current;
		struct summary_snapshot delta;
		uint8_t pending_h2r;
		uint8_t pending_r2h;
		k_spinlock_key_t key = k_spin_lock(&stats.lock);
		struct channel_stats *channel_stats = &stats.channels[channel];

		name = channel_stats->name;
		current.host_tx_count = channel_stats->host_tx_count;
		current.remote_tx_count = channel_stats->remote_tx_count;
		current.host_to_remote_pass = channel_stats->host_to_remote_pass;
		current.remote_to_host_pass = channel_stats->remote_to_host_pass;
		current.host_to_remote_timeout = channel_stats->host_to_remote_timeout;
		current.remote_to_host_timeout = channel_stats->remote_to_host_timeout;
		current.host_send_err = channel_stats->host_send_err;
		current.remote_send_err = channel_stats->remote_send_err;
		current.host_backpressure = channel_stats->host_backpressure;
		current.remote_backpressure = channel_stats->remote_backpressure;
		current.unexpected_host_rx = channel_stats->unexpected_host_rx;
		current.unexpected_remote_rx = channel_stats->unexpected_remote_rx;
		pending_h2r = pending_count(&channel_stats->host_to_remote);
		pending_r2h = pending_count(&channel_stats->remote_to_host);
		k_spin_unlock(&stats.lock, key);

		delta.host_tx_count = current.host_tx_count - previous[channel].host_tx_count;
		delta.remote_tx_count = current.remote_tx_count - previous[channel].remote_tx_count;
		delta.host_to_remote_pass =
			current.host_to_remote_pass - previous[channel].host_to_remote_pass;
		delta.remote_to_host_pass =
			current.remote_to_host_pass - previous[channel].remote_to_host_pass;
		delta.host_to_remote_timeout =
			current.host_to_remote_timeout - previous[channel].host_to_remote_timeout;
		delta.remote_to_host_timeout =
			current.remote_to_host_timeout - previous[channel].remote_to_host_timeout;
		delta.host_send_err = current.host_send_err - previous[channel].host_send_err;
		delta.remote_send_err = current.remote_send_err - previous[channel].remote_send_err;
		delta.host_backpressure =
			current.host_backpressure - previous[channel].host_backpressure;
		delta.remote_backpressure =
			current.remote_backpressure - previous[channel].remote_backpressure;
		delta.unexpected_host_rx =
			current.unexpected_host_rx - previous[channel].unexpected_host_rx;
		delta.unexpected_remote_rx =
			current.unexpected_remote_rx - previous[channel].unexpected_remote_rx;
		previous[channel] = current;

		LOG_INF("%s %us tx[h2r=%u r2h=%u] ok[h2r=%u r2h=%u] err[send=%u timeout=%u unexpected=%u bp=%u] pending[h2r=%u r2h=%u] total_ok[h2r=%u r2h=%u]",
			name, SUMMARY_INTERVAL_MS / 1000U, delta.host_tx_count,
			delta.remote_tx_count, delta.host_to_remote_pass,
			delta.remote_to_host_pass, delta.host_send_err + delta.remote_send_err,
			delta.host_to_remote_timeout + delta.remote_to_host_timeout,
			delta.unexpected_host_rx + delta.unexpected_remote_rx,
			delta.host_backpressure + delta.remote_backpressure, pending_h2r, pending_r2h,
			current.host_to_remote_pass, current.remote_to_host_pass);
	}
}

static void remote_rx_callback(const struct device *dev, struct can_frame *frame, void *user_data)
{
	const struct endpoint_ctx *ctx = user_data;
	struct channel_stats *channel_stats = &stats.channels[ctx->channel];
	uint16_t seq = frame_seq_get(frame);
	const char *channel_name;
	bool matched = false;

	k_spinlock_key_t key = k_spin_lock(&stats.lock);
	channel_name = channel_stats->name;

	if (frame_has_signature(frame, channel_stats->host_to_remote_id,
				channel_stats->host_to_remote_tag, ctx->channel)) {
		matched = pending_complete(&channel_stats->host_to_remote, seq);
	}

	if (matched) {
		channel_stats->host_to_remote_pass++;
	} else {
		channel_stats->unexpected_remote_rx++;
	}

	k_spin_unlock(&stats.lock, key);

	if (!matched) {
		LOG_WRN_RATELIMIT_RATE(HOST_DEMO_LOG_INTERVAL_MS,
				       "%s unexpected remote RX dev=%s id=0x%03x seq=%u",
				       channel_name, dev->name, frame->id, seq);
	}
}

static void host_rx_callback(const struct device *dev, struct can_frame *frame, void *user_data)
{
	const struct endpoint_ctx *ctx = user_data;
	struct channel_stats *channel_stats = &stats.channels[ctx->channel];
	uint16_t seq = frame_seq_get(frame);
	const char *channel_name;
	bool matched = false;

	k_spinlock_key_t key = k_spin_lock(&stats.lock);
	channel_name = channel_stats->name;

	if (frame_has_signature(frame, channel_stats->remote_to_host_id,
				channel_stats->remote_to_host_tag, ctx->channel)) {
		matched = pending_complete(&channel_stats->remote_to_host, seq);
	}

	if (matched) {
		channel_stats->remote_to_host_pass++;
	} else {
		channel_stats->unexpected_host_rx++;
	}

	k_spin_unlock(&stats.lock, key);

	if (!matched) {
		LOG_WRN_RATELIMIT_RATE(HOST_DEMO_LOG_INTERVAL_MS,
				       "%s unexpected host RX dev=%s id=0x%03x seq=%u",
				       channel_name, dev->name, frame->id, seq);
	}
}

static int can_prepare(const struct device *dev, can_rx_callback_t callback, void *cb_user_data,
		       can_state_change_callback_t state_cb, void *state_user_data)
{
	struct can_filter filter = {
		.id = 0U,
		.mask = 0U,
	};
	int filter_id;
	int ret;

	if (!device_is_ready(dev)) {
		LOG_ERR("device %s not ready", dev->name);
		return -ENODEV;
	}

	filter_id = can_add_rx_filter(dev, callback, cb_user_data, &filter);
	if (filter_id < 0) {
		LOG_ERR("failed to add RX filter on %s (%d)", dev->name, filter_id);
		return filter_id;
	}

	can_set_state_change_callback(dev, state_cb, state_user_data);

	ret = can_start(dev);
	if (ret < 0 && ret != -EALREADY) {
		LOG_ERR("failed to start %s (%d)", dev->name, ret);
		return ret;
	}

	return 0;
}

static int send_host_can(uint8_t channel, uint16_t seq)
{
	struct channel_stats *channel_stats = &stats.channels[channel];
	struct can_frame frame;
	int64_t now = k_uptime_get();
	int ret;

	k_spinlock_key_t key = k_spin_lock(&stats.lock);

	ret = pending_add(&channel_stats->host_to_remote, seq, now);
	if (ret < 0) {
		channel_stats->host_backpressure++;
		k_spin_unlock(&stats.lock, key);
		return -EAGAIN;
	}
	k_spin_unlock(&stats.lock, key);

	frame_fill(&frame, channel_stats->host_to_remote_id, channel_stats->host_to_remote_tag,
		   channel, seq);

	ret = can_send(channel_stats->host_can, &frame, K_MSEC(100), tx_callback,
		       (void *)host_ctx[channel].name);
	if (ret < 0) {
		key = k_spin_lock(&stats.lock);

		(void)pending_remove(&channel_stats->host_to_remote, seq);
		channel_stats->host_send_err++;
		k_spin_unlock(&stats.lock, key);
		LOG_ERR_RATELIMIT_RATE(HOST_DEMO_LOG_INTERVAL_MS,
				       "%s host send failed seq=%u ret=%d(%s)",
				       channel_stats->name, seq, ret, can_err_name(ret));
		return ret;
	}

	key = k_spin_lock(&stats.lock);

	channel_stats->host_tx_count++;
	k_spin_unlock(&stats.lock, key);

	return 0;
}

static int send_remote_can(uint8_t channel, uint16_t seq)
{
	struct channel_stats *channel_stats = &stats.channels[channel];
	struct can_frame frame;
	int64_t now = k_uptime_get();
	int ret;

	k_spinlock_key_t key = k_spin_lock(&stats.lock);

	ret = pending_add(&channel_stats->remote_to_host, seq, now);
	if (ret < 0) {
		channel_stats->remote_backpressure++;
		k_spin_unlock(&stats.lock, key);
		return -EAGAIN;
	}
	k_spin_unlock(&stats.lock, key);

	frame_fill(&frame, channel_stats->remote_to_host_id, channel_stats->remote_to_host_tag,
		   channel, seq);

	ret = can_send(channel_stats->remote_can, &frame, K_MSEC(100), tx_callback,
		       (void *)remote_ctx[channel].name);
	if (ret < 0) {
		key = k_spin_lock(&stats.lock);

		(void)pending_remove(&channel_stats->remote_to_host, seq);
		channel_stats->remote_send_err++;
		k_spin_unlock(&stats.lock, key);
		LOG_ERR_RATELIMIT_RATE(HOST_DEMO_LOG_INTERVAL_MS,
				       "%s remote send failed seq=%u ret=%d(%s)",
				       channel_stats->name, seq, ret, can_err_name(ret));
		return ret;
	}

	key = k_spin_lock(&stats.lock);

	channel_stats->remote_tx_count++;
	k_spin_unlock(&stats.lock, key);

	return 0;
}

static void sender_worker_thread(void *p1, void *p2, void *p3)
{
	struct sender_worker *worker = p1;
	int64_t cycle_start_ms = k_uptime_get() + worker->start_delay_ms;
	uint32_t frame_credit = 0U;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	sleep_until_ms(cycle_start_ms);

	while (1) {
		uint32_t burst_count;

		frame_credit += STRESS_FRAMES_PER_PERIOD_NUMERATOR;
		burst_count = frame_credit / 1000U;
		frame_credit %= 1000U;

		for (uint32_t burst = 0U; burst < burst_count; burst++) {
			if (worker->remote_to_host) {
				(void)send_remote_can(worker->channel, worker->next_seq++);
			} else {
				(void)send_host_can(worker->channel, worker->next_seq++);
			}
		}

		cycle_start_ms += TEST_PERIOD_MS;
		if (cycle_start_ms < k_uptime_get()) {
			cycle_start_ms = k_uptime_get();
		}

		sleep_until_ms(cycle_start_ms);
	}
}

int main(void)
{
	int64_t last_summary_ms;
	int ret;

	LOG_INF("host stress cap can_ideal=%ufps/dir/ch code=%ufps/dir/ch applied=%ufps/dir/ch",
		CAN_IDEAL_MAX_FPS_PER_DIRECTION, CODE_MAX_FPS_PER_DIRECTION,
		TARGET_FPS_PER_DIRECTION);
	if (STRESS_BATCH_BASE == STRESS_BATCH_PEAK) {
		LOG_INF("host stress start mode=4workers target=%ufps/dir/ch period=%ums gap=%ums batch=%u timeout=%ums pending=%u",
			TARGET_FPS_PER_DIRECTION, TEST_PERIOD_MS, TEST_GAP_MS,
			STRESS_BATCH_BASE, TEST_TIMEOUT_MS, PENDING_DEPTH);
	} else {
		LOG_INF("host stress start mode=4workers target=%ufps/dir/ch period=%ums gap=%ums batch=%u..%u timeout=%ums pending=%u",
			TARGET_FPS_PER_DIRECTION, TEST_PERIOD_MS, TEST_GAP_MS,
			STRESS_BATCH_BASE, STRESS_BATCH_PEAK, TEST_TIMEOUT_MS,
			PENDING_DEPTH);
	}
	LOG_INF("host paths h2r:[can1->vcan0 can2->vcan1] r2h:[vcan0->can1 vcan1->can2]");

	for (uint8_t channel = 0U; channel < VCAN_CHANNEL_COUNT; channel++) {
		ret = can_prepare(stats.channels[channel].remote_can, remote_rx_callback,
				  (void *)&remote_ctx[channel], state_callback,
				  (void *)&remote_ctx[channel]);
		if (ret < 0) {
			return ret;
		}

		ret = can_prepare(stats.channels[channel].host_can, host_rx_callback,
				  (void *)&host_ctx[channel], state_callback,
				  (void *)&host_ctx[channel]);
		if (ret < 0) {
			return ret;
		}
	}

	for (uint8_t i = 0U; i < ARRAY_SIZE(workers); i++) {
		k_thread_create(&worker_threads[i], worker_stacks[i], WORKER_STACK_SIZE,
				sender_worker_thread, &workers[i], NULL, NULL,
				WORKER_PRIORITY, 0, K_NO_WAIT);
		k_thread_name_set(&worker_threads[i], workers[i].name);
	}

	last_summary_ms = k_uptime_get();

	while (1) {
		check_timeouts();
		if ((k_uptime_get() - last_summary_ms) >= SUMMARY_INTERVAL_MS) {
			check_timeouts();
			summary_log();
			last_summary_ms = k_uptime_get();
		}

		k_sleep(K_MSEC(HOUSEKEEPING_PERIOD_MS));
	}
}
