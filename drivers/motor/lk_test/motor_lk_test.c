#include "motor_lk_test.h"

#include <errno.h>
#include <math.h>
#include <string.h>

#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#define DT_DRV_COMPAT lk_test_motor

LOG_MODULE_REGISTER(motor_lk_test, CONFIG_MOTOR_LOG_LEVEL);

/* ── Lifecycle helpers ─────────────────────────────────────────────── */

static const char *mode_name(enum motor_mode mode)
{
	switch (mode) {
	case ML_ANGLE:
		return "angle";
	case ML_SPEED:
		return "speed";
	case ML_TORQUE:
		return "torque";
	case MIT:
		return "mit";
	case PV:
		return "pv";
	case VO:
		return "vo";
	case HYBRID:
		return "hybrid";
	default:
		return "unknown";
	}
}

static const char *cmd_name(enum motor_cmd cmd)
{
	switch (cmd) {
	case ENABLE_MOTOR:
		return "enable";
	case DISABLE_MOTOR:
		return "disable";
	case SET_ZERO:
		return "set-zero";
	case CLEAR_PID:
		return "clear-pid";
	case CLEAR_ERROR:
		return "clear-error";
	default:
		return "unsupported";
	}
}

static const char *pid_slot_name(enum lk_test_pid_slot slot)
{
	switch (slot) {
	case LK_TEST_PID_ANGLE:
		return "angle";
	case LK_TEST_PID_SPEED:
		return "speed";
	case LK_TEST_PID_TORQUE:
		return "torque";
	default:
		return "unknown";
	}
}

static bool lk_test_command_tracks_reply(uint8_t cmd)
{
	switch (cmd) {
	case LK_TEST_CMD_READ_STAT1:
	case LK_TEST_CMD_READ_STAT:
	case LK_TEST_CMD_CLEAR_ERR:
	case LK_TEST_CMD_MOTOR_OFF:
	case LK_TEST_CMD_MOTOR_RUN:
	case LK_TEST_CMD_SET_ZERO:
	case LK_TEST_SET_PARAM:
		return true;
	default:
		return false;
	}
}

static void log_target(const struct device *dev, const char *prefix,
		       const struct lk_test_target_state *target)
{
	LOG_INF("%s %s target mode=%s angle=%.2fdeg speed=%.2fdps torque=%.3fNm "
		"limit_speed=%.2fdps limit_torque=%.3fNm",
		dev->name, prefix, mode_name(target->mode), (double)target->angle_deg,
		(double)target->speed_dps, (double)target->torque,
		(double)target->limit_speed_dps, (double)target->limit_torque);
}

static void lifecycle_init(struct lk_test_motor_data *data)
{
	data->target.mode = ML_ANGLE;
	data->target.limit_speed_dps = 8000.0f;
	data->target.limit_torque = 3.0f;
}

static int lifecycle_apply_target(struct lk_test_motor_data *data,
				  const motor_status_t *status)
{
	switch (status->mode) {
	case ML_ANGLE:
		data->target.mode = ML_ANGLE;
		data->target.angle_deg = status->angle;
		data->target.limit_speed_dps = (status->speed_limit[0] == 0.0f)
						       ? 8000.0f
						       : fabsf(status->speed_limit[0] * 6.0f);
		break;
	case ML_SPEED:
		data->target.mode = ML_SPEED;
		data->target.speed_dps = status->rpm * 6.0f;
		data->target.limit_torque =
			(status->torque_limit[0] == 0.0f) ? 3.0f : fabsf(status->torque_limit[0]);
		break;
	case ML_TORQUE:
		data->target.mode = ML_TORQUE;
		data->target.torque = status->torque;
		break;
	default:
		return -ENOTSUP;
	}

	data->common.mode = data->target.mode;
	data->comm.target_valid = true;
	return 0;
}

static void lifecycle_export_status(const struct lk_test_motor_data *data,
				    motor_status_t *status)
{
	status->angle = data->common.angle;
	status->rpm = data->common.rpm;
	status->torque = data->common.torque;
	status->temperature = data->common.temperature;
	status->mode = data->common.mode;
	status->speed_limit[0] = data->target.limit_speed_dps / 6.0f;
	status->torque_limit[0] = data->target.limit_torque;
}

static void lifecycle_apply_feedback(struct lk_test_motor_data *data)
{
	data->common.temperature = data->feedback.temperature;
	data->common.torque = data->feedback.torque;
	data->common.rpm = data->feedback.rpm;
	data->common.angle = data->feedback.angle_deg;
	data->comm.feedback_seen = true;

	if (!data->comm.target_valid) {
		data->target.mode = ML_ANGLE;
		data->target.angle_deg = data->feedback.angle_deg;
		data->target.speed_dps = 0.0f;
		data->target.torque = 0.0f;
		data->target.limit_speed_dps = 600.0f;
		data->target.limit_torque = 1.0f;
		data->comm.target_valid = true;
	}
}

static void lifecycle_on_rx(const struct device *dev,
			     struct lk_test_motor_data *data,
			     const struct can_frame *frame)
{
	const struct lk_test_motor_cfg *cfg = dev->config;

	if (lk_test_codec_is_status1_frame(frame)) {
		data->feedback.raw_temp = (int8_t)frame->data[1];
		data->feedback.temperature = (float)data->feedback.raw_temp;
		data->feedback.err = frame->data[7];
		data->feedback.last_rx_ms = k_uptime_get_32();
		data->comm.missed_count = 0U;
		if (!data->comm.online) {
			data->comm.online = true;
			data->comm.reconnect_pending = true;
			LOG_INF("%s online (feedback restored)", dev->name);
		}
		if (frame->data[7] != 0U) {
			LOG_WRN("%s status1 cmd=0x%02x error=0x%02x", dev->name,
				frame->data[0], frame->data[7]);
		} else {
			LOG_DBG("%s status1 ok cmd=0x%02x temp=%d", dev->name,
				frame->data[0], frame->data[1]);
		}
		return;
	}

	if (lk_test_codec_is_echo_frame(frame)) {
		data->feedback.last_rx_ms = k_uptime_get_32();
		data->comm.missed_count = 0U;
		if (!data->comm.online) {
			data->comm.online = true;
			data->comm.reconnect_pending = true;
			LOG_INF("%s online (feedback restored)", dev->name);
		}
		LOG_DBG("%s echo cmd=0x%02x", dev->name, frame->data[0]);
		return;
	}

	if (!lk_test_codec_is_feedback_frame(frame)) {
		LOG_DBG("%s ignore unsupported lk reply cmd=0x%02x", dev->name,
			frame->data[0]);
		return;
	}

	lk_test_codec_parse_feedback(frame, &data->feedback);
	data->feedback.last_rx_ms = k_uptime_get_32();
	data->comm.missed_count = 0U;
	lifecycle_apply_feedback(data);
	if (cfg->trace_lifecycle) {
		LOG_INF("%s fb cmd=0x%02x angle=%.2f rpm=%.2f torque=%.3f temp=%.1f "
			"raw={%d,%d,%u}",
			dev->name, frame->data[0], (double)data->feedback.angle_deg,
			(double)data->feedback.rpm, (double)data->feedback.torque,
			(double)data->feedback.temperature, data->feedback.raw_temp,
			data->feedback.raw_speed, data->feedback.raw_encoder);
	}

	if (!data->comm.online) {
		data->comm.online = true;
		data->comm.reconnect_pending = true;
		LOG_INF("%s online (feedback restored)", dev->name);
	}
	if (data->comm.target_valid &&
	    (fabsf(data->target.angle_deg - data->feedback.angle_deg) < 0.01f)) {
		LOG_INF("%s first feedback confirms hold target %.2fdeg", dev->name,
			(double)data->target.angle_deg);
	}
}

static void lifecycle_mark_enabled(struct lk_test_motor_data *data, bool enabled)
{
	data->comm.enabled = enabled;
	if (!enabled) {
		data->comm.online = false;
		data->comm.last_recovery_tx_ms = 0U;
	}
}

static void lifecycle_clear_pid(struct lk_test_motor_data *data)
{
	memset(data->params, 0, sizeof(data->params));
	memset(data->pid_dirty, true, sizeof(data->pid_dirty));
}

static int pid_slot_from_capability(const char *cap_name)
{
	if (strcmp(cap_name, "angle") == 0) {
		return LK_TEST_PID_ANGLE;
	}
	if (strcmp(cap_name, "speed") == 0) {
		return LK_TEST_PID_SPEED;
	}
	if (strcmp(cap_name, "torque") == 0) {
		return LK_TEST_PID_TORQUE;
	}
	return -EINVAL;
}

static void lifecycle_refresh_pid_targets(const struct lk_test_motor_cfg *cfg,
					  struct lk_test_motor_data *data)
{
	for (size_t i = 0; i < ARRAY_SIZE(cfg->common.pid_datas); i++) {
		struct pid_config params;
		int pid_idx;

		if ((cfg->common.pid_datas[i] == NULL) ||
		    (cfg->common.pid_datas[i]->pid_dev == NULL)) {
			break;
		}

		pid_idx = pid_slot_from_capability(cfg->common.capabilities[i]);
		if (pid_idx < 0) {
			continue;
		}

		pid_get_params(cfg->common.pid_datas[i], &params);
		if ((data->params[pid_idx].k_p != params.k_p) ||
		    (data->params[pid_idx].k_i != params.k_i) ||
		    (data->params[pid_idx].k_d != params.k_d)) {
			data->params[pid_idx] = params;
			data->pid_dirty[pid_idx] = true;
		}
	}
}

/* ── Transport helpers (forward declarations) ──────────────────────── */

static int transport_sync_pid(const struct device *dev);
static int transport_update_periodic(const struct device *dev);
static void watchdog_handler(struct k_work *work);

static void log_frame(const char *prefix, const struct device *dev,
		      const struct can_frame *frame, const char *tag)
{
	LOG_INF("%s %s%s%s id=0x%03x cmd=0x%02x data=%02x %02x %02x %02x "
		"%02x %02x %02x %02x",
		dev->name, prefix, tag != NULL ? " " : "", tag != NULL ? tag : "",
		frame->id, frame->dlc > 0U ? frame->data[0] : 0U, frame->data[0],
		frame->data[1], frame->data[2], frame->data[3], frame->data[4],
		frame->data[5], frame->data[6], frame->data[7]);
}

static void rx_handler(const struct device *can_dev, struct can_frame *frame,
		       void *user_data)
{
	const struct device *dev = user_data;
	struct lk_test_motor_data *data = dev->data;
	const struct lk_test_motor_cfg *cfg = dev->config;

	ARG_UNUSED(can_dev);

	if (cfg->trace_lifecycle) {
		log_frame("rx", dev, frame, NULL);
	}

	motor_can_sched_report_rx(cfg->common.phy, frame);
	lifecycle_on_rx(dev, data, frame);
}

static int transport_init(const struct device *dev)
{
	struct lk_test_motor_data *data = dev->data;
	const struct lk_test_motor_cfg *cfg = dev->config;
	int ret;

	if (!device_is_ready(cfg->common.phy)) {
		LOG_ERR("CAN device %s not ready", cfg->common.phy->name);
		return -ENODEV;
	}

	ret = motor_can_sched_register_can(cfg->common.phy);
	if (ret != 0) {
		LOG_ERR("failed to attach scheduler to %s: %d",
			cfg->common.phy->name, ret);
		return ret;
	}

	data->comm.filter.id = lk_test_codec_rx_id(cfg);
	data->comm.filter.mask = CAN_STD_ID_MASK;
	data->comm.filter.flags = 0;
	LOG_INF("%s transport can=%s rx_filter id=0x%03x mask=0x%03x",
		dev->name, cfg->common.phy->name, data->comm.filter.id,
		data->comm.filter.mask);
	ret = can_add_rx_filter(cfg->common.phy, rx_handler, (void *)dev,
				&data->comm.filter);
	if (ret < 0) {
		LOG_ERR("%s rx filter add failed: %d", dev->name, ret);
		/* NOTE: motor_can_sched has no unregister API; scheduler
		 * registration leaks on this error path.
		 */
		return ret;
	}

	return 0;
}

static int transport_submit_once(const struct device *dev,
				 const struct can_frame *frame,
				 const char *tag, bool high_priority)
{
	const struct lk_test_motor_cfg *cfg = dev->config;
	bool track_reply = lk_test_command_tracks_reply(frame->data[0]);
	struct motor_can_sched_tx_param param = {
		.track_reply = track_reply,
		.high_priority = high_priority,
		.immediate_reply = track_reply,
		.trace_lifecycle = cfg->trace_lifecycle,
		.ack_timeout_ms = LK_TEST_REPLY_TIMEOUT_MS,
		.max_retries = 2U,
		.reply_id = lk_test_codec_rx_id(cfg),
		.reply_mask = CAN_STD_ID_MASK,
		.reply_match = lk_test_codec_reply_matches,
		.tag = tag,
	};

	if (cfg->trace_lifecycle) {
		log_frame("tx", dev, frame, tag);
	} else {
		LOG_INF("%s tx %s cmd=0x%02x id=0x%03x prio=%s reply=%u", dev->name,
			tag != NULL ? tag : "frame", frame->data[0], frame->id,
			high_priority ? "high" : "low", track_reply ? 1U : 0U);
	}

	return motor_can_sched_send(cfg->common.phy, frame, &param, NULL);
}

static int transport_update_periodic(const struct device *dev)
{
	struct lk_test_motor_data *data = dev->data;
	const struct lk_test_motor_cfg *cfg = dev->config;
	struct can_frame frame;
	int ret;

	/* Offline detection: check feedback timeout */
	if (data->comm.online && data->feedback.last_rx_ms != 0U) {
		uint32_t now = k_uptime_get_32();
		uint32_t elapsed = now - data->feedback.last_rx_ms;

		if (elapsed > LK_TEST_ONLINE_TIMEOUT_MS) {
			data->comm.online = false;
			data->comm.last_recovery_tx_ms = 0U;
			LOG_WRN("%s offline (no feedback for %u ms)", dev->name,
				elapsed);
		}
	}

	if (data->comm.enabled && !data->comm.target_valid) {
		LOG_WRN("%s periodic blocked: no valid target yet", dev->name);
		return -EAGAIN;
	}

	/* Reconnect: re-sync PID after coming back online */
	if (data->comm.reconnect_pending) {
		data->comm.reconnect_pending = false;
		lifecycle_refresh_pid_targets(cfg, data);
		memset(data->pid_dirty, true, sizeof(data->pid_dirty));
		transport_sync_pid(dev);
		LOG_INF("%s re-synced PID after reconnect", dev->name);
	}

	ret = lk_test_codec_pack_control(cfg, &data->target, &frame);
	if (ret != 0) {
		return ret;
	}

	if (data->comm.periodic == NULL) {
		uint16_t rate = cfg->control_rate_hz != 0U
					? cfg->control_rate_hz
					: LK_TEST_DEFAULT_RATE_HZ;
		struct motor_can_sched_tx_param param = {
			.loop = true,
			.track_reply = false,
			.immediate_reply = false,
			.trace_lifecycle = cfg->trace_lifecycle,
			.loop_hz = rate,
			.ack_timeout_ms = LK_TEST_REPLY_TIMEOUT_MS,
			.max_retries = 1U,
			.reply_id = lk_test_codec_rx_id(cfg),
			.reply_mask = CAN_STD_ID_MASK,
			.tag = dev->name,
		};

		if (cfg->trace_lifecycle) {
			LOG_INF("%s periodic rate=%uHz", dev->name, rate);
			log_frame("tx-loop", dev, &frame, NULL);
		}
		log_target(dev, "start periodic", &data->target);

		return motor_can_sched_send(cfg->common.phy, &frame, &param,
					    &data->comm.periodic);
	}

	if (cfg->trace_lifecycle) {
		log_frame("tx-loop-update", dev, &frame, NULL);
	}

	return motor_can_sched_update(data->comm.periodic, &frame);
}

static void watchdog_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct lk_test_comm_state *comm =
		CONTAINER_OF(dwork, struct lk_test_comm_state, watchdog_work);
	const struct device *dev = comm->dev;
	struct lk_test_motor_data *data;
	const struct lk_test_motor_cfg *cfg;
	uint32_t now;

	if (dev == NULL) {
		return;
	}

	data = dev->data;
	cfg = dev->config;
	now = k_uptime_get_32();

	if (data->comm.enabled && data->comm.online && data->feedback.last_rx_ms != 0U) {
		uint32_t elapsed = now - data->feedback.last_rx_ms;

		if (elapsed > LK_TEST_ONLINE_TIMEOUT_MS) {
			data->comm.online = false;
			data->comm.last_recovery_tx_ms = 0U;
			LOG_WRN("%s offline (no feedback for %u ms)", dev->name, elapsed);
		}
	}

	if (data->comm.enabled && !data->comm.online &&
	    (data->comm.last_recovery_tx_ms == 0U ||
	     now - data->comm.last_recovery_tx_ms >= LK_TEST_RECOVERY_ENABLE_MS)) {
		struct can_frame enable_frame;
		int ret = lk_test_codec_pack_command(cfg, ENABLE_MOTOR, &enable_frame);

		if (ret == 0) {
			ret = transport_submit_once(dev, &enable_frame, "lk-reco", true);
			if (ret == 0) {
				data->comm.last_recovery_tx_ms = now;
			} else {
				LOG_WRN("%s recovery enable submit failed: %d", dev->name, ret);
			}
		}
	}

	if (data->comm.enabled && data->comm.online && data->comm.reconnect_pending) {
		int ret = transport_update_periodic(dev);

		if (ret != 0 && ret != -EAGAIN) {
			LOG_WRN("%s reconnect service failed: %d", dev->name, ret);
		}
	}

	if (data->comm.enabled) {
		(void)k_work_reschedule(&data->comm.watchdog_work,
					K_MSEC(LK_TEST_WATCHDOG_PERIOD_MS));
	}
}

static void transport_stop_periodic(const struct device *dev)
{
	struct lk_test_motor_data *data = dev->data;

	if (data->comm.periodic != NULL) {
		const struct lk_test_motor_cfg *cfg = dev->config;

		if (cfg->trace_lifecycle) {
			LOG_INF("%s periodic stop", dev->name);
		}
		(void)motor_can_sched_remove(data->comm.periodic);
		data->comm.periodic = NULL;
	}
}

static int transport_sync_pid(const struct device *dev)
{
	struct lk_test_motor_data *data = dev->data;
	const struct lk_test_motor_cfg *cfg = dev->config;

	for (enum lk_test_pid_slot slot = LK_TEST_PID_ANGLE;
	     slot < LK_TEST_PID_COUNT; slot++) {
		struct can_frame frame;
		int ret;

		if (!data->pid_dirty[slot]) {
			continue;
		}

		ret = lk_test_codec_pack_pid(cfg, slot, &data->params[slot],
					     &frame);
		if (ret != 0) {
			return ret;
		}

		LOG_INF("%s pid sync slot=%s kp=%.3f ki=%.3f kd=%.3f",
			dev->name, pid_slot_name(slot), (double)data->params[slot].k_p,
			(double)data->params[slot].k_i, (double)data->params[slot].k_d);
		ret = transport_submit_once(dev, &frame, "lk-pid", true);
		if (ret != 0) {
			return ret;
		}
		data->pid_dirty[slot] = false;
	}

	return 0;
}

/* ── Public API (facade) ───────────────────────────────────────────── */

const struct motor_driver_api lk_test_motor_api = {
	.motor_get = lk_test_get,
	.motor_set = lk_test_set,
	.motor_control = lk_test_control,
};

int lk_test_init(const struct device *dev)
{
	struct lk_test_motor_data *data = dev->data;
	const struct lk_test_motor_cfg *cfg = dev->config;
	struct can_frame frame;
	int ret;

	lifecycle_init(data);
	data->comm.dev = dev;
	data->comm.enabled = false;
	data->comm.online = false;
	data->comm.target_valid = false;
	data->comm.feedback_seen = false;
	data->comm.last_recovery_tx_ms = 0U;
	data->feedback.last_rx_ms = 0U;
	k_work_init_delayable(&data->comm.watchdog_work, watchdog_handler);

	ret = transport_init(dev);
	if (ret != 0) {
		return ret;
	}

	ret = lk_test_codec_pack_command(cfg, DISABLE_MOTOR, &frame);
	if (ret == 0) {
		LOG_WRN("%s startup safety: force motor-off before any enable", dev->name);
		(void)transport_submit_once(dev, &frame, "lk-safe-off", true);
	}
	(void)lk_test_codec_pack_read_status(cfg, &frame);
	(void)transport_submit_once(dev, &frame, "lk-read", true);

	lifecycle_refresh_pid_targets(cfg, data);
	ret = transport_sync_pid(dev);
	if (ret != 0) {
		LOG_WRN("%s initial pid sync failed: %d", dev->name, ret);
	}

	LOG_INF("%s init tx=0x%03x rx=0x%03x rate=%uHz (reply timeout=%dms)",
		dev->name, lk_test_codec_tx_id(cfg), lk_test_codec_rx_id(cfg),
		cfg->control_rate_hz != 0U ? cfg->control_rate_hz
					   : LK_TEST_DEFAULT_RATE_HZ,
		LK_TEST_REPLY_TIMEOUT_MS);
	LOG_INF("%s init state enabled=%d online=%d target_valid=%d feedback_seen=%d",
		dev->name, data->comm.enabled, data->comm.online,
		data->comm.target_valid, data->comm.feedback_seen);
	return 0;
}

void lk_test_control(const struct device *dev, enum motor_cmd cmd)
{
	struct lk_test_motor_data *data = dev->data;
	const struct lk_test_motor_cfg *cfg = dev->config;
	struct can_frame frame;
	int ret;

	if (cmd == CLEAR_PID) {
		lifecycle_clear_pid(data);
		LOG_INF("%s control cmd=%s", dev->name, cmd_name(cmd));
		/* PID 同步也改为异步 */
		ret = transport_sync_pid(dev);
		/* 注意：transport_sync_pid 仍然会等待 PID 帧回复，
		 * 因为 PID 参数同步需要确认是否真正写入 */
		if (ret != 0) {
			LOG_ERR("%s clear pid sync failed: %d", dev->name, ret);
		}
		return;
	}

	ret = lk_test_codec_pack_command(cfg, cmd, &frame);
	if (ret != 0) {
		LOG_ERR("%s unsupported command %d", dev->name, cmd);
		return;
	}

	LOG_INF("%s control cmd=%s enabled=%d online=%d target_valid=%d feedback_seen=%d",
		dev->name, cmd_name(cmd), data->comm.enabled, data->comm.online,
		data->comm.target_valid, data->comm.feedback_seen);

	/* 所有 control 命令都立即返回 */
	ret = transport_submit_once(dev, &frame, "lk-cmd", true);
	if (ret != 0) {
		LOG_ERR("%s command submit failed: %d", dev->name, ret);
		return;
	}

	if (cmd == ENABLE_MOTOR) {
		(void)lk_test_codec_pack_read_status(cfg, &frame);
		(void)transport_submit_once(dev, &frame, "lk-read", true);
		lifecycle_mark_enabled(data, true);
		(void)k_work_reschedule(&data->comm.watchdog_work,
					K_MSEC(LK_TEST_WATCHDOG_PERIOD_MS));
		ret = transport_update_periodic(dev);
		if ((ret != 0) && (ret != -EAGAIN)) {
			LOG_ERR("%s enable periodic start failed: %d", dev->name, ret);
		}
	} else if (cmd == DISABLE_MOTOR) {
		lifecycle_mark_enabled(data, false);
		(void)k_work_cancel_delayable(&data->comm.watchdog_work);
		transport_stop_periodic(dev);
	}
}

int lk_test_set(const struct device *dev, motor_status_t *status)
{
	struct lk_test_motor_data *data = dev->data;
	const struct lk_test_motor_cfg *cfg = dev->config;
	int ret;

	ret = lifecycle_apply_target(data, status);
	if (ret != 0) {
		return ret;
	}
	log_target(dev, "apply", &data->target);

	lifecycle_refresh_pid_targets(cfg, data);
	ret = transport_sync_pid(dev);
	if (ret != 0) {
		return ret;
	}

	if (data->comm.enabled) {
		ret = transport_update_periodic(dev);
		if (ret != 0) {
			LOG_ERR("%s periodic update failed: %d", dev->name, ret);
			return ret;
		}
	}

	return 0;
}

int lk_test_get(const struct device *dev, motor_status_t *status)
{
	struct lk_test_motor_data *data = dev->data;

	lifecycle_export_status(data, status);
	return 0;
}

DT_INST_FOREACH_STATUS_OKAY(LK_TEST_MOTOR_INST)
