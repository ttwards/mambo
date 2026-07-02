#include "ares/protocol/ares_protocol.h"
#include "ares/protocol/dual/dual_protocol.h"
#include "zephyr/kernel.h"
#include "zephyr/net_buf.h"
#include <zephyr/logging/log.h>
#include <sys/errno.h>
#include "ares/interface/ares_interface.h"
#include <stdint.h>

LOG_MODULE_REGISTER(dual_protocol, LOG_LEVEL_INF);

K_HEAP_DEFINE(dual_protocol_heap, 4096);

#define HEART_BEAT_DELAY 200
#define CRC16_SIZE       2

// CRC16-CCITT计算函数
static uint16_t crc16_ccitt(uint8_t *data, size_t length)
{
	uint16_t crc = 0xFFFF;
	for (size_t i = 0; i < length; i++) {
		crc ^= (uint16_t)data[i] << 8;
		for (int j = 0; j < 8; j++) {
			if (crc & 0x8000) {
				crc = (crc << 1) ^ 0x1021;
			} else {
				crc <<= 1;
			}
		}
	}
	return crc;
}

// 添加CRC16到缓冲区末尾
static void add_crc16(uint8_t *buf, size_t data_len)
{
	uint16_t crc = crc16_ccitt(buf, data_len);
	GET_16BITS(buf, data_len) = crc;
}

// 验证CRC16
static bool verify_crc16(uint8_t *buf, size_t total_len)
{
	if (total_len < CRC16_SIZE) {
		return false;
	}

	size_t data_len = total_len - CRC16_SIZE;
	uint16_t received_crc = GET_16BITS(buf, data_len);
	uint16_t calculated_crc = crc16_ccitt(buf, data_len);

	return received_crc == calculated_crc;
}

struct tx_msg_bck {
	uint32_t ret;
	uint16_t id;
};

#define MAKE_REPL_FRAME(id, ret, buf)                                                              \
	({                                                                                         \
		uint8_t *repl_frame = buf;                                                         \
		GET_16BITS(repl_frame, REPL_HEAD_IDX) = REPL_FRAME_HEAD;                           \
		GET_16BITS(repl_frame, REPL_FUNC_ID_IDX) = id;                                     \
		GET_32BITS(repl_frame, REPL_RET_IDX) = (uint32_t)ret;                              \
		repl_frame;                                                                        \
	})

struct k_thread ares_usb_tid;
K_THREAD_STACK_DEFINE(ares_usb_stack, 2048);

struct net_buf *alloc_and_add_data(struct AresProtocol *protocol, uint8_t *data, size_t len)
{
	struct dual_protocol_data *pdata = protocol->priv_data;
	if (protocol->interface->api->alloc_buf_with_data) {
		return protocol->interface->api->alloc_buf_with_data(protocol->interface, data,
								     len);
	} else {
		struct net_buf *buf = protocol->interface->api->alloc_buf(protocol->interface);
		if (buf == NULL) {
			LOG_ERR("%s Failed to allocate buffer.", pdata->name);
			return NULL;
		}
		net_buf_add_mem(buf, data, len);
		return buf;
	}
}

int send_try_lock(struct AresProtocol *protocol, struct net_buf *buf, struct k_mutex *mutex)
{
	if (protocol->interface->api->send_with_lock && mutex) {
		return protocol->interface->api->send_with_lock(protocol->interface, buf, mutex);
	} else {
		k_mutex_unlock(mutex);
		return protocol->interface->api->send(protocol->interface, buf);
	}
}

static void error_handle(struct AresProtocol *protocol, uint16_t req_id, uint16_t error)
{
	struct dual_protocol_data *data = protocol->priv_data;
	GET_16BITS(data->error_frame_buf, ERROR_HEAD_IDX) = ERROR_FRAME_HEAD;
	GET_8BITS(data->error_frame_buf, ERROR_ID_IDX) = req_id;
	GET_16BITS(data->error_frame_buf, ERROR_CODE_IDX) = error;
	if (error != HEART_BEAT) {
		LOG_ERR("%s Error code: %x, Request ID: %x", data->name, error, req_id);
	} else {
		data->last_heart_beat = k_uptime_get_32();
	}

	size_t frame_len = ERROR_FRAME_LENGTH;
	if (data->crc_enabled) {
		add_crc16(data->error_frame_buf, ERROR_FRAME_LENGTH);
		frame_len += CRC16_SIZE;
	}

	struct net_buf *buf = protocol->interface->api->alloc_buf(protocol->interface);
	if (buf == NULL) {
		LOG_ERR("%s Failed to allocate buffer for error frame.", data->name);
		return;
	}
	net_buf_add_mem(buf, data->error_frame_buf, frame_len);
	int err = protocol->interface->api->send(protocol->interface, buf);
	if (err != 0) {
		// LOG_ERR("Failed to send error frame. %d", err);
	}
}

static bool proceed_tx_bck(struct AresProtocol *protocol, uint8_t ID)
{
	struct dual_protocol_data *data = protocol->priv_data;
	// 创建一个临时队列
	struct k_msgq temp_msgq;
	uint8_t *temp_msgq_buf = k_heap_aligned_alloc(&dual_protocol_heap, 4,
						      sizeof(struct tx_msg_bck) * 10, K_NO_WAIT);
	k_msgq_init(&temp_msgq, temp_msgq_buf, sizeof(struct tx_msg_bck) * 10, 10);
	bool found = false;

	// 临时存储所有数据项
	struct tx_msg_bck *item;
	while ((k_msgq_get(&data->func_tx_bckup_msgq, &item, K_NO_WAIT)) == 0) {
		if (item->id == ID && !found) {
			struct dual_protocol_data *pdata = protocol->priv_data;
			size_t frame_len = REPL_FRAME_LENGTH;
			if (pdata->crc_enabled) {
				frame_len += CRC16_SIZE;
			}

			struct net_buf *buf =
				protocol->interface->api->alloc_buf(protocol->interface);

			if (buf == NULL) {
				LOG_ERR("%s Failed to allocate buffer for FUNC frame.", data->name);
				return false;
			}

			net_buf_add(buf, frame_len);

			MAKE_REPL_FRAME(ID, item->ret, buf->data);

			if (pdata->crc_enabled) {
				add_crc16(buf->data, REPL_FRAME_LENGTH);
			}

			protocol->interface->api->send(protocol->interface, buf);

			found = true;
			continue;
		} else {
			// 保存到临时队列
			k_msgq_put(&temp_msgq, item, K_NO_WAIT);
		}
	}

	// 将所有项目放回原始队列
	while ((k_msgq_get(&temp_msgq, &item, K_NO_WAIT)) == 0) {
		k_msgq_put(&data->func_tx_bckup_msgq, item, K_NO_WAIT);
	}

	return found;
}

static sync_table_t *find_pack(struct dual_protocol_data *data, uint16_t ID)
{
	for (int i = 0; i < data->sync_cnt; i++) {
		if (ID == data->sync_table[i].ID) {
			return &data->sync_table[i];
		}
	}
	return NULL;
}

static func_table_t *find_func(struct dual_protocol_data *data, uint16_t ID)
{
	for (int i = 0; i < data->func_cnt; i++) {
		if (ID == data->func_table[i].id) {
			return &data->func_table[i];
		}
	}
	return NULL;
}

static void usb_offline_clean(struct AresProtocol *protocol)
{
	struct tx_msg_bck msg;
	struct dual_protocol_data *data = protocol->priv_data;
	while (k_msgq_num_used_get(&data->func_tx_bckup_msgq) > 0) {
		k_msgq_get(&data->func_tx_bckup_msgq, &msg, K_NO_WAIT);
	}
}

static void usb_trans_heart_beat(struct k_timer *timer)
{
	struct AresProtocol *protocol = k_timer_user_data_get(timer);
	struct dual_protocol_data *data = protocol->priv_data;

	if (data->online) {
		if ((k_uptime_get_32() - data->last_heart_beat) <= HEART_BEAT_DELAY &&
		    ((int32_t)(k_uptime_get_32() - data->last_receive) >= 10 * HEART_BEAT_DELAY)) {
			LOG_ERR("%s Connection lost. last_heart_beat: %d, last_receive: %d, "
				"current: "
				"%d",
				data->name, data->last_heart_beat, data->last_receive,
				k_uptime_get_32());
			data->online = false;
			usb_offline_clean(protocol);
		}
	} else {
		if ((k_uptime_get_32() - data->last_receive) <= 10 * HEART_BEAT_DELAY) {
			LOG_INF("%s Connection established.", data->name);
			data->online = true;
		}
	}
	error_handle(protocol, HEARTBEAT_ID, HEART_BEAT);

	data->last_heart_beat = k_uptime_get_32();
}

static void parse_sync(struct AresProtocol *protocol, uint8_t *buf, size_t len)
{
	struct dual_protocol_data *data = protocol->priv_data;
	LOG_HEXDUMP_DBG(buf, len, "SYNC frame received");

	uint16_t ID = GET_16BITS(buf, SYNC_ID_IDX);
	uint8_t *pdata = buf + SYNC_DATA_IDX;
	size_t data_len = len - SYNC_FRAME_LENGTH_OFFSET;
	static uint32_t sync_rx_count;

	sync_rx_count++;
	if (sync_rx_count <= 16U || (sync_rx_count % 100U) == 0U) {
		LOG_INF("%s SYNC RX #%u id=0x%04x len=%u", data->name, sync_rx_count, ID, len);
	}

	sync_table_t *pack = find_pack(protocol->priv_data, ID);
	if (pack == NULL) {
		LOG_ERR("%s Cannot find corresponding ID from sync table: 0x%x", data->name, ID);
		return;
	}

	if (pack->len + SYNC_FRAME_LENGTH_OFFSET != len) {
		LOG_ERR("%s SYNC frame length mismatch: %d vs %d", data->name,
			pack->len + SYNC_FRAME_LENGTH_OFFSET, len);
		error_handle(protocol, pack->ID, UNKNOWN_TAIL);
		return;
	}

	if (pack->cb != NULL) {
		pack->cb(SYNC_PACK_STATUS_WRITE);
	}

	memcpy(pack->data, pdata, data_len);

	if (pack->cb != NULL) {
		pack->cb(SYNC_PACK_STATUS_DONE);
	}

	return;
}

static void parse_repl(struct AresProtocol *protocol, uint8_t *buf, size_t len)
{
	uint16_t ID = GET_16BITS(buf, REPL_FUNC_ID_IDX);
	uint16_t req_id = GET_16BITS(buf, REPL_REQ_ID_IDX);
	uint32_t ret = GET_32BITS(buf, REPL_RET_IDX);

	struct dual_protocol_data *data = protocol->priv_data;
	if (data->func_ret_cb != NULL) {
		data->func_ret_cb(ID, req_id, ret);
	}
	LOG_DBG("%s REPL frame received: ID %x, req_id %x, ret %x", data->name, ID, req_id, ret);
	return;
}

static void parse_func(struct AresProtocol *protocol, func_table_t *map)
{
	uint32_t ret = map->cb(map->arg1, map->arg2, map->arg3);

	struct dual_protocol_data *data = protocol->priv_data;
	if (data->func_tx_bckup_cnt == MAX_FUNC_COUNT) {
		struct tx_msg_bck *rm_frame = NULL;
		k_msgq_get(&data->func_tx_bckup_msgq, &rm_frame, K_NO_WAIT);
		if (rm_frame == NULL) {
			return;
		}
		data->func_tx_bckup_cnt--;
	}

	struct tx_msg_bck msg = {
		.ret = ret,
		.id = map->id,
	};
	k_msgq_put(&data->func_tx_bckup_msgq, &msg, K_NO_WAIT);
	data->func_tx_bckup_cnt++;

	uint8_t *repl_frame = map->buf;
	if (k_mutex_lock(&map->mutex, K_NO_WAIT) != 0) {
		LOG_DBG("%s Failed to lock FUNC frame mutex.", data->name);
		return;
	}
	GET_16BITS(repl_frame, REPL_HEAD_IDX) = REPL_FRAME_HEAD;
	GET_16BITS(repl_frame, REPL_FUNC_ID_IDX) = map->id;
	GET_32BITS(repl_frame, REPL_RET_IDX) = (uint32_t)ret;
	GET_8BITS(repl_frame, REPL_REQ_ID_IDX) = map->req_id;

	size_t frame_len = REPL_FRAME_LENGTH;
	if (data->crc_enabled) {
		add_crc16(repl_frame, REPL_FRAME_LENGTH);
		frame_len += CRC16_SIZE;
	}

	int err = 0;
	struct net_buf *buf = alloc_and_add_data(protocol, repl_frame, frame_len);
	err = send_try_lock(protocol, buf, &map->mutex);

	if (err != 0) {
		LOG_ERR("%s Failed to send REPLY frame.", data->name);
		return;
	}
	return;
}

static void parse_error(struct AresProtocol *protocol, uint8_t data[], int8_t len)
{
	struct dual_protocol_data *pdata = protocol->priv_data;
	uint8_t req_id = GET_8BITS(data, ERROR_ID_IDX);
	if (req_id == HEARTBEAT_ID) {
		LOG_DBG("%s Heartbeat frame received.", pdata->name);
		return;
	}
	if (!proceed_tx_bck(protocol, req_id)) {
		sync_table_t *pack = find_pack(protocol->priv_data,
					       sys_be16_to_cpu(GET_16BITS(data, ERROR_ID_IDX)));
		if (pack == NULL) {
			LOG_ERR("%s Cannot find corresponding ID from error "
				"frame.",
				pdata->name);
			error_handle(protocol, req_id, CLEANED_PACK);
			return;
		}
		if (pack->cb != NULL) {
			pack->cb(SYNC_PACK_STATUS_READ);
		}
		dual_sync_flush(protocol, pack);
	}
	LOG_INF("%s Error frame received: id =%d; code=%d", pdata->name, req_id,
		GET_16BITS(data, ERROR_CODE_IDX));
	return;
}

int dual_ret_cb_set(struct AresProtocol *protocol, dual_func_ret_cb_t cb)
{
	struct dual_protocol_data *data = protocol->priv_data;
	if (cb == NULL) {
		LOG_ERR("%s Callback function is NULL.", data->name);
		return -EINVAL;
	}
	if (data->func_ret_cb != NULL) {
		LOG_ERR("%s Callback function already set.", data->name);
		return -EALREADY;
	}
	data->func_ret_cb = cb;
	return 0;
}

int dual_sync_flush(struct AresProtocol *protocol, sync_table_t *pack)
{
	struct dual_protocol_data *data = protocol->priv_data;
	if (!data->online) {
		return -EUNATCH;
	}
	if (pack == NULL) {
		LOG_ERR("%s Sync pack is NULL.", data->name);
		return -EINVAL;
	}
	memcpy(pack->buf + SYNC_DATA_IDX, pack->data, pack->len);

	size_t frame_len = pack->len + SYNC_FRAME_LENGTH_OFFSET;
	if (data->crc_enabled) {
		add_crc16(pack->buf, pack->len + SYNC_FRAME_LENGTH_OFFSET);
		frame_len += CRC16_SIZE;
	}

	// LOG_HEXDUMP_DBG(pack->buf, frame_len, "Sync data to send:");

	struct net_buf *buf = alloc_and_add_data(protocol, pack->buf, frame_len);
	int ret = send_try_lock(protocol, buf, &pack->mutex);

	if (ret != 0) {
		// LOG_ERR("Failed to send SYNC frame. %d", ret);
		return -EBUSY;
	}
	return 0;
}

int dual_func_call(struct AresProtocol *protocol, uint16_t id, uint32_t arg1, uint32_t arg2,
		   uint32_t arg3)
{
	struct dual_protocol_data *data = protocol->priv_data;
	if (!data->online) {
		return -EUNATCH;
	}
	size_t frame_len = FUNC_FRAME_LENGTH;
	if (data->crc_enabled) {
		frame_len += CRC16_SIZE;
	}

	struct net_buf *buf = protocol->interface->api->alloc_buf(protocol->interface);
	if (buf == NULL) {
		LOG_ERR("%s Failed to allocate buffer for FUNC frame.", data->name);
		return -ENOMEM;
	}
	net_buf_add(buf, frame_len);
	GET_16BITS(buf->data, FUNC_HEAD_IDX) = FUNC_FRAME_HEAD;
	GET_16BITS(buf->data, FUNC_ID_IDX) = id;
	GET_32BITS(buf->data, FUNC_ARG1_IDX) = arg1;
	GET_32BITS(buf->data, FUNC_ARG2_IDX) = arg2;
	GET_32BITS(buf->data, FUNC_ARG3_IDX) = arg3;
	uint16_t req_id = k_uptime_get_32();
	GET_16BITS(buf->data, FUNC_REQ_IDX) = req_id;

	if (data->crc_enabled) {
		add_crc16(buf->data, FUNC_FRAME_LENGTH);
	}

	int err = protocol->interface->api->send(protocol->interface, buf);
	if (err != 0) {
		LOG_ERR("%s Failed to send FUNC frame.", data->name);
		return -EBUSY;
	}
	return req_id;
}

sync_table_t *dual_sync_add(struct AresProtocol *protocol, uint16_t ID, uint8_t *buf, size_t len,
			    dual_trans_cb_t cb)
{
	struct dual_protocol_data *data = protocol->priv_data;
	if (data->sync_cnt >= MAX_PACK_COUNT) {
		LOG_ERR("%s Sync pack count exceeds maximum.", data->name);
		return NULL;
	}
	data->sync_table[data->sync_cnt].interface = protocol->interface;
	data->sync_table[data->sync_cnt].data = buf;
	data->sync_table[data->sync_cnt].ID = ID;
	data->sync_table[data->sync_cnt].len = len;
	data->sync_table[data->sync_cnt].cb = cb;

	size_t buf_size = len + SYNC_FRAME_LENGTH_OFFSET;
	if (data->crc_enabled) {
		buf_size += CRC16_SIZE;
	}

	data->sync_table[data->sync_cnt].buf =
		k_heap_aligned_alloc(&dual_protocol_heap, 4, buf_size, K_NO_WAIT);
	memset(data->sync_table[data->sync_cnt].buf, 0, buf_size);
	if (data->sync_table[data->sync_cnt].buf == NULL) {
		LOG_ERR("%s Failed to allocate memory for SYNC frame. Count: %d", data->name,
			data->sync_cnt);
		return NULL;
	}
	GET_16BITS(data->sync_table[data->sync_cnt].buf, SYNC_HEAD_IDX) = SYNC_FRAME_HEAD;
	GET_16BITS(data->sync_table[data->sync_cnt].buf, SYNC_ID_IDX) = ID;
	data->sync_cnt++;
	if (cb) {
		cb(SYNC_PACK_STATUS_READ);
	}
	LOG_INF("%s Sync pack added: ID %x, data %p, len %d", data->name, ID, (void *)data, len);
	dual_sync_flush(protocol, &data->sync_table[data->sync_cnt - 1]);

	return &data->sync_table[data->sync_cnt - 1];
}

void dual_func_add(struct AresProtocol *protocol, uint16_t id, dual_trans_func_t cb)
{
	struct dual_protocol_data *data = protocol->priv_data;
	if (cb == NULL) {
		LOG_ERR("%s Callback function is NULL.", data->name);
		return;
	}
	if (data->func_cnt >= MAX_PACK_COUNT) {
		LOG_ERR("%s Func count exceeds maximum.", data->name);
		return;
	}
	data->func_table[data->func_cnt].id = id;
	data->func_table[data->func_cnt].cb = cb;
	data->func_cnt++;
}

void dual_func_remove(struct AresProtocol *protocol, uint16_t header)
{
	struct dual_protocol_data *data = protocol->priv_data;
	for (int i = 0; i < data->func_cnt; i++) {
		if (data->func_table[i].id == header) {
			data->func_cnt--;
			data->func_table[i] = data->func_table[data->func_cnt];
			return;
		}
	}
	LOG_ERR("%s Cannot find corresponding ID from func table.", data->name);
	return;
}

// 根据帧头确定帧类型
static enum frame_type get_frame_type(uint16_t header)
{
	switch (header) {
	case SYNC_FRAME_HEAD:
		return FRAME_TYPE_SYNC;
	case FUNC_FRAME_HEAD:
		return FRAME_TYPE_FUNC;
	case ERROR_FRAME_HEAD:
		return FRAME_TYPE_ERROR;
	case REPL_FRAME_HEAD:
		return FRAME_TYPE_REPL;
	default:
		return FRAME_TYPE_UNKNOWN;
	}
}

// 根据帧类型和数据确定帧长度
static uint16_t get_frame_length(struct AresProtocol *protocol, enum frame_type type, uint8_t *buf)
{
	struct dual_protocol_data *data = protocol->priv_data;
	uint16_t base_length = 0;

	switch (type) {
	case FRAME_TYPE_FUNC:
		base_length = FUNC_FRAME_LENGTH;
		break;
	case FRAME_TYPE_ERROR:
		base_length = ERROR_FRAME_LENGTH;
		break;
	case FRAME_TYPE_REPL:
		base_length = REPL_FRAME_LENGTH;
		break;
	case FRAME_TYPE_SYNC:
		if (buf != NULL) {
			// 需要根据sync_table中的信息确定长度
			uint16_t sync_id = GET_16BITS(buf, SYNC_ID_IDX);
			sync_table_t *pack = find_pack(protocol->priv_data, sync_id);
			if (pack != NULL) {
				base_length = pack->len + SYNC_FRAME_LENGTH_OFFSET;
			} else {
				return 0; // 无法确定长度
			}
		} else {
			return 0; // 无法确定长度
		}
		break;
	default:
		return 0;
	}

	// 如果启用CRC，增加CRC16长度
	if (data->crc_enabled) {
		base_length += CRC16_SIZE;
	}

	return base_length;
}

// 重置状态机
static void reset_parser_state(struct dual_protocol_data *data)
{
	data->state = PARSER_STATE_IDLE;
	data->current_frame_type = FRAME_TYPE_UNKNOWN;
	data->rx_buffer_pos = 0;
	data->expected_frame_length = 0;
	data->header_value = 0;
}

// 处理完整的帧
static void process_complete_frame(struct AresProtocol *protocol)
{
	struct dual_protocol_data *data = protocol->priv_data;
	uint8_t *buf = data->rx_buffer;
	uint16_t len = data->rx_buffer_pos;

	LOG_DBG("%s Processing complete frame: type %d, length %d", data->name,
		data->current_frame_type, len);

	// 如果启用CRC，先验证CRC
	if (data->crc_enabled) {
		if (!verify_crc16(buf, len)) {
			LOG_ERR("%s CRC16 verification failed for frame type %d", data->name,
				data->current_frame_type);
			reset_parser_state(data);
			return;
		}
		// CRC验证通过，调整帧长度（去除CRC部分）
		len -= CRC16_SIZE;
	}

	switch (data->current_frame_type) {
	case FRAME_TYPE_FUNC:
		if (len != FUNC_FRAME_LENGTH) {
			LOG_ERR("%s FUNC frame length mismatch: %d vs %d", data->name,
				FUNC_FRAME_LENGTH, len);
			error_handle(protocol, GET_16BITS(buf, FUNC_ID_IDX), UNKNOWN_TAIL);
			break;
		}
		func_table_t *map = find_func(data, GET_16BITS(buf, FUNC_ID_IDX));
		LOG_HEXDUMP_DBG(buf, len, "FUNC frame received");
		LOG_DBG("%s FUNC frame received: ID %x", data->name, GET_16BITS(buf, FUNC_ID_IDX));
		if (map == NULL) {
			LOG_ERR("%s Cannot find corresponding ID 0x%x in func mappings.",
				data->name, GET_16BITS(buf, FUNC_ID_IDX));
			LOG_HEXDUMP_DBG(buf, len, "FUNC frame");
			// error_handle(protocol, GET_16BITS(buf, FUNC_ID_IDX), UNKNOWN_FUNC);
			break;
		}
		map->arg1 = GET_32BITS(buf, FUNC_ARG1_IDX);
		map->arg2 = GET_32BITS(buf, FUNC_ARG2_IDX);
		map->arg3 = GET_32BITS(buf, FUNC_ARG3_IDX);
		map->req_id = GET_16BITS(buf, FUNC_REQ_IDX);
		parse_func(protocol, map);
		break;
	case FRAME_TYPE_SYNC:
		parse_sync(protocol, buf, len);
		LOG_DBG("%s SYNC frame received: ID %x", data->name, GET_16BITS(buf, SYNC_ID_IDX));
		break;
	case FRAME_TYPE_ERROR:
		if (len != ERROR_FRAME_LENGTH) {
			LOG_ERR("%s ERROR frame length mismatch: %d vs %d", data->name,
				ERROR_FRAME_LENGTH, len);
			LOG_HEXDUMP_ERR(buf, len, "ERROR frame");
			error_handle(protocol, GET_16BITS(buf, ERROR_ID_IDX), UNKNOWN_TAIL);
			break;
		}
		parse_error(protocol, buf, len);
		break;
	case FRAME_TYPE_REPL:
		parse_repl(protocol, buf, len);
		break;
	default:
		LOG_ERR("%s Unknown frame type: %d", data->name, data->current_frame_type);
		LOG_HEXDUMP_ERR(buf, len, "Unknown frame");
		break;
	}

	// 重置状态机
	reset_parser_state(data);
}

// 状态机处理单个字节
static void process_byte(struct AresProtocol *protocol, uint8_t byte)
{
	struct dual_protocol_data *data = protocol->priv_data;
parse:
	switch (data->state) {
	case PARSER_STATE_IDLE:
		// 等待帧头第一个字节
		data->rx_buffer[0] = byte;
		data->rx_buffer_pos = 1;
		data->state = PARSER_STATE_HEADER_FIRST;
		break;

	case PARSER_STATE_HEADER_FIRST:
		// 接收帧头第二个字节
		data->rx_buffer[1] = byte;
		data->rx_buffer_pos = 2;
		data->header_value = GET_16BITS(data->rx_buffer, 0);
		data->current_frame_type = get_frame_type(data->header_value);

		if (data->current_frame_type == FRAME_TYPE_UNKNOWN) {
			reset_parser_state(data);
			goto parse;
		}

		// 对于非SYNC帧，可以立即确定长度
		if (data->current_frame_type != FRAME_TYPE_SYNC) {
			data->expected_frame_length =
				get_frame_length(protocol, data->current_frame_type, NULL);
			if (data->expected_frame_length == 0) {
				LOG_ERR("%s Cannot determine frame length for type %d", data->name,
					data->current_frame_type);
				reset_parser_state(data);
				break;
			}
		}

		// 检查是否已经收到完整帧（只有2字节的帧不存在）
		if (data->expected_frame_length > 0 &&
		    data->rx_buffer_pos >= data->expected_frame_length) {
			data->state = PARSER_STATE_FRAME_COMPLETE;
			process_complete_frame(protocol);
		} else {
			data->state = PARSER_STATE_RECEIVING_DATA;
		}
		break;

	case PARSER_STATE_RECEIVING_DATA:
		// 接收数据字节
		if (data->rx_buffer_pos >= sizeof(data->rx_buffer)) {
			LOG_ERR("%s RX buffer overflow", data->name);
			reset_parser_state(data);
			break;
		}

		data->rx_buffer[data->rx_buffer_pos++] = byte;

		// 对于SYNC帧，需要在接收到ID后确定长度
		if (data->current_frame_type == FRAME_TYPE_SYNC && data->rx_buffer_pos == 4) {
			data->expected_frame_length = get_frame_length(
				protocol, data->current_frame_type, data->rx_buffer);
			if (data->expected_frame_length == 0) {
				LOG_ERR("%s Cannot determine SYNC frame length with ID: 0x%x",
					data->name, GET_16BITS(data->rx_buffer, SYNC_ID_IDX));
				reset_parser_state(data);
			}
		}

		// 检查是否收到完整帧
		if (data->expected_frame_length > 0 &&
		    data->rx_buffer_pos >= data->expected_frame_length) {
			data->state = PARSER_STATE_FRAME_COMPLETE;
			process_complete_frame(protocol);
		}
		break;

	case PARSER_STATE_FRAME_COMPLETE:
		// 这种状态不应该出现，重置状态机
		LOG_ERR("%s Unexpected byte in FRAME_COMPLETE state", data->name);
		reset_parser_state(data);
		process_byte(protocol, byte); // 重新处理这个字节
		break;
	}
}

static void rx_frame_parser(struct AresProtocol *protocol, uint8_t *buf, uint8_t len)
{
	if (len == 0) {
		return;
	}

	struct dual_protocol_data *data = protocol->priv_data;
	data->last_receive = k_uptime_get_32();

	// 逐字节处理
	for (uint8_t i = 0; i < len; i++) {
		process_byte(protocol, buf[i]);
	}
}

void ares_dual_protocol_handle_byte(struct AresProtocol *protocol, uint8_t byte)
{
	struct dual_protocol_data *data = protocol->priv_data;
	data->last_receive = k_uptime_get_32();
	process_byte(protocol, byte);
}

int ares_dual_protocol_init(struct AresProtocol *protocol)
{
	struct dual_protocol_data *data = protocol->priv_data;
	LOG_INF("%s Dual protocol init", data->name);
	k_mutex_init(&data->err_frame_mutex);
	k_timer_init(&data->heart_beat_timer, usb_trans_heart_beat, NULL);
	k_timer_user_data_set(&data->heart_beat_timer, protocol);
	k_timer_start(&data->heart_beat_timer, K_MSEC(100), K_MSEC(HEART_BEAT_DELAY));

	uint8_t *msgq_buf = k_heap_aligned_alloc(&dual_protocol_heap, 4,
						 sizeof(struct tx_msg_bck) * 10, K_NO_WAIT);
	k_msgq_init(&data->func_tx_bckup_msgq, msgq_buf, sizeof(struct tx_msg_bck), 10);

	// 初始化状态机
	data->state = PARSER_STATE_IDLE;
	data->current_frame_type = FRAME_TYPE_UNKNOWN;
	data->rx_buffer_pos = 0;
	data->expected_frame_length = 0;
	data->header_value = 0;

	return 0;
}

void ares_dual_protocol_handle(struct AresProtocol *protocol, struct net_buf *buf)
{
	// LOG_INF("handle");
	// struct dual_protocol_data *data = protocol->priv_data;
	rx_frame_parser(protocol, buf->data, buf->len);
	// LOG_HEXDUMP_DBG(buf->data, buf->len, "RX frame");
}

void ares_dual_protocol_event(struct AresProtocol *protocol, enum AresProtocolEvent event)
{
	// LOG_INF("event");
	struct dual_protocol_data *data = protocol->priv_data;
	if (event == ARES_PROTOCOL_EVENT_CONNECTED) {
		LOG_INF("%s Connection established due to event.", data->name);
		k_msleep(600);
		data->online = true;
		// 重置状态机状态
		reset_parser_state(data);
	} else if (event == ARES_PROTOCOL_EVENT_DISCONNECTED) {
		LOG_INF("%s Connection lost due to event.", data->name);
		data->online = false;
		// 重置状态机状态
		reset_parser_state(data);
	}
}
