#include <stdint.h>
#include "ares_sbus.h"
#include <string.h>
#include <sys/_intsup.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/sbus.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include "zephyr/logging/log.h"
#include "zephyr/sys/time_units.h"

#define DT_DRV_COMPAT ares_sbus

LOG_MODULE_REGISTER(ares_sbus, CONFIG_SBUS_LOG_LEVEL);

// serial buffer pool
#define BUF_SIZE 64
K_MEM_SLAB_DEFINE_IN_SECT(sbus_uart_slab, __nocache, BUF_SIZE, 4, 4);

static const struct device *uart_dev = DEVICE_DT_GET(DT_CHOSEN(sbus_uart));

int sbus_parseframe_chan(const struct device *dev, int chan)
{
	// 检查帧是否为空
	struct sbus_driver_data *data = dev->data;

	int curr_time = k_uptime_get();
	if (curr_time - data->recv_time > 100) {
		return 1024;
	}

	// 数据转换成通道值
	switch (chan) {
	case 0:
		return (data->data[1] >> 0 | (data->data[2] << 8)) & 0x07FF;
	case 1:
		return (data->data[2] >> 3 | (data->data[3] << 5)) & 0x07FF;
	case 2:
		return (data->data[3] >> 6 | (data->data[4] << 2) | data->data[5] << 10) & 0x07FF;
	case 3:
		return (data->data[5] >> 1 | (data->data[6] << 7)) & 0x07FF;
	case 4:
		return (data->data[6] >> 4 | (data->data[7] << 4)) & 0x07FF;
	case 5:
		return (data->data[7] >> 7 | (data->data[8] << 1) | data->data[9] << 9) & 0x07FF;
	case 6:
		return (data->data[9] >> 2 | (data->data[10] << 6)) & 0x07FF;
	case 7:
		return (data->data[10] >> 5 | (data->data[11] << 3)) & 0x07FF;
	case 8:
		return (data->data[12] << 0 | (data->data[13] << 8)) & 0x07FF;
	case 9:
		return (data->data[13] >> 3 | (data->data[14] << 5)) & 0x07FF;
	case 10:
		return (data->data[14] >> 6 | (data->data[16] << 2) | data->data[15] << 10) &
		       0x07FF;
	case 11:
		return (data->data[16] >> 1 | (data->data[17] << 7)) & 0x07FF;
	case 12:
		return (data->data[17] >> 4 | (data->data[18] << 4)) & 0x07FF;
	case 13:
		return (data->data[18] >> 7 | (data->data[19] << 1) | data->data[20] << 9) & 0x07FF;
	case 14:
		return (data->data[20] >> 2 | (data->data[21] << 6)) & 0x07FF;
	case 15:
		return (data->data[21] >> 5 | (data->data[22] << 3)) & 0x07FF;
	default:
		return 1024;
	}
}

void sbus_parseframe(const struct device *dev)
{
	struct sbus_driver_data *data = dev->data;

	for (int i = 0; i < 16; i++) {
		data->channels[i] = sbus_parseframe_chan(dev, i);
	}

	data->frameLost = (data->data[23] & 0x04) >> 2;
	data->failSafe = (data->data[23] & 0x08) >> 3;
	data->digitalChannels[0] = (data->data[23] & 0x01);
	data->digitalChannels[1] = (data->data[23] & 0x02) >> 1;
}

static int16_t find_begin(const uint8_t *data, uint16_t len)
{
	if (len < 25) {
		return -1;
	}
	for (int i = len - 1; i >= 24; i--) {
		if (data[i - 24] == 0x0F && data[i] == 0x00) {
			return i - 24;
		}
	}
	return -1;
}

// async serial callback
static void uart_callback(const struct device *dev, struct uart_event *evt, void *user_data)
{
	int err;

	struct device *sbus_dev = (struct device *)user_data;
	struct sbus_driver_data *data = sbus_dev->data;
	switch (evt->type) {
	case UART_RX_RDY: {
		uint8_t *p = &(evt->data.rx.buf[evt->data.rx.offset]);
		uint16_t len = evt->data.rx.len;

		uint8_t *ptr_offset = p + find_begin(p, len);
		if (ptr_offset >= p && ptr_offset + 25 <= p + len) {
			memcpy(data->data, ptr_offset, 25);
		} else {
			return;
		}

		data->recv_time = k_uptime_get();
		break;
	}

	case UART_RX_BUF_REQUEST: {
		uint8_t *buf = NULL; // 使用 void* 类型

		err = k_mem_slab_alloc(&sbus_uart_slab, (void **)&buf, K_NO_WAIT);
		if (err == 0 && ((uintptr_t)buf & 0x3) == 0 && buf != NULL) {
			uart_rx_buf_rsp(dev, buf, BUF_SIZE);
		}
		break;
	}

	case UART_RX_BUF_RELEASED: {
		void *buf = evt->data.rx_buf.buf;
		if (buf != NULL) {
			k_mem_slab_free(&sbus_uart_slab, buf);
		}
		break;
	}

	case UART_RX_DISABLED:
		break;

	// 添加错误处理
	case UART_RX_STOPPED: {
		uint8_t *buf = NULL;
		err = k_mem_slab_alloc(&sbus_uart_slab, (void **)&buf, K_NO_WAIT);
		if (err == 0 && buf != NULL && ((uintptr_t)buf & 0x3) == 0) {
			memset(buf, 0, BUF_SIZE);
			err = uart_rx_enable(dev, buf, BUF_SIZE, 100);
			if (err) {
				LOG_ERR("Failed to enable RX: %d", err);
				k_mem_slab_free(&sbus_uart_slab, buf);
			}
		} else {
			LOG_ERR("Failed to allocate memory: %d", err);
		}
		break;
	}
	default:
		break;
	}
}

// 初始化sbus，打开DMA接收，关闭DMA接收半完成中断
static int sbus_init(const struct device *dev)
{
	uint8_t *buf;
	int err;

	if (!device_is_ready(uart_dev)) {
		LOG_ERR("UART device not ready");
		return -ENODEV;
	}

	// 将所有通道值初始化为中间值
	struct sbus_driver_data *data = dev->data;
	for (int i = 0; i < 16; i++) {
		data->channels[i] = 1024;
	}

	// 确保 UART 设备初始化完成
	err = uart_callback_set(uart_dev, uart_callback, (void *)dev);
	if (err) {
		LOG_ERR("Failed to set callback: %d", err);
		return err;
	}

	err = k_mem_slab_alloc(&sbus_uart_slab, (void **)&buf, K_NO_WAIT);
	if (err) {
		LOG_ERR("Failed to allocate memory: %d", err);
		return err;
	}
	memset(buf, 0, BUF_SIZE);

	// 启用接收前确保串口配置正确
	const struct uart_config config = {.baudrate = 100000,
					   .parity = UART_CFG_PARITY_EVEN,
					   .stop_bits = UART_CFG_STOP_BITS_2,
					   .data_bits = UART_CFG_DATA_BITS_8,
					   .flow_ctrl = UART_CFG_FLOW_CTRL_NONE};

	err = uart_configure(uart_dev, &config);
	if (err) {
		LOG_ERR("Failed to configure UART: %d", err);
		k_mem_slab_free(&sbus_uart_slab, buf);
		return err;
	}

	// 在启用接收前打印 DMA 状态
	// LOG_INF("DMA status: %x", DMA1->ISR); // 需要包含相应的寄存器头文件
	uart_rx_disable(uart_dev);
	err = uart_rx_enable(uart_dev, buf, BUF_SIZE, 100);
	if (err) {
		LOG_ERR("Failed to enable RX: %d", err);
		k_mem_slab_free(&sbus_uart_slab, buf);
		return err;
	}

	return 0;
}

// 获取通道百分比
float sbus_getchannel_percentage(const struct device *dev, uint8_t channelid)
{
	int chan = sbus_parseframe_chan(dev, channelid);
	float scale = 2.0f / (SBUS_MAX - SBUS_MIN);
	float out = (int16_t)(chan - 1024) * scale;
	return out;
}

// 获取通道数字值
int sbus_getchannel_digital(const struct device *dev, uint8_t channelid)
{
	int chan = sbus_parseframe_chan(dev, channelid);
	return chan;
}

static struct sbus_driver_api sbus_api = {
	.getchannel_percentage = sbus_getchannel_percentage,
	.getchannel_digital = sbus_getchannel_digital,
};

#define SBUS_DT_DATA_DEFINE(inst) static struct sbus_driver_data sbus_data_##inst = {0};
#define SBUS_DT_CFG_DEFINE(inst)                                                                   \
	static const struct sbus_driver_config sbus_cfg_##inst = {                                 \
		.startByte = DT_PROP(DT_DRV_INST(inst), start_byte),                               \
		.endByte = DT_PROP(DT_DRV_INST(inst), end_byte),                                   \
		.frameSize = DT_PROP(DT_DRV_INST(inst), frame_size),                               \
		.inputChannels = DT_PROP(DT_DRV_INST(inst), input_channels),                       \
	};

#define SBUS_DT_DEVICE_DEFINE(inst)                                                                \
	DEVICE_DT_INST_DEFINE(inst, sbus_init, NULL, &sbus_data_##inst, &sbus_cfg_##inst,          \
			      POST_KERNEL, CONFIG_SBUS_INIT_PRIORITY, &sbus_api);

#define SBUS_DEVICE_INIT(inst)                                                                     \
	SBUS_DT_DATA_DEFINE(inst);                                                                 \
	SBUS_DT_CFG_DEFINE(inst);                                                                  \
	SBUS_DT_DEVICE_DEFINE(inst);

DT_INST_FOREACH_STATUS_OKAY(SBUS_DEVICE_INIT)
