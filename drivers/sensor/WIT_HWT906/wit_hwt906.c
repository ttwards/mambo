/*
 * Copyright (c) 2024 Librgod
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT wit_hwt906

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(wit_hwt906, LOG_LEVEL_INF);

#define DATA(idx) (short)((short)buf[idx + 1] << 8 | buf[idx]) // 组合高低字节

struct wit_hwt906_data {
	const struct device *dev; // 指向自身的指针

	/*缓存解析后的数据(后面统一单位)*/
	struct {
		uint8_t time[8];  // 时间戳，对应 0x50 包
		int16_t acc[3];   // 加速度，对应 0x51 包: Ax, Ay, Az
		int16_t gyro[3];  // 角速度，对应 0x52 包: Wx, Wy, Wz
		int16_t angle[3]; // 角度，对应 0x53 包: Roll, Pitch, Yaw
		int16_t mag[3];   // 磁场，对应 0x54 包: Hx, Hy, Hz
		int16_t temp;     // 温度，对应 0x51 包: TL, TH
		int16_t Vol;      // 电压，对应 0x52 包: VL, VH
		int16_t Version;  // 版本号，对应 0x53 包: VerL, VerH
	} raw;

	uint8_t parse_buf[11];
	uint8_t last_acc_packet[11];
	uint8_t last_angle_packet[11];
	uint8_t parse_len;
	uint32_t total_bytes_received;
	uint32_t total_bytes_dropped;
	uint32_t bad_header_count;
	uint32_t bad_checksum_count;
	uint32_t unknown_packet_count;
	uint32_t packet_count[5];
	uint32_t last_diag_ms;
};

static void wit_hwt906_parse_packet(struct wit_hwt906_data *data, const uint8_t *buf)
{
	/* 解析数据 (低8位在前，高8位在后) */
	switch (buf[1]) {
	case 0x50: // 时间包
		for (int i = 0; i < 8; i++) {
			data->raw.time[i] = buf[i + 2];
		}
		data->packet_count[0]++;
		break;
	case 0x51:                          // 加速度包
		memcpy(data->last_acc_packet, buf, sizeof(data->last_acc_packet));
		data->raw.acc[0] = DATA(2); // AxL, AxH
		data->raw.acc[1] = DATA(4); // AyL, AyH
		data->raw.acc[2] = DATA(6); // AzL, AzH
		data->raw.temp = DATA(8);   // TL, TH
		data->packet_count[1]++;
		break;
	case 0x52:                           // 角速度包
		data->raw.gyro[0] = DATA(2); // WxL, WxH
		data->raw.gyro[1] = DATA(4); // WyL, WyH
		data->raw.gyro[2] = DATA(6); // WzL, WzH
		data->raw.Vol = DATA(8);     // VL, VH
		data->packet_count[2]++;
		break;
	case 0x53:                            // 角度包
		memcpy(data->last_angle_packet, buf, sizeof(data->last_angle_packet));
		data->raw.angle[0] = DATA(2); // RollL, RollH
		data->raw.angle[1] = DATA(4); // PitchL, PitchH
		data->raw.angle[2] = DATA(6); // YawL, YawH
		data->raw.Version = DATA(8);  // VerL, VerH
		data->packet_count[3]++;
		break;
	case 0x54:                          // 磁场包
		data->raw.mag[0] = DATA(2); // HxL, HxH
		data->raw.mag[1] = DATA(4); // HyL, HyH
		data->raw.mag[2] = DATA(6); // HzL, HzH
		data->raw.temp = DATA(8);   // TL, TH
		data->packet_count[4]++;
		break;
	default:
		// 其他类型的包我们暂时不关心
		data->unknown_packet_count++;
		break;
	}
}

static void wit_hwt906_parse_byte(struct wit_hwt906_data *data, uint8_t byte)
{
	uint8_t sum = 0;

	data->total_bytes_received++;

	if (data->parse_len == 0U) {
		if (byte != 0x55) {
			data->bad_header_count++;
			return;
		}
		data->parse_buf[data->parse_len++] = byte;
		return;
	}

	data->parse_buf[data->parse_len++] = byte;
	if (data->parse_len < sizeof(data->parse_buf)) {
		return;
	}

	for (int i = 0; i < 10; i++) {
		sum = (uint8_t)(sum + data->parse_buf[i]);
	}

	if (sum == data->parse_buf[10]) {
		wit_hwt906_parse_packet(data, data->parse_buf);
		data->parse_len = 0U;
		return;
	}

	data->bad_checksum_count++;
	for (int i = 1; i < sizeof(data->parse_buf); i++) {
		if (data->parse_buf[i] == 0x55) {
			uint8_t remaining = sizeof(data->parse_buf) - i;

			memmove(data->parse_buf, &data->parse_buf[i], remaining);
			data->parse_len = remaining;
			return;
		}
	}

	data->parse_len = 0U;
}

static void wit_hwt906_uart_cb(const struct device *uart_dev, void *user_data)
{
	const struct device *sensor_dev = user_data;
	struct wit_hwt906_data *data = sensor_dev->data;

	if (!uart_irq_update(uart_dev)) {
		return;
	}

	if (uart_irq_rx_ready(uart_dev)) {
		uint8_t buffer[64];
		// 从硬件FIFO中读取数据到临时缓冲区
		int len = uart_fifo_read(uart_dev, buffer, sizeof(buffer));

		if (len > 0) {
			for (int i = 0; i < len; i++) {
				wit_hwt906_parse_byte(data, buffer[i]);
			}
		}
	}
}

/* 从设备树配置里获取 UART 设备的指针 */
struct wit_hwt906_config {
	const struct device *uart_dev;
};

static int wit_hwt906_init(const struct device *dev)
{
	const struct wit_hwt906_config *cfg = dev->config;

	printk("[Driver] WIT HWT906 initializing...\n");

	if (!device_is_ready(cfg->uart_dev)) {
		printk("[Driver] Error: UART device not ready\n");
		return -ENODEV;
	}

	uart_irq_callback_user_data_set(cfg->uart_dev, wit_hwt906_uart_cb, (void *)dev);

	/* 开启接收中断 */
	uart_irq_rx_enable(cfg->uart_dev);

	printk("[Driver] UART interrupts enabled.\n");
	return 0;
}

static int wit_hwt906_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct wit_hwt906_data *data = dev->data;
	uint32_t now_ms = k_uptime_get_32();

	if (now_ms - data->last_diag_ms >= 1000U) {
		int16_t angle_raw[3];
		int16_t acc_raw[3];
		int16_t version_raw;
		uint8_t acc_packet[11];
		uint8_t angle_packet[11];
		unsigned int key;

		data->last_diag_ms = now_ms;

		key = irq_lock();
		angle_raw[0] = data->raw.angle[0];
		angle_raw[1] = data->raw.angle[1];
		angle_raw[2] = data->raw.angle[2];
		acc_raw[0] = data->raw.acc[0];
		acc_raw[1] = data->raw.acc[1];
		acc_raw[2] = data->raw.acc[2];
		version_raw = data->raw.Version;
		memcpy(acc_packet, data->last_acc_packet, sizeof(acc_packet));
		memcpy(angle_packet, data->last_angle_packet, sizeof(angle_packet));
		irq_unlock(key);

		LOG_INF("diag pkts 50=%u 51=%u 52=%u 53=%u 54=%u unknown=%u bad_head=%u bad_sum=%u rx=%u drop=%u parse_len=%u angle_raw=(%d %d %d) angle_deg=(%.2f %.2f %.2f) acc_raw=(%d %d %d) ver=0x%04x",
			data->packet_count[0], data->packet_count[1], data->packet_count[2],
			data->packet_count[3], data->packet_count[4], data->unknown_packet_count,
			data->bad_header_count, data->bad_checksum_count,
			data->total_bytes_received, data->total_bytes_dropped, data->parse_len,
			angle_raw[0], angle_raw[1], angle_raw[2],
			(double)(angle_raw[0] / 32768.0 * 180.0),
			(double)(angle_raw[1] / 32768.0 * 180.0),
			(double)(angle_raw[2] / 32768.0 * 180.0),
			acc_raw[0], acc_raw[1], acc_raw[2], (uint16_t)version_raw);
		LOG_INF("diag raw pkt51=%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x pkt53=%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
			acc_packet[0], acc_packet[1], acc_packet[2], acc_packet[3],
			acc_packet[4], acc_packet[5], acc_packet[6], acc_packet[7],
			acc_packet[8], acc_packet[9], acc_packet[10],
			angle_packet[0], angle_packet[1], angle_packet[2], angle_packet[3],
			angle_packet[4], angle_packet[5], angle_packet[6], angle_packet[7],
			angle_packet[8], angle_packet[9], angle_packet[10]);
	}

	if (data->packet_count[3] == 0U) {
		return -EAGAIN;
	}
	return 0;
}

static void wit_convert_acc(struct sensor_value *val, int16_t raw)
{
	// 公式: raw / 32768 * 16 * 9.80665
	sensor_value_from_double(val, raw / 32768.0 * 16.0 * 9.80665);
}

static void wit_convert_gyro(struct sensor_value *val, int16_t raw)
{
	// 公式: raw / 32768 * 2000 * (PI / 180) -> 转为弧度/秒
	// 2000度/秒 约等于 34.906585 弧度/秒
	sensor_value_from_double(val, raw / 32768.0 * 34.906585);
}

static void wit_convert_angle(struct sensor_value *val, int16_t raw)
{
	// 公式: raw / 32768 * 180 (度)
	sensor_value_from_double(val, raw / 32768.0 * 180.0);
}

static void wit_convert_temp(struct sensor_value *val, int16_t raw)
{
	// 公式: raw / 100 (摄氏度)
	sensor_value_from_double(val, raw / 100.0);
}

static void wit_convert_voltage(struct sensor_value *val, int16_t raw)
{
	// 公式: raw / 100 (伏特)
	sensor_value_from_double(val, raw / 100.0);
}

static int wit_hwt906_channel_get(const struct device *dev, enum sensor_channel chan,
				  struct sensor_value *val)
{
	struct wit_hwt906_data *data = dev->data;
	unsigned int key;
	int16_t raw[3];
	int16_t raw_single;
	uint8_t time[8];

	switch (chan) {
	case SENSOR_CHAN_ACCEL_XYZ: // 支持加速度接口
		key = irq_lock();
		raw[0] = data->raw.acc[0];
		raw[1] = data->raw.acc[1];
		raw[2] = data->raw.acc[2];
		irq_unlock(key);
		wit_convert_acc(&val[0], raw[0]);
		wit_convert_acc(&val[1], raw[1]);
		wit_convert_acc(&val[2], raw[2]);
		break;

	case SENSOR_CHAN_GYRO_XYZ: // 支持角速度接口
		key = irq_lock();
		raw[0] = data->raw.gyro[0];
		raw[1] = data->raw.gyro[1];
		raw[2] = data->raw.gyro[2];
		irq_unlock(key);
		wit_convert_gyro(&val[0], raw[0]);
		wit_convert_gyro(&val[1], raw[1]);
		wit_convert_gyro(&val[2], raw[2]);
		break;

	case SENSOR_CHAN_ROTATION: // 支持欧拉角接口 (Roll, Pitch, Yaw)
		key = irq_lock();
		raw[0] = data->raw.angle[0];
		raw[1] = data->raw.angle[1];
		raw[2] = data->raw.angle[2];
		irq_unlock(key);
		wit_convert_angle(&val[0], raw[0]);
		wit_convert_angle(&val[1], raw[1]);
		wit_convert_angle(&val[2], raw[2]);
		break;

	case SENSOR_CHAN_MAGN_XYZ: // 支持磁场接口
		key = irq_lock();
		raw[0] = data->raw.mag[0];
		raw[1] = data->raw.mag[1];
		raw[2] = data->raw.mag[2];
		irq_unlock(key);
		sensor_value_from_double(&val[0], raw[0]);
		sensor_value_from_double(&val[1], raw[1]);
		sensor_value_from_double(&val[2], raw[2]);
		break;

	case SENSOR_CHAN_DIE_TEMP: // 支持芯片温度接口
		key = irq_lock();
		raw_single = data->raw.temp;
		irq_unlock(key);
		wit_convert_temp(&val[0], raw_single);
		break;

	case SENSOR_CHAN_VOLTAGE: // Zephyr 标准电压通道
		key = irq_lock();
		raw_single = data->raw.Vol;
		irq_unlock(key);
		wit_convert_voltage(&val[0], raw_single);
		break;

	/* 新增：时间戳通道 (使用 PRIV_START 自定义) */
	case SENSOR_CHAN_PRIV_START: // 自定义通道起始位置
		// 时间戳格式: YY MM DD hh mm ss ms ms，这里只使用时、分、秒、毫秒
		key = irq_lock();
		for (int i = 0; i < 8; i++) {
			time[i] = data->raw.time[i];
		}
		irq_unlock(key);
		val[0].val1 = time[3];                    // hh
		val[1].val1 = time[4];                    // mm
		val[2].val1 = time[5];                    // ss
		val[3].val1 = (time[7] << 8 | time[6]);   // ms
		break;

	/* 新增：版本号通道 */
	case SENSOR_CHAN_PRIV_START + 1:
		key = irq_lock();
		raw_single = data->raw.Version;
		irq_unlock(key);
		val[0].val1 = raw_single;
		val[0].val2 = 0;
		break;

	default:
		return -ENOTSUP; // 不支持的通道返回错误
	}

	return 0;
}

static const struct sensor_driver_api wit_hwt906_api = {
	.sample_fetch = wit_hwt906_sample_fetch,
	.channel_get = wit_hwt906_channel_get,
};

/* 实例化 Config 结构体 (从设备树里拿 UART 设备) */
#define WIT_HWT906_CONFIG_DEFINE(inst)                                                             \
	static const struct wit_hwt906_config wit_hwt906_config_##inst = {                         \
		.uart_dev = DEVICE_DT_GET(DT_INST_BUS(inst)),                                      \
	};

/* 实例化 Data 结构体 */
#define WIT_HWT906_DATA_DEFINE(inst) static struct wit_hwt906_data wit_hwt906_data_##inst;

/* 初始化宏 */
#define WIT_HWT906_DEFINE(inst)                                                                    \
	WIT_HWT906_CONFIG_DEFINE(inst)                                                             \
	WIT_HWT906_DATA_DEFINE(inst)                                                               \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, wit_hwt906_init, NULL, &wit_hwt906_data_##inst,         \
				     &wit_hwt906_config_##inst, POST_KERNEL,                       \
				     CONFIG_SENSOR_INIT_PRIORITY, &wit_hwt906_api);

DT_INST_FOREACH_STATUS_OKAY(WIT_HWT906_DEFINE)
