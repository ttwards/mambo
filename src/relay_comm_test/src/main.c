/*
 * 串口双向通信测试包 (relay_comm 的对端)
 *
 * 与 relay_comm 通过 UART 双协议进行双向 4 float 数据收发,
 * 定时发送测试数据并统计收发, 验证双向通信稳定性。
 */

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <math.h>

#include <ares/interface/uart/uart.h>
#include <ares/protocol/dual/dual_protocol.h>
#include <ares/ares_comm.h>

LOG_MODULE_REGISTER(comm_test, LOG_LEVEL_DBG);

/* ==================== 串口通信 ==================== */

/* UART 设备, 和 relay_comm 用同一个串口 (硬件上交叉连接) */
#define UART_NODE DT_NODELABEL(usart6)
static const struct device *uart_dev = DEVICE_DT_GET(UART_NODE);

ARES_UART_INTERFACE_DEFINE(uart_interface);
DUAL_PROPOSE_PROTOCOL_DEFINE(uart_protocol);

/*
 * SYNC ID 与 relay_comm 镜像:
 * 本板 TX(0x0200) -> relay_comm RX(0x0200)
 * 本板 RX(0x0100) <- relay_comm TX(0x0100)
 */
#define SYNC_ID_TX 0x0200
#define SYNC_ID_RX 0x0100

/* 4 个 float = 16 字节 */
#define DATA_LEN 16
static float tx_data[4];
static float rx_data[4];

static sync_table_t *tx_pack;
static sync_table_t *rx_pack;

/* 统计 */
static uint32_t rx_cnt;
static uint32_t tx_cnt;
static uint32_t tx_fail;

/* 接收回调 */
static void rx_cb(int status)
{
	if (status == SYNC_PACK_STATUS_DONE) {
		rx_cnt++;
		/* 每 200 帧打印一次接收数据 */
		if (rx_cnt % 200 == 0) {
			LOG_INF("[RX #%u] %.1f %.1f %.1f %.1f",
				rx_cnt,
				(double)rx_data[0], (double)rx_data[1],
				(double)rx_data[2], (double)rx_data[3]);
		}
	}
}

/* 通信初始化 */
static int uart_comm_init(void)
{
	if (!device_is_ready(uart_dev)) {
		LOG_ERR("UART 设备未就绪");
		return -ENODEV;
	}

	ares_uart_init_dev(&uart_interface, uart_dev);
	int ret = ares_bind_interface(&uart_interface, &uart_protocol);
	if (ret < 0) {
		LOG_ERR("UART 绑定失败: %d", ret);
		return ret;
	}

	tx_pack = dual_sync_add(&uart_protocol, SYNC_ID_TX, (uint8_t *)tx_data,
				DATA_LEN, NULL);
	rx_pack = dual_sync_add(&uart_protocol, SYNC_ID_RX, (uint8_t *)rx_data,
				DATA_LEN, rx_cb);

	LOG_INF("通信初始化完成 (TX:0x%04X RX:0x%04X len=%d)",
		SYNC_ID_TX, SYNC_ID_RX, DATA_LEN);
	return 0;
}

static int uart_tx_flush(void)
{
	return dual_sync_flush(&uart_protocol, tx_pack);
}

/* ==================== 主函数 ==================== */

int main(void)
{
	LOG_INF("===== 串口双向通信测试 =====");

	if (uart_comm_init() < 0) {
		return -1;
	}

	float t = 0.0f;

	while (1) {
		/* 填充测试数据: 4 个 float */
		tx_data[0] = sinf(t);        /* 正弦波 */
		tx_data[1] = (float)tx_cnt;  /* 发送计数 */
		tx_data[2] = 1.0f;           /* 心跳标记: 1=在线 */
		tx_data[3] = 0.0f;           /* 备用 */
		t += 0.1f;

		if (uart_tx_flush() != 0) {
			tx_fail++;
		}
		tx_cnt++;

		/* 每 200 帧打印一次统计 (200*3ms = 600ms) */
		if (tx_cnt % 200 == 0) {
			LOG_INF("[STAT] TX=%u RX=%u fail=%u 丢包=%d",
				tx_cnt, rx_cnt, tx_fail,
				tx_cnt - rx_cnt);
		}

		k_msleep(10); /* 100Hz */
	}

	return 0;
}
