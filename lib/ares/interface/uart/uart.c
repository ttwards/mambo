/*
 * Copyright (c) 2025 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/linker/section_tags.h>
#include <zephyr/logging/log.h>
#include <zephyr/net_buf.h>

#include "ares/interface/ares_interface.h"
#include "ares/interface/uart/uart.h"
#include "zephyr/sys/ring_buffer.h"
#include <ares/protocol/ares_protocol.h>
#include <string.h>

LOG_MODULE_REGISTER(ares_uart, LOG_LEVEL_INF); // 日志级别可以按需调整

/* === 配置常量 === */
#define ARES_UART_PROCESSING_THREAD_STACK_SIZE CONFIG_ARES_UART_THREAD_STACK_SIZE
#define ARES_UART_PROCESSING_THREAD_PRIORITY   K_PRIO_PREEMPT(5)
#define ARES_UART_RX_INACTIVE_TIMEOUT          10

struct ares_uart_tx_meta {
	ares_interface_tx_done_cb_t tx_done;
	void *tx_user_data;
};

NET_BUF_POOL_DEFINE(uart_net_buf_pool, 16, 128, sizeof(struct ares_uart_tx_meta),
		    NULL); // 增加缓冲区数量和大小
K_MEM_SLAB_DEFINE_IN_SECT(uart_rx_slab, __nocache, ARES_UART_BLOCK_SIZE, 8, 4);

static uint8_t __nocache __aligned(32) uart_tx_dma_buf[MAX_FRAME_PAYLOAD_SIZE];

static void ares_uart_complete_tx(struct AresInterface *interface, struct net_buf *buf, int status)
{
	struct ares_uart_tx_meta *meta;

	if (buf == NULL) {
		return;
	}

	meta = net_buf_user_data(buf);
	if (meta && meta->tx_done) {
		meta->tx_done(interface, buf, status, meta->tx_user_data);
	}
}

/**
 * @brief 发送一个数据包到队列中
 *
 * 这个函数现在是非阻塞的。它将数据包（net_buf）放入发送队列后立即返回。
 * 真正的发送操作由 ares_uart_tx_thread_entry 线程完成。
 * @param interface 接口实例
 * @param buf 包含载荷数据的 net_buf
 * @return 0 on success, -ENOMEM if the queue is full.
 */
int ares_uart_send(struct AresInterface *interface, struct net_buf *buf)
{
	return ares_uart_send_with_callback(interface, buf, NULL, NULL);
}

int ares_uart_send_with_callback(struct AresInterface *interface, struct net_buf *buf,
				 ares_interface_tx_done_cb_t cb, void *user_data)
{
	struct AresUartInterface *uart_if = interface->priv_data;
	struct ares_uart_tx_meta *meta = net_buf_user_data(buf);

	if (meta) {
		meta->tx_done = cb;
		meta->tx_user_data = user_data;
	}

	if (k_msgq_put(&uart_if->tx_msgq, &buf, K_NO_WAIT) != 0) {
		LOG_ERR("TX message queue is full. Dropping packet!");
		ares_uart_complete_tx(interface, buf, -ENOMEM);
		net_buf_unref(buf);
		return -ENOMEM;
	}

	return 0;
}

int ares_uart_send_raw(struct AresInterface *interface, uint8_t *data, uint16_t len)
{
	struct AresUartInterface *uart_if = interface->priv_data;
	int err;

	err = uart_tx(uart_if->uart_dev, data, len, SYS_FOREVER_US);
	if (err != 0) {
		LOG_ERR("uart_tx failed with error %d", err);
		return -EIO;
	}
	return 0;
}

struct net_buf *ares_uart_interface_alloc_buf(struct AresInterface *interface)
{
	struct net_buf *buf = net_buf_alloc(&uart_net_buf_pool, K_NO_WAIT);
	if (buf) {
		struct ares_uart_tx_meta *meta = net_buf_user_data(buf);
		if (meta) {
			meta->tx_done = NULL;
			meta->tx_user_data = NULL;
		}
	}
	return buf;
}

uint32_t ares_uart_caps(struct AresInterface *interface)
{
	ARG_UNUSED(interface);

	return ARES_INTERFACE_CAP_STREAM | ARES_INTERFACE_CAP_TX_COMPLETE |
	       ARES_INTERFACE_CAP_TX_QUEUE;
}

size_t ares_uart_mtu(struct AresInterface *interface)
{
	ARG_UNUSED(interface);

	return MAX_FRAME_PAYLOAD_SIZE;
}

/**
 * @brief (新增) 发送线程入口函数
 *
 * 该线程是数据发送的唯一执行者。它从队列中获取数据，
 * 并通过信号量与 UART TX 完成中断同步，实现流量控制。
 */
static void ares_uart_tx_thread_entry(void *p1, void *p2, void *p3)
{
	struct AresInterface *interface = p1;
	struct AresUartInterface *uart_if = interface->priv_data;
	struct net_buf *buf;
	int err;

	while (1) {
		k_msgq_get(&uart_if->tx_msgq, &buf, K_FOREVER);
		k_sem_take(&uart_if->tx_sem, K_FOREVER);
		if (buf->len > sizeof(uart_tx_dma_buf)) {
			LOG_ERR("UART frame too large for DMA buffer: %u", buf->len);
			ares_uart_complete_tx(interface, buf, -EMSGSIZE);
			net_buf_unref(buf);
			k_sem_give(&uart_if->tx_sem);
			continue;
		}
		memcpy(uart_tx_dma_buf, buf->data, buf->len);
		uart_if->current_tx_buf = buf;
		err = uart_tx(uart_if->uart_dev, uart_tx_dma_buf, buf->len, SYS_FOREVER_US);
		if (err != 0) {
			LOG_ERR("uart_tx failed with error %d, even with flow control!", err);
			k_sem_give(&uart_if->tx_sem);
			ares_uart_complete_tx(interface, uart_if->current_tx_buf, -EIO);
			net_buf_unref(uart_if->current_tx_buf);
			uart_if->current_tx_buf = NULL;
		}
	}
}

// async serial callback
static void uart_callback(const struct device *dev, struct uart_event *evt, void *user_data)
{
	struct AresUartInterface *uart_if = user_data;
	int err;

	switch (evt->type) {
	/* --- TX 事件处理 --- */
	case UART_TX_DONE:
		if (uart_if->current_tx_buf != NULL) {
			ares_uart_complete_tx(uart_if->interface, uart_if->current_tx_buf, 0);
			net_buf_unref(uart_if->current_tx_buf);
			uart_if->current_tx_buf = NULL;
		}
		k_sem_give(&uart_if->tx_sem);
		break;

	case UART_TX_ABORTED:
		LOG_WRN("UART TX aborted");
		if (uart_if->current_tx_buf != NULL) {
			ares_uart_complete_tx(uart_if->interface, uart_if->current_tx_buf,
					      -ECANCELED);
			net_buf_unref(uart_if->current_tx_buf);
			uart_if->current_tx_buf = NULL;
		}
		k_sem_give(&uart_if->tx_sem);
		break;

	/* --- RX 事件处理 (保持不变) --- */
	case UART_RX_RDY: {
		if (evt->data.rx.len > 0) {
			ring_buf_put(&uart_if->uart_rb, evt->data.rx.buf + evt->data.rx.offset,
				     evt->data.rx.len);
			k_sem_give(&uart_if->sem);
		}
		break;
	}

	case UART_RX_BUF_REQUEST: {
		uint8_t *buf;
		err = k_mem_slab_alloc(&uart_rx_slab, (void **)&buf, K_NO_WAIT);
		if (err != 0) {
			LOG_ERR("Failed to allocate memory for RX buffer.");
			return;
		}
		err = uart_rx_buf_rsp(uart_if->uart_dev, buf, ARES_UART_BLOCK_SIZE);
		if (err != 0) {
			LOG_ERR("Failed to provide new buffer.");
			k_mem_slab_free(&uart_rx_slab, (void **)&buf);
			return;
		}
		break;
	}

	case UART_RX_BUF_RELEASED:
		k_mem_slab_free(&uart_rx_slab, (void *)evt->data.rx_buf.buf);
		break;

	case UART_RX_DISABLED:
		break;

	case UART_RX_STOPPED:
		break;
	}
}

void frame_parser_process(struct AresInterface *interface, struct ring_buf *rb)
{
	uint8_t byte;

	while (ring_buf_get(rb, &byte, 1) > 0) {
		interface->protocol->api->handle_byte(interface->protocol, byte);
	}
}

// 接收线程入口函数 (保持不变)
void ares_uart_thread_entry(void *p1, void *p2, void *p3)
{
	struct AresInterface *interface = p1;
	struct AresUartInterface *uart_if = interface->priv_data;

	while (1) {
		k_sem_take(&uart_if->sem, K_FOREVER);
		frame_parser_process(interface, &uart_if->uart_rb);
	}
}

void ares_uart_init_dev(struct AresInterface *interface, const struct device *uart_dev)
{
	struct AresUartInterface *uart_if = interface->priv_data;
	uart_if->interface = interface;
	uart_if->uart_dev = (struct device *)uart_dev;
}

// 初始化函数 (修改)
int ares_uart_init(struct AresInterface *interface)
{
	struct AresUartInterface *uart_if = interface->priv_data;
	uart_if->interface = interface;

	if (!device_is_ready(uart_if->uart_dev)) {
		LOG_ERR("UART device not ready");
		return -ENODEV;
	}

	int err = 0;

	/* --- 初始化 RX 部分 (和原来一样) --- */
	ring_buf_init(&uart_if->uart_rb, sizeof(uart_if->uart_rb_buf), uart_if->uart_rb_buf);
	k_sem_init(&uart_if->sem, 0, 1);

	err = uart_callback_set(uart_if->uart_dev, uart_callback, (void *)uart_if);
	if (err) {
		LOG_ERR("uart_callback_set failed: %d", err);
		return err;
	}

	uint8_t *buf;
	err = k_mem_slab_alloc(&uart_rx_slab, (void **)&buf, K_NO_WAIT);
	__ASSERT(err == 0, "Failed to alloc slab");
	err = uart_rx_enable(uart_if->uart_dev, buf, ARES_UART_BLOCK_SIZE,
			     ARES_UART_RX_INACTIVE_TIMEOUT);
	__ASSERT(err == 0, "Failed to enable rx");

	k_thread_create(&uart_if->thread, uart_if->thread_stack,
			ARES_UART_PROCESSING_THREAD_STACK_SIZE, ares_uart_thread_entry,
			(void *)interface, NULL, NULL, ARES_UART_PROCESSING_THREAD_PRIORITY, 0,
			K_NO_WAIT);
	k_thread_name_set(&uart_if->thread, "ares_uart_rx");

	/* --- 新增: 初始化 TX 部分 --- */
	uart_if->current_tx_buf = NULL;
	k_msgq_init(&uart_if->tx_msgq, uart_if->tx_msgq_buffer, sizeof(struct net_buf *),
		    ARES_UART_TX_QUEUE_SIZE);
	k_sem_init(&uart_if->tx_sem, 1, 1);

	k_thread_create(&uart_if->tx_thread, uart_if->tx_thread_stack,
			ARES_UART_TX_THREAD_STACK_SIZE, ares_uart_tx_thread_entry,
			(void *)interface, NULL, NULL, ARES_UART_PROCESSING_THREAD_PRIORITY, 0,
			K_NO_WAIT);
	k_thread_name_set(&uart_if->tx_thread, "ares_uart_tx");

	LOG_INF("Ares UART Interface initialized with TX queue.");
	return 0;
}
