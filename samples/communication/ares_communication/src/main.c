// main.c
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net_buf.h>
#include <ares/interface/usb/usb_bulk.h>
#if IS_ENABLED(CONFIG_UART_INTERFACE)
#include <ares/interface/uart/uart.h>
#endif
#include <ares/protocol/dual/dual_protocol.h>
#include <ares/ares_comm.h>

#include <zephyr/debug/thread_analyzer.h>

LOG_MODULE_REGISTER(main_app, LOG_LEVEL_INF);

#if IS_ENABLED(CONFIG_UART_INTERFACE)
#define UART_DEV DT_NODELABEL(usart6)
static const struct device *uart_dev = DEVICE_DT_GET(UART_DEV);
#endif

DUAL_PROPOSE_PROTOCOL_DEFINE(dual_protocol);
#if IS_ENABLED(CONFIG_UART_INTERFACE)
DUAL_PROPOSE_PROTOCOL_DEFINE(uart_protocol);
#endif
// 建议对UART使用CRC校验
// DUAL_PROPOSE_PROTOCOL_DEFINE_CRC(uart_protocol);
ARES_BULK_INTERFACE_DEFINE(usb_bulk_interface);
#if IS_ENABLED(CONFIG_UART_INTERFACE)
ARES_UART_INTERFACE_DEFINE(uart_interface);
#endif

int cnt = 0;
int func_cb(uint32_t param1, uint32_t param2, uint32_t param3)
{
	cnt++;
	LOG_INF("func_cb");
	LOG_INF("params: %2x,%2x,%2x", param1, param2, param3);
	return cnt;
}

void sync_cb(uint32_t status)
{
	LOG_INF("sync_cb: %d", status);
}

void func_ret_cb(uint16_t id, uint16_t req_id, uint32_t ret)
{
	LOG_INF("func_ret_cb: ID %x, req_id %x, ret %x", id, req_id, ret);
}

uint8_t test[59] = {0};
int main(void)
{
	LOG_INF("ARES USB Bulk & UART device is ready.");

#if IS_ENABLED(CONFIG_UART_INTERFACE)
	// Initialize UART interface
	ares_uart_init_dev(&uart_interface, uart_dev);
	int err_uart = ares_bind_interface(&uart_interface, &uart_protocol);
	if (err_uart) {
		LOG_ERR("Failed to initialize ARES UART device (%d)", err_uart);
	} else {
		LOG_INF("ARES UART interface initialized successfully");
	}
#endif

	// Initialize USB bulk interface
	int err_usb = ares_bind_interface(&usb_bulk_interface, &dual_protocol);
	if (err_usb) {
		LOG_ERR("Failed to initialize ARES USB device (%d)", err_usb);
	} else {
		LOG_INF("ARES USB Bulk interface initialized successfully");
	}

	// Setup callbacks and functions for both protocols
	dual_ret_cb_set(&dual_protocol, (dual_func_ret_cb_t)func_ret_cb);
	dual_func_add(&dual_protocol, 0x1, (dual_trans_func_t)func_cb);
	sync_table_t *usb_pack =
		dual_sync_add(&dual_protocol, 0x1, test, sizeof(test), (dual_trans_cb_t)sync_cb);

#if IS_ENABLED(CONFIG_UART_INTERFACE)
	dual_ret_cb_set(&uart_protocol, (dual_func_ret_cb_t)func_ret_cb);
	dual_func_add(&uart_protocol, 0x1, (dual_trans_func_t)func_cb);
	sync_table_t *uart_pack =
		dual_sync_add(&uart_protocol, 0x1, test, sizeof(test), (dual_trans_cb_t)sync_cb);
#endif

	LOG_INF("ARES Dual Interface (USB Bulk + UART) device is ready.");

	// The main thread is now free to do other things.
	// All USB and UART data processing happens in the background.
	int cnt = 0;
	while (1) {
		k_sleep(K_MSEC(1000));

#if IS_ENABLED(CONFIG_UART_INTERFACE)
		// Send data through both interfaces alternately
		if (cnt % 2 == 0) {
			// Send through USB
			if (err_usb == 0) {
				dual_func_call(&dual_protocol, 0x1, 0x1, 0x2, cnt);
				dual_sync_flush(&dual_protocol, usb_pack);
				LOG_INF("Data sent via USB: %d", cnt);
			}
		} else {
			// Send through UART
			if (err_uart == 0) {
				dual_func_call(&uart_protocol, 0x1, 0x1, 0x2, cnt);
				dual_sync_flush(&uart_protocol, uart_pack);
				LOG_INF("Data sent via UART: %d", cnt);
			}
		}
#else
		if (err_usb == 0) {
			dual_func_call(&dual_protocol, 0x1, 0x1, 0x2, cnt);
			dual_sync_flush(&dual_protocol, usb_pack);
			LOG_INF("Data sent via USB: %d", cnt);
		}
#endif
		cnt++;
	}

	return 0;
}
