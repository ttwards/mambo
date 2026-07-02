/*
 * Copyright (c) 2022, 2025 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/usb/usbd.h>
#include <zephyr/usb/usb_ch9.h>
#include <zephyr/drivers/usb/udc.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#if IS_ENABLED(CONFIG_HWINFO)
#include <zephyr/drivers/hwinfo.h>
#endif
#include <zephyr/net_buf.h>

#include "usb_bulk.h"
#include <ares/protocol/ares_protocol.h>

// Define a log level for this module
#ifndef CONFIG_ARES_USB_BULK_LOG_LEVEL
#define CONFIG_ARES_USB_BULK_LOG_LEVEL LOG_LEVEL_INF
#endif
LOG_MODULE_REGISTER(ares_usb_bulk_async, CONFIG_ARES_USB_BULK_LOG_LEVEL);

/* === Pipeline and Threading Configuration === */
#define ARES_PROCESSING_THREAD_STACK_SIZE CONFIG_ARES_USB_THREAD_STACK_SIZE
#define ARES_PROCESSING_THREAD_PRIORITY   K_PRIO_PREEMPT(5)

// Message queue to hold incoming net_buf pointers from ISR to thread
#define INCOMING_MSGQ_MAX_MSGS (CONFIG_UDC_BUF_COUNT / 2) // Half of UDC bufs
K_MSGQ_DEFINE(incoming_data_msgq, sizeof(struct net_buf *), INCOMING_MSGQ_MAX_MSGS, 4);

// Processing thread
K_THREAD_STACK_DEFINE(processing_thread_stack_area, ARES_PROCESSING_THREAD_STACK_SIZE);
struct k_thread processing_thread_data;

static struct AresInterface *ares_interface;

/* Device and Endpoint definitions */
#define ARES_USB_VID 0x1209 // Test VID
#define ARES_USB_PID 0x0001 // Product PID
#define BULK_EP_OUT  0x01   // OUT endpoint address
#define BULK_EP_IN   0x81   // IN endpoint address

/* State bits for the interface */
#define ARES_IF_FUNCTION_ENABLED     0
#define ARES_IF_FUNCTION_OUT_ENGAGED 1
#define ARES_IF_FUNCTION_IN_ENGAGED  2

/* === USB Device and Descriptor Definitions === */

USBD_DEVICE_DEFINE(ares_usbd, DEVICE_DT_GET(DT_NODELABEL(zephyr_udc0)), ARES_USB_VID, ARES_USB_PID);

USBD_DESC_LANG_DEFINE(ares_lang);
USBD_DESC_MANUFACTURER_DEFINE(ares_mfr, "ARES");
USBD_DESC_PRODUCT_DEFINE(ares_product, "ARES Bulk Interface Async");
IF_ENABLED(CONFIG_HWINFO, (USBD_DESC_SERIAL_NUMBER_DEFINE(ares_sn)));

USBD_DESC_CONFIG_DEFINE(fs_cfg_desc, "FS Configuration");
USBD_DESC_CONFIG_DEFINE(hs_cfg_desc, "HS Configuration");

static const uint8_t attributes = USB_SCD_SELF_POWERED;

USBD_CONFIGURATION_DEFINE(ares_fs_config, attributes, 250, &fs_cfg_desc);
USBD_CONFIGURATION_DEFINE(ares_hs_config, attributes, 250, &hs_cfg_desc);

/* === Custom Class Implementation === */

// Forward declaration
struct usbd_class_data *ares_if_class_data;

struct ares_if_desc {
	struct usb_if_descriptor if0;
	struct usb_ep_descriptor if0_out_ep;
	struct usb_ep_descriptor if0_in_ep;
	struct usb_ep_descriptor if0_hs_out_ep;
	struct usb_ep_descriptor if0_hs_in_ep;
	struct usb_desc_header nil_desc;
};

struct ares_if_data {
	struct ares_if_desc *const desc;
	const struct usb_desc_header **const fs_desc;
	const struct usb_desc_header **const hs_desc;
	atomic_t state;
};

static uint8_t ares_if_get_bulk_out(struct usbd_class_data *const c_data)
{
	struct ares_if_data *data = usbd_class_get_private(c_data);
	struct usbd_context *uds_ctx = usbd_class_get_ctx(c_data);
	struct ares_if_desc *desc = data->desc;

	if (usbd_bus_speed(uds_ctx) == USBD_SPEED_HS) {
		return desc->if0_hs_out_ep.bEndpointAddress;
	}
	return desc->if0_out_ep.bEndpointAddress;
}

static uint8_t ares_if_get_bulk_in(struct usbd_class_data *const c_data)
{
	struct ares_if_data *data = usbd_class_get_private(c_data);
	struct usbd_context *uds_ctx = usbd_class_get_ctx(c_data);
	struct ares_if_desc *desc = data->desc;

	if (usbd_bus_speed(uds_ctx) == USBD_SPEED_HS) {
		return desc->if0_hs_in_ep.bEndpointAddress;
	}
	return desc->if0_in_ep.bEndpointAddress;
}

static uint16_t ares_if_get_mps(struct usbd_class_data *const c_data)
{
	struct ares_if_data *data = usbd_class_get_private(c_data);
	struct usbd_context *uds_ctx = usbd_class_get_ctx(c_data);
	struct ares_if_desc *desc = data->desc;

	if (usbd_bus_speed(uds_ctx) == USBD_SPEED_HS) {
		return sys_le16_to_cpu(desc->if0_hs_out_ep.wMaxPacketSize);
	}
	return sys_le16_to_cpu(desc->if0_out_ep.wMaxPacketSize);
}

static int ares_if_submit_bulk_out(struct usbd_class_data *const c_data)
{
	struct ares_if_data *data = usbd_class_get_private(c_data);
	struct net_buf *buf;
	int err;

	if (!atomic_test_bit(&data->state, ARES_IF_FUNCTION_ENABLED)) {
		return -EPERM;
	}

	if (atomic_test_and_set_bit(&data->state, ARES_IF_FUNCTION_OUT_ENGAGED)) {
		return -EBUSY; /* Already waiting for data */
	}

	if (k_msgq_num_free_get(&incoming_data_msgq) == 0) {
		atomic_clear_bit(&data->state, ARES_IF_FUNCTION_OUT_ENGAGED);
		return 0;
	}

	buf = usbd_ep_buf_alloc(c_data, ares_if_get_bulk_out(c_data), ares_if_get_mps(c_data));
	if (buf == NULL) {
		LOG_ERR("Failed to allocate buffer for OUT endpoint");
		atomic_clear_bit(&data->state, ARES_IF_FUNCTION_OUT_ENGAGED);
		return -ENOMEM;
	}

	err = usbd_ep_enqueue(c_data, buf);
	if (err) {
		LOG_ERR("Failed to enqueue buffer for OUT endpoint (err %d)", err);
		net_buf_unref(buf);
		atomic_clear_bit(&data->state, ARES_IF_FUNCTION_OUT_ENGAGED);
	} else {
		LOG_DBG("Enqueued read request on OUT endpoint");
	}
	return err;
}

static int ares_if_request_handler(struct usbd_class_data *const c_data, struct net_buf *const buf,
				   const int err_code)
{
	struct udc_buf_info *bi = udc_get_buf_info(buf);
	struct ares_if_data *data = usbd_class_get_private(c_data);
	const uint8_t ep = bi->ep;

	if (ep == ares_if_get_bulk_in(c_data)) {
		atomic_clear_bit(&data->state, ARES_IF_FUNCTION_IN_ENGAGED);
		net_buf_unref(buf);

		/*
		 * IN transfer is complete. The 'buf' will be unref'd by the caller (USBD stack)
		 * which balances the initial alloc/ref.
		 */
		LOG_DBG("IN transfer complete for buf %p", buf);
	} else if (ep == ares_if_get_bulk_out(c_data)) {
		atomic_clear_bit(&data->state, ARES_IF_FUNCTION_OUT_ENGAGED);

		if (buf->len == 12) {
			LOG_HEXDUMP_ERR(buf->data, buf->len, "ERROR frame");
			LOG_ERR("ERROR frame received");
			// k_msgq_put(&incoming_data_msgq, &buf, K_NO_WAIT);
		}

		if (err_code == 0 && buf->len > 0) {
			static uint32_t rx_count;

			rx_count++;
			if (rx_count <= 16U || (rx_count % 100U) == 0U) {
				uint16_t head = 0U;
				uint8_t b0 = 0U;
				uint8_t b1 = 0U;
				uint8_t b2 = 0U;
				uint8_t b3 = 0U;

				if (buf->len > 0U) {
					b0 = buf->data[0];
				}
				if (buf->len > 1U) {
					b1 = buf->data[1];
					head = (uint16_t)b0 | ((uint16_t)b1 << 8);
				}
				if (buf->len > 2U) {
					b2 = buf->data[2];
				}
				if (buf->len > 3U) {
					b3 = buf->data[3];
				}

				LOG_INF("ARES Bulk OUT #%u len=%u head=0x%04x bytes=%02x %02x %02x %02x",
					rx_count, buf->len, head, b0, b1, b2, b3);
			}
			if (k_msgq_put(&incoming_data_msgq, &buf, K_NO_WAIT) != 0) {
				/* Queue is full (back-pressure!). Drop packet and log. */
				{
					static int64_t last_log_time = 0;
					int64_t now = k_uptime_get();
					if (now - last_log_time >= 500) { // 2Hz = 500ms
						LOG_WRN("Incoming data queue full, packet "
							"dropped.");
						last_log_time = now;
					}
				}
				net_buf_unref(buf);
			} else {
				LOG_DBG("Queued incoming buffer %p", buf);
			}
		} else if (err_code != 0 && err_code != -ECONNABORTED) {
			LOG_ERR("OUT ep 0x%02x failed, err %d", ep, err_code);
			net_buf_unref(buf);
		} else {
			net_buf_unref(buf);
		}

		if (atomic_test_bit(&data->state, ARES_IF_FUNCTION_ENABLED) &&
		    err_code != -ECONNABORTED) {
			if (ares_if_submit_bulk_out(c_data) < 0) {
				LOG_ERR("Failed to re-submit bulk OUT request");
			}
		}
	}

	return 0;
}

static void ares_if_update(struct usbd_class_data *c_data, uint8_t iface, uint8_t alternate)
{
	LOG_DBG("Interface %u alternate %u changed", iface, alternate);
}

static void *ares_if_get_desc(struct usbd_class_data *const c_data, const enum usbd_speed speed)
{
	struct ares_if_data *data = usbd_class_get_private(c_data);
	if (USBD_SUPPORTS_HIGH_SPEED && speed == USBD_SPEED_HS) {
		return data->hs_desc;
	}
	return data->fs_desc;
}

static void ares_if_enable(struct usbd_class_data *const c_data)
{
	struct ares_if_data *data = usbd_class_get_private(c_data);
	LOG_INF("Enable ARES Bulk interface");
	if (atomic_test_and_set_bit(&data->state, ARES_IF_FUNCTION_ENABLED)) {
		return;
	}
	if (ares_if_submit_bulk_out(c_data) != 0) {
		LOG_ERR("Failed to submit initial bulk OUT request on enable");
	}
}

static void ares_if_disable(struct usbd_class_data *const c_data)
{
	struct ares_if_data *data = usbd_class_get_private(c_data);
	struct net_buf *buf;

	LOG_INF("Disable ARES Bulk interface");
	atomic_clear_bit(&data->state, ARES_IF_FUNCTION_ENABLED);
	atomic_clear_bit(&data->state, ARES_IF_FUNCTION_IN_ENGAGED);
	atomic_clear_bit(&data->state, ARES_IF_FUNCTION_OUT_ENGAGED);

	while (k_msgq_get(&incoming_data_msgq, &buf, K_NO_WAIT) == 0) {
		net_buf_unref(buf);
	}
}

static int ares_if_init(struct usbd_class_data *c_data)
{
	struct ares_if_data *data = usbd_class_get_private(c_data);
	atomic_set(&data->state, 0);
	ares_if_class_data = c_data;
	LOG_DBG("Init ARES Bulk class instance %p", c_data);
	return 0;
}

struct usbd_class_api ares_if_api = {
	.update = ares_if_update,
	.request = ares_if_request_handler,
	.get_desc = ares_if_get_desc,
	.enable = ares_if_enable,
	.disable = ares_if_disable,
	.init = ares_if_init,
};

/* --- Processing Thread --- */
static void ares_processing_thread_entry(void *p1, void *p2, void *p3)
{
	struct net_buf *buf;
	// struct ares_if_data *data = usbd_class_get_private(ares_if_class_data);
	// LOG_INF("ARES data processing thread started");

	while (1) {
		/* Wait forever for a data buffer from the ISR */
		k_msgq_get(&incoming_data_msgq, &buf, K_FOREVER);

		if (k_msgq_num_free_get(&incoming_data_msgq) > 0) {
			ares_if_submit_bulk_out(ares_if_class_data);
		}

		LOG_DBG("Processing thread got buffer %p with len %u", buf, buf->len);

		if (ares_interface && ares_interface->protocol && ares_interface->protocol->api &&
		    ares_interface->protocol->api->handle) {
			ares_interface->protocol->api->handle(ares_interface->protocol, buf);
		}
		// k_busy_wait(3);
		// LOG_INF("Processing thread got buffer %p with len %u", buf, buf->len);
		/*
		 * We are done with the buffer. Unref it to return it to the pool.
		 * This balances the net_buf_ref() done in the ISR.
		 */
		net_buf_unref(buf);
	}
}

/* === Public API and Setup Function === */

struct ares_udc_buf_info {
	struct udc_buf_info udc_buf_info;
	struct k_mutex *mutex;
} __attribute__((packed));

static const char *const ares_usbd_bulk_first_blocklist[] = {
	"ares_if_0",
	NULL,
};

/* To send data, we can allocate from the general-purpose system pool */
void buf_cb_unlock(struct net_buf *buf)
{
	struct ares_udc_buf_info *buf_info = net_buf_user_data(buf);
	if (buf_info->mutex) {
		LOG_DBG("Unlocking mutex %p", buf_info->mutex);
		k_mutex_unlock(buf_info->mutex);
	}
	LOG_DBG("Destroying buffer %p", buf);
	net_buf_destroy(buf);
}

UDC_BUF_POOL_DEFINE(tx_pool, 8, 512, sizeof(struct ares_udc_buf_info), buf_cb_unlock);

int ares_usbd_write(struct AresInterface *interface, struct net_buf *buf)
{
	struct usbd_class_data *c_data = ares_if_class_data;
	struct ares_if_data *data = NULL;
	int err;

	if (c_data == NULL) {
		err = -ENODEV;
		goto clear_exit;
	}

	data = usbd_class_get_private(c_data);
	if (data == NULL) {
		err = -EINVAL;
		goto clear_exit;
	}
	if (!atomic_test_bit(&data->state, ARES_IF_FUNCTION_ENABLED)) {
		err = -EPERM;
		goto clear_exit;
	}
	if (atomic_test_and_set_bit(&data->state, ARES_IF_FUNCTION_IN_ENGAGED)) {
		err = -EBUSY;
		goto clear_exit;
	}

	struct ares_udc_buf_info *buf_info = net_buf_user_data(buf);
	if (buf_info) {
		buf_info->mutex = NULL;
		buf_info->udc_buf_info.ep = ares_if_get_bulk_in(c_data);
	}

	err = usbd_ep_enqueue(c_data, buf);
	if (err) {
		atomic_clear_bit(&data->state, ARES_IF_FUNCTION_IN_ENGAGED);
		LOG_ERR("Enqueue error %d", err);
clear_exit:
		net_buf_unref(buf);
	} else {
		LOG_DBG("Enqueueing buffer %p", buf);
	}
	return err;
}

int ares_usbd_write_with_lock(struct AresInterface *interface, struct net_buf *buf,
			      struct k_mutex *mutex)
{
	struct usbd_class_data *c_data = ares_if_class_data;
	struct ares_if_data *data = NULL;
	int err;

	if (c_data == NULL) {
		err = -ENODEV;
		goto clear_exit;
	}

	data = usbd_class_get_private(c_data);
	if (data == NULL) {
		err = -EINVAL;
		goto clear_exit;
	}
	if (!atomic_test_bit(&data->state, ARES_IF_FUNCTION_ENABLED)) {
		err = -EPERM;
		goto clear_exit;
	}
	if (atomic_test_and_set_bit(&data->state, ARES_IF_FUNCTION_IN_ENGAGED)) {
		err = -EBUSY;
		goto clear_exit;
	}

	struct ares_udc_buf_info *buf_info = net_buf_user_data(buf);
	if (buf_info) {
		buf_info->mutex = mutex;
		buf_info->udc_buf_info.ep = ares_if_get_bulk_in(c_data);
	}

	err = usbd_ep_enqueue(c_data, buf);
	if (err) {
		atomic_clear_bit(&data->state, ARES_IF_FUNCTION_IN_ENGAGED);
		LOG_DBG("locked Enqueue error %d", err);
clear_exit:
		net_buf_unref(buf);
	} else {
		LOG_DBG("locked Enqueueing buffer %p", buf);
	}
	return err;
}

struct net_buf *ares_interface_alloc_buf(struct AresInterface *interface)
{
	struct net_buf *buf = net_buf_alloc(&tx_pool, K_NO_WAIT);
	if (buf) {
		struct ares_udc_buf_info *buf_info = net_buf_user_data(buf);
		if (buf_info) {
			buf_info->mutex = NULL;
		}
	}
	return buf;
}

struct net_buf *ares_interface_alloc_buf_with_data(struct AresInterface *interface, void *data,
						   size_t size)
{
	struct net_buf *buf = net_buf_alloc_with_data(&tx_pool, data, size, K_NO_WAIT);
	if (buf) {
		struct ares_udc_buf_info *buf_info = net_buf_user_data(buf);
		if (buf_info) {
			buf_info->mutex = NULL;
		}
	}
	return buf;
}

static void ares_usbd_protocol_event(enum AresProtocolEvent event)
{
	if (ares_interface && ares_interface->protocol && ares_interface->protocol->api &&
	    ares_interface->protocol->api->event) {
		ares_interface->protocol->api->event(ares_interface->protocol, event);
	}
}

static void ares_usbd_fix_code_triple(struct usbd_context *uds_ctx, enum usbd_speed speed)
{
	if (IS_ENABLED(CONFIG_USBD_CDC_ACM_CLASS)) {
		usbd_device_set_code_triple(uds_ctx, speed, USB_BCC_MISCELLANEOUS, 0x02, 0x01);
	} else {
		usbd_device_set_code_triple(uds_ctx, speed, 0, 0, 0);
	}
}

static int ares_usbd_register_classes(enum usbd_speed speed)
{
	int err;

	err = usbd_register_class(&ares_usbd, "ares_if_0", speed, 1);
	if (err) {
		LOG_ERR("Failed to register ARES Bulk class for %s (%d)",
			speed == USBD_SPEED_HS ? "HS" : "FS", err);
		return err;
	}

	err = usbd_register_all_classes(&ares_usbd, speed, 1, ares_usbd_bulk_first_blocklist);
	if (err) {
		LOG_ERR("Failed to register composite USB classes for %s (%d)",
			speed == USBD_SPEED_HS ? "HS" : "FS", err);
		return err;
	}

	return 0;
}

// Corrected message callback, same as in sample_usbd.c
static void ares_usbd_msg_cb(struct usbd_context *const usbd_ctx, const struct usbd_msg *const msg)
{
	switch (msg->type) {
	case USBD_STATE_CONFIGURED:
		LOG_INF("USB device configured");
		ares_usbd_protocol_event(ARES_PROTOCOL_EVENT_CONNECTED);
		break;
	case USBD_MSG_RESET:
		LOG_INF("USB device reset");
		ares_usbd_protocol_event(ARES_PROTOCOL_EVENT_DISCONNECTED);
		break;
	case USBD_MSG_SUSPEND:
		LOG_INF("USB device suspended");
		ares_usbd_protocol_event(ARES_PROTOCOL_EVENT_DISCONNECTED);
		break;
	default:
		break;
	}
}

int ares_usbd_init(struct AresInterface *interface)
{
	int err;

	if (interface != NULL) {
		struct AresBulkInterface *bulk_interface = interface->priv_data;
		bulk_interface->c_data = ares_if_class_data;
		bulk_interface->usbd_ctx = usbd_class_get_ctx(ares_if_class_data);
		bulk_interface->state = 0;
		bulk_interface->incoming_data_msgq = &incoming_data_msgq;
		bulk_interface->processing_thread_data = &processing_thread_data;
		ares_interface = interface;
	}

	err = usbd_add_descriptor(&ares_usbd, &ares_lang);
	err |= usbd_add_descriptor(&ares_usbd, &ares_mfr);
	err |= usbd_add_descriptor(&ares_usbd, &ares_product);
	IF_ENABLED(CONFIG_HWINFO, (err |= usbd_add_descriptor(&ares_usbd, &ares_sn)));
	if (err) {
		LOG_ERR("Failed to add descriptors (%d)", err);
		return err;
	}

	err = usbd_add_configuration(&ares_usbd, USBD_SPEED_FS, &ares_fs_config);
	if (err) {
		LOG_ERR("Failed to add FS config");
		return err;
	}

	// Keep ARES Bulk registered first so it keeps EP OUT 0x01 and IN 0x81.
	err = ares_usbd_register_classes(USBD_SPEED_FS);
	if (err) {
		return err;
	}
	ares_usbd_fix_code_triple(&ares_usbd, USBD_SPEED_FS);

	if (USBD_SUPPORTS_HIGH_SPEED && usbd_caps_speed(&ares_usbd) == USBD_SPEED_HS) {
		err = usbd_add_configuration(&ares_usbd, USBD_SPEED_HS, &ares_hs_config);
		if (err) {
			LOG_ERR("Failed to add HS config");
			return err;
		}
		err = ares_usbd_register_classes(USBD_SPEED_HS);
		if (err) {
			return err;
		}
		ares_usbd_fix_code_triple(&ares_usbd, USBD_SPEED_HS);
	}
	usbd_self_powered(&ares_usbd, attributes & USB_SCD_SELF_POWERED);

	// Correctly register the message callback
	err = usbd_msg_register_cb(&ares_usbd, ares_usbd_msg_cb);
	if (err) {
		LOG_ERR("Failed to register message callback");
		return err;
	}

	err = usbd_init(&ares_usbd);
	if (err) {
		LOG_ERR("Failed to initialize device support (%d)", err);
		return err;
	}

	err = usbd_enable(&ares_usbd);
	if (err) {
		LOG_ERR("Failed to enable device support (%d)", err);
		return err;
	}

	k_thread_create(&processing_thread_data, processing_thread_stack_area,
			K_THREAD_STACK_SIZEOF(processing_thread_stack_area),
			ares_processing_thread_entry, NULL, NULL, NULL,
			ARES_PROCESSING_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&processing_thread_data, "ares_usb_proc");

	LOG_INF("USB device initialized");
	return 0;
}

/* === Descriptor and Class Data Definitions === */
#define DEFINE_INTERFACE_DESCRIPTOR(x, _)                                                          \
	static struct ares_if_desc ares_if_desc_##x = {                                            \
		.if0 =                                                                             \
			{                                                                          \
				.bLength = sizeof(struct usb_if_descriptor),                       \
				.bDescriptorType = USB_DESC_INTERFACE,                             \
				.bInterfaceNumber = 0,                                             \
				.bAlternateSetting = 0,                                            \
				.bNumEndpoints = 2,                                                \
				.bInterfaceClass = USB_BCC_VENDOR,                                 \
				.bInterfaceSubClass = 0,                                           \
				.bInterfaceProtocol = 0,                                           \
				.iInterface = 0,                                                   \
			},                                                                         \
		.if0_out_ep =                                                                      \
			{                                                                          \
				.bLength = sizeof(struct usb_ep_descriptor),                       \
				.bDescriptorType = USB_DESC_ENDPOINT,                              \
				.bEndpointAddress = BULK_EP_OUT,                                   \
				.bmAttributes = USB_EP_TYPE_BULK,                                  \
				.wMaxPacketSize = sys_cpu_to_le16(64U),                            \
				.bInterval = 0x00,                                                 \
			},                                                                         \
		.if0_in_ep =                                                                       \
			{                                                                          \
				.bLength = sizeof(struct usb_ep_descriptor),                       \
				.bDescriptorType = USB_DESC_ENDPOINT,                              \
				.bEndpointAddress = BULK_EP_IN,                                    \
				.bmAttributes = USB_EP_TYPE_BULK,                                  \
				.wMaxPacketSize = sys_cpu_to_le16(64U),                            \
				.bInterval = 0x00,                                                 \
			},                                                                         \
		.if0_hs_out_ep =                                                                   \
			{                                                                          \
				.bLength = sizeof(struct usb_ep_descriptor),                       \
				.bDescriptorType = USB_DESC_ENDPOINT,                              \
				.bEndpointAddress = BULK_EP_OUT,                                   \
				.bmAttributes = USB_EP_TYPE_BULK,                                  \
				.wMaxPacketSize = sys_cpu_to_le16(512U),                           \
				.bInterval = 0x00,                                                 \
			},                                                                         \
		.if0_hs_in_ep =                                                                    \
			{                                                                          \
				.bLength = sizeof(struct usb_ep_descriptor),                       \
				.bDescriptorType = USB_DESC_ENDPOINT,                              \
				.bEndpointAddress = BULK_EP_IN,                                    \
				.bmAttributes = USB_EP_TYPE_BULK,                                  \
				.wMaxPacketSize = sys_cpu_to_le16(512U),                           \
				.bInterval = 0x00,                                                 \
			},                                                                         \
		.nil_desc =                                                                        \
			{                                                                          \
				.bLength = 0,                                                      \
				.bDescriptorType = 0,                                              \
			},                                                                         \
	};                                                                                         \
	const static struct usb_desc_header *ares_if_fs_desc_##x[] = {                             \
		(struct usb_desc_header *)&ares_if_desc_##x.if0,                                   \
		(struct usb_desc_header *)&ares_if_desc_##x.if0_in_ep,                             \
		(struct usb_desc_header *)&ares_if_desc_##x.if0_out_ep,                            \
		(struct usb_desc_header *)&ares_if_desc_##x.nil_desc,                              \
	};                                                                                         \
	const static struct usb_desc_header *ares_if_hs_desc_##x[] = {                             \
		(struct usb_desc_header *)&ares_if_desc_##x.if0,                                   \
		(struct usb_desc_header *)&ares_if_desc_##x.if0_hs_in_ep,                          \
		(struct usb_desc_header *)&ares_if_desc_##x.if0_hs_out_ep,                         \
		(struct usb_desc_header *)&ares_if_desc_##x.nil_desc,                              \
	};

#define DEFINE_FUNCTION_DATA(n, _)                                                                 \
	static struct ares_if_data ares_if_data_##n = {                                            \
		.desc = (struct ares_if_desc *)&ares_if_desc_##n,                                  \
		.fs_desc = ares_if_fs_desc_##n,                                                    \
		.hs_desc = ares_if_hs_desc_##n,                                                    \
	};                                                                                         \
	USBD_DEFINE_CLASS(ares_if_##n, &ares_if_api, &ares_if_data_##n, NULL);

LISTIFY(1, DEFINE_INTERFACE_DESCRIPTOR, ())
LISTIFY(1, DEFINE_FUNCTION_DATA, ())
