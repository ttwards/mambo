// usb_bulk.h
#ifndef ARES_USB_BULK_H__
#define ARES_USB_BULK_H__

#include <stddef.h>
#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/net_buf.h>
#include <ares/interface/ares_interface.h>

int ares_usbd_init(struct AresInterface *interface);
int ares_usbd_write(struct AresInterface *interface, struct net_buf *buf);
struct net_buf *ares_interface_alloc_buf(struct AresInterface *interface);
struct net_buf *ares_interface_alloc_buf_with_data(struct AresInterface *interface, void *data,
						   size_t len);
int ares_usbd_write_with_callback(struct AresInterface *interface, struct net_buf *buf,
				  ares_interface_tx_done_cb_t cb, void *user_data);
uint32_t ares_usbd_caps(struct AresInterface *interface);
size_t ares_usbd_mtu(struct AresInterface *interface);

struct AresBulkInterface {
	struct AresInterface *interface;

	struct usbd_class_data *c_data;
	struct usbd_context *usbd_ctx;

	atomic_t state;

	struct k_msgq *incoming_data_msgq;
	struct k_thread *processing_thread_data;
};

#define ARES_BULK_INTERFACE_DEFINE(Interface_name)                                                 \
	struct AresInterfaceAPI ares_bulk_interface_api = {                                        \
		.init = ares_usbd_init,                                                            \
		.send = ares_usbd_write,                                                           \
		.send_with_callback = ares_usbd_write_with_callback,                               \
		.caps = ares_usbd_caps,                                                            \
		.mtu = ares_usbd_mtu,                                                              \
		.alloc_buf = ares_interface_alloc_buf,                                             \
		.alloc_buf_with_data = ares_interface_alloc_buf_with_data,                         \
	};                                                                                         \
	struct AresBulkInterface Internal_##Interface_name = {NULL};                               \
	struct AresInterface Interface_name = {                                                    \
		.name = #Interface_name,                                                           \
		.api = &ares_bulk_interface_api,                                                   \
		.protocol = NULL,                                                                  \
		.priv_data = &Internal_##Interface_name,                                           \
	};

#endif // ARES_USB_BULK_H__
