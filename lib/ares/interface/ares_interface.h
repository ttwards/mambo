#ifndef _ARES_INTERFACE_H_
#define _ARES_INTERFACE_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <zephyr/net_buf.h>
#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

struct AresProtocol;
struct AresInterface;

typedef void (*ares_interface_tx_done_cb_t)(struct AresInterface *interface,
					    struct net_buf *buf, int status,
					    void *user_data);

enum AresInterfaceCaps {
	ARES_INTERFACE_CAP_STREAM = BIT(0),
	ARES_INTERFACE_CAP_PACKET = BIT(1),
	ARES_INTERFACE_CAP_ZERO_COPY = BIT(2),
	ARES_INTERFACE_CAP_TX_COMPLETE = BIT(3),
	ARES_INTERFACE_CAP_TX_QUEUE = BIT(4),
	ARES_INTERFACE_CAP_ETHERNET_FRAME = BIT(5),
};

/**
 * @brief API that an interface must implement.
 *
 * This structure defines the set of functions that a specific interface
 * (e.g., a UART or USB driver) must provide. It is primarily used by the
 * protocol layer to send data.
 */
struct AresInterfaceAPI {
	int (*send)(struct AresInterface *interface, struct net_buf *buf);
	int (*send_with_callback)(struct AresInterface *interface, struct net_buf *buf,
				  ares_interface_tx_done_cb_t cb, void *user_data);
	int (*send_raw)(struct AresInterface *interface, uint8_t *data, uint16_t len);

	int (*connect)(struct AresInterface *interface);
	int (*disconnect)(struct AresInterface *interface);
	bool (*is_connected)(struct AresInterface *interface);
	uint32_t (*caps)(struct AresInterface *interface);
	size_t (*mtu)(struct AresInterface *interface);

	struct net_buf *(*alloc_buf)(struct AresInterface *interface);
	struct net_buf *(*alloc_buf_with_data)(struct AresInterface *interface, void *data,
					       size_t size);

	int (*init)(struct AresInterface *interface);
};

/**
 * @brief Represents a specific communication interface instance (e.g., a UART port).
 *
 * This structure holds the state of the interface, including the loaded protocol
 * and any driver-specific data.
 */
struct AresInterface {
	const char *name;
	const struct AresInterfaceAPI *api;
	struct AresProtocol *protocol; // The protocol loaded onto this interface

	void *priv_data; // Interface-specific private data (e.g., UART device ptr)
};

#ifdef __cplusplus
}
#endif

#endif
