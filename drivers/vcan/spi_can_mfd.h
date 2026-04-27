/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_VCAN_SPI_CAN_MFD_H_
#define ZEPHYR_DRIVERS_VCAN_SPI_CAN_MFD_H_

#include <zephyr/device.h>
#include <zephyr/drivers/can.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SPI_CAN_MFD_MAX_CHANNELS 2U

int spi_can_mfd_set_bitrate(const struct device *parent, uint8_t channel, uint32_t bitrate);
int spi_can_mfd_get_core_clock(const struct device *parent, uint32_t *rate);
int spi_can_mfd_send(const struct device *parent, uint8_t channel,
		     const struct can_frame *frame, k_timeout_t timeout);
int spi_can_mfd_get_state(const struct device *parent, uint8_t channel,
			  enum can_state *state, struct can_bus_err_cnt *err_cnt);

int spi_can_node_handle_rx_frame(const struct device *dev, const struct can_frame *frame);
void spi_can_node_report_state(const struct device *dev, enum can_state state,
			       const struct can_bus_err_cnt *err_cnt);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_VCAN_SPI_CAN_MFD_H_ */
