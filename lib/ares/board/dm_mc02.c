/*
 * Copyright (c) 2025 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "init.h"

LOG_MODULE_DECLARE(board_init, CONFIG_BOARD_LOG_LEVEL);

#define XT30_1_NODE DT_NODELABEL(power1)
#define XT30_2_NODE DT_NODELABEL(power2)
#define STRIP_NODE  DT_ALIAS(led_strip)

static const struct device *const strip = DEVICE_DT_GET(STRIP_NODE);

#if DT_NODE_EXISTS(XT30_1_NODE) && DT_NODE_HAS_PROP(XT30_1_NODE, gpios) &&                         \
	DT_NODE_EXISTS(XT30_2_NODE) && DT_NODE_HAS_PROP(XT30_2_NODE, gpios)
static const struct gpio_dt_spec pwr1 = GPIO_DT_SPEC_GET(XT30_1_NODE, gpios);
static const struct gpio_dt_spec pwr2 = GPIO_DT_SPEC_GET(XT30_2_NODE, gpios);
#endif

void ares_board_power_init(void)
{
#if DT_NODE_EXISTS(XT30_1_NODE) && DT_NODE_HAS_PROP(XT30_1_NODE, gpios) &&                         \
	DT_NODE_EXISTS(XT30_2_NODE) && DT_NODE_HAS_PROP(XT30_2_NODE, gpios)
	gpio_pin_configure_dt(&pwr1, GPIO_OUTPUT_HIGH);
	gpio_pin_configure_dt(&pwr2, GPIO_OUTPUT_HIGH);
#endif
}

int ares_board_status_led_set_rgb(const struct ares_led_rgb *color)
{
	struct led_rgb led_color = {
		.r = color->r,
		.g = color->g,
		.b = color->b,
	};

	return led_strip_update_rgb(strip, &led_color, 1);
}

uint8_t ares_board_status_led_max_channel(void)
{
	return 0x7e;
}

void ares_board_status_led_init(void)
{
	struct ares_led_rgb color = {
		.r = 0x4f,
		.g = 0x4f,
		.b = 0x4f,
	};

	if (!device_is_ready(strip)) {
		LOG_ERR("WS2812 LED strip device is not ready");
		return;
	}

	ares_board_status_led_set_rgb(&color);
	k_sleep(K_MSEC(300));
	ares_board_status_led_service_start();
}
