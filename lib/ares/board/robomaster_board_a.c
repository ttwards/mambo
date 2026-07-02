/*
 * Copyright (c) 2025 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>

#include "init.h"

#define XT30_1_NODE DT_NODELABEL(power1)
#define XT30_2_NODE DT_NODELABEL(power2)
#define XT30_3_NODE DT_NODELABEL(power3)
#define XT30_4_NODE DT_NODELABEL(power4)

static const struct gpio_dt_spec pwr1 = GPIO_DT_SPEC_GET(XT30_1_NODE, gpios);
static const struct gpio_dt_spec pwr2 = GPIO_DT_SPEC_GET(XT30_2_NODE, gpios);
static const struct gpio_dt_spec pwr3 = GPIO_DT_SPEC_GET(XT30_3_NODE, gpios);
static const struct gpio_dt_spec pwr4 = GPIO_DT_SPEC_GET(XT30_4_NODE, gpios);

void ares_board_power_init(void)
{
	gpio_pin_configure_dt(&pwr1, GPIO_OUTPUT_HIGH);
	gpio_pin_configure_dt(&pwr2, GPIO_OUTPUT_HIGH);
	gpio_pin_configure_dt(&pwr3, GPIO_OUTPUT_HIGH);
	gpio_pin_configure_dt(&pwr4, GPIO_OUTPUT_HIGH);
}

void ares_board_status_led_init(void)
{
}

int ares_board_status_led_set_rgb(const struct ares_led_rgb *color)
{
	ARG_UNUSED(color);
	return -ENOTSUP;
}

uint8_t ares_board_status_led_max_channel(void)
{
	return 0xff;
}
