/*
 * Copyright (c) 2025 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "init.h"

LOG_MODULE_DECLARE(board_init, CONFIG_BOARD_LOG_LEVEL);

#define LED_BLUE  DT_ALIAS(led_blue)
#define LED_GREEN DT_ALIAS(led_green)
#define LED_RED   DT_ALIAS(led_red)

static const struct pwm_dt_spec pwm_led_r = PWM_DT_SPEC_GET(LED_RED);
static const struct pwm_dt_spec pwm_led_g = PWM_DT_SPEC_GET(LED_GREEN);
static const struct pwm_dt_spec pwm_led_b = PWM_DT_SPEC_GET(LED_BLUE);

void ares_board_power_init(void)
{
}

int ares_board_status_led_set_rgb(const struct ares_led_rgb *color)
{
	int ret;
	uint32_t pulse_r = (uint32_t)(((float)color->r / 255.0f) * pwm_led_r.period);
	uint32_t pulse_g = (uint32_t)(((float)color->g / 255.0f) * pwm_led_g.period);
	uint32_t pulse_b = (uint32_t)(((float)color->b / 255.0f) * pwm_led_b.period);

	ret = pwm_set_dt(&pwm_led_r, pwm_led_r.period, pulse_r);
	if (ret < 0) {
		LOG_ERR("Failed to set red LED PWM (%d)", ret);
		return ret;
	}

	ret = pwm_set_dt(&pwm_led_g, pwm_led_g.period, pulse_g);
	if (ret < 0) {
		LOG_ERR("Failed to set green LED PWM (%d)", ret);
		return ret;
	}

	ret = pwm_set_dt(&pwm_led_b, pwm_led_b.period, pulse_b);
	if (ret < 0) {
		LOG_ERR("Failed to set blue LED PWM (%d)", ret);
		return ret;
	}

	return 0;
}

uint8_t ares_board_status_led_max_channel(void)
{
	return 0xff;
}

void ares_board_status_led_init(void)
{
	struct ares_led_rgb color = {
		.r = 0x4f,
		.g = 0x4f,
		.b = 0x4f,
	};

	if (!device_is_ready(pwm_led_r.dev) || !device_is_ready(pwm_led_g.dev) ||
	    !device_is_ready(pwm_led_b.dev)) {
		LOG_ERR("One or more PWM LED devices are not ready");
		return;
	}

	ares_board_status_led_set_rgb(&color);
	k_sleep(K_MSEC(300));
	ares_board_status_led_service_start();
}
