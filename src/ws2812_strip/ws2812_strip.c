/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * WS2812B LED strip 驱动实现
 *
 * 依赖：
 *   - Zephyr led_strip API (worldsemi,ws2812-spi)
 *   - app.overlay 中配置好 SPI + WS2812 节点
 *   - prj.conf: CONFIG_LED_STRIP=y, CONFIG_WS2812_STRIP=y
 */

#include <ws2812_strip.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(ws2812_strip, CONFIG_WS2812_STRIP_LOG_LEVEL);

/* ---- 从设备树获取 led-strip 设备 ---- */
#define STRIP_NODE  DT_ALIAS(led_strip)
#define STRIP_LEN   DT_PROP(STRIP_NODE, chain_length)

static const struct device *strip_dev = DEVICE_DT_GET(STRIP_NODE);

/* ---- 像素缓存 ---- */
static struct led_rgb pixels[STRIP_LEN];

/* ---- 色相基准 ---- */
static uint16_t current_hue;

/* ------------------------------------------------------------------ */
/*  HSV → RGB                                                         */
/* ------------------------------------------------------------------ */
static void hsv_to_rgb(uint16_t h, uint8_t s, uint8_t v,
		       uint8_t *r, uint8_t *g, uint8_t *b)
{
	uint8_t region, remainder, p, q, t;

	if (s == 0) {
		*r = *g = *b = v;
		return;
	}

	region    = h / 60;
	remainder = (uint8_t)(((uint16_t)(h - region * 60) * 255) / 60);

	p = (uint8_t)((v * (255 - s)) / 255);
	q = (uint8_t)((v * (255 - ((uint16_t)s * remainder) / 255)) / 255);
	t = (uint8_t)((v * (255 - ((uint16_t)s * (255 - remainder)) / 255)) / 255);

	switch (region) {
	case 0:  *r = v; *g = t; *b = p; break;
	case 1:  *r = q; *g = v; *b = p; break;
	case 2:  *r = p; *g = v; *b = t; break;
	case 3:  *r = p; *g = q; *b = v; break;
	case 4:  *r = t; *g = p; *b = v; break;
	default: *r = v; *g = p; *b = q; break;
	}
}

/* ------------------------------------------------------------------ */
/*  内部刷屏                                                           */
/* ------------------------------------------------------------------ */
static void ws2812_strip_flush(void)
{
	if (!device_is_ready(strip_dev)) {
		return;
	}
	led_strip_update_rgb(strip_dev, pixels, STRIP_LEN);
}

/* ------------------------------------------------------------------ */
/*  Public API                                                        */
/* ------------------------------------------------------------------ */

int ws2812_strip_init(void)
{
	if (!device_is_ready(strip_dev)) {
		LOG_ERR("LED strip device %s not ready", strip_dev->name);
		return -ENODEV;
	}

	ws2812_strip_clear();
	LOG_INF("WS2812B strip ready (%u LEDs)", STRIP_LEN);
	return 0;
}

size_t ws2812_strip_length(void)
{
	return STRIP_LEN;
}

void ws2812_strip_set_led(int index, uint8_t r, uint8_t g, uint8_t b)
{
	if (index < 0 || (size_t)index >= STRIP_LEN) {
		return;
	}

	pixels[index].r = r;
	pixels[index].g = g;
	pixels[index].b = b;
	ws2812_strip_flush();
}

void ws2812_strip_set_all(uint8_t r, uint8_t g, uint8_t b)
{
	for (size_t i = 0; i < STRIP_LEN; i++) {
		pixels[i].r = r;
		pixels[i].g = g;
		pixels[i].b = b;
	}
	ws2812_strip_flush();
}

void ws2812_strip_clear(void)
{
	ws2812_strip_set_all(0, 0, 0);
}

void ws2812_strip_set_hsv_spread(uint16_t hue, uint8_t sat, uint8_t val,
				 uint16_t spread)
{
	for (size_t i = 0; i < STRIP_LEN; i++) {
		uint16_t h = (hue + i * spread) % 360;
		uint8_t r, g, b;

		hsv_to_rgb(h, sat, val, &r, &g, &b);
		pixels[i].r = r;
		pixels[i].g = g;
		pixels[i].b = b;
	}
	current_hue = hue;
	ws2812_strip_flush();
}

void ws2812_strip_hue_shift(uint16_t delta)
{
	current_hue = (current_hue + delta) % 360;
	ws2812_strip_set_hsv_spread(current_hue, 255, 128, 30);
}

uint16_t ws2812_strip_get_hue(void)
{
	return current_hue;
}
