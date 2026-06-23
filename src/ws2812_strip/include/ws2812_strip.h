/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * WS2812B LED strip 驱动
 *
 * 基于 Zephyr led_strip API，通过 SPI 驱动 WS2812B/SK6812 等可寻址 RGB LED 灯带。
 *
 * 使用前需在 app overlay 中配置 worldsemi,ws2812-spi 节点（参考 dts/ws2812_strip.overlay），
 * 并在 prj.conf 中启用 CONFIG_LED_STRIP=y 和 CONFIG_WS2812_STRIP=y。
 */

#ifndef WS2812_STRIP_H_
#define WS2812_STRIP_H_

#include <stdint.h>
#include <stddef.h>

/**
 * @brief 初始化 WS2812B 灯带驱动。
 *
 * 调用 led_strip_update_rgb 前必须先调用此函数。
 *
 * @retval 0  成功
 * @retval -ENODEV  设备未就绪
 */
int ws2812_strip_init(void);

/**
 * @brief 获取灯带中灯珠的总数。
 *
 * @return 灯珠数量（由设备树 chain-length 决定）。
 */
size_t ws2812_strip_length(void);

/**
 * @brief 设置指定灯珠的颜色并立即刷新到灯带。
 *
 * @param index  灯珠序号 (0 ~ length-1)
 * @param r      红色通道 (0-255)
 * @param g      绿色通道 (0-255)
 * @param b      蓝色通道 (0-255)
 */
void ws2812_strip_set_led(int index, uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief 设置所有灯珠为同一颜色并立即刷新到灯带。
 *
 * @param r, g, b  颜色通道 (0-255)
 */
void ws2812_strip_set_all(uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief 关闭所有灯珠。
 */
void ws2812_strip_clear(void);

/**
 * @brief 以 HSV 色相均分方式设置灯带。
 *
 * 每颗灯珠的色相按 spread 等差偏移，适合跑马灯等视觉效果。
 *
 * @param hue     起始色相 (0-360)
 * @param sat     饱和度 (0-255)
 * @param val     亮度 (0-255)
 * @param spread  相邻灯珠色相差
 */
void ws2812_strip_set_hsv_spread(uint16_t hue, uint8_t sat, uint8_t val,
				 uint16_t spread);

/**
 * @brief 色相循环偏移（跑马灯效果）。
 *
 * 每次调用将所有灯珠色相增加 delta。
 *
 * @param delta  色相步进值
 */
void ws2812_strip_hue_shift(uint16_t delta);

/**
 * @brief 获取当前色相基准。
 */
uint16_t ws2812_strip_get_hue(void);

#endif /* WS2812_STRIP_H_ */
