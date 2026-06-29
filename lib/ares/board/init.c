/*
 * Copyright (c) 2025 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "zephyr/toolchain.h"
#include <zephyr/drivers/pwm.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/kernel/thread.h>
#include <zephyr/logging/log.h>
#include <zephyr/debug/thread_analyzer.h>
#include "init.h"

LOG_MODULE_REGISTER(board_init, CONFIG_BOARD_LOG_LEVEL);

struct led_rgb {
	uint8_t r;
	uint8_t g;
	uint8_t b;
};

#ifdef CONFIG_BOARD_DM_MC02
#define XT30_1_NODE DT_NODELABEL(power1)
#define XT30_2_NODE DT_NODELABEL(power2)

#if DT_NODE_EXISTS(XT30_1_NODE) && DT_NODE_HAS_PROP(XT30_1_NODE, gpios) &&                         \
	DT_NODE_EXISTS(XT30_2_NODE) && DT_NODE_HAS_PROP(XT30_2_NODE, gpios)
static const struct gpio_dt_spec pwr1 = GPIO_DT_SPEC_GET(XT30_1_NODE, gpios);
static const struct gpio_dt_spec pwr2 = GPIO_DT_SPEC_GET(XT30_2_NODE, gpios);

void pwr_init(void)
{
	gpio_pin_configure_dt(&pwr1, GPIO_OUTPUT_HIGH);
	gpio_pin_configure_dt(&pwr2, GPIO_OUTPUT_HIGH);
}

#define PWR_INIT pwr_init();
#else
#define PWR_INIT
#endif

#define LED_INIT

#define set_rgb_led_brightness(color)

#endif /* CONFIG_BOARD_DM_MC02 */

#ifdef CONFIG_BOARD_ROBOMASTER_BOARD_A

#define XT30_1_NODE DT_NODELABEL(power1)
#define XT30_2_NODE DT_NODELABEL(power2)
#define XT30_3_NODE DT_NODELABEL(power3)
#define XT30_4_NODE DT_NODELABEL(power4)

static const struct gpio_dt_spec pwr1 = GPIO_DT_SPEC_GET(XT30_1_NODE, gpios);
static const struct gpio_dt_spec pwr2 = GPIO_DT_SPEC_GET(XT30_2_NODE, gpios);
static const struct gpio_dt_spec pwr3 = GPIO_DT_SPEC_GET(XT30_3_NODE, gpios);
static const struct gpio_dt_spec pwr4 = GPIO_DT_SPEC_GET(XT30_4_NODE, gpios);

void pwr_init(void)
{
	gpio_pin_configure_dt(&pwr1, GPIO_OUTPUT_HIGH);
	gpio_pin_configure_dt(&pwr2, GPIO_OUTPUT_HIGH);
	gpio_pin_configure_dt(&pwr3, GPIO_OUTPUT_HIGH);
	gpio_pin_configure_dt(&pwr4, GPIO_OUTPUT_HIGH);
}

// #define LED_GREEN DT_ALIAS(led8)
// #define LED_RED DT_ALIAS(led9)

// const struct device *led_green = DEVICE_DT_GET(LED_GREEN);
// const struct device *led_red = DEVICE_DT_GET(LED_RED);

#define PWR_INIT pwr_init();
#define LED_INIT
#define set_rgb_led_brightness(color)

#endif /* CONFIG_BOARD_ROBOMASTER_BOARD_A */

#ifdef CONFIG_BOARD_ROBOMASTER_BOARD_C

#define LED_BLUE  DT_ALIAS(led_blue)
#define LED_GREEN DT_ALIAS(led_green)
#define LED_RED   DT_ALIAS(led_red)

static const struct pwm_dt_spec pwm_led_r = PWM_DT_SPEC_GET(LED_RED);   // 红色LED
static const struct pwm_dt_spec pwm_led_g = PWM_DT_SPEC_GET(LED_GREEN); // 绿色LED
static const struct pwm_dt_spec pwm_led_b = PWM_DT_SPEC_GET(LED_BLUE);  // 蓝色LED

// 定义PWM周期 (单位：纳秒)
// 这里我们假设亮度值是 0-255，所以周期也设置为 255 个单位时间
// 你可以根据实际需求调整周期，例如 10000 (10kHz 对应 100us 周期，如果亮度是0-100)
// PWM_DT_SPEC_GET 会从设备树中获取周期，所以这里定义的 period 仅用于计算 pulse_width
// 通常，我们会让 period 与亮度的最大值相对应，以简化计算。
// 例如，如果亮度范围是 0-255，并且PWM硬件的计数器可以达到255或更高，
// 那么将PWM周期设置为能表示256个级别的值是合适的。
// pwm_set_cycles 使用的是纳秒，但 pwm_dt_spec 会处理转换。
// 我们这里假设亮度值 (0-255) 直接映射到脉冲宽度。
#define PWM_PERIOD_FOR_BRIGHTNESS (pwm_led_r.period) // 使用从设备树获取的周期

/**
 * @brief 设置RGB LED的亮度
 *
 * @param r 红色LED的亮度 (0-255, 0为灭, 255为最亮)
 * @param g 绿色LED的亮度 (0-255, 0为灭, 255为最亮)
 * @param b 蓝色LED的亮度 (0-255, 0为灭, 255为最亮)
 *
 * @return 0 如果成功, 负数错误码如果失败.
 */
int set_rgb_led_brightness(struct led_rgb *color)
{
	int ret;
	uint32_t pulse_r, pulse_g, pulse_b;

	// 将亮度值 (0-255) 映射到脉冲宽度 (单位：纳秒)
	// 假设PWM周期 (pwm_led_x.period) 对应最大亮度 (255)
	// pulse_width = (brightness / 255.0) * period_ns
	pulse_r = (uint32_t)(((float)color->r / 255.0f) * PWM_PERIOD_FOR_BRIGHTNESS);
	pulse_g = (uint32_t)(((float)color->g / 255.0f) * PWM_PERIOD_FOR_BRIGHTNESS);
	pulse_b = (uint32_t)(((float)color->b / 255.0f) * PWM_PERIOD_FOR_BRIGHTNESS);

	// 设置红色LED的PWM占空比
	ret = pwm_set_dt(&pwm_led_r, pwm_led_r.period, pulse_r);
	if (ret < 0) {
		LOG_ERR("Failed to set red LED PWM (%d)", ret);
		return ret;
	}

	// 设置绿色LED的PWM占空比
	ret = pwm_set_dt(&pwm_led_g, pwm_led_g.period, pulse_g);
	if (ret < 0) {
		LOG_ERR("Failed to set green LED PWM (%d)", ret);
		return ret;
	}

	// 设置蓝色LED的PWM占空比
	ret = pwm_set_dt(&pwm_led_b, pwm_led_b.period, pulse_b);
	if (ret < 0) {
		LOG_ERR("Failed to set blue LED PWM (%d)", ret);
		return ret;
	}

	// LOG_DBG("RGB LED brightness set: R=%u, G=%u, B=%u (pulse width R:%u, G:%u, B:%u ns)",
	// 	color->r, color->g, color->b, pulse_r, pulse_g, pulse_b);

	return 0;
}
#define PWR_INIT
#define LED_INIT led_init();

#endif /* CONFIG_BOARD_ROBOMASTER_BOARD_C */

#define RGB(_r, _g, _b) ((struct led_rgb){.r = (_r), .g = (_g), .b = (_b)})

void led_serivce_func(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	struct k_thread_runtime_stats idle_stats_old, idle_stats_new;
	uint64_t idle_cycles_diff;
	uint64_t total_cycles_diff;
	struct led_rgb color;

	// uint8_t blue = 0u;
	// bool blue_decrease = false;

	uint32_t cnt = 0;

	while (1) {
		// 获取统计数据
		k_thread_runtime_stats_all_get(&idle_stats_old);
		k_msleep(100);
		k_thread_runtime_stats_all_get(&idle_stats_new);

#if CONFIG_SCHED_THREAD_USAGE_ALL
		// 计算CPU使用率 (0-100)
		idle_cycles_diff = idle_stats_new.idle_cycles - idle_stats_old.idle_cycles;
		total_cycles_diff =
			idle_stats_new.execution_cycles - idle_stats_old.execution_cycles;
		float cpu_usage = 100.0f * (1.0f - ((float)idle_cycles_diff / total_cycles_diff));
#else
		float cpu_usage = 0.2f;
#endif

		// // 映射到RGB值 (红色表示高负载，绿色表示低负载)
		uint8_t red = (uint8_t)((cpu_usage / 100.0f) * 0xff);
		uint8_t green = (uint8_t)((1.0f - cpu_usage / 100.0f) * 0xff);

		// if (blue_decrease) {
		// 	blue--;
		// 	if (blue == 0) {
		// 		blue_decrease = false;
		// 	}
		// } else {
		// 	blue++;
		// 	if (blue == 0xff) {
		// 		blue_decrease = true;
		// 	}
		// }

		color = RGB(red, green, 0);
		set_rgb_led_brightness(&color);

		if (cnt % 10 == 0) {
			LOG_DBG("CPU: %.1f%%, RGB: %02x%02x%02x", (double)cpu_usage, red, green, 0);
		}
		cnt++;
	}
}

static K_THREAD_STACK_DEFINE(led_serivce_stack, 1024); // 定义线程栈
static struct k_thread led_service_thread;
void led_init(void)
{
	struct led_rgb color = RGB(0x4F, 0x4F, 0x4F);
	set_rgb_led_brightness(&color);
	k_sleep(K_MSEC(300));

#ifdef CONFIG_BOARD_ROBOMASTER_BOARD_C
	// 检查PWM设备是否就绪
	if (!device_is_ready(pwm_led_r.dev) || !device_is_ready(pwm_led_g.dev) ||
	    !device_is_ready(pwm_led_b.dev)) {
		LOG_ERR("One or more PWM LED devices are not ready");
	}

	set_rgb_led_brightness(&color);
#endif /* CONFIG_BOARD_ROBOMASTER_BOARD_C */

	k_thread_create(&led_service_thread, led_serivce_stack,
			K_THREAD_STACK_SIZEOF(led_serivce_stack), led_serivce_func, NULL, NULL,
			NULL, -1, 0, K_NO_WAIT);
}

#ifndef PWR_INIT
#define PWR_INIT
#endif
#ifndef LED_INIT
#define LED_INIT
#endif

int board_init(void)
{
	k_sleep(K_MSEC(550));
	PWR_INIT
	LED_INIT
	LOG_INF("Board init done.");
	return 0;
}

SYS_INIT(board_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
