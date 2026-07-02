/*
 * Copyright (c) 2025 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/kernel/thread.h>
#include <zephyr/logging/log.h>
#include <zephyr/debug/thread_analyzer.h>

#include "init.h"

LOG_MODULE_REGISTER(board_init, CONFIG_BOARD_LOG_LEVEL);

#define RGB(_r, _g, _b) ((struct ares_led_rgb){.r = (_r), .g = (_g), .b = (_b)})

static float read_cpu_usage_percent(void)
{
#if IS_ENABLED(CONFIG_SCHED_THREAD_USAGE_ALL)
	struct k_thread_runtime_stats idle_stats_old;
	struct k_thread_runtime_stats idle_stats_new;

	k_thread_runtime_stats_all_get(&idle_stats_old);
	k_msleep(100);
	k_thread_runtime_stats_all_get(&idle_stats_new);

	uint64_t idle_cycles_diff = idle_stats_new.idle_cycles - idle_stats_old.idle_cycles;
	uint64_t total_cycles_diff =
		idle_stats_new.execution_cycles - idle_stats_old.execution_cycles;

	if (total_cycles_diff > 0U) {
		return 100.0f * (1.0f - ((float)idle_cycles_diff / total_cycles_diff));
	}

	return 0.0f;
#else
	k_msleep(100);
	return 0.2f;
#endif
}

static void led_service_func(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	uint32_t cnt = 0;

	while (1) {
		float cpu_usage = read_cpu_usage_percent();
		uint8_t max_channel = ares_board_status_led_max_channel();
		uint8_t red = (uint8_t)((cpu_usage / 100.0f) * max_channel);
		uint8_t green = (uint8_t)((1.0f - cpu_usage / 100.0f) * max_channel);
		struct ares_led_rgb color = RGB(red, green, 0);

		ares_board_status_led_set_rgb(&color);

		if (cnt % 10 == 0) {
			LOG_DBG("CPU: %.1f%%, RGB: %02x%02x%02x", (double)cpu_usage, red, green, 0);
		}
		cnt++;
	}
}

static K_THREAD_STACK_DEFINE(led_service_stack, 1024);
static struct k_thread led_service_thread;

void ares_board_status_led_service_start(void)
{
	k_thread_create(&led_service_thread, led_service_stack,
			K_THREAD_STACK_SIZEOF(led_service_stack), led_service_func, NULL, NULL,
			NULL, -1, 0, K_NO_WAIT);
}

static int board_init(void)
{
	k_sleep(K_MSEC(550));
	ares_board_power_init();
	if (IS_ENABLED(CONFIG_ARES_BOARD_STATUS_LED)) {
		ares_board_status_led_init();
	}
	LOG_INF("Board init done.");
	return 0;
}

SYS_INIT(board_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
