/*
 * Copyright (c) 2025 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Yinshi LA Series Micro Servo Electric Cylinder Demo
 *
 * Demonstrates:
 *   - Device initialization via devicetree
 *   - Position control with automatic feedback + retry (grasp/release)
 *   - Status query (position, temperature, current, faults)
 *   - Enable/disable/clear-fault commands
 *   - Error handling (timeout, invalid position)
 */

#include <stdio.h>
#include <stdlib.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/linear_actuator.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(la_demo, LOG_LEVEL_INF);

/*---------------------------------------------------------------------------*
 * Devicetree: pick up the demo device by node label "la_demo"
 *---------------------------------------------------------------------------*/

#define LA_DEMO_NODE DT_NODELABEL(la_demo)

#if !DT_NODE_EXISTS(LA_DEMO_NODE)
#error "No la_demo node found in devicetree overlay"
#endif

static const struct device *const la_dev = DEVICE_DT_GET(LA_DEMO_NODE);

/*---------------------------------------------------------------------------*
 * Grasp parameters (tune for your actuator stroke and mechanical setup)
 *---------------------------------------------------------------------------*/

/* Actuator position range is 0 ~ 2000.
 * 0   = fully retracted
 * 2000 = fully extended
 *
 * Adjust these to match your gripper mechanics. */
#define POS_OPEN  1400 /**< Open / release position */
#define POS_GRASP 2000 /**< Grasp / close position */
#define POS_MID   1700 /**< Midpoint for testing */

#define CYCLE_COUNT    3    /**< Number of open/close cycles */
#define CYCLE_DELAY_MS 2000 /**< Delay between moves (ms) */

/*---------------------------------------------------------------------------*
 * Helper: print status
 *---------------------------------------------------------------------------*/

static void print_status(const struct la_status *s)
{
	LOG_INF("Status:");
	LOG_INF("  position:  %d / %d (current/target)", s->current_pos, s->target_pos);
	LOG_INF("  temperature: %d C", s->temperature);
	LOG_INF("  current:    %u mA", s->current_ma);
	LOG_INF("  force:      %d g", s->force_grams);
	LOG_INF("  faults:     0x%02x %s%s%s%s", s->faults,
		(s->faults & LA_FAULT_STALL) ? "[STALL] " : "",
		(s->faults & LA_FAULT_OVERTEMP) ? "[OVERTEMP] " : "",
		(s->faults & LA_FAULT_OVERCURRENT) ? "[OVERCURRENT] " : "",
		(s->faults & LA_FAULT_MOTOR_ERROR) ? "[MOTOR_ERR] " : "");
	LOG_INF("  online:      %s", s->online ? "yes" : "no");
}

/*---------------------------------------------------------------------------*
 * Helper: move to position and wait for settling
 *---------------------------------------------------------------------------*/

static int move_to(uint16_t position)
{
	int ret;

	LOG_INF("Moving to position %u ...", position);
	ret = la_set_position(la_dev, position);
	if (ret < 0) {
		LOG_ERR("la_set_position(%u) failed: %d", position, ret);
		return ret;
	}

	/* Give the actuator time to settle at target position */
	k_sleep(K_MSEC(CYCLE_DELAY_MS));
	return 0;
}

/*---------------------------------------------------------------------------*
 * Helper: query and display status
 *---------------------------------------------------------------------------*/

static int query_and_show(void)
{
	struct la_status status;
	int ret;

	ret = la_get_status(la_dev, &status);
	if (ret < 0) {
		LOG_ERR("la_get_status failed: %d", ret);
		return ret;
	}

	print_status(&status);

	/* Check for faults */
	if (status.faults != 0) {
		LOG_WRN("Fault detected, attempting clear ...");
		ret = la_clear_fault(la_dev);
		if (ret < 0) {
			LOG_ERR("la_clear_fault failed: %d", ret);
		} else {
			LOG_INF("Fault cleared");
		}
	}

	return 0;
}

/*---------------------------------------------------------------------------*
 * Main demo
 *---------------------------------------------------------------------------*/

void main(void)
{
	struct la_status status;
	int ret;

	LOG_INF("=== Yinshi LA Electric Cylinder Demo ===");

	/* --- Check device ready --- */
	if (!device_is_ready(la_dev)) {
		LOG_ERR("LA device not ready");
		return;
	}
	LOG_INF("Device '%s' ready", la_dev->name);

	/* --- Enable actuator --- */
	LOG_INF("Enabling actuator ...");
	ret = la_enable(la_dev);
	if (ret < 0) {
		LOG_ERR("la_enable failed: %d", ret);
		return;
	}
	LOG_INF("Actuator enabled");
	k_sleep(K_MSEC(100));

	/* --- Initial status --- */
	LOG_INF("--- Initial Status ---");
	query_and_show();

	/* --- Grasp cycle loop --- */
	LOG_INF("--- Starting %d grasp/release cycles ---", CYCLE_COUNT);

	for (int i = 1; i <= CYCLE_COUNT; i++) {
		LOG_INF("=== Cycle %d/%d ===", i, CYCLE_COUNT);

		/* Close (grasp) */
		ret = move_to(POS_GRASP);
		if (ret < 0) {
			LOG_ERR("Grasp failed at cycle %d", i);
			break;
		}
		query_and_show();

		/* Open (release) */
		ret = move_to(POS_OPEN);
		if (ret < 0) {
			LOG_ERR("Release failed at cycle %d", i);
			break;
		}
		query_and_show();
	}

	/* --- Mid-point and final status --- */
	LOG_INF("--- Moving to mid-point ---");
	move_to(POS_MID);
	query_and_show();

	/* --- Demonstrate clear-fault robustness (no-op if no fault) --- */
	LOG_INF("--- Clear fault test ---");
	ret = la_clear_fault(la_dev);
	if (ret == 0) {
		LOG_INF("Clear fault OK");
	} else {
		LOG_WRN("Clear fault returned: %d (may be OK if no fault)", ret);
	}

	/* --- Demonstrate save-params (be careful: writes Flash!) --- */
	LOG_INF("--- Save params (skipped by default, uncomment to enable) ---");
	/* ret = la_save_params(la_dev);
	 * if (ret == 0) {
	 *     LOG_INF("Params saved to Flash");
	 * } else {
	 *     LOG_ERR("Save params failed: %d", ret);
	 * }
	 */

	/* --- Demonstrate invalid position rejection --- */
	LOG_INF("--- Invalid position test ---");
	ret = la_set_position(la_dev, 3000); /* exceeds max 2000 */
	LOG_INF("la_set_position(3000) -> %d (expected -EINVAL=%d)", ret, -EINVAL);

	/* --- Disable --- */
	LOG_INF("Disabling actuator ...");
	ret = la_disable(la_dev);
	if (ret < 0) {
		LOG_ERR("la_disable failed: %d", ret);
	} else {
		LOG_INF("Actuator disabled");
	}

	/* --- Final status --- */
	k_sleep(K_MSEC(50));
	la_get_status(la_dev, &status);
	LOG_INF("Final online: %s", status.online ? "yes" : "no");

	LOG_INF("=== Demo complete ===");
}
