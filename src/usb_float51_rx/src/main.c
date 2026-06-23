#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include <ares/ares_comm.h>
#include <ares/interface/usb/usb_bulk.h>
#include <ares/protocol/dual/dual_protocol.h>

LOG_MODULE_REGISTER(usb_float51_rx, LOG_LEVEL_INF);

#define CTRL_SYNC_ID 0x0101
#define CTRL_FLOAT_NUM 7
#define CTRL_BUTTON_NUM 48
#define CTRL_BUTTON_MASK_NUM 3
#define CTRL_BUTTONS_BUF_LEN 256

enum ctrl_data_index {
	CTRL_IDX_BUTTONS_0_15 = 0,
	CTRL_IDX_BUTTONS_16_31,
	CTRL_IDX_BUTTONS_32_47,
	CTRL_IDX_LX,
	CTRL_IDX_LY,
	CTRL_IDX_RX,
	CTRL_IDX_RY,
};

DUAL_PROPOSE_PROTOCOL_DEFINE(control_protocol);
ARES_BULK_INTERFACE_DEFINE(usb_bulk_interface);

static float ctrl_rx[CTRL_FLOAT_NUM];
static float ctrl_data[CTRL_FLOAT_NUM];
static uint32_t ctrl_rx_count;

K_SEM_DEFINE(ctrl_rx_sem, 0, 1);

static const char *const ctrl_button_names[CTRL_BUTTON_NUM] = {
	[0] = "LEFT_TRACKPAD",
	[1] = "RIGHT_TRACKPAD",
	[2] = "QUICK_ACCESS",
	[3] = "A",
	[4] = "B",
	[5] = "X",
	[6] = "Y",
	[7] = "LB",
	[8] = "RB",
	[9] = "LT_FULL",
	[10] = "RT_FULL",
	[11] = "VIEW",
	[12] = "MENU",
	[13] = "STEAM",
	[14] = "L3",
	[15] = "R3",
	[16] = "DPAD_UP",
	[17] = "DPAD_DOWN",
	[18] = "DPAD_LEFT",
	[19] = "DPAD_RIGHT",
	[20] = "L4",
	[21] = "R4",
	[22] = "L5",
	[23] = "R5",
	[32] = "VIRTUAL_BUTTON_1",
	[33] = "VIRTUAL_BUTTON_2",
	[34] = "VIRTUAL_BUTTON_3",
	[35] = "VIRTUAL_BUTTON_4",
	[36] = "VIRTUAL_BUTTON_5",
	[37] = "VIRTUAL_BUTTON_6",
	[38] = "VIRTUAL_BUTTON_7",
	[39] = "VIRTUAL_BUTTON_8",
	[40] = "ACTION_SELECT_3_LEFT",
	[41] = "ACTION_SELECT_3_MID",
	[42] = "ACTION_SELECT_3_RIGHT",
	[43] = "ACTION_SELECT_2_LEFT",
	[44] = "ACTION_SELECT_2_MID",
	[45] = "ACTION_SELECT_2_RIGHT",
	[46] = "ACTION_RELEASE",
	[47] = "ACTION_PLACE",
};

static uint16_t ctrl_float_to_mask(float value)
{
	if (value <= 0.0f) {
		return 0U;
	}

	if (value >= 65535.0f) {
		return UINT16_MAX;
	}

	return (uint16_t)(value + 0.5f);
}

static bool ctrl_button_pressed(const uint16_t masks[CTRL_BUTTON_MASK_NUM], uint8_t button_id)
{
	return (masks[button_id / 16U] & BIT(button_id % 16U)) != 0U;
}

static void ctrl_format_pressed_buttons(const uint16_t masks[CTRL_BUTTON_MASK_NUM],
					char *buf, size_t buf_len)
{
	size_t pos = 0U;

	if (buf_len == 0U) {
		return;
	}

	buf[0] = '\0';

	for (uint8_t button_id = 0U; button_id < CTRL_BUTTON_NUM; button_id++) {
		const char *name;
		int written;

		if (!ctrl_button_pressed(masks, button_id)) {
			continue;
		}

		name = ctrl_button_names[button_id];
		if (name == NULL) {
			continue;
		}

		written = snprintk(&buf[pos], buf_len - pos, "%s%s",
				   pos == 0U ? "" : ",", name);
		if (written < 0) {
			buf[0] = '\0';
			return;
		}

		if ((size_t)written >= buf_len - pos) {
			buf[buf_len - 1U] = '\0';
			return;
		}

		pos += (size_t)written;
	}

	if (pos == 0U) {
		(void)snprintk(buf, buf_len, "-");
	}
}

static void control_rx_cb(int status)
{
	if (status != SYNC_PACK_STATUS_DONE) {
		return;
	}

	memcpy(ctrl_data, ctrl_rx, sizeof(ctrl_data));
	ctrl_rx_count++;
	k_sem_give(&ctrl_rx_sem);
}

static void ctrl_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);
	int log_count = 0;
	while (1) {
		uint16_t masks[CTRL_BUTTON_MASK_NUM];
		char buttons[CTRL_BUTTONS_BUF_LEN];

		k_sem_take(&ctrl_rx_sem, K_FOREVER);

		masks[0] = ctrl_float_to_mask(ctrl_data[CTRL_IDX_BUTTONS_0_15]);
		masks[1] = ctrl_float_to_mask(ctrl_data[CTRL_IDX_BUTTONS_16_31]);
		masks[2] = ctrl_float_to_mask(ctrl_data[CTRL_IDX_BUTTONS_32_47]);
		ctrl_format_pressed_buttons(masks, buttons, sizeof(buttons));
		if(log_count++ % 10 != 0) {
			continue;
		}
		LOG_INF("rx=%u masks=0x%04x,0x%04x,0x%04x buttons=%s",
			ctrl_rx_count, masks[0], masks[1], masks[2], buttons);
		LOG_INF("sticks L(%.3f, %.3f) R(%.3f, %.3f)",
			(double)ctrl_data[CTRL_IDX_LX], (double)ctrl_data[CTRL_IDX_LY],
			(double)ctrl_data[CTRL_IDX_RX], (double)ctrl_data[CTRL_IDX_RY]);
	}
}

K_THREAD_DEFINE(ctrl_log, 2048, ctrl_thread, NULL, NULL, NULL, 6, 0, 0);

int main(void)
{
	int ret;
	sync_table_t *control_rx;

	LOG_INF("usb_float51_rx start");

	ret = ares_bind_interface(&usb_bulk_interface, &control_protocol);
	if (ret != 0) {
		LOG_ERR("failed to initialize ARES USB interface: %d", ret);
		return ret;
	}

	control_rx = dual_sync_add(&control_protocol, CTRL_SYNC_ID, (uint8_t *)ctrl_rx,
				   sizeof(ctrl_rx), control_rx_cb);
	if (control_rx == NULL) {
		LOG_ERR("failed to register control sync pack");
		return -ENOMEM;
	}

	LOG_INF("rx sync id=0x%04x, floats=%d", CTRL_SYNC_ID, CTRL_FLOAT_NUM);

	while (1) {
		k_sleep(K_SECONDS(1));
	}

	return 0;
}
