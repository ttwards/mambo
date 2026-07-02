/**
 * @file usb_arm.c
 * @brief 机械臂 USB SYNC 通信实现
 *
 * 复用 R2usb 主程序已经绑定到 USB bulk 的 dual_protocol。
 */

#include "usb_arm.h"
#include "arm_poses.h"
#include "arm_control.h"

#include <stddef.h>
#include <stdbool.h>
#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <ares/protocol/dual/dual_protocol.h>

LOG_MODULE_REGISTER(usb_arm, LOG_LEVEL_DBG);

/* =======================================================================
 * SYNC 帧收发缓冲区
 * ======================================================================= */

/*
 * SYNC payload 结构：action(u32) + 4×float32 reserved = 20字节
 * 完整帧：head(2) + id(2) + payload(20) = 24字节
 */
struct arm_sync_frame {
	uint32_t action;
	float reserved[4];
};

static struct arm_sync_frame arm_cmd_rx;

static struct arm_sync_frame arm_done_tx;

static struct AresProtocol *arm_protocol;

static sync_table_t *arm_done_pack;

/* =======================================================================
 * 命令分发：回调 → 信号量 → 工作线程
 * ======================================================================= */

/* 待执行的命令（由回调写入，由工作线程读取） */
static volatile enum usb_arm_action pending_action;

/* 通用参数快照（reserved[4]） */
static float pending_args[4];

/* 命令到达信号量：回调 give，工作线程 take */
K_SEM_DEFINE(arm_cmd_sem, 0, 1);

static void arm_cmd_cb(int status)
{
	if (status != SYNC_PACK_STATUS_DONE) {
		return;
	}

	enum usb_arm_action action = (enum usb_arm_action)arm_cmd_rx.action;


	/* 快照通用参数（回调返回后 arm_cmd_rx 可能被下一帧覆盖） */
	for (int i = 0; i < 4; i++) {
		pending_args[i] = arm_cmd_rx.reserved[i];
	}

	pending_action = action;
	k_sem_give(&arm_cmd_sem);
}

void usb_arm_send_done(enum usb_arm_action action)
{
	arm_done_tx.action = (uint32_t)action;
	arm_done_tx.reserved[0] = 0.0f;
	arm_done_tx.reserved[1] = 0.0f;
	arm_done_tx.reserved[2] = 0.0f;
	arm_done_tx.reserved[3] = 0.0f;

	if (arm_protocol == NULL || arm_done_pack == NULL) {
		LOG_WRN("arm protocol not ready");
		return;
	}

	if (dual_sync_flush(arm_protocol, arm_done_pack) != 0) {
		LOG_WRN("failed to send arm done frame");
	}
}

static void arm_cmd_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	while (1) {
		/* 阻塞等待上位机命令 */
		k_sem_take(&arm_cmd_sem, K_FOREVER);

		enum usb_arm_action action = pending_action;

		float speed = pending_args[0];

		LOG_INF("USB cmd: action=%u, speed=%.1f deg/s", (unsigned)action, (double)speed);

		switch (action) {
		case STAY_INIT: {
			execute_pose_sequence(init, init_count);
			break;
		}


		case FIRST_PICK_FORWARD_HIGH: {
			execute_path((const motor_angles_t[]){ init[0], forward_positive_200[0] }, 2, speed);
			xipan_on();
			execute_path((const motor_angles_t[]){ forward_positive_200[0], store1[0] }, 2, speed);
			xipan_off();
			execute_path((const motor_angles_t[]){ store1[0], init[0] }, 2, speed);
			break;
		}

		case FIRST_PICK_FORWARD_LOW: {
			execute_path((const motor_angles_t[]){ init[0], forward_negative_200[0] }, 2, speed);
			xipan_on();
			execute_path((const motor_angles_t[]){ forward_negative_200[0], store1[0] }, 2, speed);
			xipan_off();
			execute_path((const motor_angles_t[]){ store1[0], init[0] }, 2, speed);
			break;
		}

		case FIRST_PICK_LEFT_HIGH: {
			execute_path((const motor_angles_t[]){ init[0], left_positive_200[0] }, 2, speed);
			xipan_on();
			execute_path((const motor_angles_t[]){ left_positive_200[0], store1[0] }, 2, speed);
			xipan_off();
			execute_path((const motor_angles_t[]){ store1[0], init[0] }, 2, speed);
			break;
		}

		case FIRST_PICK_LEFT_LOW: {
			execute_path((const motor_angles_t[]){ init[0], left_negative_200[0] }, 2, speed);
			xipan_on();
			execute_path((const motor_angles_t[]){ left_negative_200[0], store1[0] }, 2, speed);
			xipan_off();
			execute_path((const motor_angles_t[]){ store1[0], init[0] }, 2, speed);
			break;
		}

		case FIRST_PICK_RIGHT_HIGH: {
			execute_path((const motor_angles_t[]){ init[0], right_positive_200[0] }, 2, speed);
			xipan_on();
			execute_path((const motor_angles_t[]){ right_positive_200[0], store1[0] }, 2, speed);
			xipan_off();
			execute_path((const motor_angles_t[]){ store1[0], init[0] }, 2, speed);
			break;
		}

		case FIRST_PICK_RIGHT_LOW: {
			execute_path((const motor_angles_t[]){ init[0], right_negative_200[0] }, 2, speed);
			xipan_on();
			execute_path((const motor_angles_t[]){ right_negative_200[0], store1[0] }, 2, speed);
			xipan_off();
			execute_path((const motor_angles_t[]){ store1[0], init[0] }, 2, speed);
			break;
		}


		case SECOND_PICK_FORWARD_HIGH: {
			execute_path((const motor_angles_t[]){ init[0], forward_positive_200[0] }, 2, speed);
			xipan_on();
			k_msleep(1000);
			execute_path((const motor_angles_t[]){ forward_positive_200[0], store2[0] }, 2, speed);
			break;
		}

		case SECOND_PICK_FORWARD_LOW: {
			execute_path((const motor_angles_t[]){ init[0], forward_negative_200[0] }, 2, speed);
			xipan_on();
			k_msleep(1000);
			execute_path((const motor_angles_t[]){ forward_negative_200[0], store2[0] }, 2, speed);
			break;
		}

		case SECOND_PICK_LEFT_HIGH: {
			execute_path((const motor_angles_t[]){ init[0], left_positive_200[0] }, 2, speed);
			xipan_on();
			k_msleep(1000);
			execute_path((const motor_angles_t[]){ left_positive_200[0], store2[0] }, 2, speed);
			break;
		}

		case SECOND_PICK_LEFT_LOW: {
			execute_path((const motor_angles_t[]){ init[0], left_negative_200[0] }, 2, speed);
			xipan_on();
			execute_path((const motor_angles_t[]){ left_negative_200[0], store2[0] }, 2, speed);
			break;
		}

		case SECOND_PICK_RIGHT_HIGH: {
			execute_path((const motor_angles_t[]){ init[0], right_positive_200[0] }, 2, speed);
			xipan_on();
			execute_path((const motor_angles_t[]){ right_positive_200[0], store2[0] }, 2, speed);
			break;
		}

		case SECOND_PICK_RIGHT_LOW: {
			execute_path((const motor_angles_t[]){ init[0], right_negative_200[0] }, 2, speed);
			xipan_on();
			execute_path((const motor_angles_t[]){ right_negative_200[0], store2[0] }, 2, speed);
			break;
		}


		case FIRST_PUT_2_FLOOR: {
			execute_path((const motor_angles_t[]){ store2[0], put_2_floor[0] }, 2, speed);
			xipan_off();
			execute_path((const motor_angles_t[]){ put_2_floor[0], init[0] }, 2, speed);
			break;
		}


		case SECOND_PUT_3_FLOOR: {
			execute_path((const motor_angles_t[]){ init[0], store1[0] }, 2, speed);
			xipan_on();
			execute_path((const motor_angles_t[]){ store1[0], put_3_floor[0] }, 2, speed);
			xipan_off();
			execute_path((const motor_angles_t[]){ put_3_floor[0], init[0] }, 2, speed);
			break;
		}

		default:
			LOG_WRN("unhandled action: %u", (unsigned)action);
			break;
		}

		/* 动作执行完毕，发送反馈帧 */
		usb_arm_send_done(action);
		LOG_INF("USB cmd done: action=%u", (unsigned)action);
	}
}

K_THREAD_DEFINE(arm_usb_cmd, 4096, arm_cmd_thread, NULL, NULL, NULL, 5, 0, 0);


int usb_arm_init(struct AresProtocol *protocol)
{
	if (protocol == NULL) {
		return -EINVAL;
	}

	arm_protocol = protocol;

	/* 注册命令帧 pack：收到 0x5A5A+0x0203 时解析到 arm_cmd_rx 并调用回调 */
	if (dual_sync_add(arm_protocol, SYNC_ID_ARM, (uint8_t *)&arm_cmd_rx,
			  sizeof(arm_cmd_rx), arm_cmd_cb) == NULL) {
		LOG_ERR("failed to register arm cmd sync pack");
		return -ENOMEM;
	}

	/* 注册反馈帧 pack：无回调，仅用于 dual_sync_flush 发送 */
	arm_done_pack = dual_sync_add(arm_protocol, SYNC_ID_ARM,
				      (uint8_t *)&arm_done_tx, sizeof(arm_done_tx), NULL);
	if (arm_done_pack == NULL) {
		LOG_ERR("failed to register arm done sync pack");
		return -ENOMEM;
	}

	LOG_INF("USB arm comm initialized (SYNC ID=0x%04X)", SYNC_ID_ARM);
	return 0;
}
