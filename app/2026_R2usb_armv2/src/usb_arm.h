#ifndef USB_ARM_H
#define USB_ARM_H

#include "zephyr/kernel.h"
#include <stdint.h>

struct AresProtocol;

/**
 * @file usb_arm.h
 * @brief 机械臂 USB SYNC 通信层
 *
 * 复用 ARES dual_protocol SYNC 帧承载，走 USB bulk。
 * 指令帧与反馈帧共用同一动作对象 ID(0x0203)，靠收/发方向区分。
 *
 * 帧格式: head=0x5A5A, id=0x0203, action(u32), reserved(f32×4) = 24字节
 */

/* SYNC 动作对象 ID（与 ares_r2_tool 一致） */
#define SYNC_ID_ARM 0x0203

/* 动作枚举：上位机 action 字符串映射到这些值 */
enum usb_arm_action {
	STAY_INIT = 1,                  /* 保持在home位 */

	FIRST_PICK_FORWARD_HIGH = 2,               /* 第一次取前方高位 */
	FIRST_PICK_FORWARD_LOW = 3,                /* 第一次取前方低位 */
	FIRST_PICK_LEFT_HIGH = 4,                  /* 第一次取左方高位 */
	FIRST_PICK_LEFT_LOW = 5,                   /* 第一次取左方低位 */
	FIRST_PICK_RIGHT_HIGH = 6,                 /* 第一次取右方高位 */
	FIRST_PICK_RIGHT_LOW = 7,                  /* 第一次取右方低位 */

	SECOND_PICK_FORWARD_HIGH = 8,              /* 第二次取前方高位 */
	SECOND_PICK_FORWARD_LOW = 9,               /* 第二次取前方低位 */
	SECOND_PICK_LEFT_HIGH = 10,                /* 第二次取左方高位 */
	SECOND_PICK_LEFT_LOW = 11,                 /* 第二次取左方低位 */
	SECOND_PICK_RIGHT_HIGH = 12,               /* 第二次取右方高位 */
	SECOND_PICK_RIGHT_LOW = 13,                /* 第二次取右方低位 */

	FIRST_PUT_2_FLOOR = 14,               /* 第一次放2层 */
	SECOND_PUT_3_FLOOR = 15,              /* 第二次放3层 */

};

/**
 * @brief 初始化 USB 通信并启动命令接收线程
 *
 * 在 main() 中调用，复用 R2usb 已绑定的 dual_protocol。
 */
int usb_arm_init(struct AresProtocol *protocol);

/**
 * @brief 发送动作完成反馈帧给上位机
 *
 * 由命令处理线程在动作执行完毕后调用。
 * @param action 刚完成的动作枚举值
 */
void usb_arm_send_done(enum usb_arm_action action);

void xipan_on(void);
void xipan_off(void);

#endif /* USB_ARM_H */
