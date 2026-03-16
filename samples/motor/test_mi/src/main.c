#include "ares/board/init.h"
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <zephyr/debug/thread_analyzer.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/motor.h>
#include <zephyr/drivers/sbus.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

#define MOTOR1_NODE DT_INST(0, mi_motor)
const struct device *motor1 = DEVICE_DT_GET(MOTOR1_NODE);

float motor1_rpm = 0;

/* CAN Feedback to console*/
k_tid_t feedback_tid = 0;

K_THREAD_STACK_DEFINE(feedback_stack_area, 4096); // 定义线程栈

void console_feedback(void *arg1, void *arg2, void *arg3)
{

	while (1) {
		LOG_INF("rpm: motor1: %.2f %.2f\n", (double)motor1_rpm,
			(double)motor_get_speed(motor1));
		k_msleep(200);
	}
}

int main(void)
{
	motor_control(motor1, ENABLE_MOTOR);
	k_sleep(K_MSEC(500));

	motor_set_mode(motor1, MIT);
	/* Start Feedback thread*/
	struct k_thread feedback_thread_data;
	feedback_tid = k_thread_create(&feedback_thread_data,
				       feedback_stack_area, // 修改为 can_send_stack_area
				       K_THREAD_STACK_SIZEOF(feedback_stack_area), console_feedback,
				       (void *)motor1, NULL, NULL, 0, 0, K_MSEC(300));
					   
	motor_control(motor1, SET_ZERO);
	while (1) {

		motor_set_mit(motor1, 10, 80, 0.3);
		k_msleep(500);
		motor_set_mit(motor1, 0, 0, 0);
		k_msleep(500);
	}
}
