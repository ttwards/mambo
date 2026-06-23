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

#define G 9.80665f

#define HIGH_BYTE(x) ((x) >> 8)
#define LOW_BYTE(x) ((x)&0xFF)
#define COMBINE_HL8(HIGH, LOW) ((HIGH << 8) + LOW)

#define MOTOR1_NODE DT_INST(0, mi_motor)

const struct device *motor1 = DEVICE_DT_GET(MOTOR1_NODE);

#define CPU_NODE DT_NODELABEL(cpu0)

const struct device *cpu_dev = DEVICE_DT_GET(CPU_NODE);

k_tid_t feedback_tid = 0;

float motor1_rpm = 0;
float motor2_rpm = 0;

/* CAN Feedback to console*/
K_THREAD_STACK_DEFINE(feedback_stack_area, 4096); // 定义线程栈
void console_feedback(void *arg1, void *arg2, void *arg3) {

  while (1) {
    LOG_INF(" motor1: %.2f %.2f\n", (double)motor1_rpm,
            (double)motor_get_angle(motor1));

    k_msleep(200);
  }
}

int main(void) {
  // board_init();
  LOG_INF("yes");
  k_sleep(K_MSEC(1000));
  // motor_control(motor1, ENABLE_MOTOR);

  motor_set_mode(motor1, MIT);
  /* Start Feedback thread*/
  struct k_thread feedback_thread_data;
  feedback_tid = k_thread_create(
      &feedback_thread_data,
      feedback_stack_area, // 修改为 can_send_stack_area
      K_THREAD_STACK_SIZEOF(feedback_stack_area), console_feedback,
      (void *)motor1, NULL, NULL, 0, 0, K_MSEC(300));
  motor_control(motor1, SET_ZERO);
  while (1) {

    motor_set_speed(motor1, 10);
    motor_set_torque(motor1, 0.1);
    motor_set_angle(motor1, 360);

    k_msleep(500);
  }
}