#include "ares/board/init.h"
#include "math.h"
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
#include "cycloid.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

#define G 9.80665f

#define HIGH_BYTE(x) ((x) >> 8)
#define LOW_BYTE(x) ((x)&0xFF)
#define COMBINE_HL8(HIGH, LOW) ((HIGH << 8) + LOW)

#define MOTOR1_NODE DT_INST(0, mi_motor)
#define MOTOR2_NODE DT_INST(1, mi_motor)
#define MOTOR3_NODE DT_INST(2, mi_motor)
#define MOTOR4_NODE DT_INST(3, mi_motor)
#define MOTOR5_NODE DT_INST(4, mi_motor)
#define MOTOR6_NODE DT_INST(5, mi_motor)
#define MOTOR7_NODE DT_INST(6, mi_motor)
#define MOTOR8_NODE DT_INST(7, mi_motor)
#define MOTOR9_NODE DT_INST(8, mi_motor)

const struct device *motor1 = DEVICE_DT_GET(MOTOR1_NODE);
const struct device *motor2 = DEVICE_DT_GET(MOTOR2_NODE);
const struct device *motor3 = DEVICE_DT_GET(MOTOR3_NODE);
const struct device *motor4 = DEVICE_DT_GET(MOTOR4_NODE);
const struct device *motor5 = DEVICE_DT_GET(MOTOR5_NODE);
const struct device *motor6 = DEVICE_DT_GET(MOTOR6_NODE);
const struct device *motor7 = DEVICE_DT_GET(MOTOR7_NODE);
const struct device *motor8 = DEVICE_DT_GET(MOTOR8_NODE);
const struct device *motor9 = DEVICE_DT_GET(MOTOR8_NODE);

// #define TIMER_NODE DT_NODELABEL(timers10)
// const struct device *timer = DEVICE_DT_GET(TIMER_NODE);
k_tid_t feedback_tid = 0;
k_tid_t leg1_tid = 0;
k_tid_t leg2_tid = 0;
k_tid_t leg3_tid = 0;
k_tid_t leg4_tid = 0;
k_tid_t waist_tid = 0;

/* CAN Feedback to console*/
K_THREAD_STACK_DEFINE(feedback_stack_area, 4096); // 定义线程栈
void console_feedback(void *arg1, void *arg2, void *arg3) {

  while (1) {
    // LOG_INF("rpm: motor1: %.2f %.2f\n", (double)motor1_rpm,
    //         (double)motor_get_speed(motor1));
    // LOG_INF("rpm: motor2: %.2f %.2f\n", (double)motor2_rpm,
    //         (double)motor_get_speed(motor2));
    k_msleep(200);
  }
}

K_THREAD_STACK_DEFINE(leg1_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(leg2_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(leg3_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(leg4_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(waist_stack, STACK_SIZE);

float forward_angle1[TRAJ_POINTS];
float forward_angle4[TRAJ_POINTS];
float backward_angle1[TRAJ_POINTS];
float backward_angle4[TRAJ_POINTS];

int main(void) 
{
  Point2D start = {-0.15f, -0.20f};
  Point2D end = {0.15f, -0.2f};

  leg_motors leg1 = { motor1, motor2, LEFT_BACK};
  leg_motors leg2 = { motor4, motor3, RIGHT_BACK};
  leg_motors leg3 = { motor5, motor6, LEFT_FRONT};
  leg_motors leg4 = { motor8, motor7, RIGHT_FRONT};
  
  dog_initialization(start,end,forward_angle1,forward_angle4,backward_angle1,backward_angle4);
  k_sleep(K_MSEC(500)); 
  // motor_control(motor1, ENABLE_MOTOR); k_sleep(K_MSEC(10));
  // motor_control(motor2, ENABLE_MOTOR); k_sleep(K_MSEC(10));
  // motor_control(motor3, ENABLE_MOTOR); k_sleep(K_MSEC(10));
  // motor_control(motor4, ENABLE_MOTOR); k_sleep(K_MSEC(10));
  // motor_control(motor5, ENABLE_MOTOR); k_sleep(K_MSEC(10));
  // motor_control(motor6, ENABLE_MOTOR); k_sleep(K_MSEC(10));
  // motor_control(motor7, ENABLE_MOTOR); k_sleep(K_MSEC(10));
  // motor_control(motor8, ENABLE_MOTOR); k_sleep(K_MSEC(10));
  
  motor_set_mode(motor1, MIT);  k_sleep(K_MSEC(1));
  motor_set_mode(motor2, MIT);  k_sleep(K_MSEC(1));
  motor_set_mode(motor3, MIT);  k_sleep(K_MSEC(1));
  motor_set_mode(motor4, MIT);  k_sleep(K_MSEC(10));
  motor_set_mode(motor5, MIT);  k_sleep(K_MSEC(1));
  motor_set_mode(motor6, MIT);  k_sleep(K_MSEC(1));
  motor_set_mode(motor7, MIT);  k_sleep(K_MSEC(10));
  motor_set_mode(motor8, MIT);  k_sleep(K_MSEC(10));

  // Leg leg1 = { motor1, motor2, INITIAL_ANGLE1, INITIAL_ANGLE2, LEFT_BACK};
  // Leg leg2 = { motor4, motor3, INITIAL_ANGLE3, INITIAL_ANGLE4, RIGHT_BACK};
  // Leg leg3 = { motor5, motor6, INITIAL_ANGLE5, INITIAL_ANGLE6, LEFT_FRONT};
  // Leg leg4 = { motor8, motor7, INITIAL_ANGLE7, INITIAL_ANGLE8, RIGHT_FRONT};

  /* Start Feedback thread*/
  struct k_thread feedback_thread_data;
  struct k_thread leg1_thread_data;
  struct k_thread leg2_thread_data;
  struct k_thread leg3_thread_data;
  struct k_thread leg4_thread_data;
  struct k_thread waist_thread_data;

  k_sleep(K_MSEC(500));

  feedback_tid = k_thread_create(
      &feedback_thread_data,
      feedback_stack_area, // 修改为 can_send_stack_area
      K_THREAD_STACK_SIZEOF(feedback_stack_area), console_feedback,
      (void *)motor1, NULL, NULL, 0, 0, K_MSEC(300));

  leg1_tid = k_thread_create(&leg1_thread_data, leg1_stack, K_THREAD_STACK_SIZEOF(leg1_stack),
      leg_control_thread, &leg1, NULL, NULL,
      THREAD_PRIORITY, 0, K_NO_WAIT);

  leg2_tid = k_thread_create(&leg2_thread_data, leg2_stack, K_THREAD_STACK_SIZEOF(leg2_stack),
      leg_control_thread, &leg2, NULL, NULL,
      THREAD_PRIORITY, 0, K_NO_WAIT);

  leg3_tid = k_thread_create(&leg3_thread_data, leg3_stack, K_THREAD_STACK_SIZEOF(leg3_stack),
      leg_control_thread, &leg3, NULL, NULL,
      THREAD_PRIORITY, 0, K_NO_WAIT);

  leg4_tid = k_thread_create(&leg4_thread_data, leg4_stack, K_THREAD_STACK_SIZEOF(leg4_stack),
      leg_control_thread, &leg4, NULL, NULL,
      THREAD_PRIORITY, 0, K_NO_WAIT); 

  waist_tid = k_thread_create(&waist_thread_data, waist_stack, K_THREAD_STACK_SIZEOF(waist_stack),
      waist_control_thread, motor9, NULL, NULL,
      THREAD_PRIORITY, 0, K_NO_WAIT);

  k_sleep(K_MSEC(500));

  while (1) {  
  }
}