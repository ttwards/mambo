#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/motor.h>
#include <zephyr/drivers/sbus.h>
#include <ares/board/init.h>
#include <ares/ekf/imu_task.h>
#include <ares/interface/usb/usb_bulk.h>
#include <ares/protocol/dual/dual_protocol.h>
#include <ares/ares_comm.h>
#include <sys/_stdint.h>
#include "ares/ekf/QuaternionEKF.h"
#include <arm_math.h>
#include <zephyr/drivers/gpio.h>


LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

#ifndef M_PI
#define M_PI 3.14159265f
#endif


#define STACK_SIZE 4096


#define JOINT1_NODE DT_NODELABEL(rs_motor1) // yaw轴
#define JOINT2_NODE DT_NODELABEL(rs_motor2) // 大臂
#define JOINT3_NODE DT_NODELABEL(rs_motor3) // 小臂
#define JOINT4_NODE DT_NODELABEL(dm_motor)  // 末端执行器
#define EMVALVE1_NODE DT_NODELABEL(emvalve1)//夹爪电磁阀
#define EMVALVE2_NODE DT_NODELABEL(emvalve2)//吸盘电磁阀


const struct device *joint1_motor = DEVICE_DT_GET(JOINT1_NODE);
const struct device *joint2_motor = DEVICE_DT_GET(JOINT2_NODE);
const struct device *joint3_motor = DEVICE_DT_GET(JOINT3_NODE);
const struct device *joint4_motor = DEVICE_DT_GET(JOINT4_NODE);

static const struct gpio_dt_spec emvalve1 = GPIO_DT_SPEC_GET(EMVALVE1_NODE, gpios);
static const struct gpio_dt_spec emvalve2 = GPIO_DT_SPEC_GET(EMVALVE2_NODE, gpios);

// ============== 反馈电机角度线程 ================
float m1_fb = 0.0f;
float m2_fb = 0.0f;
float m3_fb = 0.0f;
float m4_fb = 0.0f;

void feedback_thread(void *arg1, void *arg2, void *arg3)
{
    while (1) {
        // m1_fb = motor_get_angle(joint1_motor);
        m2_fb = motor_get_angle(joint2_motor);
        m3_fb = motor_get_angle(joint3_motor);
        m4_fb = motor_get_angle(joint4_motor);
        k_msleep(10);
    }
    
}

K_THREAD_DEFINE(arm_thread_id, 4096, feedback_thread, NULL, NULL, NULL, 2, 0, 100);

// ============== 硬编码位姿 ================
typedef struct {
    float joint1;
    float joint2;
    float joint3;
    float joint4;
} arm_pose_t;

arm_pose_t init_to_1[] = {
    {0.0f,   0.00f,   0.00f,   0.00f},
    {0.0f,   2.27f,  -0.54f,   0.94f},
    {0.0f,   2.27f,  -0.56f,   1.88f},
    {0.0f,   2.38f,  -0.58f,   2.82f},
    {0.0f,   3.77f,  -1.88f,   3.76f},
    {0.0f,   6.34f,  -4.43f,   4.69f},
    {0.0f,   8.78f,  -6.87f,   5.63f},
    {0.0f,  12.76f, -10.65f,   6.57f},
    {0.0f,  16.23f, -13.97f,   7.51f},
    {0.0f,  18.58f, -15.97f,   8.45f},
    {0.0f,  22.78f, -20.12f,   9.39f},
    {0.0f,  25.86f, -23.42f,  10.33f},
    {0.0f,  26.01f, -24.91f,  11.27f},
    {0.0f,  25.99f, -27.95f,  12.21f},
    {0.0f,  24.32f, -30.58f,  13.15f},
    {0.0f,  21.37f, -30.89f,  14.08f},
    {0.0f,  18.12f, -31.11f,  15.02f},
    {0.0f,  14.80f, -31.33f,  15.96f},
    {0.0f,  11.64f, -33.27f,  16.90f},
    {0.0f,   7.88f, -34.34f,  17.84f},
    {0.0f,   2.58f, -35.18f,  18.78f},
    {0.0f,  -3.48f, -37.05f,  19.72f},
    {0.0f, -10.01f, -38.06f,  20.66f},
    {0.0f, -16.87f, -39.66f,  21.60f},
    {0.0f, -22.98f, -40.45f,  22.54f},
    {0.0f, -26.72f, -40.61f,  23.47f},
    {0.0f, -30.94f, -41.46f,  24.41f},
    {0.0f, -37.31f, -43.05f,  25.35f},
    {0.0f, -43.57f, -44.17f,  26.29f},
    {0.0f, -47.24f, -44.15f,  27.23f},
    {0.0f, -50.17f, -44.43f,  28.17f},
    {0.0f, -51.75f, -44.15f,  29.11f},
    {0.0f, -52.72f, -44.27f,  30.05f},
    {0.0f, -52.46f, -44.91f,  31.99f},
    {0.0f, -52.43f, -45.94f,  33.93f},
    {0.0f, -52.10f, -46.94f,  35.87f},
    {0.0f, -52.30f, -48.87f,  37.80f},
};

arm_pose_t init_to_2[] = {
    {0.0f,   0.30f,  -0.05f,  -2.50f},
    {0.0f,   1.46f,  -0.19f,  -2.51f},
    {0.0f,   3.03f,  -1.19f,  -2.81f},
    {0.0f,   5.23f,  -2.22f,  -3.11f},
    {0.0f,   8.16f,  -3.53f,  -3.41f},
    {0.0f,  10.50f,  -4.53f,  -3.41f},
    {0.0f,  12.69f,  -5.80f,  -3.40f},
    {0.0f,  14.33f,  -6.72f,  -3.41f},
    {0.0f,  15.87f,  -10.02f, -3.41f},
    {0.0f,  15.92f,  -11.28f, -5.03f},
    {0.0f,  16.06f,  -12.40f, -5.03f},
    {0.0f,  15.98f,  -13.44f, -6.98f},
    {0.0f,  14.11f,  -14.44f, -9.05f},
    {0.0f,  11.44f,  -15.17f, -9.41f},
    {0.0f,   7.40f,  -15.50f, -10.29f},
    {0.0f,   2.79f,  -16.59f, -13.62f},
    {0.0f,  -2.92f,  -17.14f, -15.69f},
    {0.0f,  -6.24f,  -17.74f, -15.69f},
    {0.0f,  -9.63f,  -18.47f, -15.69f},
    {0.0f, -12.96f,  -19.38f, -17.22f},
    {0.0f, -16.32f,  -20.07f, -19.10f},
    {0.0f, -19.67f,  -21.46f, -21.17f},
    {0.0f, -23.48f,  -22.23f, -24.55f},
    {0.0f, -27.90f,  -24.22f, -26.54f},
    {0.0f, -32.19f,  -25.30f, -28.34f},
    {0.0f, -36.51f,  -27.06f, -31.38f},
    {0.0f, -40.49f,  -28.61f, -33.06f},
    {0.0f, -44.34f,  -30.04f, -33.61f},
    {0.0f, -47.77f,  -31.30f, -34.76f},
    {0.0f, -51.14f,  -32.80f, -36.35f},
    {0.0f, -54.01f,  -34.37f, -38.14f},
    {0.0f, -56.98f,  -36.12f, -39.74f},
    {0.0f, -59.89f,  -38.06f, -40.84f},
    {0.0f, -62.59f,  -39.88f, -41.87f},
    {0.0f, -65.25f,  -41.80f, -42.74f},
    {0.0f, -67.16f,  -44.06f, -43.08f},
    {0.0f, -68.88f,  -46.02f, -44.22f},
    {0.0f, -70.38f,  -48.08f, -44.22f},
    {0.0f, -71.76f,  -50.06f, -45.28f},
    {0.0f, -73.20f,  -52.58f, -46.08f},
    {0.0f, -74.54f,  -54.96f, -46.40f},
    {0.0f, -75.85f,  -57.76f, -46.88f},
    {0.0f, -76.69f,  -59.75f, -47.93f},
    {0.0f, -76.72f,  -61.47f, -47.71f},
    {0.0f, -76.59f,  -62.68f, -47.79f},
    {0.0f, -76.59f,  -63.49f, -48.11f},
    {0.0f, -76.86f,  -64.30f, -49.98f},
    {0.0f, -76.88f,  -64.93f, -51.14f},
    {0.0f, -76.46f,  -65.66f, -54.19f},
};

arm_pose_t one_to_up[] = {
    {0.0f, -58.30f, -42.87f,  37.80f},
    {0.0f, -57.55f, -44.15f,  34.59f},
    {0.0f, -53.20f, -46.15f,  30.91f},
    {0.0f, -49.75f, -48.15f,  27.23f},
    {0.0f, -45.49f, -48.12f,  25.54f},
    {0.0f, -41.05f, -47.66f,  24.86f},
    {0.0f, -37.33f, -47.66f,  24.17f},
    {0.0f, -33.77f, -47.33f,  23.49f},
    {0.0f, -29.49f, -46.96f,  22.81f},
    {0.0f, -23.70f, -45.16f,  22.12f},
    {0.0f, -19.46f, -44.80f,  21.44f},
    {0.0f, -15.37f, -44.80f,  20.75f},
    {0.0f, -11.04f, -44.80f,  20.07f},
    {0.0f,  -7.15f, -44.80f,  19.39f},
    {0.0f,  -3.90f, -44.91f,  18.70f},
    {0.0f,   0.56f, -46.96f,  18.02f},
    {0.0f,   4.98f, -51.97f,  17.33f},
    {0.0f,   8.63f, -55.18f,  16.65f},
    {0.0f,  11.26f, -58.30f,  15.97f},
    {0.0f,  14.50f, -61.55f,  15.28f},
    {0.0f,  18.39f, -63.53f,  14.60f},
    {0.0f,  20.87f, -65.29f,  13.91f},
    {0.0f,  22.34f, -65.91f,  13.23f},
    {0.0f,  25.13f, -66.37f,  12.55f},
    {0.0f,  26.52f, -66.34f,  11.86f},
    {0.0f,  27.05f, -66.50f,  11.18f},
    {0.0f,  27.40f, -66.48f,  10.49f},
    {0.0f,  27.57f, -66.45f,   9.81f},
    {0.0f,  27.68f, -66.45f,   9.13f},
    {0.0f,  27.77f, -66.45f,   8.44f},
    {0.0f,  27.84f, -66.45f,   7.76f},
    {0.0f,  28.14f, -66.45f,   7.07f},
    {0.0f,  28.45f, -66.48f,   6.39f},
    {0.0f,  29.05f, -66.48f,   5.71f},
    {0.0f,  29.24f, -66.45f,   5.02f},
    {0.0f,  29.31f, -66.52f,   4.34f},
    {0.0f,  29.62f, -67.97f,   3.65f},
    {0.0f,  29.90f, -68.08f,   2.97f},
    {0.0f,  29.79f, -68.06f,   2.29f},
    {0.0f,  29.81f, -68.06f,   1.60f},
    {0.0f,  29.71f, -68.06f,   0.92f},
    {0.0f,  29.31f, -68.04f,   0.23f},
    {0.0f,  29.22f, -68.06f,  -0.45f},
    {0.0f,  29.20f, -68.06f,  -1.14f},
    {0.0f,  29.22f, -70.06f,  -1.82f},
    {0.0f,  28.30f, -73.14f,  -2.48f},
};

arm_pose_t two_to_up[] = {
    {0.0f, -76.46f, -65.66f, -54.19f},
    {0.0f, -76.46f, -63.66f, -53.18f},
    {0.0f, -76.46f, -62.40f, -52.16f},
    {0.0f, -76.08f, -60.36f, -51.15f},
    {0.0f, -72.15f, -59.38f, -50.13f},
    {0.0f, -66.92f, -59.27f, -49.12f},
    {0.0f, -62.34f, -59.25f, -48.11f},
    {0.0f, -57.22f, -59.25f, -47.09f},
    {0.0f, -51.86f, -59.27f, -46.08f},
    {0.0f, -46.10f, -59.27f, -45.06f},
    {0.0f, -40.01f, -59.27f, -44.05f},
    {0.0f, -34.34f, -59.27f, -43.04f},
    {0.0f, -27.81f, -59.09f, -42.02f},
    {0.0f, -24.06f, -59.33f, -41.01f},
    {0.0f, -18.94f, -59.71f, -39.99f},
    {0.0f, -10.47f, -59.88f, -38.98f},
    {0.0f,  -4.52f, -59.88f, -37.97f},
    {0.0f,  -1.40f, -59.90f, -36.95f},
    {0.0f,   3.07f, -60.01f, -35.94f},
    {0.0f,  10.98f, -63.09f, -34.92f},
    {0.0f,  15.86f, -65.58f, -33.91f},
    {0.0f,  17.66f, -67.96f, -32.89f},
    {0.0f,  20.54f, -69.25f, -31.88f},
    {0.0f,  24.39f, -71.22f, -30.87f},
    {0.0f,  25.35f, -73.22f, -29.85f},
    {0.0f,  25.46f, -73.22f, -28.84f},
    {0.0f,  26.12f, -73.25f, -27.82f},
    {0.0f,  26.83f, -73.22f, -26.81f},
    {0.0f,  27.02f, -73.18f, -25.80f},
    {0.0f,  27.00f, -73.14f, -24.78f},
    {0.0f,  27.05f, -73.18f, -23.77f},
    {0.0f,  27.05f, -73.14f, -22.75f},
    {0.0f,  27.20f, -73.16f, -21.74f},
    {0.0f,  27.42f, -73.16f, -20.72f},
    {0.0f,  27.46f, -73.18f, -19.71f},
    {0.0f,  27.44f, -73.14f, -18.70f},
    {0.0f,  27.55f, -73.16f, -17.68f},
    {0.0f,  27.55f, -73.16f, -16.67f},
    {0.0f,  27.68f, -73.18f, -15.65f},
    {0.0f,  27.81f, -73.18f, -14.64f},
    {0.0f,  28.14f, -73.20f, -13.63f},
    {0.0f,  28.32f, -73.18f, -12.61f},
    {0.0f,  28.30f, -73.16f, -11.60f},
    {0.0f,  28.30f, -73.18f, -10.58f},
    {0.0f,  28.32f, -73.18f,  -9.57f},
    {0.0f,  28.32f, -73.18f,  -8.55f},
    {0.0f,  28.30f, -73.16f,  -7.54f},
    {0.0f,  28.30f, -73.18f,  -6.53f},
    {0.0f,  28.30f, -73.16f,  -5.51f},
    {0.0f,  28.28f, -73.16f,  -4.50f},
    {0.0f,  28.30f, -73.14f,  -3.48f},
    {0.0f,  28.30f, -73.14f,  -2.48f},
};

arm_pose_t up_to_put[] = {
    {0.0f,  28.30f, -73.14f,  -2.48f},
    {0.0f,  26.31f, -73.97f,  -2.50f},
    {0.0f,  24.32f, -74.80f,  -2.52f},
    {0.0f,  22.33f, -75.62f,  -2.55f},
    {0.0f,  20.34f, -76.45f,  -2.57f},
    {0.0f,  18.35f, -77.28f,  -2.59f},
    {0.0f,  16.36f, -78.11f,  -2.61f},
    {0.0f,  14.37f, -78.93f,  -2.64f},
    {0.0f,  12.38f, -79.76f,  -2.66f},
    {0.0f,  10.39f, -80.59f,  -2.68f},
    {0.0f,   8.40f, -81.42f,  -2.70f},
    {0.0f,   6.41f, -82.24f,  -2.73f},
    {0.0f,   4.42f, -83.07f,  -2.75f},
    {0.0f,   2.43f, -83.90f,  -2.77f},
    {0.0f,   0.44f, -84.73f,  -2.79f},
    {0.0f,  -1.55f, -85.55f,  -2.82f},
    {0.0f,  -3.54f, -86.38f,  -2.84f},
    {0.0f,  -5.53f, -87.21f,  -2.86f},
    {0.0f,  -7.52f, -88.04f,  -2.88f},
    {0.0f,  -9.51f, -88.86f,  -2.91f},
    {0.0f, -11.50f, -89.69f,  -2.93f},
    {0.0f, -13.49f, -90.52f,  -2.95f},
    {0.0f, -15.48f, -91.35f,  -2.97f},
    {0.0f, -17.47f, -92.18f,  -3.00f},
    {0.0f, -19.46f, -93.01f,  -3.01f},
};

// ============== 运动执行 ================
#define POSE_INTERVAL_MS  100  // 每个插值点的间隔时间(ms)

void execute_pose_sequence(const arm_pose_t *poses, size_t count)
{
    for (size_t i = 0; i < count; i++) {
        // motor_set_mit(joint1_motor, 0.0f, poses[i].joint1, 0.0f);
        motor_set_mit(joint2_motor, 0.0f, poses[i].joint2, 0.0f);
        motor_set_mit(joint3_motor, 0.0f, poses[i].joint3, 0.0f);
        motor_set_mit(joint4_motor, 0.0f, poses[i].joint4, 0.0f);
        k_msleep(POSE_INTERVAL_MS);
    }
}

int main(void)
{

    k_msleep(100);

    motor_control(joint1_motor, ENABLE_MOTOR);k_msleep(100);motor_control(joint1_motor, ENABLE_MOTOR);k_msleep(100);motor_set_mode(joint1_motor, MIT);k_msleep(100);motor_control(joint1_motor, SET_ZERO);
	k_msleep(100);

    motor_control(joint2_motor, ENABLE_MOTOR);k_msleep(100);motor_control(joint2_motor, ENABLE_MOTOR);k_msleep(100);motor_set_mode(joint2_motor, MIT);k_msleep(100);motor_control(joint2_motor, SET_ZERO);
	k_msleep(100);

    motor_control(joint3_motor, ENABLE_MOTOR);k_msleep(100);motor_control(joint3_motor, ENABLE_MOTOR);k_msleep(100);motor_set_mode(joint3_motor, MIT);k_msleep(100);motor_control(joint3_motor, SET_ZERO);
    k_msleep(100);

    motor_control(joint4_motor, ENABLE_MOTOR);k_msleep(100);motor_set_mode(joint4_motor, MIT);k_msleep(100);
    
    gpio_pin_configure_dt(&emvalve1, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&emvalve2, GPIO_OUTPUT_INACTIVE);
    

    motor_set_mit(joint1_motor, 0.0f, 0.0f, 0.0f);

    k_msleep(1000);
	


    // if (1) {
    //     while (!device_is_ready(joint1_motor) || !device_is_ready(joint2_motor) ||
    //            !device_is_ready(joint3_motor) || !device_is_ready(joint4_motor))
    //     {
    //         LOG_ERR("Motor not ready, waiting...");
    //         k_msleep(100);
    //         break;
    //     }
    //         LOG_INF("All motors ready!");
    // } else {
    //     LOG_INF("RUNNING IN SIMULATION MODE (No Motors Required)");
    // }

    // 执行位姿序列
    execute_pose_sequence(init_to_1, ARRAY_SIZE(init_to_1));
    // k_msleep(500);
    // execute_pose_sequence(two_to_up, ARRAY_SIZE(two_to_up));
    // k_msleep(500);
    // execute_pose_sequence(up_to_put, ARRAY_SIZE(up_to_put));    

	while (1) {
		k_msleep(200);
		// LOG_WRN("Target Pos [deg] -> J1:%.2f, J2:%.2f, J3:%.2f, J4:%.2f",
		// 	(double)joint1_pos, (double)joint2_pos, (double)joint3_pos, (double)joint4_pos);
        // LOG_WRN("Current Pos [deg] -> J1:%.2f, J2:%.2f, J3:%.2f, J4:%.2f",
        //     (double)j1_fb_deg, (double)j2_fb_deg, (double)j3_fb_deg, (double)j4_fb_deg);
        // LOG_INF("Motor Target [deg] -> M1:%.2f, M2:%.2f, M3:%.2f, M4:%.2f",
        //     (double)motor1_ang, (double)motor2_ang, (double)motor3_ang, (double)motor4_ang);
		LOG_INF("Motor Feedback [deg] -> , M2:%.2f, M3:%.2f, M4:%.2f",
			 (double)m2_fb, (double)m3_fb, (double)m4_fb);
	}

	return 0;
}