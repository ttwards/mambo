#include <zephyr/device.h>
#include <zephyr/drivers/motor.h>
#define ANGLE_OUTER 30
#define ANGLE_INNER 60

#define STACK_SIZE 4096

#define INITIAL_ANGLE1 ANGLE_OUTER 
#define INITIAL_ANGLE2 -ANGLE_INNER
#define INITIAL_ANGLE3 -ANGLE_OUTER
#define INITIAL_ANGLE4 ANGLE_INNER
#define INITIAL_ANGLE5 ANGLE_INNER
#define INITIAL_ANGLE6 -ANGLE_OUTER
#define INITIAL_ANGLE7 -ANGLE_INNER
#define INITIAL_ANGLE8 ANGLE_OUTER

#define MOTOR1_NODE DT_INST(0, mi_motor)
#define MOTOR2_NODE DT_INST(1, mi_motor)
#define MOTOR3_NODE DT_INST(2, mi_motor)
#define MOTOR4_NODE DT_INST(3, mi_motor)
#define MOTOR5_NODE DT_INST(4, mi_motor)
#define MOTOR6_NODE DT_INST(5, mi_motor)
#define MOTOR7_NODE DT_INST(6, mi_motor)
#define MOTOR8_NODE DT_INST(7, mi_motor)
#define MOTOR9_NODE DT_INST(8, mi_motor)

const struct device *motor1 = DEVICE_DT_GET(MOTOR1_NODE);//left back external motor
const struct device *motor2 = DEVICE_DT_GET(MOTOR2_NODE);//left back internal motor
const struct device *motor3 = DEVICE_DT_GET(MOTOR3_NODE);//left front external motor
const struct device *motor4 = DEVICE_DT_GET(MOTOR4_NODE);//left front internal motor
const struct device *motor5 = DEVICE_DT_GET(MOTOR5_NODE);//right back internal motor
const struct device *motor6 = DEVICE_DT_GET(MOTOR6_NODE);//right back external motor
const struct device *motor7 = DEVICE_DT_GET(MOTOR7_NODE);//right front internal motor
const struct device *motor8 = DEVICE_DT_GET(MOTOR8_NODE);//right front external motor
const struct device *motor9 = DEVICE_DT_GET(MOTOR9_NODE);//waist motor

// #define ACCEL_NODE DT_NODELABEL(bmi08x_accel)
// extern const struct device *accel_dev;

// #define GYRO_NODE DT_NODELABEL(bmi08x_gyro)
// extern const struct device *gyro_dev;

// const struct device *accel_dev = DEVICE_DT_GET(ACCEL_NODE);
// const struct device *gyro_dev = DEVICE_DT_GET(GYRO_NODE);

#define SBUS_NODE DT_NODELABEL(sbus0)
const struct device *sbus = DEVICE_DT_GET(SBUS_NODE);

void imu_tx_data_handler(struct k_work *work);

void tx_isr_handler(struct k_timer *dummy);
void tx_data_handler(struct k_work *work);
void action_rx_data_handler(struct k_work *work);
void torque_cmd_rx_data_handler(struct k_work *work);
struct k_work_q work_queue;

K_THREAD_STACK_DEFINE(work_queue_stack, STACK_SIZE);

K_WORK_DEFINE(imu_tx_data_handle, imu_tx_data_handler);

K_WORK_DEFINE(tx_data_handle, tx_data_handler);

K_WORK_DEFINE(action_rx_data_handle, action_rx_data_handler);

K_WORK_DEFINE(torque_cmd_rx_data_handle, torque_cmd_rx_data_handler);

K_TIMER_DEFINE(tx_timer, tx_isr_handler, NULL);