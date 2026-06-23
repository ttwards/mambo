
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

/* Devicetree */
// #define ACCEL_NODE DT_NODELABEL(bmi08x_accel)
// const struct device *accel_dev = DEVICE_DT_GET(ACCEL_NODE);

// #define GYRO_NODE DT_NODELABEL(bmi08x_gyro)
// const struct device *gyro_dev = DEVICE_DT_GET(GYRO_NODE);



#define WHEELMOTOR1_NODE DT_NODELABEL(motor_wheel0)
#define WHEELMOTOR2_NODE DT_NODELABEL(motor_wheel1)
#define WHEELMOTOR3_NODE DT_NODELABEL(motor_wheel2)
#define WHEELMOTOR4_NODE DT_NODELABEL(motor_wheel3)

#define STEERMOTOR1_NODE DT_NODELABEL(motor_steer0)
#define STEERMOTOR2_NODE DT_NODELABEL(motor_steer1)
#define STEERMOTOR3_NODE DT_NODELABEL(motor_steer2)
#define STEERMOTOR4_NODE DT_NODELABEL(motor_steer3)

#define DMOTOR_NODE1 DT_NODELABEL(dm_motor0)
#define DMOTOR_NODE2 DT_NODELABEL(dm_motor1)
#define DMOTOR_NODE3 DT_NODELABEL(dm_motor2)
#define DMOTOR_NODE4 DT_NODELABEL(dm_motor3)

#define CHASSIS_NODE DT_NODELABEL(chassis)

const struct device *wheel_motor1 = DEVICE_DT_GET(WHEELMOTOR1_NODE);
const struct device *wheel_motor2 = DEVICE_DT_GET(WHEELMOTOR2_NODE);
const struct device *wheel_motor3 = DEVICE_DT_GET(WHEELMOTOR3_NODE);
const struct device *wheel_motor4 = DEVICE_DT_GET(WHEELMOTOR4_NODE);

const struct device *steer_motor1 = DEVICE_DT_GET(STEERMOTOR1_NODE);
const struct device *steer_motor2 = DEVICE_DT_GET(STEERMOTOR2_NODE);
const struct device *steer_motor3 = DEVICE_DT_GET(STEERMOTOR3_NODE);
const struct device *steer_motor4 = DEVICE_DT_GET(STEERMOTOR4_NODE);

const struct device *dm_motor1 = DEVICE_DT_GET(DMOTOR_NODE1);
const struct device *dm_motor2 = DEVICE_DT_GET(DMOTOR_NODE2);
const struct device *dm_motor3 = DEVICE_DT_GET(DMOTOR_NODE3);
const struct device *dm_motor4 = DEVICE_DT_GET(DMOTOR_NODE4);

const struct device *chassis = DEVICE_DT_GET(CHASSIS_NODE);

#define SBUS_NODE DT_NODELABEL(sbus0)
const struct device *sbus = DEVICE_DT_GET(SBUS_NODE);

