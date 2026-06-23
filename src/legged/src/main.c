#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/motor.h>
#include <zephyr/drivers/sbus.h>
#include <ares/board/init.h>
#include <ares/ekf/imu_task.h>
#include <ares/ares_comm.h>
#include <ares/protocol/dual/dual_protocol.h>
#include "ares/ekf/QuaternionEKF.h"
#include <zephyr/debug/thread_analyzer.h>
#include <ares/interface/usb/usb_bulk.h>
#include <zephyr/drivers/gpio.h>
#include "main.h"
#include "ares/protocol/dual/dual_protocol.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

#ifndef PI
#define PI 3.14159265358979323846f
#endif

#define STACK_SIZE 4096

#define RPM2RADPS(rpm) ((rpm) * (PI / 30.0f))
#define RADPS2RPM(radps) ((radps) * (30.0f / PI))
#define DEG2RAD(deg) ((deg) * (PI / 180.0f))
#define RAD2DEG(rad) ((rad) * (180.0f / PI))
/**
 * @brief 计算重力向量在地面坐标系中的单位化投影
 * @param q 四元数 [q0, q1, q2, q3] (归一化)
 * @param body_accel 载体坐标系下的加速度 [ax, ay, az]（未使用，保留接口兼容性）
 * @param g 重力加速度大小（未使用，保留接口兼容性）
 * @param gravity_projection 输出：重力向量在地面坐标系的单位化投影 [gx, gy, gz]
 */
void calculate_gravity_projection(float *q, float *body_accel, float g, float *gravity_projection)
{
    // 四元数转旋转矩阵公式
    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
    
    // 确保四元数是归一化的
    float qnorm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if (qnorm > 0.0001f) {
        q0 /= qnorm; q1 /= qnorm; q2 /= qnorm; q3 /= qnorm;
    }
    
    // 载体坐标系各轴在重力方向上的投影
    // 即载体坐标系XYZ轴与重力向量（垂直向下）的夹角余弦值
    // 这等价于将载体坐标系的单位向量投影到重力方向上
    
    // 载体坐标系X轴在重力方向的投影 = 载体X轴与重力向量的点积
    // 载体X轴: [1,0,0] 变换到地面坐标系，然后与重力向量[0,0,-1]点积
    // 结果就是旋转矩阵第一行第三列的负值
    gravity_projection[0] = -(2 * (q1 * q3 + q0 * q2));  // 载体X轴在重力方向的投影
    
    // 载体坐标系Y轴在重力方向的投影
    gravity_projection[1] = -(2 * (q2 * q3 - q0 * q1));  // 载体Y轴在重力方向的投影
    
    // 载体坐标系Z轴在重力方向的投影  
    gravity_projection[2] = -(1 - 2 * (q1 * q1 + q2 * q2));  // 载体Z轴在重力方向的投影
    
    // 这些值的范围是[-1, 1]，表示各轴与重力方向的夹角余弦值
    // 当载体某轴与重力方向平行时，对应投影为±1
    // 当载体某轴与重力方向垂直时，对应投影为0
}

/**
 * @brief 计算四元数对应的欧拉角
 * @param q 四元数 [q0, q1, q2, q3]
 * @param euler 输出欧拉角 [roll, pitch, yaw] (弧度)
 */
void quaternion_to_euler(float *q, float *euler)
{
    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
    
    // Roll (绕X轴旋转)
    euler[0] = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));
    
    // Pitch (绕Y轴旋转)
    float sin_pitch = 2.0f * (q0 * q2 - q3 * q1);
    if (fabsf(sin_pitch) >= 1.0f) {
        euler[1] = copysignf(PI / 2.0f, sin_pitch); // 万向锁情况
    } else {
        euler[1] = asinf(sin_pitch);
    }
    
    // Yaw (绕Z轴旋转)
    euler[2] = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));
}
float kp_cmd[9] = {0};
float kd_cmd[9] = {0};
float action[9]= {0};
float torque_cmd[9] = {0};
float motor_angle[9] = {0};
float motor_speed[9] = {0};
float motor_torque[9] = {0};
float imu_pack[10] = {0};
float current_accel[3] = {0};  // 存储当前加速度数据
float sbus_data[11] = {0};
uint8_t actionbuf[36]= {0};
uint8_t torque_cmdbuf[36]= {0};
uint8_t anglebuf[36]= {0};
uint8_t speedbuf[36]= {0};
uint8_t torquebuf[36]= {0};
uint8_t imudatabuf[40]= {0};
uint8_t sbus_buf[44]= {0};
uint8_t kp_cmdbuf[36]= {0};
uint8_t kd_cmdbuf[36]= {0};
sync_table_t *angle_tx;
sync_table_t *speed_tx;
sync_table_t *torque_tx;
sync_table_t *imu_tx;
sync_table_t *sbus_tx;

struct pid_config new_config={.k_p = 25.0f, .k_d = 0.5f};
static int pub_cnt = 0;  // 添加发布计数器

DUAL_PROPOSE_PROTOCOL_DEFINE(dual_protocol);
ARES_BULK_INTERFACE_DEFINE(usb_bulk_interface);

// leg_motors leg1 = { motor1, motor2, LEFT_BACK};
// leg_motors leg2 = { motor3, motor4, LEFT_FRONT};
// leg_motors leg3 = { motor6, motor5, RIGHT_BACK};
// leg_motors leg4 = { motor8, motor7, RIGHT_FRONT};

const struct device *motors[9]={
    DEVICE_DT_GET(DT_NODELABEL(motor3)), DEVICE_DT_GET(DT_NODELABEL(motor2)), DEVICE_DT_GET(DT_NODELABEL(motor6)), DEVICE_DT_GET(DT_NODELABEL(motor7)), DEVICE_DT_GET(DT_NODELABEL(motor8)), DEVICE_DT_GET(DT_NODELABEL(motor1)), DEVICE_DT_GET(DT_NODELABEL(motor0)), DEVICE_DT_GET(DT_NODELABEL(motor4)), DEVICE_DT_GET(DT_NODELABEL(motor5))
};

const struct device *mit_pid[9] ={
    DEVICE_DT_GET(DT_NODELABEL(mit_pid_3)),
    DEVICE_DT_GET(DT_NODELABEL(mit_pid_2)),
    DEVICE_DT_GET(DT_NODELABEL(mit_pid_6)),
    DEVICE_DT_GET(DT_NODELABEL(mit_pid_7)),
    DEVICE_DT_GET(DT_NODELABEL(mit_pid_8)),
    DEVICE_DT_GET(DT_NODELABEL(mit_pid_1)),
    DEVICE_DT_GET(DT_NODELABEL(mit_pid_0)),
    DEVICE_DT_GET(DT_NODELABEL(mit_pid_4)),
    DEVICE_DT_GET(DT_NODELABEL(mit_pid_5)),
};

void stand(){
    motor_control(motor9, SET_ZERO);
    motor_set_mit(motor9, 0.0f, 0.0f, 0.0f);
    
    /* 渐进式移动到初始位置 */
    for(int i = 0; i < 10; i++) {
        float progress = (i + 1) * 0.1f;
        motor_set_mit(motor1, 0.0f, progress * INITIAL_ANGLE1, 0.0f);
        motor_set_mit(motor2, 0.0f, progress * INITIAL_ANGLE2, 0.0f);
        motor_set_mit(motor3, 0.0f, progress * INITIAL_ANGLE3, 0.0f);
        motor_set_mit(motor4, 0.0f, progress * INITIAL_ANGLE4, 0.0f);
        motor_set_mit(motor5, 0.0f, progress * INITIAL_ANGLE5, 0.0f);
        motor_set_mit(motor6, 0.0f, progress * INITIAL_ANGLE6, 0.0f);
        motor_set_mit(motor7, 0.0f, progress * INITIAL_ANGLE7, 0.0f);
        motor_set_mit(motor8, 0.0f, progress * INITIAL_ANGLE8, 0.0f);
        k_msleep(50);  // 增加延时确保平滑运动
    }
    
    k_sleep(K_MSEC(500));
    
    /* 重新设置零点并释放力矩 */
    for(int i = 0; i < 9; i++) {
        motor_control(motors[i], SET_ZERO);
        motor_set_mit(motors[i], 0.0f, 0.0f, 0.0f);
        k_sleep(K_MSEC(10));
    }
}

// void Sensor_update_cb(QEKF_INS_t *QEKF)
// {
//     pub_cnt++;
    
//     // 存储四元数
//     imu_pack[0] = QEKF->q[0];
//     imu_pack[1] = QEKF->q[1];
//     imu_pack[2] = QEKF->q[2];
//     imu_pack[3] = QEKF->q[3];

//     // 存储角速度  
//     imu_pack[4] = QEKF->Gyro[0];
//     imu_pack[5] = QEKF->Gyro[1];
//     imu_pack[6] = QEKF->Gyro[2];
//     // if(pub_cnt % 3000 == 0) {
//     //     LOG_ERR(" [%f, %f, %f, %f]", 
//     //         QEKF->q[0], QEKF->q[1], QEKF->q[2], QEKF->q[3]);
//     // }
    
//     // 存储当前加速度数据
//     current_accel[0] = QEKF->Accel[0];
//     current_accel[1] = QEKF->Accel[1]; 
//     current_accel[2] = QEKF->Accel[2];
    
//     k_work_submit_to_queue(&work_queue, &imu_tx_data_handle);
// }
int action_rx_cb(int status)
{
	// LOG_ERR("action_rx_cb called with status: %d", status);
	if(status == SYNC_PACK_STATUS_DONE) {
		k_work_submit_to_queue(&work_queue, &action_rx_data_handle);
	}else{
		return;
	}
	return 0;
}
int torque_cmd_rx_cb(int status)
{
    // LOG_ERR("torque_cmd_rx_cb called with status: %d", status);
	if(status == SYNC_PACK_STATUS_DONE) {
		k_work_submit_to_queue(&work_queue, &torque_cmd_rx_data_handle);
	}else{
		return 0;
	}
	return 0;
}

int kp_cmd_rx_cb(int status)
{
    // LOG_ERR("kp_cmd_rx_cb called with status: %d", status);
    if(status == SYNC_PACK_STATUS_DONE) {
     
        // LOG_ERR("kp_cmd: %f, %f, %f, %f, %f, %f, %f, %f, %f", 
        //         kp_cmd[0], kp_cmd[1], kp_cmd[2], kp_cmd[3], 
        //         kp_cmd[4], kp_cmd[5], kp_cmd[6], kp_cmd[7], kp_cmd[8]);
        memcpy(&kp_cmd, &kp_cmdbuf, 36);
        for(int i = 0; i < 9; i++) {
            new_config.k_p = kp_cmd[i];
        new_config.k_d = kd_cmd[i];
        pid_set_params(mit_pid[i], &new_config);
        }
    }else{
        return 0;
    }
    return 0;
}
int kd_cmd_rx_cb(int status)
{
    // LOG_ERR("kd_cmd_rx_cb called with status: %d", status);
    if(status == SYNC_PACK_STATUS_DONE) {
        // LOG_ERR("kd_cmd: %f, %f, %f, %f, %f, %f, %f, %f, %f", 
        //         kd_cmd[0], kd_cmd[1], kd_cmd[2], kd_cmd[3], 
        //         kd_cmd[4], kd_cmd[5], kd_cmd[6], kd_cmd[7], kd_cmd[8]);
        memcpy(&kd_cmd, &kd_cmdbuf, 36);
        for(int i = 0; i < 9; i++) {
            new_config.k_p = kp_cmd[i];
        new_config.k_d = kd_cmd[i];
        pid_set_params(mit_pid[i], &new_config);
        }
    }else{
        return 0;
    }
    return 0;
}
void imu_tx_data_handler(struct k_work *work)
{   
    // 计算重力向量在地面坐标系中的单位化投影
    static float gravity_projection[3] = {0};

    // 直接使用四元数计算重力投影（不需要加速度数据）
    calculate_gravity_projection(imu_pack, current_accel, 0, gravity_projection);
    
    // 上传重力向量在地面坐标系的单位化投影
    imu_pack[7] = gravity_projection[0];
    imu_pack[8] = gravity_projection[1];
    imu_pack[9] = gravity_projection[2];
    
    
    
    memcpy(imudatabuf, imu_pack, sizeof(imu_pack));
    dual_sync_flush(&dual_protocol, imu_tx);
}

void tx_isr_handler(struct k_timer *dummy)
{
	k_work_submit_to_queue(&work_queue, &tx_data_handle);
}
uint16_t log_cnt = 0;
void tx_data_handler(struct k_work *work)
{
    for(int i = 0; i < 9; i++) {
        motor_angle[i] = DEG2RAD(motor_get_angle(motors[i]));
    }
    memcpy(anglebuf, motor_angle, sizeof(motor_angle));
    dual_sync_flush(&dual_protocol, angle_tx);
    k_sleep(K_USEC(500));
    for(int i = 0; i < 9; i++) {
        motor_speed[i] = RPM2RADPS(motor_get_speed(motors[i]));
    }
    memcpy(speedbuf, motor_speed, sizeof(motor_speed));
    dual_sync_flush(&dual_protocol, speed_tx);
    k_sleep(K_USEC(500));
    for(int i = 0; i < 9; i++) {
        motor_torque[i] = (motor_get_torque(motors[i]));
    }
    memcpy(torquebuf, motor_torque, sizeof(motor_torque));
    dual_sync_flush(&dual_protocol, torque_tx);
    k_sleep(K_USEC(500));
    for(int i = 0; i < 11; i++) {
        sbus_data[i] = sbus_get_percent(sbus,i);
    }
    // if(log_cnt++ % 100 == 0) {
       
    // LOG_ERR("sbus_data: %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f", 
    //         sbus_data[0], sbus_data[1], sbus_data[2], sbus_data[3], sbus_data[4], sbus_data[5], sbus_data[6], sbus_data[7], sbus_data[8], sbus_data[9], sbus_data[10]);}
    memcpy(sbus_buf, sbus_data, sizeof(sbus_data));
    dual_sync_flush(&dual_protocol, sbus_tx);
}
void action_rx_data_handler(struct k_work *work)
{
    memcpy(&action, &actionbuf, 36);
   
    for(int i = 0; i < 9; i++) {
        // float deg_value = fmaxf(fminf(RAD2DEG(action[i]), 70.0f), -70.0f);
        // if(i==4){
        //     deg_value = fmaxf(fminf(deg_value, 30.0f), -30.0f);
        // }

        motor_set_mit(motors[i], 0.0f, RAD2DEG(action[i]), torque_cmd[i]);

    }
}

void torque_cmd_rx_data_handler(struct k_work *work)
{
    memcpy(&torque_cmd, &torque_cmdbuf, 36);
    
    for(int i = 0; i < 9; i++) {
        // float deg_value = fmaxf(fminf(RAD2DEG(action[i]), 70.0f), -70.0f);
        // if(i==4){
        //     deg_value = fmaxf(fminf(deg_value, 30.0f), -30.0f);
        // }

        motor_set_mit(motors[i], 0.0f, RAD2DEG(action[i]), torque_cmd[i]);

    }

    
}



/* USB COMMUNICATION CONFIG END */

int main()
{
    /* USB COMMUNICATION INITIATION START */
    ares_bind_interface(&usb_bulk_interface, &dual_protocol);
    /*
    * 0x0201 : MCU -> Host  (MotorState-A)
    * 0x0101 : Host -> MCU  (MotorCmd)
    */
    // IMU_Sensor_trig_init(accel_dev, gyro_dev);
    // IMU_Sensor_set_update_cb(Sensor_update_cb);
    angle_tx = dual_sync_add(&dual_protocol, 0x0201, anglebuf, 36, NULL);
    speed_tx = dual_sync_add(&dual_protocol, 0x0202, speedbuf, 36, NULL);
    torque_tx = dual_sync_add(&dual_protocol, 0x0203, torquebuf, 36, NULL);
    imu_tx = dual_sync_add(&dual_protocol, 0x0301, imu_pack, 40, NULL);
    sbus_tx = dual_sync_add(&dual_protocol, 0x0400, sbus_buf, 44, NULL);
     for(int i = 0; i < 9; i++) {
        motor_control(motors[i], SET_ZERO);
        k_sleep(K_MSEC(1));
    }
    /* 设置所有电机为MIT模式 */
    for(int i = 0; i < 9; i++) {
        motor_set_mode(motors[i], MIT);
        k_sleep(K_MSEC(1));
    }
    k_sleep(K_MSEC(100));  // 等待电机模式设置完成
    for(int i = 0; i < 9; i++) {
        motor_control(motors[i], ENABLE_MOTOR);
        k_sleep(K_MSEC(1));
    }
    k_sleep(K_MSEC(100));
    LOG_INF("Executing stand sequence...");
    stand();
    LOG_INF("Stand sequence completed.");
    /* 初始化工作队列 */
    k_work_queue_init(&work_queue);
    k_work_queue_start(&work_queue, work_queue_stack, STACK_SIZE, -1, NULL);
    dual_sync_add(&dual_protocol, 0x0101, &actionbuf, 36, (dual_trans_cb_t)action_rx_cb);
    dual_sync_add(&dual_protocol, 0x0102, &torque_cmdbuf, 36, (dual_trans_cb_t)torque_cmd_rx_cb);
    dual_sync_add(&dual_protocol, 0x0100, &kp_cmdbuf, 36, (dual_trans_cb_t)kp_cmd_rx_cb);
    dual_sync_add(&dual_protocol, 0x0200, &kd_cmdbuf, 36, (dual_trans_cb_t)kd_cmd_rx_cb);
    /* 初始化定时器 */
    tx_timer.expiry_fn = tx_isr_handler;

    /* 启动数据传输定时器 */
    k_timer_start(&tx_timer, K_NO_WAIT, K_MSEC(10));
    while (1) {  
        LOG_INF("Main loop iteration - pub_cnt: %d", pub_cnt);
        k_msleep(2000);  // 减少日志输出频率

       
    }
}
