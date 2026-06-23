/*大臂平着0,与标签相同，小臂*/
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/motor.h>
#include <zephyr/logging/log.h>
#include <math.h>
#include <zephyr/drivers/sbus.h>
#include "zephyr/drivers/gpio.h"
#include <zephyr/drivers/uart.h>


#include <stdio.h>
#include <math.h>
#include <stdbool.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ==========================================
// ⚙️ 机械臂物理参数配置
// ==========================================
#define L1 440.0         // 大臂长度
#define L2 460.0         // 小臂长度
#define L_CUP 0.0       // 吸嘴尖端到腕部轴心的直线距离

// ==========================================
// 📍 点位数据表 (Hardcoded 宏定义)
// ==========================================
#define P0_X    300.0f
#define P0_Y   100.0f

// 目标 1：正前朝前吸
#define T1_P1_X   400.0f
#define T1_P1_Y   50.0f
#define T1_P2_X   757.32f
#define T1_P2_Y  -107.3f
#define T1_P3_X   837.32f
#define T1_P3_Y  -107.3f

// 目标 2：正前朝下吸
#define T2_P1_X   400.0f
#define T2_P1_Y   50.0f
#define T2_P2_X   700.64f
#define T2_P2_Y     7.7f
#define T2_P3_X   893.64f
#define T2_P3_Y   -72.3f

// 目标 3：背后朝前吸
#define T3_P1_X  -550.0f
#define T3_P1_Y   300.0f
#define T3_P2_X  -600.19f
#define T3_P2_Y   279.7f
#define T3_P3_X  -659.19f
#define T3_P3_Y   279.7f

//目标 4：放置


#define HIGH_BYTE(x)           ((x) >> 8)
#define LOW_BYTE(x)            ((x) & 0xFF)
#define COMBINE_HL8(HIGH, LOW) ((HIGH << 8) + LOW)

// 电机设备定义
#define MOTOR1_NODE DT_NODELABEL(rs_motor1)
#define MOTOR2_NODE DT_NODELABEL(rs_motor2)
#define MOTOR3_NODE DT_NODELABEL(rs_motor3)
//#define DM_MOTOR_NODE DT_NODELABEL(dm_motor)
const struct device *motor1 = DEVICE_DT_GET(MOTOR1_NODE);
const struct device *motor2 = DEVICE_DT_GET(MOTOR2_NODE);
const struct device *motor3 = DEVICE_DT_GET(MOTOR3_NODE);
//const struct device *dm_motor = DEVICE_DT_GET(DM_MOTOR_NODE);
static const struct device *dm_motor = DEVICE_DT_GET(DT_PATH(motor, dm_motor));
//static const struct device *dm_motor = DEVICE_DT_GET(DT_NODELABEL(dm_motor));

//电磁阀
#define SBUS_NODE DT_NODELABEL(sbus0)
const struct device *sbus = DEVICE_DT_GET(SBUS_NODE);

#define EMValve1 DT_NODELABEL(emvalve1)
#define EMValve2 DT_NODELABEL(emvalve2)
const struct gpio_dt_spec emvalve1 = GPIO_DT_SPEC_GET_BY_IDX(EMValve1, gpios, 0);
const struct gpio_dt_spec emvalve2 = GPIO_DT_SPEC_GET_BY_IDX(EMValve2, gpios, 0);

#define songkai false
#define jia true
LOG_MODULE_REGISTER(path_planning_demo, LOG_LEVEL_INF);

// ==========================================
// 📊 数据结构与枚举
// ==========================================
typedef struct { double x; double y; } Point2D;
typedef struct { double alpha; double beta; bool is_reachable; } JointAngles;
typedef struct {double x; double y } Error;



// 定义工作模式枚举
typedef enum {
    MODE_STANDBY,     // 待机模式
    MODE_PICK_T1,     // 执行目标 1 抓取
    MODE_PICK_T2,     // 执行目标 2 抓取
    MODE_PICK_T3      // 执行目标 3 抓取
} RobotMode;


// ==========================================
// 🧮 核心算法层 
// ==========================================
Point2D CalculateBezierPoint(double t, Point2D p0, Point2D p1, Point2D p2, Point2D p3) {
    Point2D p;
    double u = 1.0 - t, tt = t * t, uu = u * u, uuu = uu * u, ttt = tt * t;
    p.x = uuu * p0.x + 3 * uu * t * p1.x + 3 * u * tt * p2.x + ttt * p3.x;
    p.y = uuu * p0.y + 3 * uu * t * p1.y + 3 * u * tt * p2.y + ttt * p3.y;
    return p;
}

JointAngles CalculateIK(Point2D target,int is_backward) {
    JointAngles angles = {0.0, 0.0, true};
    double r_sq = target.x * target.x + target.y * target.y;
    double r = sqrt(r_sq);

    if (r > (L1 + L2) || r < fabs(L1 - L2)) { angles.is_reachable = false; return angles; }

    double cos_beta = (L1 * L1 + L2 * L2 - r_sq) / (2 * L1 * L2);
    if (cos_beta > 1.0) cos_beta = 1.0; if (cos_beta < -1.0) cos_beta = -1.0;
    double beta_rad = acos(cos_beta);

    double theta_rad = atan2(target.y, target.x); 
    double cos_gamma = (L1 * L1 + r_sq - L2 * L2) / (2 * L1 * r);
    if (cos_gamma > 1.0) cos_gamma = 1.0; if (cos_gamma < -1.0) cos_gamma = -1.0;
    double gamma_rad = acos(cos_gamma);

    
    double alpha_rad;
    if (is_backward == 0) {
        // 模式 1、2 专用：强制前方肘部朝上
        alpha_rad = theta_rad + gamma_rad;  
    } else {
        // 模式 3 专用：强制后方肘部朝上
        alpha_rad = theta_rad - gamma_rad;  
    }

    angles.alpha = alpha_rad * (180.0 / M_PI);
    angles.beta = beta_rad * (180.0 / M_PI);
    return angles;
}

// ==========================================
// 🚀 业务逻辑层：模式切换与轨迹执行
// ==========================================
void ExecuteTrajectory(RobotMode current_mode) {
    Point2D p0 = {P0_X, P0_Y};
    Point2D p1, p2, p3;
    double offset_x = 0.0;
    double offset_y = 0.0;
    double theta;
    double real_beta;
    int theta_mode;
    int backward;
    //Error error_1, error_2, error_3;

    // 巧妙利用 switch case 来加载不同的硬编码参数
    switch (current_mode) {
        case MODE_PICK_T1:
            p1 = (Point2D){T1_P1_X, T1_P1_Y};
            p2 = (Point2D){T1_P2_X, T1_P2_Y};
            p3 = (Point2D){T1_P3_X, T1_P3_Y};
            // T1 朝前吸 (+X方向)：腕部在尖端左侧 (X - L_CUP)
            offset_x = -L_CUP;
            offset_y = 0.0;
            theta_mode = 1;
            backward =0;
            printf("\n>>> 开始执行 [目标 1: 正前朝前吸] <<<\n");
            break;

        case MODE_PICK_T2:
            p1 = (Point2D){T2_P1_X, T2_P1_Y};
            p2 = (Point2D){T2_P2_X, T2_P2_Y};
            p3 = (Point2D){T2_P3_X, T2_P3_Y};
            // T2 朝下吸 (-Y方向)：腕部在尖端上方 (Y + L_CUP)
            offset_x = 0.0;
            offset_y = L_CUP;
            theta_mode = 2;
            backward =0;
            printf("\n>>> 开始执行 [目标 2: 正前朝下吸] <<<\n");
            break;

        case MODE_PICK_T3:
            p0 = (Point2D){-453.0, 360.0};
            p1 = (Point2D){T3_P1_X, T3_P1_Y};
            p2 = (Point2D){T3_P2_X, T3_P2_Y};
            p3 = (Point2D){T3_P3_X, T3_P3_Y};
            // T3 背后朝前吸 (-X方向)：尖端在 -659，腕部在 -579，所以腕部在尖端右侧 (X + L_CUP)
            offset_x = L_CUP; 
            offset_y = 0.0;
            theta_mode = 3;
            backward =1;
            printf("\n>>> 开始执行 [目标 3: 背后朝前吸] <<<\n");
            break;

        default:
            printf("系统待机中...\n");
            return;
    }

    // --- 开始执行插补循环 ---
    int steps = 20; // 离散成 20 个点，实际控制中可能要几百个点
    for (int i = 0; i <= steps; ++i) {
        double t = (double)i / steps;
        
        // 1. 算吸盘尖端坐标
        Point2D tip_target = CalculateBezierPoint(t, p0, p1, p2, p3);
        
        // 2. 算腕部关节坐标 (套用 switch case 里定好的偏移)
        Point2D wrist_target;
        wrist_target.x = tip_target.x + offset_x;
        wrist_target.y = tip_target.y + offset_y;

        // 3. 逆运动学求解
        JointAngles joint_cmd = CalculateIK(wrist_target,backward);
        

        if (joint_cmd.is_reachable) {
            //mode 1
            Error error_1 = {2, -12};
            real_beta =180 - (joint_cmd.alpha + joint_cmd.beta) + error_1.y;
            motor_set_mit(motor2, 0.0,joint_cmd.alpha + error_1.x ,0.0);
            motor_set_mit(motor3, 0.0,real_beta,0.0);
            
             //mode2
            Error error_2 = {15.0, -9.0};
            // real_beta =180 - (joint_cmd.alpha + joint_cmd.beta) + error_2.y;
            // motor_set_mit(motor2, 0.0,joint_cmd.alpha + error_2.x,0.0);
            // motor_set_mit(motor3, 0.0,real_beta,0.0);

            //mode 3
            Error error_3 ={-5, 10};
            // real_beta =180 - (joint_cmd.alpha + joint_cmd.beta) + error_3.y;
            // motor_set_mit(motor2, 0.0,joint_cmd.alpha + error_3.x ,0.0);
            // motor_set_mit(motor3, 0.0,real_beta,0.0);
            
            if(i == 0){  
            k_msleep(2000);
            }
            switch (theta_mode)
            {
                case 1 :
                    theta =180 - joint_cmd.alpha - joint_cmd.beta + error_1.y;
                    break;
                case 2:
                    theta =90 - joint_cmd.alpha - joint_cmd.beta + error_2.y;
                    break;
                case 3:
                    theta =270 - joint_cmd.alpha - joint_cmd.beta + error_3.y;
                    break;
                default:
                printf("error...\n");
                return;

            }
            motor_set_mit(dm_motor, 0.0,theta,0.0);

            // 打印演示
            if(i == 0 || i == steps) { // 只打印起点和终点，防止刷屏
                printf("t=%.2f | 腕部(%.1f, %.1f) | Alpha=%.2f°, Beta=%.2f°\n", 
                       t, wrist_target.x, wrist_target.y, joint_cmd.alpha, joint_cmd.beta);
            }
        } else {
            printf("警告: 轨迹点超出范围，急停！\n");
            break;
        }
        k_msleep(700);
    

    }
    //夹爪执行
	gpio_pin_set_dt(&emvalve1,jia);
	k_msleep(5000);
	//gpio_pin_set_dt(&emvalve1, songkai);

    printf("动作执行完毕。\n");
}


void motor_monitor_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	while (1) {
		motor_status_t status;
		int ret = motor_get(motor3, &status);

		while (ret != 0) {
			LOG_ERR("读取电机状态失败: %d", ret);
			k_msleep(5000); // 1秒后重试
			ret = motor_get(motor3, &status);
			continue;
		}

		LOG_INF("motor torque: %.2f, speed: %.2f, angle: %.2f", status.torque, status.rpm,
			status.angle);

		k_msleep(200);
	}
}
// ==========================================
// 🏁 主函数测试
// ==========================================
int main() {
    
    gpio_pin_configure_dt(&emvalve1, GPIO_OUTPUT_INACTIVE);//默认关闭
	gpio_pin_configure_dt(&emvalve2, GPIO_OUTPUT_INACTIVE);
    // 模拟接收到了不同的任务指令，通过改变枚举变量来切换模式
        motor_control(motor1, ENABLE_MOTOR);
        k_msleep(100);
        motor_control(motor1, AUTO_REPORT_ENABLE);
        k_msleep(100);
        motor_control(motor2, ENABLE_MOTOR);
        k_msleep(100);
        motor_control(motor2, AUTO_REPORT_ENABLE);
        k_msleep(100);
        motor_control(motor3, ENABLE_MOTOR);
        k_msleep(100);
        motor_control(motor3, AUTO_REPORT_ENABLE);
	    k_msleep(100); // 等待自动报告使能
        motor_control(dm_motor, ENABLE_MOTOR);
	    LOG_INF("电机已启用");
        k_msleep(100);
        motor_set_mode(dm_motor, MIT);
	    k_msleep(1000);
    RobotMode current_task;
    k_msleep(1000);
    motor_set_mit(motor2, 0.0,90,0.0);
    motor_set_mit(motor3, 0.0,90,0.0);
    k_msleep(1000);

    //测试模式 1
    current_task = MODE_PICK_T1;
    ExecuteTrajectory(current_task);

    //测试模式 2
    // current_task = MODE_PICK_T2;
    // ExecuteTrajectory(current_task);

    // 测试模式 3
    // current_task = MODE_PICK_T3;
    // float start_angle_b = -193.3;
    // float start_angle_a = 118.7;
    // for(int i =0; i<= 8;i++){
    //     motor_set_mit(motor3,0,90 - 28.3*i, 0);
    //     motor_set_mit(motor2,0,90 + 2.87*i, 0);
    //     k_msleep(500);
    // }

    //ExecuteTrajectory(current_task);
k_msleep(2000);
    //放置
    motor_set_mit(motor2, 0.0,90,0.0);
    motor_set_mit(motor3, 0.0,-5.2,0.0);
    for(int i =0;i<=5;i++){
        motor_set_mit(motor2, 0.0,13.4 + 8.6*i,0.0);
        k_msleep(500);
    }

    

    return 0;
}
