#ifndef CYCLOID_H
#define CYCLOID_H

#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/motor.h>
#include "math.h"

#define L1 0.09f
#define L2 0.224f
#define L3 0.224f
#define L4 0.09f
#define M_PI 3.14159265358979323846

#define CONTROL_CYCLE 10 // 控制周期10ms
#define TRAJ_POINTS 30   // 轨迹点数
#define BACK_TRAJ_POINTS 30
#define P_MOTOR  10
#define D_MOTOR  1
#define F_MOTOR  7
#define Torque 5

#define INITIAL_ANGLE1 45 //站定时刻对应电机角度
#define INITIAL_ANGLE2 -90
#define INITIAL_ANGLE3 90
#define INITIAL_ANGLE4 -45
#define INITIAL_ANGLE5 -45
#define INITIAL_ANGLE6 90
#define INITIAL_ANGLE7 -90
#define INITIAL_ANGLE8 45

#define TRAJ_INTERVAL 10

#define STACK_SIZE 4096
#define THREAD_PRIORITY -1

extern float g_target_theta1;
extern float g_target_theta4;

extern float forward_angle1[TRAJ_POINTS];
extern float forward_angle4[TRAJ_POINTS];
extern float backward_angle1[TRAJ_POINTS];
extern float backward_angle4[TRAJ_POINTS];

typedef enum
{
  GET_DOWN,
  STAND_UP,
  MOVE
}State;

typedef enum
{
  FORWARD,
  PAUSE_END,  // 终点停留
  BACKWARD,   // 返回起点
  PAUSE_START // 起点停留
}GaitState;

typedef enum
{
  LEFT_FRONT,
  RIGHT_FRONT,
  LEFT_BACK,
  RIGHT_BACK,
}legside;

typedef struct{
  const struct device *motor_a;
  const struct device *motor_b;
  const legside leg_side;
}leg_motors; 


typedef struct{
  const struct device *motor_a;
  const struct device *motor_b;
  const float initial_angle_a;
  const float initial_angle_b;
  const legside side;
  GaitState gait_state;
  // uint8_t leg_status;
  // uint32_t last_tick;
  // uint32_t start_time;
  uint8_t traj_index;
}Leg; 

typedef struct
{
  float x;
  float z;
}Point2D;

uint8_t inverse_kinematic(float x, float z, float *theta1, float *theta4);
void generate_cycloid(Point2D start, Point2D end, float height, Point2D *traj, int num_points);
void MotionControl(const struct device* motor, uint32_t total_time, uint8_t move, float start_angle, float target_angle, float speed);
void motor_control_thread(void* arg1 , void* arg2, void* arg3);
void leg_control_thread(void* arg1 , void* arg2, void* arg3);
void waist_control_thread(void* arg1 , void* arg2, void* arg3);
void dog_initialization(Point2D start, Point2D end, float *forward_theta1, float *forward_theta4, float *backward_theta1, float *backward_theta4);

#endif