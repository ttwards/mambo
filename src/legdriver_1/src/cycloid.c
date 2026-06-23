#include "cycloid.h"

float g_target_theta1 = 0;
float g_target_theta4 = 0;

/**  
 * @brief 插值运动控制
 * @param motor 电机设备
 * @param total_time 计划运行时间
 * @param move 计划运行步数
 * @param start_angle 初始角度
 * @param target_angle 目标角度
 */
void MotionControl(const struct device* motor, uint32_t total_time, uint8_t move, float start_angle, float target_angle, float speed)
{
  uint32_t each_time = total_time / move;
  float each_move = (target_angle - start_angle)/ move;
  for (int i = 0; i < move; i++)
  {
    motor_set_speed(motor, speed);
    motor_set_angle(motor, start_angle + each_move * (i + 1));
    k_msleep(each_time);
  }
}
/**
 *@brief 运动控制线程
 */
void motor_control_thread(void* arg1 , void* arg2, void* arg3)
{
  Leg *leg = (Leg *)arg1;
  ARG_UNUSED(arg2);
  ARG_UNUSED(arg3);

  extern State dog_state;
  while(1){
    switch(dog_state)
    {
      case GET_DOWN:
        break;
      case STAND_UP:
        break;
      case MOVE:
        break;
    }
  }
}
/**
 *@brief 站立控制线程
 */
void leg_control_thread(void* arg1 , void* arg2, void* arg3)
{

  leg_motors *leg = (leg_motors *)arg1;
  ARG_UNUSED(arg2);
  ARG_UNUSED(arg3);

  uint8_t init = 0;
  
  while(1){
    if (init == 0){   
      switch(leg->leg_side) {
        case LEFT_FRONT:
          MotionControl(leg->motor_a, 200, 5, 0, INITIAL_ANGLE5,100);
          MotionControl(leg->motor_b, 200, 5, 0, INITIAL_ANGLE6,100);
          k_msleep(TRAJ_INTERVAL);
          break;
          
        case RIGHT_FRONT:
          MotionControl(leg->motor_a, 200, 5, 0, INITIAL_ANGLE7,100);
          MotionControl(leg->motor_b, 200, 5, 0, INITIAL_ANGLE8,100);
          k_msleep(TRAJ_INTERVAL);
          break;

        case LEFT_BACK:
          MotionControl(leg->motor_a, 200, 5, 0, INITIAL_ANGLE1,100);
          MotionControl(leg->motor_b, 200, 5, 0, INITIAL_ANGLE2,100);
          k_msleep(TRAJ_INTERVAL);
          break;

        case RIGHT_BACK:
          MotionControl(leg->motor_a, 200, 5, 0, INITIAL_ANGLE3,100);
          MotionControl(leg->motor_b, 200, 5, 0, INITIAL_ANGLE4,100);
          k_msleep(TRAJ_INTERVAL);
          break;
      }
      init++;
    }
    else{
      for(int i = 0; i < TRAJ_POINTS; i++){
        switch(leg->leg_side){
          case LEFT_FRONT:
            motor_set_speed(leg->motor_a, 100);
            motor_set_speed(leg->motor_b, 100);
            motor_set_angle(leg->motor_a, INITIAL_ANGLE3 + forward_angle4[i]);
            motor_set_angle(leg->motor_b, INITIAL_ANGLE4 + forward_angle1[i]);
            k_msleep(TRAJ_INTERVAL);
            break;
          case LEFT_BACK:
            motor_set_speed(leg->motor_a, 100);
            motor_set_speed(leg->motor_b, 100);
            motor_set_angle(leg->motor_a, INITIAL_ANGLE1 - backward_angle1[i]);
            motor_set_angle(leg->motor_b, INITIAL_ANGLE2 - backward_angle4[i]);
            k_msleep(TRAJ_INTERVAL);
            break;
          case RIGHT_FRONT:
            motor_set_speed(leg->motor_a, 100);
            motor_set_speed(leg->motor_b, 100);
            motor_set_angle(leg->motor_a, INITIAL_ANGLE7 - backward_angle4[i]);
            motor_set_angle(leg->motor_b, INITIAL_ANGLE8 - backward_angle1[i]);
            k_msleep(TRAJ_INTERVAL);
            break;
          case RIGHT_BACK:
            motor_set_speed(leg->motor_a, 100);
            motor_set_speed(leg->motor_b, 100);
            motor_set_angle(leg->motor_a, INITIAL_ANGLE5 + forward_angle1[i]);
            motor_set_angle(leg->motor_b, INITIAL_ANGLE6 + forward_angle4[i]);
            k_msleep(TRAJ_INTERVAL);
            break;
        }
      }
      for(int i = 0; i < BACK_TRAJ_POINTS; i++){
        switch(leg->leg_side){
          case LEFT_BACK:
            motor_set_speed(leg->motor_a, 100);
            motor_set_speed(leg->motor_b, 100);
            motor_set_angle(leg->motor_a, INITIAL_ANGLE1 - forward_angle1[i]);
            motor_set_angle(leg->motor_b, INITIAL_ANGLE2 - forward_angle4[i]);
            k_msleep(TRAJ_INTERVAL);
            break;
          case LEFT_FRONT:
            motor_set_speed(leg->motor_a, 100);
            motor_set_speed(leg->motor_b, 100);
            motor_set_angle(leg->motor_a, INITIAL_ANGLE3 + backward_angle4[i]);
            motor_set_angle(leg->motor_b, INITIAL_ANGLE4 + backward_angle1[i]);
            k_msleep(TRAJ_INTERVAL);
            break;
          case RIGHT_BACK:
            motor_set_speed(leg->motor_a, 100);
            motor_set_speed(leg->motor_b, 100);
            motor_set_angle(leg->motor_a, INITIAL_ANGLE5 + backward_angle1[i]);
            motor_set_angle(leg->motor_b, INITIAL_ANGLE6 + backward_angle4[i]);
            k_msleep(TRAJ_INTERVAL);
            break;
          case RIGHT_FRONT:
            motor_set_speed(leg->motor_a, 100);
            motor_set_speed(leg->motor_b, 100);
            motor_set_angle(leg->motor_a, INITIAL_ANGLE7 - forward_angle4[i]);
            motor_set_angle(leg->motor_b, INITIAL_ANGLE8 - forward_angle1[i]);
            k_msleep(TRAJ_INTERVAL);
            break;
        }
      }
    }  
  }  
}
/**  
 * @brief 逆运动学解算
 * @param x: x坐标
 * @param z: z坐标
 * @param theta1: 电机1角度
 * @param theta4: 电机4角度
 * @return 0: 表示解算失败
 * @return 1: 表示解算成功
 */
uint8_t inverse_kinematic(float x, float z, float *theta1, float *theta4)
{
  float l0 = sqrtf(x * x + z * z);

  if (fabsf(L3 - L4) >= l0 || l0 >= (L3 + L4))
  {
    return 0; // 返回错误标志
  }

  float theta_inside = acosf((L4 * L4 + l0 * l0 - L3 * L3) / (2 * L4 * l0));
  float theta = atan2f(z, x);

  *theta1 = theta + theta_inside;
  *theta4 = theta - theta_inside;
  return 1; // 成功标志
}

/**
 * @brief 生成摆线轨迹
 * @param start: 起点
 * @param end: 终点
 * @param height: 高度
 * @param traj: 轨迹
 * @param num_points: 轨迹点数
 */
void generate_cycloid(Point2D start, Point2D end, float height, Point2D *traj, int num_points)
{
  float delta_x = end.x - start.x;
  float delta_z = end.z - start.z; // 起点与落点可能存在高度差

  for (int i = 0; i < num_points; ++i)
  {
    float t = 2 * M_PI * i / (num_points - 1);

    // X方向摆线
    traj[i].x = start.x + delta_x * (t - sinf(t)) / (2 * M_PI);

    traj[i].z = start.z + delta_z * (i / (float)num_points) + height * (1 - cosf(t)) / 2; // 关键修改：height取反
  }
}
/**
*@brief 初始化步态
*@param start 目标电位起始点
*@param end 目标电位终止点
*@param forward_theta1 电机1去程角度
*@param forward_theta4 电机4去程角度
*@param backward_theta1 电机1回程角度
*@param backward_theta4 电机4回程角度
**/
void dog_initialization(Point2D start, Point2D end, float *forward_theta1, float *forward_theta4, float *backward_theta1, float *backward_theta4)
{
  Point2D forward_trajectory[TRAJ_POINTS];
  Point2D backward_trajectory[BACK_TRAJ_POINTS];
  generate_cycloid(start, end, 0.03f, forward_trajectory, TRAJ_POINTS);
  generate_cycloid(end, start, -0.03f, backward_trajectory, BACK_TRAJ_POINTS);

  for (int i = 0; i < TRAJ_POINTS; i++)
  {
    Point2D current = forward_trajectory[i];
    float theta1, theta4;
    if(inverse_kinematic(current.x, current.z, &theta1, &theta4))
    {
      forward_theta1[i] = -theta1 * 180/M_PI;
      forward_theta4[i] = (theta4 + M_PI) * 180/M_PI;
    }
  }
  for(int i = 0; i < BACK_TRAJ_POINTS; i++)
  {
    Point2D current = backward_trajectory[i];
    float theta1, theta4;
    if(inverse_kinematic(current.x, current.z, &theta1, &theta4))
    {
      backward_theta1[i] = -theta1 * 180/M_PI;;
      backward_theta4[i] = (theta4 + M_PI) * 180/M_PI;
    }
  }
  k_msleep(1000);
}