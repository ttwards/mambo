#ifndef ARM_POSES_H
#define ARM_POSES_H

#include <stddef.h>
#include <stdbool.h>

/**
 * @brief 电机角度结构体
 */
typedef struct {
    float motor1_angle;  /**< yaw轴电机角度 */
    float motor2_angle;  /**< 大臂电机角度 */
    float motor3_angle;  /**< 小臂电机角度 */
    float motor4_angle;  /**< 末端电机角度 */
} motor_angles_t;

// ============== 位姿序列声明 ==============

/** @brief 初始位置 */
extern const motor_angles_t init[];
extern const size_t init_count;

/** @brief 存储位1 (第一次取物放置位) */
extern const motor_angles_t store1[];
extern const size_t store1_count;

/** @brief 存储位2 (第二次取物放置位) */
extern const motor_angles_t store2[];
extern const size_t store2_count;

/** @brief 前进正向200 */
extern const motor_angles_t forward_positive_200[];
extern const size_t forward_positive_200_count;

/** @brief 前进正向400 */
extern const motor_angles_t forward_positive_400[];
extern const size_t forward_positive_400_count;

/** @brief 前进负向200 */
extern const motor_angles_t forward_negative_200[];
extern const size_t forward_negative_200_count;

/** @brief 左移正向200 */
extern const motor_angles_t left_positive_200[];
extern const size_t left_positive_200_count;

/** @brief 左移正向400 */
extern const motor_angles_t left_positive_400[];
extern const size_t left_positive_400_count;

/** @brief 左移负向200 */
extern const motor_angles_t left_negative_200[];
extern const size_t left_negative_200_count;

/** @brief 右移正向200 */
extern const motor_angles_t right_positive_200[];
extern const size_t right_positive_200_count;

/** @brief 右移正向400 */
extern const motor_angles_t right_positive_400[];
extern const size_t right_positive_400_count;

/** @brief 右移负向200 */
extern const motor_angles_t right_negative_200[];
extern const size_t right_negative_200_count;

/** @brief 放置到2层 */
extern const motor_angles_t put_2_floor[];
extern const size_t put_2_floor_count;

/** @brief 放置到3层 */
extern const motor_angles_t put_3_floor[];
extern const size_t put_3_floor_count;

/** @brief 初始位置到前进正向200 */
extern const motor_angles_t init_to_forward_positive_200[];
extern const size_t init_to_forward_positive_200_count;

/** @brief 初始位置到前进正向400 */
extern const motor_angles_t init_to_forward_positive_400[];
extern const size_t init_to_forward_positive_400_count;

/** @brief 位姿1到上举 */
extern const motor_angles_t one_to_up[];
extern const size_t one_to_up_count;

/** @brief 位姿2到上举 */
extern const motor_angles_t two_to_up[];
extern const size_t two_to_up_count;

/** @brief 上举到放置 */
extern const motor_angles_t up_to_put[];
extern const size_t up_to_put_count;

/** @brief 放置回初始 */
extern const motor_angles_t put_to_init[];
extern const size_t put_to_init_count;

/** @brief 保持初始位置 */
extern const motor_angles_t stay_init[];
extern const size_t stay_init_count;

// ============== 插值函数 ==============

/**
 * @brief 线性插值函数
 *
 * 找出两个位姿中角度差最大的电机，以指定步长插入中间值
 * 其他电机按比例插值，到达最终值后保持
 *
 * @param[in] start 起始位姿
 * @param[in] end 结束位姿
 * @param[out] result 插值结果数组（需预先分配足够空间）
 * @param[in] step_deg 插值步长(度)
 * @return 插值点数量（包含起始和结束）
 */
size_t interpolate_poses(const motor_angles_t *start, const motor_angles_t *end,
                         motor_angles_t *result, float step_deg);

#endif /* ARM_POSES_H */