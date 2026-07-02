#include "arm_poses.h"

#include <math.h>

// ============== 位姿序列数据 ==============

const motor_angles_t init[] = {
    {0.0f, -135.0f, 152.31f, 22.8f},
};
const size_t init_count = sizeof(init) / sizeof(init[0]);


const motor_angles_t store1[] = {
    {0.0f, -135.0f, 152.31f, 22.8f},  /* 暂用 init 位姿，需修改 */
};
const size_t store1_count = sizeof(store1) / sizeof(store1[0]);

const motor_angles_t store2[] = {
    {0.0f, -135.0f, 152.31f, 0.00f},
};
const size_t store2_count = sizeof(store2) / sizeof(store2[0]);

const motor_angles_t forward_positive_200[] = {
    {0.0f, -61.02f, 86.51f, -64.50f},
};
const size_t forward_positive_200_count = sizeof(forward_positive_200) / sizeof(forward_positive_200[0]);

const motor_angles_t forward_positive_400[] = {
    {0.0f, -61.02f, 86.51f, -64.50f},
};
const size_t forward_positive_400_count = sizeof(forward_positive_400) / sizeof(forward_positive_400[0]);

// const motor_angles_t forward_negative_200[] = {
//     {0.0f, -40.61f, 75.00f, -130.61f},
// };
const motor_angles_t forward_negative_200[] = {
    {0.0f, 0.00f, 0.00f, -170.00f},//有问题
};

const size_t forward_negative_200_count = sizeof(forward_negative_200) / sizeof(forward_negative_200[0]);

const motor_angles_t left_positive_200[] = {
    {-90.0f, -39.11f, 51.44f, -77.67f},
};
const size_t left_positive_200_count = sizeof(left_positive_200) / sizeof(left_positive_200[0]);

const motor_angles_t left_positive_400[] = {
    {-90.0f, -39.11f, 51.44f, -77.67f},
};
const size_t left_positive_400_count = sizeof(left_positive_400) / sizeof(left_positive_400[0]);

const motor_angles_t left_negative_200[] = {
    {-90.0f, -15.22f, 27.90f, -167.32f},//有问题
};
const size_t left_negative_200_count = sizeof(left_negative_200) / sizeof(left_negative_200[0]);

const motor_angles_t right_positive_200[] = {
    {90.0f, -39.11f, 51.44f, -77.67f},
};
const size_t right_positive_200_count = sizeof(right_positive_200) / sizeof(right_positive_200[0]);

const motor_angles_t right_positive_400[] = {
    {90.0f, -39.11f, 51.44f, -77.67f},
};
const size_t right_positive_400_count = sizeof(right_positive_400) / sizeof(right_positive_400[0]);

const motor_angles_t right_negative_200[] = {
    {90.0f, -15.22f, 27.90f, -167.32f},//有问题
};
const size_t right_negative_200_count = sizeof(right_negative_200) / sizeof(right_negative_200[0]);

// const motor_angles_t put_2_floor[] = {
//     {0.0f, -91.39f, 76.41f, -104.98f},
// };
const motor_angles_t put_2_floor[] = {
    {0.0f, -125.55f, 100.47f, -115.08f},
};
const size_t put_2_floor_count = sizeof(put_2_floor) / sizeof(put_2_floor[0]);

const motor_angles_t put_3_floor[] = {
    {0.0f, -114.46f, 73.65f, -130.81f},
};
const size_t put_3_floor_count = sizeof(put_3_floor) / sizeof(put_3_floor[0]);




const motor_angles_t init_to_forward_positive_200[] = {
};
const size_t init_to_forward_positive_200_count = sizeof(init_to_forward_positive_200) / sizeof(init_to_forward_positive_200[0]);

const motor_angles_t init_to_forward_positive_400[] = {
};
const size_t init_to_forward_positive_400_count = sizeof(init_to_forward_positive_400) / sizeof(init_to_forward_positive_400[0]);

const motor_angles_t one_to_up[] = {
};
const size_t one_to_up_count = sizeof(one_to_up) / sizeof(one_to_up[0]);

const motor_angles_t two_to_up[] = {
};
const size_t two_to_up_count = sizeof(two_to_up) / sizeof(two_to_up[0]);

const motor_angles_t up_to_put[] = {
};
const size_t up_to_put_count = sizeof(up_to_put) / sizeof(up_to_put[0]);

const motor_angles_t put_to_init[] = {
};
const size_t put_to_init_count = sizeof(put_to_init) / sizeof(put_to_init[0]);

const motor_angles_t stay_init[] = {
    {0.0f,   0.00f,   0.00f,   0.00f},
};
const size_t stay_init_count = sizeof(stay_init) / sizeof(stay_init[0]);

size_t interpolate_poses(const motor_angles_t *start, const motor_angles_t *end,
                         motor_angles_t *result, float step_deg)
{
    float diff1 = fabsf(end->motor1_angle - start->motor1_angle);
    float diff2 = fabsf(end->motor2_angle - start->motor2_angle);
    float diff3 = fabsf(end->motor3_angle - start->motor3_angle);
    float diff4 = fabsf(end->motor4_angle - start->motor4_angle);
    float max_diff = diff1;

    if (diff2 > max_diff) max_diff = diff2;
    if (diff3 > max_diff) max_diff = diff3;
    if (diff4 > max_diff) max_diff = diff4;

    size_t steps = (size_t)ceilf(max_diff / step_deg);
    if (steps == 0) steps = 1;

    for (size_t i = 0; i <= steps; i++) {
        float t = (float)i / (float)steps;
        float m1_diff = end->motor1_angle - start->motor1_angle;
        float m2_diff = end->motor2_angle - start->motor2_angle;
        float m3_diff = end->motor3_angle - start->motor3_angle;
        float m4_diff = end->motor4_angle - start->motor4_angle;
        float progress1 = (diff1 > 0.001f) ? (t * max_diff / diff1) : 1.0f;
        float progress2 = (diff2 > 0.001f) ? (t * max_diff / diff2) : 1.0f;
        float progress3 = (diff3 > 0.001f) ? (t * max_diff / diff3) : 1.0f;
        float progress4 = (diff4 > 0.001f) ? (t * max_diff / diff4) : 1.0f;

        if (progress1 > 1.0f) progress1 = 1.0f;
        if (progress2 > 1.0f) progress2 = 1.0f;
        if (progress3 > 1.0f) progress3 = 1.0f;
        if (progress4 > 1.0f) progress4 = 1.0f;

        result[i].motor1_angle = start->motor1_angle + m1_diff * progress1;
        result[i].motor2_angle = start->motor2_angle + m2_diff * progress2;
        result[i].motor3_angle = start->motor3_angle + m3_diff * progress3;
        result[i].motor4_angle = start->motor4_angle + m4_diff * progress4;
    }

    return steps + 1;
}
