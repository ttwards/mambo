#ifndef ARM_CONTROL_H
#define ARM_CONTROL_H

#include <stddef.h>

#include "arm_poses.h"

int arm_control_prepare(void);
void execute_path(const motor_angles_t *poses, size_t count, float speed_dps);
void execute_pose_sequence(const motor_angles_t *poses, size_t count);
void xipan_on(void);
void xipan_off(void);

#endif /* ARM_CONTROL_H */
