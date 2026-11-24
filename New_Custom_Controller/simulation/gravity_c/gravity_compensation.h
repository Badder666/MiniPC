#ifndef GRAVITY_COMPENSATION_H
#define GRAVITY_COMPENSATION_H

#include <stdbool.h>
#include "gravity_model.h"

/* q 为 URDF 关节角度，gravity 为模型根坐标系中的重力加速度。
 * 返回抵消重力所需的关节力矩，语义与 Pinocchio g(q) 一致。
 * 不包含电机映射、补偿倍率或速度阻尼；无动态分配、无 RTOS/HAL 依赖。 */
bool GravityCompensation_Calculate(const float q[GRAVITY_MODEL_DOF],
                                   const float gravity[3],
                                   float torque[GRAVITY_MODEL_DOF]);

#endif /* GRAVITY_COMPENSATION_H */
