#include "gravity_compensation.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static void RotateVector(const GravityRotation *rotation, const float vector[3],
                         float result[3])
{
    for (unsigned int row = 0U; row < 3U; ++row) {
        result[row] = rotation->m[row][0] * vector[0] +
                      rotation->m[row][1] * vector[1] +
                      rotation->m[row][2] * vector[2];
    }
}

static void MultiplyRotation(const GravityRotation *left, const GravityRotation *right,
                             GravityRotation *result)
{
    for (unsigned int row = 0U; row < 3U; ++row) {
        for (unsigned int column = 0U; column < 3U; ++column) {
            result->m[row][column] = left->m[row][0] * right->m[0][column] +
                                     left->m[row][1] * right->m[1][column] +
                                     left->m[row][2] * right->m[2][column];
        }
    }
}

/* Rodrigues 旋转：模型参数中的 axis 已在导出时归一化。 */
static void JointRotation(const float axis[3], float angle, GravityRotation *rotation)
{
    const float c = cosf(angle);
    const float s = sinf(angle);
    const float t = 1.0f - c;
    const float x = axis[0], y = axis[1], z = axis[2];
    rotation->m[0][0] = c + x * x * t;
    rotation->m[0][1] = x * y * t - z * s;
    rotation->m[0][2] = x * z * t + y * s;
    rotation->m[1][0] = y * x * t + z * s;
    rotation->m[1][1] = c + y * y * t;
    rotation->m[1][2] = y * z * t - x * s;
    rotation->m[2][0] = z * x * t - y * s;
    rotation->m[2][1] = z * y * t + x * s;
    rotation->m[2][2] = c + z * z * t;
}

bool GravityCompensation_Calculate(const float q[GRAVITY_MODEL_DOF],
                                   const float gravity[3],
                                   float torque[GRAVITY_MODEL_DOF])
{
    if (q == NULL || gravity == NULL || torque == NULL) {
        return false;
    }
    for (unsigned int i = 0U; i < GRAVITY_MODEL_DOF; ++i) {
        if (!isfinite(q[i])) {
            return false;
        }
    }
    for (unsigned int i = 0U; i < 3U; ++i) {
        if (!isfinite(gravity[i])) {
            return false;
        }
    }

    float joint_position[GRAVITY_MODEL_DOF][3];
    float joint_axis[GRAVITY_MODEL_DOF][3];
    float center_of_mass[GRAVITY_MODEL_DOF][3];
    GravityRotation parent_rotation = gravity_mount_rotation;
    float parent_position[3];
    memcpy(parent_position, gravity_mount_translation, sizeof(parent_position));

    /* 正运动学：关节中心、旋转轴和质心都转换到 controller_mount 坐标系。 */
    for (unsigned int i = 0U; i < GRAVITY_MODEL_DOF; ++i) {
        const GravityModelJoint *joint = &gravity_model_joints[i];
        GravityRotation origin_rotation, motion_rotation, link_rotation;
        float translation[3], com_offset[3];
        MultiplyRotation(&parent_rotation, &joint->origin_rotation, &origin_rotation);
        RotateVector(&parent_rotation, joint->origin_translation, translation);
        RotateVector(&origin_rotation, joint->axis, joint_axis[i]);
        JointRotation(joint->axis, q[i], &motion_rotation);
        MultiplyRotation(&origin_rotation, &motion_rotation, &link_rotation);
        RotateVector(&link_rotation, joint->center_of_mass, com_offset);
        for (unsigned int k = 0U; k < 3U; ++k) {
            joint_position[i][k] = parent_position[k] + translation[k];
            center_of_mass[i][k] = joint_position[i][k] + com_offset[k];
            parent_position[k] = joint_position[i][k];
        }
        parent_rotation = link_rotation;
    }

    /* 每个关节承担自身和下游 Link 的重力；负号将重力负载转成补偿力矩。 */
    for (unsigned int i = 0U; i < GRAVITY_MODEL_DOF; ++i) {
        float moment[3] = {0.0f, 0.0f, 0.0f};
        for (unsigned int link = i; link < GRAVITY_MODEL_DOF; ++link) {
            const float mass = gravity_model_joints[link].mass;
            float r[3], force[3];
            for (unsigned int k = 0U; k < 3U; ++k) {
                r[k] = center_of_mass[link][k] - joint_position[i][k];
                force[k] = mass * gravity[k];
            }
            moment[0] += r[1] * force[2] - r[2] * force[1];
            moment[1] += r[2] * force[0] - r[0] * force[2];
            moment[2] += r[0] * force[1] - r[1] * force[0];
        }
        torque[i] = -(joint_axis[i][0] * moment[0] +
                      joint_axis[i][1] * moment[1] +
                      joint_axis[i][2] * moment[2]);
        if (!isfinite(torque[i])) {
            return false;
        }
    }
    return true;
}
