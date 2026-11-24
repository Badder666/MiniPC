#ifndef GRAVITY_MODEL_H
#define GRAVITY_MODEL_H

#define GRAVITY_MODEL_DOF 6U

typedef struct {
    float m[3][3];
} GravityRotation;

/* 一条串联关节及其子 Link；坐标和质量单位为 m、kg。 */
typedef struct {
    GravityRotation origin_rotation;
    float origin_translation[3];
    float axis[3];
    float mass;
    float center_of_mass[3];
} GravityModelJoint;

/* controller_mount → base_link 的固定安装变换。 */
extern const GravityRotation gravity_mount_rotation;
extern const float gravity_mount_translation[3];
extern const GravityModelJoint gravity_model_joints[GRAVITY_MODEL_DOF];

#endif /* GRAVITY_MODEL_H */
