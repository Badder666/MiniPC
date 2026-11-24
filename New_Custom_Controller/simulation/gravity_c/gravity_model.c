#include "gravity_model.h"

/* 导出自当前运行的 controller_identified.urdf（2026-10-11）。
 * SHA256: 0b717261f6c01a3a1392398b1b5ccb77986d4be4c72fd7c6cdfe7cddb9da1422
 * 质量和质心为最终手动采集辨识结果；固定安装变换同时保留。
 * base_link 固定在根部，其重力不产生六个活动关节的广义力矩。 */
const GravityRotation gravity_mount_rotation = {.m = {
        {-1.0f, -1.186845315e-20f, 3.231089149e-15f},
        {-3.231089149e-15f, 3.673205103e-06f, -1.0f},
        {-0.0f, -1.0f, -3.673205103e-06f}
    }};
const float gravity_mount_translation[3] = {0.0f, 0.0f, 0.0f};

const GravityModelJoint gravity_model_joints[GRAVITY_MODEL_DOF] = {
    { /* J1 → link1 */
        .origin_rotation = {.m = {
        {1.0f, 0.0f, 0.0f},
        {0.0f, -3.673205103e-06f, -1.0f},
        {-0.0f, 1.0f, -3.673205103e-06f}
    }},
        .origin_translation = {0.0f, 0.006f, 0.0f},
        .axis = {0.0f, 0.0f, -1.0f},
        .mass = 0.3847706108f,
        .center_of_mass = {0.001598804355f, 0.003164657495f, 0.04651919185f},
    },
    { /* J2 → link2 */
        .origin_rotation = {.m = {
        {0.9659256678f, -9.506976094e-07f, 0.2588196364f},
        {-0.2588196364f, -3.548043093e-06f, 0.9659256678f},
        {-0.0f, -1.0f, -3.673205103e-06f}
    }},
        .origin_translation = {0.0064705f, 0.024148f, 0.051f},
        .axis = {0.0f, 0.0f, -1.0f},
        .mass = 0.3632108124f,
        .center_of_mass = {-0.06987581934f, 0.02850000001f, -0.02800061155f},
    },
    { /* J3 → link3 */
        .origin_rotation = {.m = {
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {-0.0f, 0.0f, 1.0f}
    }},
        .origin_translation = {-0.13f, 0.0f, -0.0045f},
        .axis = {0.0f, 0.0f, -1.0f},
        .mass = 0.3709381712f,
        .center_of_mass = {-0.008563901505f, -0.05962771186f, -0.03776062134f},
    },
    { /* J4 → link4 */
        .origin_rotation = {.m = {
        {-3.538280741e-10f, 7.346410207e-06f, 1.0f},
        {-3.673205086e-06f, -1.0f, 7.346410206e-06f},
        {1.0f, -3.673205084e-06f, 3.808129454e-10f}
    }},
        .origin_translation = {0.062989f, -0.06f, -0.0345f},
        .axis = {0.0f, 0.0f, -1.0f},
        .mass = 0.3534461818f,
        .center_of_mass = {0.04847773635f, 0.003379186393f, 0.01924875588f},
    },
    { /* J5 → link5 */
        .origin_rotation = {.m = {
        {-1.17407439e-08f, -0.0001999999986f, -0.99999998f},
        {3.67318634e-06f, 0.99999998f, -0.0001999999987f},
        {1.0f, -3.673188614e-06f, -1.100610641e-08f}
    }},
        .origin_translation = {0.0462f, 0.0f, 0.047011f},
        .axis = {0.0f, 0.0f, -1.0f},
        .mass = 0.4265849206f,
        .center_of_mass = {0.07219899628f, 0.008807673324f, 0.05331280764f},
    },
    { /* J6 → link6 */
        .origin_rotation = {.m = {
        {-4.516579214e-08f, -8.294753773e-14f, 1.0f},
        {-3.672927413e-06f, 1.0f, -8.2945803e-14f},
        {-1.0f, -3.672927413e-06f, -4.516579214e-08f}
    }},
        .origin_translation = {0.1335f, 0.0f, 0.0462f},
        .axis = {0.0f, 0.0f, -1.0f},
        .mass = 0.09198231162f,
        .center_of_mass = {0.01304669206f, -0.03616048378f, -0.001412644128f},
    },
};
