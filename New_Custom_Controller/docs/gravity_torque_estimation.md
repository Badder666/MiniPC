# URDF 静态重力力矩估算

`v2/urdf/v2.urdf` 包含六个转动关节、连杆质量和质心信息，可以在上位机计算一个静态重力力矩近似值。
模型使用每个连杆的质量和质心，不使用惯量矩阵，因此只适用于低速或静止保持姿态：

```text
tau_i = sum( Jv_i(link_com)^T * (mass_link * gravity) )
```

它不包含加速度、科氏力、摩擦、减速器损耗和电机效率。计算结果单位为 N*m，只能作为初始前馈值，必须在低增益、机械支撑和急停条件下标定。

## 运行

在上位机工作区中：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run custom_controller_usb gravity_torque \
  --urdf /path/to/v2/urdf/v2.urdf \
  --q 0 0 0 0 0 0
```

`v2.urdf` 的全零姿态是折叠姿态。命令中的 `q` 是自定义控制器六个电机的零位坐标；不要把 RViz 使用的 `new_hj_arm2` 折叠参考姿态 `[0, 2.4, -2.4, 0, 0, 0]` 直接作为该模型输入。程序按 URDF 链顺序使用关节：`jonit1`, `joint2`, ..., `joint6`。URDF 中的关节限位目前都是 0，占位值不能用于运动范围校验；计算器只使用几何、质量和质心字段。实际采样范围以固件 `g_gravity_sampling_min_rad` 和 `g_gravity_sampling_max_rad` 为准。

当前命令只计算并打印力矩，不会修改 64 字节控制协议，也不会自动发送到下位机。要真正使用补偿值，需要在确认符号和零位后，把力矩字段加入上位机到下位机协议，并在下位机 MIT 控制中叠加。
