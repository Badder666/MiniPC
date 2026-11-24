# 辨识质心接入与 RViz 显示

## 本次接入的模型

使用 `controller.csv` 的 45 个稳定窗口、合并后的 15 个姿态。J2 使用修正后的电机设置，不除以 4。虚拟机中保存的零位比第一次拟合时更新过，因此用同一批数据按最新 offset 重新表达姿态、重新拟合。

原模型：`src/custom_controller_usb/config/controller_model_urdf2/controller.urdf`。

候选模型：同目录 `controller_identified.urdf`。RViz、robot_state_publisher 和 Pinocchio 通过同一 `urdf_path` 加载它。

模型记录：同目录 `controller_identified_fit.json`，包含采集时的映射、修正量、拟合指标和适用范围。

| Link | 当前质心 xyz，单位 m | 来源 |
| --- | --- | --- |
| base_link | 0.008978, -0.015407, -0.035343 | URDF |
| link1 | 0.001599, 0.003165, 0.046519 | URDF |
| link2 | -0.094329, 0.028500, -0.028001 | 约束拟合候选 |
| link3 | -0.019671, -0.024878, -0.037761 | 后段等效修正附着在 link3 |
| link4 | 0.060823, 0.000036, 0.044256 | URDF |
| link5 | 0.095892, -0.000005, 0.038413 | URDF |
| link6 | 0.000574, -0.050879, 0.003697 | URDF |

质心 xyz 是相对各 Link 局部坐标系的位置，不是世界坐标，也不是旋转中心。修改质心只改变重力计算和质心标记，不改变模型关节转轴、旋转中心或可视化运动几何。

link3 的候选由 `原 link3 质心 + [delta_h3_x, delta_h3_y, 0] / link3质量` 得到。后段整体质心不能直接赋给 link3；此次后段采样变化较小，无法分别辨识 link3～link6 的真实质心。黄色整体质心按当前 J4～J6 角度重新计算，随姿态变化。

质量、z 分量和惯量张量保留 URDF 先验。这是用于静态重力的候选模型，尚未辨识完整动力学。link2 的 y 分量仍触及 STL 几何约束边界；后段大幅转动的效果还需实测。

拟合报告中的常值偏置仅用于诊断，不加入实时重力输出。纯重力对这批数据的 RMSE：J2 约 0.145 N·m，J3 约 0.095 N·m；加诊断偏置时约 0.096、0.066 N·m，不能混作当前实时补偿误差。

## RViz 标记

左侧 `Link Centers of Mass` 默认开启，话题为 `/custom_controller/center_of_mass_markers`。

- 绿色：保留原值的 Link 质心。
- 紫色：当前候选模型的 link2、link3 质心。
- 蓝色：link2、link3 的原 URDF 质心，作为对照。
- 黄色：Link3～Link6 在当前姿态下的整体质心。

每个 Link 标注名称、质量，以及局部 xyz（显示单位 mm）。没有质量数据的 `controller_mount` 不显示质心。紫色 link3 标签 `EQUIVALENT CORRECTION` 表示等效校正，并非单独测得的真实 link3 质心。

质心标记附着在实时模型 `target/<link>` 上。已有橙色/绿色关节中心和轴标记、绿色零位快照继续可用。可以在质心显示的 Namespaces 中取消 `link_com_original` 或 `rear_assembly_com`，减少遮挡。

## 编译和启动

在 Ubuntu 虚拟机执行：

```bash
cd ~/minipc/New_Custom_Controller
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select custom_controller_usb
source install/setup.bash
```

本次已后台打开实时预览：串口 `/dev/ttyACM0`，比例 `1.0`，读取电机位置、显示质心和计算重力，`gravity_dry_run:=true` 表示暂不发送力矩。检查结果约 96 帧/秒，六轴有效掩码 `63`，20 个球/文字标记均正常发布。

重启前停止这次后台预览，避免两个接收节点竞争同一串口：

```bash
python3 tools/stop_identified_preview.py
```

只看实时模型、质心和重力计算：

```bash
ros2 launch custom_controller_usb identified_gravity.launch.py \
  port:=/dev/ttyACM0 torque_scale:=1.0 gravity_dry_run:=true
```

正式下发 USB 力矩：

```bash
ros2 launch custom_controller_usb identified_gravity.launch.py \
  port:=/dev/ttyACM0 torque_scale:=0.75 gravity_dry_run:=false
```

`torque_scale` 是统一补偿比例；改成 `1.0` 即全比例。需要结束上一条 launch 后重启，当前接收节点不支持动态更新此比例。

板端 `App/manual_mit_hold.h` 已将 `MANUAL_MIT_HOLD_ENABLED` 改为 `0U` 并编译通过，重新烧录后默认关闭手动保持。旧固件仍在运行时，可在 CubeMonitor 将 `g_manual_mit_hold_enabled` 改为 `0`。手动保持模式用于静态测量，优先级高于 USB 力矩，且把前馈清零。要实际试补偿，先支撑住机构，关闭手动保持并确保未处于自动采样模式，然后启动上面的非 dry-run 命令。USB 力矩帧会打开既有 HostTorque 接收模式。此时退出位置保持，板端 KP/KD 向零过渡，由主机力矩提供重力补偿。本次尚未烧录固件。

切回原 URDF 对照，结束当前 launch 后执行：

```bash
ros2 launch custom_controller_usb identified_gravity.launch.py \
  urdf_path:="$(ros2 pkg prefix --share custom_controller_usb)/config/controller_model_urdf2/controller.urdf" \
  torque_scale:=0.75 gravity_dry_run:=false
```

## 修改入口和验证

偏置/正负比例：`src/custom_controller_usb/config/controller_mapping_urdf2.yaml`。本次保留虚拟机最新保存的零位，同步回 Windows；J2 的位置比例为 `-1.0`，力矩反向仍保留。后续改变零位会改变拟合使用的角度，需核对或重新辨识，不能只改显示一侧。

候选质心：`controller_identified.urdf` 中各 Link 的 `inertial/origin xyz`。修改这里后重启，显示和计算同时生效，不要修改 `joint/origin` 来调整质心。

生成脚本：`tools/export_identified_urdf.py`，输入原模型和拟合 JSON，可重建候选 URDF。

校准更新脚本：`tools/refresh_centroid_calibration.py` 只允许 axis_order、scales、torque_signs 不变时平移合并姿态的 offset；修改这些映射需从原始窗口重新分组。

本次完成：13 项相关测试；15 个姿态的导出模型与拟合纯重力预测最大差约 `4.44e-16 N·m`；独立质心势能梯度与 Pinocchio 重力最大差约 `3.76e-10 N·m`。这些验证计算和显示的一致性，实机支撑效果需正式切换后观察。

本次新零位拟合结果在 Windows：`Custom_Controller_Gravity_Compensation/data/manual_static/20261009_controller_大范围/deployment_current_zero/`；虚拟机对应 `/tmp/static_centroid_wide_20261009/current_zero_fit/`。源码改动前备份在工程 `backups/before_centroid_markers_20261009/`。
