# controller2 质心模型接入与 RViz 显示

## 当前模型

2026-10-09 接入 `controller2.csv` 的各 Link Huber＋加权岭回归候选。原始 385 个稳定窗口，剔除第 89～91 条后使用 382 个窗口；J2 编码值 ≤0.05 rad 的受支撑区域仅屏蔽 J2 力矩方程。CMA-ES 没有进一步改善。

- 当前候选：`src/custom_controller_usb/config/controller_model_urdf2/controller_identified.urdf`。
- 分析记录：同目录 `controller_identified_fit.json`。
- 原始参考：同目录 `controller.urdf`。
- RViz、robot_state_publisher 和 Pinocchio 共用当前候选 URDF。
- 编码映射、零位和力矩方向：`src/custom_controller_usb/config/controller_mapping_urdf2.yaml`，与拟合输入一致。J2 比例 -1，不除以 4。

五折姿态族验证 RMSE（N·m）：J2 0.1655、J3 0.1010、J4 0.0562、J5 0.0831、J6 0.0363。这是留出数据误差，并非实机效果保证。15 个质心一阶矩参数仅 10 个独立组合可观测，各 Link 质心是约束候选，不能视为唯一真实质心。

| Link | 质心 x / y / z（m，Link 局部坐标） |
| --- | --- |
| link2 | -0.069876 / 0.028500 / -0.028001 |
| link3 | -0.008564 / -0.059628 / -0.037761 |
| link4 | 0.048478 / 0.003379 / 0.019249 |
| link5 | 0.072199 / 0.008808 / 0.053313 |
| link6 | 0.013047 / -0.036160 / -0.001413 |

## RViz 质心标记

`Link Centers of Mass` 话题为 `/custom_controller/center_of_mass_markers`：紫色是 link2～link6 当前候选，蓝色是原 URDF 质心，绿色是未修改的质心，黄色是当前姿态的 Link3～6 整体质心。标记附着在实时 Link 上，文字显示质量和局部质心坐标。关节零位快照继续共用既有 offset。

## 编译与运行

```bash
cd ~/minipc/New_Custom_Controller
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select custom_controller_usb
source install/setup.bash
```

先停止已有后台实例，避免竞争 USB：

```bash
python3 tools/stop_identified_preview.py
```

启动 RViz 和实际 USB 力矩输出，沿用比例 1.0：

```bash
ros2 launch custom_controller_usb identified_gravity.launch.py \
  port:=/dev/ttyACM0 torque_scale:=1.0 gravity_dry_run:=false
```

修改统一补偿比例：重启时将 `torque_scale:=1.0` 改为所需值，例如 `torque_scale:=0.75`。仅计算和显示时用 `gravity_dry_run:=true`。

2026-10-10：统一倍率及六轴倍率全部恢复为 1.0。单轴配置位于 `src/custom_controller_usb/config/controller_mapping_urdf2.yaml` 的 `torque_axis_scales: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]`，按电机 J1～J6 排列；例如第二项改0.8就是J2补偿乘0.8。USB下发力矩 = 模型重力力矩 × 方向符号 × 统一倍率 × 对应轴倍率。该参数不改变关节角映射，也不作为质心辨识时的测量方向转换；修改后重启生效。


实际输出要求 USB 连接到 Ubuntu 虚拟机，板端关闭手动 MIT 保持和自动采样。当前板端默认 `MANUAL_MIT_HOLD_ENABLED=0U`；旧固件可通过 CubeMonitor 将 `g_manual_mit_hold_enabled` 置 0。

当前后台实例 PID 文件 `/tmp/controller_identified_preview.pid`；日志路径文件 `/tmp/controller_identified_preview.logpath`。查看输出：

```bash
tail -f "$(cat /tmp/controller_identified_preview.logpath)"
```

## 回退与追溯

接入前文件保存在 `backups/before_controller2_robust_20261009/`，Windows 和虚拟机各有备份。

结束当前实例后，使用原 URDF 对照：

```bash
ros2 launch custom_controller_usb identified_gravity.launch.py \
  urdf_path:="$(ros2 pkg prefix --share custom_controller_usb)/config/controller_model_urdf2/controller.urdf" \
  port:=/dev/ttyACM0 torque_scale:=1.0 gravity_dry_run:=false
```

完整分析在 Windows `Custom_Controller_Gravity_Compensation/results/controller2_robust_20261009/最终拟合报告.md`；虚拟机在 `/home/badder/minipc/centroid_identification/20261009_controller2/`。`controller_identified_fit.json` 是拟合分析记录，其中运行状态描述对应导出时刻，当前接入状态以本说明和运行节点为准。
