#!/usr/bin/env python3
"""
在 Gazebo (Ignition) 中生成机械臂模型。

功能: 读取 xacro → 生成 URDF → 转换为 SDF → 通过 Gazebo Transport 生成

用法:
  python3 spawn_robot_gz.py /path/to/urdf.xacro use_gazebo:=true

替代 ros_gz_sim create，避免依赖 ros-humble-ros-gz-sim 的版本冲突。
"""

import sys
import os
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET


def urdf_to_sdf(urdf_str: str, model_name: str, pose: str = "0 0 0.5 0 0 0") -> str:
    """
    将 URDF 字符串转换为 Gazebo SDF 格式。

    基本转换: <robot> → <sdf><model>, <link> → <link>, <joint> → <joint>
    保留 <ros2_control> 和 <gazebo> 标签
    """
    # 解析 URDF
    urdf_root = ET.fromstring(urdf_str)

    # 创建 SDF 结构
    sdf = ET.Element("sdf", version="1.6")
    model = ET.SubElement(sdf, "model", name=model_name)

    # 添加 pose
    pose_elem = ET.SubElement(model, "pose")
    pose_elem.text = pose

    # 复制所有 link 元素
    for link in urdf_root.findall("link"):
        # 在 SDF 中 link name 是必须属性，直接复制即可
        model.append(link)

    # 复制所有 joint 元素
    for joint in urdf_root.findall("joint"):
        model.append(joint)

    # 复制 gazebo 标签
    for gazebo in urdf_root.findall("gazebo"):
        model.append(gazebo)

    # 复制 ros2_control 标签 (ign_ros2_control 插件会读取)
    for ros2_ctrl in urdf_root.findall("ros2_control"):
        model.append(ros2_ctrl)

    # 转换为字符串
    return ET.tostring(sdf, encoding="unicode")


def spawn_via_ign(sdf_content: str, world_name: str = "empty", timeout: int = 10):
    """
    通过 Gazebo Transport 生成模型。
    使用 ign service 命令行工具。
    """
    # 写入临时 SDF 文件
    sdf_path = f"/tmp/spawn_robot_{os.getpid()}.sdf"
    with open(sdf_path, "w") as f:
        f.write(sdf_content)

    # 构建 EntityFactory 请求
    # ign service 需要 protobuf 消息，我们用文件方式传入
    print(f"[spawn] 通过 ign transport 生成机械臂...")
    print(f"[spawn] SDF 文件: {sdf_path}")

    # 方法: 使用 Python ignition transport 绑定
    try:
        from ignition.msgs.entity_factory_pb2 import EntityFactory
        from ignition.transport import Node as IgnNode

        # 创建请求
        req = EntityFactory()
        req.sdf = sdf_content

        # 通过 transport 发送
        node = IgnNode()
        topic = f"/world/{world_name}/create"

        # 使用 request-response 模式
        response = node.request(topic, req, EntityFactory, timeout * 1000)
        if response[0]:
            print(f"[spawn] ✓ 机械臂已生成")
        else:
            print(f"[spawn] ✗ 生成失败: {response}")
    except ImportError:
        # 回退: 使用 ign service CLI
        print("[spawn] Python ignition bindings 不可用, 使用 CLI 方式...")
        _spawn_via_cli(sdf_path, world_name, timeout)


def _spawn_via_cli(sdf_path: str, world_name: str, timeout: int):
    """使用 ign service CLI 生成模型 (回退方案)"""
    # 读取 SDF 内容并转义
    with open(sdf_path) as f:
        sdf_escaped = f.read().replace('"', '\\"').replace('\n', '\\n')

    # 使用 ign service 命令
    cmd = [
        "ign", "service", "-s", f"/world/{world_name}/create",
        "--reqtype", "ignition.msgs.EntityFactory",
        "--reptype", "ignition.msgs.Boolean",
        "--timeout", str(timeout * 1000),
        "--req", f'sdf: "{sdf_escaped}"'
    ]

    print(f"[spawn] 执行: {' '.join(cmd[:6])}...")
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout + 5)

    if result.returncode == 0:
        print(f"[spawn] ✓ 机械臂生成成功")
    else:
        print(f"[spawn] ✗ 生成失败: {result.stderr}")
        print(f"[spawn] stdout: {result.stdout}")


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    urdf_xacro_path = sys.argv[1]
    xacro_args = sys.argv[2:] if len(sys.argv) > 2 else []

    # ── 步骤1: 运行 xacro 生成 URDF ──
    print(f"[spawn] 运行 xacro: {urdf_xacro_path}")
    cmd = ["xacro", urdf_xacro_path] + xacro_args
    result = subprocess.run(cmd, capture_output=True, text=True)

    if result.returncode != 0:
        print(f"[spawn] ✗ xacro 错误: {result.stderr}")
        sys.exit(1)

    urdf_content = result.stdout
    print(f"[spawn] URDF 生成成功 ({len(urdf_content)} bytes)")

    # ── 步骤2: URDF → SDF ──
    sdf_content = urdf_to_sdf(urdf_content, "new_hj_arm2")
    print(f"[spawn] SDF 转换完成 ({len(sdf_content)} bytes)")

    # ── 步骤3: 等待 Gazebo 就绪 ──
    print("[spawn] 等待 Gazebo 就绪...")
    time.sleep(2.0)

    # ── 步骤4: 生成模型 ──
    spawn_via_ign(sdf_content)


if __name__ == "__main__":
    main()
