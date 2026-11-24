#!/usr/bin/env python3
"""
重力补偿控制器 — Gazebo 物理仿真验证

功能:
  订阅 /joint_states → 用 Pinocchio 计算 G(q) → 发布 effort 命令到
  /gravity_compensation_controller/commands (forward_command_controller)

验证流程:
  1. 用 joint_trajectory_controller 将机械臂移到目标姿态
  2. 停止 position 控制器: ros2 control set_state gravity_compensation_controller active
  3. 启用重力补偿: ros2 service call /gravity_compensation/enable std_srvs/srv/SetBool "{data: true}"
  4. 在 Gazebo 中观察机械臂是否保持姿态不下坠

用法:
  终端1: ros2 launch new_hj_arm2_moveit_config gazebo_gravity.launch.py
  终端2: ros2 service call /gravity_compensation/enable std_srvs/srv/SetBool "{data: true}"
"""

import sys
import os
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool

import pinocchio

# ── 关节顺序（与 forward_command_controller 中配置一致）──
JOINT_ORDER = ["J1", "J2", "J3", "J4", "J5", "J6"]


def load_model(package_name: str = "new_hj_arm2"):
    """加载 Pinocchio 模型"""
    try:
        from ament_index_python import get_package_share_directory
        urdf_path = os.path.join(
            get_package_share_directory(package_name),
            "urdf", f"{package_name}.urdf"
        )
    except Exception:
        import subprocess
        r = subprocess.run(
            ["ros2", "pkg", "prefix", package_name],
            capture_output=True, text=True
        )
        urdf_path = os.path.join(
            r.stdout.strip(), "share", package_name,
            "urdf", f"{package_name}.urdf"
        )

    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"URDF 文件不存在: {urdf_path}")

    model = pinocchio.buildModelFromUrdf(urdf_path)
    return model


class GravityCompensationController(Node):
    """重力补偿力矩控制器"""

    def __init__(self):
        super().__init__("gravity_compensation_controller")

        # 参数
        self.declare_parameter("model_package", "new_hj_arm2")
        self.declare_parameter("update_rate", 100.0)

        model_pkg = self.get_parameter("model_package").value
        update_rate = self.get_parameter("update_rate").value

        # 加载模型
        self.get_logger().info(f"加载模型: {model_pkg}")
        self.model = load_model(model_pkg)
        self.data = self.model.createData()
        self.get_logger().info(f"模型就绪: {self.model.name} | DOF={self.model.nq}")

        # 建立关节名 → Pinocchio 索引映射
        self._build_joint_map()

        # 当前关节位置
        self.q_current = np.zeros(self.model.nq)
        self.q_received = False

        # 重力补偿开关
        self.compensation_enabled = False
        self.compensation_count = 0

        # ── 订阅 /joint_states ──
        self.joint_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10,
        )

        # ── 发布 effort 命令 ──
        self.effort_pub = self.create_publisher(
            Float64MultiArray,
            "/gravity_compensation_controller/commands",
            10,
        )

        # ── 开关服务 ──
        self.enable_srv = self.create_service(
            SetBool,
            "/gravity_compensation/enable",
            self.enable_callback,
        )

        # ── 定时器: 定期计算并发布重力补偿力矩 ──
        self.timer = self.create_timer(1.0 / update_rate, self.control_loop)

        # ── 状态定时器 ──
        self.status_timer = self.create_timer(2.0, self.print_status)

        self.get_logger().info("重力补偿控制器已启动")
        self.get_logger().info(f"服务: /gravity_compensation/enable (std_srvs/SetBool)")
        self.get_logger().info(f"发布: /gravity_compensation_controller/commands")
        self.get_logger().info("等待关节状态...")

    def _build_joint_map(self):
        """建立关节名 → 速度索引映射"""
        self.joint_to_idx_v = {}
        for jid in range(1, self.model.njoints):
            idx_v = self.model.joints[jid].idx_v
            if idx_v >= 0:
                name = self.model.names[jid]
                self.joint_to_idx_v[name] = idx_v

    def joint_state_callback(self, msg: JointState):
        """接收关节状态"""
        if not self.q_received:
            self.get_logger().info("已收到关节状态数据")
            self.q_received = True

        for name, pos in zip(msg.name, msg.position):
            if name in self.joint_to_idx_v:
                idx_v = self.joint_to_idx_v[name]
                self.q_current[idx_v] = pos

    def enable_callback(self, request, response):
        """启用/禁用重力补偿"""
        self.compensation_enabled = request.data
        state = "启用" if self.compensation_enabled else "禁用"
        self.get_logger().info(f"重力补偿已{state}")
        response.success = True
        response.message = f"重力补偿已{state}"
        return response

    def control_loop(self):
        """主控制循环: 计算 G(q) 并发布"""
        if not self.q_received:
            return

        # 计算重力补偿力矩
        tau = pinocchio.computeGeneralizedGravity(
            self.model, self.data, self.q_current
        )

        # 如果启用补偿，发布力矩；否则发布零力矩
        if self.compensation_enabled:
            effort = tau.tolist()
            self.compensation_count += 1
        else:
            effort = [0.0] * self.model.nv

        msg = Float64MultiArray()
        msg.data = effort
        self.effort_pub.publish(msg)

    def print_status(self):
        """定期打印状态"""
        if not self.q_received:
            return

        tau = pinocchio.computeGeneralizedGravity(
            self.model, self.data, self.q_current
        )

        q_deg = np.rad2deg(self.q_current)
        status = "● 启用" if self.compensation_enabled else "○ 禁用"

        # 简化输出
        parts = [f"{name}={tau[i]:+.2f}" for i, name in enumerate(JOINT_ORDER)]
        tau_str = " | ".join(parts)
        total = np.sum(np.abs(tau))

        self.get_logger().info(
            f"[{status}] 重力补偿力矩 (Nm): {tau_str}  | 合计|τ|={total:.2f}"
        )


def main():
    rclpy.init()
    node = GravityCompensationController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[INFO] 重力补偿控制器已停止")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
