#!/usr/bin/env python3
"""
MoveIt2 + 重力补偿 联动脚本

在 RViz 中拖动机械臂，点击 Plan 规划轨迹后，
本脚本自动捕获轨迹并输出重力补偿力矩。

用法:
  终端1: ros2 launch new_hj_arm2_moveit_config demo.launch.py
  终端2: python3 gravity_monitor.py
  然后在 RViz 中拖动目标位置 → Plan → 本脚本自动输出力矩
"""

import sys
import os
import numpy as np

# ── ROS 2 ──────────────────────────────────────
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import DisplayTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

# ── Pinocchio ──────────────────────────────────
import pinocchio

# ── 模型加载 (复用现有逻辑) ────────────────────

def load_model():
    # 第1层: ament_index (需要 source install/setup.bash)
    try:
        from ament_index_python import get_package_share_directory
        path = os.path.join(get_package_share_directory("new_hj_arm2"), "urdf", "new_hj_arm2.urdf")
        if os.path.exists(path):
            return pinocchio.buildModelFromUrdf(path)
    except Exception:
        pass

    # 第2层: ros2 pkg prefix
    try:
        import subprocess
        r = subprocess.run(["ros2", "pkg", "prefix", "new_hj_arm2"],
                         capture_output=True, text=True, timeout=5)
        prefix = r.stdout.strip()
        if prefix:
            path = os.path.join(prefix, "share", "new_hj_arm2", "urdf", "new_hj_arm2.urdf")
            if os.path.exists(path):
                return pinocchio.buildModelFromUrdf(path)
    except Exception:
        pass

    # 第3层: 相对路径回退 (从脚本所在目录向上找)
    d = os.path.dirname(os.path.abspath(__file__))
    path = os.path.normpath(os.path.join(d, "..", "..", "new_hj_arm2", "urdf", "new_hj_arm2.urdf"))
    if os.path.exists(path):
        return pinocchio.buildModelFromUrdf(path)

    raise FileNotFoundError(
        "无法找到 new_hj_arm2 URDF 文件。请先执行: source install/setup.bash\n"
        f"搜索路径: {path}"
    )


# ── 重力补偿节点 ───────────────────────────────

JOINT_ORDER = ["J1", "J2", "J3", "J4", "J5", "J6"]


class GravityMonitor(Node):
    """监听 MoveIt2 规划轨迹，计算重力补偿力矩"""

    def __init__(self):
        super().__init__("gravity_monitor")

        # 加载 Pinocchio 模型
        self.model = load_model()
        self.get_logger().info(f"Model loaded: {self.model.name} ({self.model.nq} DOF)")

        # 订阅 /display_planned_path (MoveIt2 规划完成后发布)
        self.sub = self.create_subscription(
            DisplayTrajectory,
            "/display_planned_path",
            self.trajectory_callback,
            10,
        )
        self.get_logger().info("Listening for planned trajectories...")
        self.get_logger().info("Drag the arm in RViz, click 'Plan', and watch here.")
        self.get_logger().info("-" * 50)

        self.count = 0

    def trajectory_callback(self, msg: DisplayTrajectory):
        """收到新的规划轨迹"""
        if not msg.trajectory:
            return

        self.count += 1

        for tidx, traj in enumerate(msg.trajectory):
            if not traj.joint_trajectory.points:
                continue

            joint_names = traj.joint_trajectory.joint_names
            points = traj.joint_trajectory.points

            if len(points) < 2:
                continue

            # 建立关节名 → 索引映射
            name_to_idx = {n: i for i, n in enumerate(joint_names)}
            pin_idx = [name_to_idx.get(jn) for jn in JOINT_ORDER]
            if None in pin_idx:
                self.get_logger().warn(f"Joint mismatch: {joint_names}")
                continue

            # 提取关节轨迹
            N = len(points)
            q = np.zeros((N, self.model.nq))
            times = np.zeros(N)

            for i, pt in enumerate(points):
                for j, pj in enumerate(pin_idx):
                    if pj is not None and pj < len(pt.positions):
                        q[i, j] = pt.positions[pj]
                times[i] = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9

            # 计算重力补偿力矩
            tau = np.zeros((N, self.model.nq))
            data = self.model.createData()
            for i in range(N):
                tau[i] = pinocchio.computeGeneralizedGravity(self.model, data, q[i])

            # ── 输出 ──────────────────────────
            print(f"\n{'='*90}")
            print(f"  Trajectory #{self.count} — Gravity Compensation Torques")
            print(f"{'='*90}")
            print(f"  Duration: {times[-1]:.2f}s | Waypoints: {N}")
            print()

            # 摘要
            print(f"  {'Joint':<8} {'τ_peak':>10} {'τ_rms':>10} {'τ_start':>10} {'τ_end':>10}")
            print(f"  {'':<8} {'(Nm)':>10} {'(Nm)':>10} {'(Nm)':>10} {'(Nm)':>10}")
            print("  " + "-" * 48)
            for j in range(self.model.nq):
                tj = tau[:, j]
                print(f"  {JOINT_ORDER[j]:<8} {np.max(np.abs(tj)):>10.3f} "
                      f"{np.sqrt(np.mean(tj**2)):>10.3f} {tj[0]:>10.3f} {tj[-1]:>10.3f}")
            total_peak = np.max(np.sum(np.abs(tau), axis=1))
            print("  " + "-" * 48)
            print(f"  {'Total':<8} {total_peak:>10.3f} Nm")

            # ── 逐路点表格（全部输出）──
            print(f"\n  {'t(s)':>6}  {'J1':>7}  {'J2':>7}  {'J3':>7}  {'J4':>7}  {'J5':>7}  {'J6':>7}  {'|τ|':>7}")
            print("  " + "-" * 66)
            for i in range(N):
                t = times[i]
                row = f"{t:>6.2f}  "
                row += "  ".join(f"{tau[i,j]:>7.3f}" for j in range(self.model.nq))
                total_i = np.sum(np.abs(tau[i]))
                row += f"  {total_i:>7.3f}"
                print(f"  {row}")

            # 能耗积分
            energy = np.trapz(np.sum(np.abs(tau), axis=1), times)
            print(f"\n  Energy ∫|τ|dt: {energy:.2f} Nm·s")
            print(f"{'='*90}\n")


def main():
    rclpy.init()
    node = GravityMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[INFO] 重力监视器已停止。")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
