#!/usr/bin/env python3
"""
URDF 重力补偿标定 — 数据采集工具。

对每个用户指定的姿态，用 5 点漂移速率 + 线性回归找到 J2/J3
的平衡 tau_offset，克服静摩擦死区对二分搜索的干扰。

原理:
  测量不同 offset 下关节的漂移速率 v (rad/s)，线性拟合 v = a·offset + b，
  过零点 (-b/a) 即为真正的平衡 offset。

用法:
  python3 calibrate_collect.py --poses 15 --step 1.0

前提:
  ros2 run arm_driver serial_dm_feedforward_bridge.py  必须已在运行
"""

import time
import json
import argparse
import subprocess
import sys
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINT_NAMES = ["J1", "J2", "J3", "J4", "J5", "J6"]
TARGET_NODE = "arm_serial_feedforward_node"

# ── 工具函数 ────────────────────────────────────────────────────────


def compute_r_squared(x, y, slope, intercept):
    """线性拟合的 R² 决定系数"""
    y_pred = [slope * xi + intercept for xi in x]
    ss_res = sum((yi - yp) ** 2 for yi, yp in zip(y, y_pred))
    ss_tot = sum((yi - np.mean(y)) ** 2 for yi in y)
    if ss_tot < 1e-12:
        return 0.0
    return 1.0 - ss_res / ss_tot


def ros2_param_set(node_name: str, param_name: str, value: float):
    """通过 subprocess 调用 ros2 param set"""
    subprocess.run(
        ["ros2", "param", "set", node_name, param_name, str(value)],
        capture_output=True,
        timeout=5.0,
    )


def ros2_param_get(node_name: str, param_name: str) -> float:
    """通过 subprocess 调用 ros2 param get，返回浮点值"""
    try:
        result = subprocess.run(
            ["ros2", "param", "get", node_name, param_name],
            capture_output=True,
            text=True,
            timeout=5.0,
        )
        return float(result.stdout.strip())
    except (ValueError, AttributeError, subprocess.TimeoutExpired):
        return 0.0


# ── 标定采集节点 ────────────────────────────────────────────────────


class CalibrationCollector(Node):
    """订阅 real_positions，通过 ros2 param 控制 bridge 的 offset 参数"""

    def __init__(self, step: float = 1.0):
        super().__init__("calibrate_collector")
        self._step = step
        self._latest_positions: list[float] | None = None

        self._pos_sub = self.create_subscription(
            JointState, "real_positions", self._pos_cb, 10
        )

        self.get_logger().info("标定采集器已启动")

    def _pos_cb(self, msg: JointState):
        self._latest_positions = list(msg.position)

    def wait_for_positions(self, timeout: float = 5.0) -> bool:
        """等待收到第一帧 real_positions 数据"""
        t0 = time.time()
        while self._latest_positions is None:
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.time() - t0 > timeout:
                self.get_logger().error(
                    "无法收到 real_positions，请确认 bridge 节点在运行"
                )
                return False
        return True

    def _spin_fresh(self, n: int = 5):
        """spin 几次以获取最新数据"""
        for _ in range(n):
            rclpy.spin_once(self, timeout_sec=0.01)

    def get_joint_angle(self, joint_index: int) -> float:
        """读取单个关节当前角度 (rad)

        Args:
            joint_index: 0-based (0=J1, …, 5=J6)
        """
        self._spin_fresh()
        if self._latest_positions is None or joint_index >= len(self._latest_positions):
            return 0.0
        return self._latest_positions[joint_index]

    def get_all_angles(self) -> list[float]:
        """读取全部 6 个关节当前角度"""
        self._spin_fresh()
        if self._latest_positions is None:
            return [0.0] * 6
        return list(self._latest_positions[:6])

    # ── offset 读写 ──────────────────────────────────────────────

    def set_offset(self, joint_index: int, value: float):
        """设置 bridge 的 j{joint_index+1}_tau_offset 参数"""
        param_name = f"j{joint_index + 1}_tau_offset"
        ros2_param_set(TARGET_NODE, param_name, value)

    def get_offset(self, joint_index: int) -> float:
        """读取 bridge 当前的 offset 值"""
        param_name = f"j{joint_index + 1}_tau_offset"
        return ros2_param_get(TARGET_NODE, param_name)

    # ── 核心: 线性拟合搜索平衡点 ──────────────────────────────────

    def find_balance_offset(self, joint_index: int) -> tuple[float, float]:
        """5 点漂移速率测量 + 线性拟合 → 返回 (平衡offset, R²)

        Args:
            joint_index: 0-based (1=J2, 2=J3)
        """
        jn = JOINT_NAMES[joint_index]
        self.get_logger().info(f"━━━ 搜索 {jn} 平衡点 ━━━")

        offset_0 = self.get_offset(joint_index)
        s = self._step

        # 5 个试探点: offset_0 ± 2s, ± s, offset_0
        trial_offsets = [
            offset_0 - 2 * s,
            offset_0 - s,
            offset_0,
            offset_0 + s,
            offset_0 + 2 * s,
        ]

        drift_rates: list[float] = []
        for i, off in enumerate(trial_offsets):
            self.set_offset(joint_index, off)
            self.get_logger().info(f"  [{i+1}/5] offset = {off:+.2f} Nm  -- 稳定中")
            time.sleep(2.0)

            q1 = self.get_joint_angle(joint_index)
            time.sleep(3.0)
            q2 = self.get_joint_angle(joint_index)

            v = (q2 - q1) / 3.0
            drift_rates.append(v)
            self.get_logger().info(f"          v = {v:+.6f} rad/s")

        # 线性拟合: v = a * offset + b
        a, b = np.polyfit(trial_offsets, drift_rates, 1)
        r2 = compute_r_squared(trial_offsets, drift_rates, a, b)

        if abs(a) < 1e-12:
            self.get_logger().warn(f"  斜率 a ≈ 0，拟合无效，沿用原值")
            balance = offset_0
        else:
            balance = float(-b / a)

        # 验证: 用算出的 balance 观测 5s
        self.set_offset(joint_index, balance)
        self.get_logger().info(f"  balance = {balance:+.3f} Nm  验证 5s ...")
        time.sleep(2.0)
        q1 = self.get_joint_angle(joint_index)
        time.sleep(5.0)
        q2 = self.get_joint_angle(joint_index)
        delta = abs(q2 - q1)

        if delta <= 0.001:
            self.get_logger().info(f"  ✓ 通过  Δ = {delta:.4f} rad")
        else:
            self.get_logger().warn(f"  ⚠ 偏差 Δ = {delta:.4f} rad (> 0.001)")

        self.get_logger().info(
            f"  拟合: a={a:.6f}  b={b:.6f}  R²={r2:.4f}"
        )
        return balance, r2


# ── 主流程 ──────────────────────────────────────────────────────────


def main():
    parser = argparse.ArgumentParser(description="URDF 重力补偿标定数据采集")
    parser.add_argument(
        "--poses", type=int, default=15,
        help="采集姿态数量 (默认 15)"
    )
    parser.add_argument(
        "--step", type=float, default=1.0,
        help="offset 试探步长 Nm (默认 1.0)"
    )
    parser.add_argument(
        "--output", type=str, default="calibration_data.json",
        help="输出 JSON 文件路径"
    )
    args = parser.parse_args()

    rclpy.init()
    node = CalibrationCollector(step=args.step)

    print()
    print("=" * 60)
    print("  URDF 重力补偿标定 — 数据采集")
    print("=" * 60)
    print()
    print("  1. 手动把机械臂摆到某个姿态（建议覆盖 J2/J3 运动范围）")
    print("  2. 按回车 → 自动搜索 J2/J3 平衡 offset")
    print("  3. 重复直到收集足够姿态")
    print()
    print(f"  目标姿态数: {args.poses}")
    print(f"  试探步长:   {args.step} Nm")
    print("=" * 60)

    if not node.wait_for_positions():
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    data: list[dict] = []

    for i in range(args.poses):
        print()
        print("=" * 60)
        input(f">>> [{i+1}/{args.poses}] 摆好姿态后按回车...")
        print()

        # 读取当前姿态
        q = node.get_all_angles()
        q_str = "  ".join(f"{JOINT_NAMES[j]}: {q[j]:+8.4f}" for j in range(6))
        print(f"  姿态: [{q_str}]")
        print()

        # 搜 J2 平衡点
        j2_bal, j2_r2 = node.find_balance_offset(joint_index=1)
        print()

        # 搜 J3 平衡点 (此时 J2 已用平衡 offset)
        j3_bal, j3_r2 = node.find_balance_offset(joint_index=2)
        print()

        entry = {
            "q": q,
            "tau_j2": j2_bal,
            "tau_j3": j3_bal,
            "r2_j2": j2_r2,
            "r2_j3": j3_r2,
        }
        data.append(entry)

        print(f"  → J2: balance={j2_bal:+.3f} Nm  R²={j2_r2:.4f}")
        print(f"  → J3: balance={j3_bal:+.3f} Nm  R²={j3_r2:.4f}")
        if j2_r2 < 0.85:
            print(f"     ⚠ J2 R² 偏低，考虑重试此姿态")
        if j3_r2 < 0.85:
            print(f"     ⚠ J3 R² 偏低，考虑重试此姿态")

    # 保存
    with open(args.output, "w") as f:
        json.dump(data, f, indent=2)

    print()
    print("=" * 60)
    print(f"  ✓ 已保存 {len(data)} 组数据到 {args.output}")
    print("=" * 60)

    # 统计 R² 分布
    r2_j2_vals = [d["r2_j2"] for d in data]
    r2_j3_vals = [d["r2_j3"] for d in data]
    print(f"  J2 R²: mean={np.mean(r2_j2_vals):.4f}  min={np.min(r2_j2_vals):.4f}")
    print(f"  J3 R²: mean={np.mean(r2_j3_vals):.4f}  min={np.min(r2_j3_vals):.4f}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
