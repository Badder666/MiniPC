#!/usr/bin/env python3
"""
机械臂重力补偿脚本 — new_hj_arm2
流程: 目标关节角度 → 轨迹规划(q) → G(q) → 重力补偿力矩序列

用法:
  python3 gravity_compensation.py                                    # 默认demo
  python3 gravity_compensation.py --q_static 0 0 0 0 0 0             # 单点静态力矩(零位/水平伸直)
  python3 gravity_compensation.py --q_static 0 137.5 -137.5 0 0 0     # 单点静态力矩(home姿态)
  python3 gravity_compensation.py --q_static 0 0 0 0 0 0 --manual     # 手动计算模式:展示质量/质心/力臂/力矩分解
  python3 gravity_compensation.py --q_target 0 45 -30 0 0 0           # 轨迹分析(度)
  python3 gravity_compensation.py --q_start 0 0 0 0 0 0 --q_target 0 45 -30 0 0 0 --duration 2.0

模型: new_hj_arm2 | 零位(全0) = 水平伸直 | home(J2=2.4,J3=-2.4) = 折叠收起
"""

import sys
import os
import numpy as np
from dataclasses import dataclass
from typing import Optional

try:
    import pinocchio
except ImportError:
    print("错误: 未找到 pinocchio。请先安装: sudo apt install ros-humble-pinocchio")
    sys.exit(1)

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    HAS_MPL = True
except ImportError:
    HAS_MPL = False


# ═══════════════════════════════════════════════════════════════
# 1. 模型加载
# ═══════════════════════════════════════════════════════════════

def find_urdf() -> str:
    """查找 new_hj_arm2 包的 URDF 文件"""
    try:
        from ament_index_python import get_package_share_directory
        path = os.path.join(get_package_share_directory("new_hj_arm2"), "urdf", "new_hj_arm2.urdf")
        if os.path.exists(path):
            return path
    except Exception:
        pass
    import subprocess
    try:
        r = subprocess.run(["ros2", "pkg", "prefix", "new_hj_arm2"], capture_output=True, text=True)
        path = os.path.join(r.stdout.strip(), "share", "new_hj_arm2", "urdf", "new_hj_arm2.urdf")
        if os.path.exists(path):
            return path
    except Exception:
        pass
    d = os.path.dirname(os.path.abspath(__file__))
    path = os.path.normpath(os.path.join(d, "..", "..", "new_hj_arm2", "urdf", "new_hj_arm2.urdf"))
    if os.path.exists(path):
        return path
    raise FileNotFoundError("无法找到 URDF 文件。请先 source install/setup.bash")


def load_model(urdf_path: str) -> pinocchio.Model:
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"URDF 文件不存在: {urdf_path}")
    model = pinocchio.buildModelFromUrdf(urdf_path)
    print(f"[模型] {model.name} | {model.nq} DOF | {model.njoints} joints")
    return model


# ═══════════════════════════════════════════════════════════════
# 2. 轨迹规划（五阶多项式）
# ═══════════════════════════════════════════════════════════════

@dataclass
class Trajectory:
    t: np.ndarray    # (N,)  时间
    q: np.ndarray    # (N, n) 位置
    v: np.ndarray    # (N, n) 速度
    a: np.ndarray    # (N, n) 加速度
    dt: float


def plan_trajectory(
    q_start: np.ndarray,
    q_target: np.ndarray,
    duration: float = 2.0,
    dt: float = 0.01,
) -> Trajectory:
    """
    五阶多项式轨迹: q(t) 在边界处速度、加速度为零

    q(t) = a0 + a1·t + a2·t² + a3·t³ + a4·t⁴ + a5·t⁵
    边界: v(0)=v(T)=0, a(0)=a(T)=0
    """
    n = len(q_start)
    T = duration
    N = int(T / dt) + 1
    t = np.linspace(0, T, N)

    q = np.zeros((N, n))
    v = np.zeros((N, n))
    a = np.zeros((N, n))

    for j in range(n):
        dq = q_target[j] - q_start[j]
        # a0=q_start, a1=0, a2=0
        # 解: a3, a4, a5 由 q(T), v(T), a(T) 确定
        A = np.array([
            [T**3,      T**4,       T**5],
            [3*T**2,    4*T**3,     5*T**4],
            [6*T,       12*T**2,    20*T**3],
        ])
        b = np.array([dq, 0.0, 0.0])
        a3, a4, a5 = np.linalg.solve(A, b)

        a0, a1, a2 = q_start[j], 0.0, 0.0
        for i in range(N):
            ti = t[i]
            q[i, j] = a0 + a1*ti + a2*ti**2 + a3*ti**3 + a4*ti**4 + a5*ti**5
            v[i, j] = a1 + 2*a2*ti + 3*a3*ti**2 + 4*a4*ti**3 + 5*a5*ti**4
            a[i, j] = 2*a2 + 6*a3*ti + 12*a4*ti**2 + 20*a5*ti**3

    return Trajectory(t=t, q=q, v=v, a=a, dt=dt)


# ═══════════════════════════════════════════════════════════════
# 3. 重力补偿计算
# ═══════════════════════════════════════════════════════════════

def compute_gravity_compensation(
    model: pinocchio.Model,
    traj: Trajectory,
) -> np.ndarray:
    """
    沿轨迹逐点计算重力补偿力矩 τ = G(q)

    返回: tau (N, nv) 力矩序列
    """
    N = len(traj.t)
    nv = model.nv
    tau = np.zeros((N, nv))
    data = model.createData()

    for i in range(N):
        tau[i] = pinocchio.computeGeneralizedGravity(model, data, traj.q[i])

    return tau


def gravity_at_point(model: pinocchio.Model, q: np.ndarray) -> np.ndarray:
    """单点重力力矩 G(q) — Pinocchio 内置算法"""
    data = model.createData()
    return pinocchio.computeGeneralizedGravity(model, data, q)


# ═══════════════════════════════════════════════════════════════
# 3b. 手动重力补偿计算（使用质量+质心+力臂）
# ═══════════════════════════════════════════════════════════════

def _is_descendant(model, link_id, ancestor_id):
    """检查 link_id 是否在 ancestor_id 的子树中（包含自身: link_id == ancestor_id → True）"""
    if link_id == ancestor_id:
        return True
    current = link_id
    while current > 0:
        parent = model.parents[current]
        if parent == ancestor_id:
            return True
        if parent == 0:
            return False
        current = parent
    return False


def compute_gravity_manual(model, q, verbose=True):
    """
    手动计算重力补偿力矩 — 只使用每个连杆的质量和质心位置。

    物理公式:
      τ_j = Σ_{i in subtree(j)} axis_j · [(p_i - p_j) × (m_i · g)]

    其中:
      - p_i   = 连杆 i 质心在世界坐标系中的位置
      - p_j   = 关节 j 在世界坐标系中的位置（在旋转轴上）
      - axis_j = 关节 j 在世界坐标系中的旋转轴方向
      - m_i   = 连杆 i 的质量
      - g     = [0, 0, -9.81] m/s²

    数据来源: URDF 中的 <mass>, <inertial><origin> (质心), <axis> (关节轴)
    计算步骤: FK 变换质心→世界坐标 → 力臂×重力 → 投影到关节轴
    """
    data = model.createData()
    pinocchio.forwardKinematics(model, data, q)
    pinocchio.updateFramePlacements(model, data)

    g = np.array([0.0, 0.0, -9.81])

    # ── 从 URDF 解析关节轴（在关节自身坐标系中）──
    # Pinocchio 不直接暴露 axis，这里用数值方法: 给关节 j 一个小扰动 ε，
    # 对比旋转前后一个测试向量的变化来提取旋转轴
    def _get_joint_axis_world(joint_id, data, model):
        """数值法提取关节轴在世界坐标系中的方向（对 1-DOF 转动关节）"""
        # 用 FK 微扰: q + ε*e_j → 看 oMi[joint_id] 的旋转变化
        eps = 1e-6
        q_pert = q.copy()
        idx_q = model.joints[joint_id].idx_q
        q_pert[idx_q] += eps

        data2 = model.createData()
        pinocchio.forwardKinematics(model, data2, q_pert)

        # 旋转变化: R_pert = R_orig * R_delta  →  R_delta = R_orig^T * R_pert
        R_orig = data.oMi[joint_id].rotation
        R_pert = data2.oMi[joint_id].rotation
        R_delta = R_orig.T @ R_pert

        # 从旋转矩阵提取旋转轴 (小角度近似)
        # axis = (R - R^T)^∨ / (2*sin(ε))  ≈ 反对称分量 / (2·ε)
        skew = (R_delta - R_delta.T) / (2.0 * eps)
        axis_local = np.array([skew[2, 1], skew[0, 2], skew[1, 0]])

        # 转换到世界坐标系
        axis_world = R_orig @ axis_local
        nrm = np.linalg.norm(axis_world)
        if nrm > 1e-10:
            axis_world /= nrm
        return axis_world

    if verbose:
        print(f"\n{'='*70}")
        print(f"  手动重力补偿 — 逐步计算")
        print(f"{'='*70}")
        print(f"  重力向量 g = [0, 0, -9.81] m/s²")
        print()
        print(f"  {'连杆':<10} {'质量':>8}  {'质心在连杆坐标系':>30}  {'质心在世界坐标系':>30}  {'重力(N)':>18}")
        print(f"  {'':10} {'(kg)':>8}  {'x':>8} {'y':>8} {'z':>8}  {'x':>8} {'y':>8} {'z':>8}  {'Fz':>10}")
        print("  " + "-" * 80)

    # ── 第1步: 计算每个连杆的质心世界坐标和重力 ──
    link_data = {}
    for link_id in range(1, model.njoints):
        mass = model.inertias[link_id].mass
        com_local = model.inertias[link_id].lever          # 质心在连杆自身坐标系
        com_world = data.oMi[link_id].act(com_local)       # 变换到世界坐标系
        force_w = mass * g                                  # 世界坐标系下的重力

        link_data[link_id] = {
            "mass": mass,
            "com_local": com_local.copy(),
            "com_world": com_world.copy(),
            "force_w": force_w.copy(),
        }

        if verbose:
            print(f"  {model.names[link_id]:<10} {mass:>8.4f}"
                  f"  {com_local[0]:>8.4f} {com_local[1]:>8.4f} {com_local[2]:>8.4f}"
                  f"  {com_world[0]:>8.4f} {com_world[1]:>8.4f} {com_world[2]:>8.4f}"
                  f"  {force_w[2]:>10.4f}")

    # ── 第2步: 对每个活动关节，计算子连杆重力产生的力矩 ──
    if verbose:
        print(f"\n  {'关节':<8} {'手动τ(Nm)':>12}  =  各连杆力矩贡献")
        print("  " + "-" * 60)

    tau = np.zeros(model.nv)

    for joint_id in range(1, model.njoints):
        idx_v = model.joints[joint_id].idx_v
        if idx_v < 0:   # 固定关节，跳过
            continue

        # 关节在世界坐标系中的位置（在旋转轴上）
        joint_pos_w = data.oMi[joint_id].translation.copy()

        # 关节旋转轴在世界坐标系中的方向（数值法从 FK 微扰提取）
        axis_w = _get_joint_axis_world(joint_id, data, model)

        torque_j = 0.0
        parts = []

        for link_id in range(1, model.njoints):
            if not _is_descendant(model, link_id, joint_id):
                continue

            ld = link_data[link_id]
            r = ld["com_world"] - joint_pos_w        # 力臂: 关节→质心
            moment = np.cross(r, ld["force_w"])       # 力矩: r × F
            contrib = np.dot(moment, axis_w)          # 投影到关节轴
            torque_j += contrib
            parts.append(f"{contrib:+.4f}({model.names[link_id]})")

        tau[idx_v] = -torque_j   # Pinocchio 约定: τ>0 表示抵抗重力

        if verbose:
            joint_name = model.names[joint_id] if model.names[joint_id] != "Joint" else f"J{idx_v+1}"
            detail = " + ".join(parts)
            print(f"  {joint_name:<8} {tau[idx_v]:>12.6f}  = {detail}")

    # ── 第3步: 与 Pinocchio 对比 ──
    if verbose:
        G_pin = gravity_at_point(model, q)
        print(f"\n  {'关节':<8} {'手动 τ (Nm)':>14}  {'Pinocchio (Nm)':>16}  {'差异':>12}")
        print("  " + "-" * 50)
        for j in range(model.nv):
            diff = tau[j] - G_pin[j]
            status = "✓" if abs(diff) < 1e-6 else f"✗ Δ={diff:.4e}"
            print(f"  {JOINT_NAMES[j]:<8} {tau[j]:>14.6f}  {G_pin[j]:>16.6f}  {status:>12}")

    return tau


# ═══════════════════════════════════════════════════════════════
# 4. 输出
# ═══════════════════════════════════════════════════════════════

JOINT_NAMES = ["J1", "J2", "J3", "J4", "J5", "J6"]


def print_summary(traj: Trajectory, tau: np.ndarray):
    """打印轨迹重力补偿摘要"""
    n = tau.shape[1]

    print(f"\n{'='*70}")
    print(f"  重力补偿力矩 — G(q) 沿轨迹")
    print(f"{'='*70}")
    print(f"  时长: {traj.t[-1]:.1f}s | 步数: {len(traj.t)} | dt: {traj.dt:.3f}s")
    print()
    print(f"  {'关节':<8} {'τ_peak':>10} {'τ_rms':>10} {'τ_start':>10} {'τ_end':>10}")
    print(f"  {'':<8} {'(Nm)':>10} {'(Nm)':>10} {'(Nm)':>10} {'(Nm)':>10}")
    print("  " + "-" * 48)

    for j in range(n):
        tj = tau[:, j]
        print(f"  {JOINT_NAMES[j]:<8} {np.max(np.abs(tj)):>10.4f} "
              f"{np.sqrt(np.mean(tj**2)):>10.4f} {tj[0]:>10.4f} {tj[-1]:>10.4f}")

    total_peak = np.max(np.sum(np.abs(tau), axis=1))
    print("  " + "-" * 48)
    print(f"  {'合计峰值':<8} {total_peak:>10.4f} Nm")

    # 关键帧
    print(f"\n  关键帧:")
    for label, idx in [("起点", 0), ("25%", len(traj.t)//4),
                        ("50%", len(traj.t)//2), ("75%", 3*len(traj.t)//4), ("终点", -1)]:
        t = traj.t[idx]
        q_deg = np.rad2deg(traj.q[idx])
        tj = tau[idx]
        parts = [f"{JOINT_NAMES[j]}={tj[j]:6.3f}" for j in range(n)]
        print(f"    t={t:5.2f}s | " + " | ".join(parts))

    # 总能耗估计 (积分 |τ| dt)
    energy = np.trapz(np.sum(np.abs(tau), axis=1), traj.t)
    print(f"\n  累计力矩积分 ∫|τ|dt: {energy:.2f} Nm·s")


def plot_result(traj: Trajectory, tau: np.ndarray, save_path: Optional[str] = None):
    """绘制轨迹和重力补偿力矩"""
    if not HAS_MPL:
        return

    n = tau.shape[1]
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    fig.suptitle("Gravity Compensation — G(q) along trajectory", fontsize=14, fontweight="bold")

    # 位置 + 力矩
    ax = axes[0]
    for j in range(n):
        ax.plot(traj.t, np.rad2deg(traj.q[:, j]), label=JOINT_NAMES[j])
    ax.set_ylabel("Joint Position (deg)")
    ax.legend(loc="upper right", ncol=n)
    ax.grid(True, alpha=0.3)
    ax.set_title("Trajectory q(t)")

    ax = axes[1]
    for j in range(n):
        ax.plot(traj.t, tau[:, j], label=JOINT_NAMES[j])
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Torque (Nm)")
    ax.legend(loc="upper right", ncol=n)
    ax.grid(True, alpha=0.3)
    ax.axhline(y=0, color="black", linewidth=0.5)
    ax.set_title("Gravity Compensation Torque G(q)")

    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches="tight")
    else:
        d = os.path.dirname(os.path.abspath(__file__))
        plt.savefig(os.path.join(d, "gravity_compensation.png"), dpi=150, bbox_inches="tight")
    print(f"\n[Plot] Saved to {save_path or os.path.join(d, 'gravity_compensation.png')}")
    plt.close()


# ═══════════════════════════════════════════════════════════════
# 5. 命令行
# ═══════════════════════════════════════════════════════════════

def parse_args():
    args = {"q_target": None, "q_start": None, "q_static": None, "duration": 2.0, "dt": 0.01, "plot": True, "manual": False}
    argv = sys.argv[1:]
    i = 0
    while i < len(argv):
        if argv[i] == "--q_target":
            vals = []; i += 1
            while i < len(argv) and len(vals) < 6:
                try:
                    vals.append(float(argv[i])); i += 1
                except ValueError:
                    break
            if vals:
                args["q_target"] = np.array(vals)
        elif argv[i] == "--q_start":
            vals = []; i += 1
            while i < len(argv) and len(vals) < 6:
                try:
                    vals.append(float(argv[i])); i += 1
                except ValueError:
                    break
            if vals:
                args["q_start"] = np.array(vals)
        elif argv[i] == "--q_static":
            vals = []; i += 1
            while i < len(argv) and len(vals) < 6:
                try:
                    vals.append(float(argv[i])); i += 1
                except ValueError:
                    break
            if vals:
                args["q_static"] = np.array(vals)
        elif argv[i] == "--duration" and i + 1 < len(argv):
            args["duration"] = float(argv[i + 1]); i += 2
        elif argv[i] == "--dt" and i + 1 < len(argv):
            args["dt"] = float(argv[i + 1]); i += 2
        elif argv[i] == "--no-plot":
            args["plot"] = False; i += 1
        elif argv[i] == "--manual":
            args["manual"] = True; i += 1
        elif argv[i] in ("--help", "-h"):
            print(__doc__); sys.exit(0)
        else:
            i += 1
    return args


def run_demo(model):
    """默认演示 — new_hj_arm2: 零位=水平伸直, home=折叠收起"""
    print("\n" + "=" * 60)
    print("  重力补偿 Demo — new_hj_arm2")
    print("  零位(全0) = 水平伸直 | home = 折叠收起")
    print("=" * 60)

    # 几个典型姿态 (角度制)
    postures = {
        "零位(水平伸直)":   [0, 0, 0, 0, 0, 0],
        "home(折叠)":       [0, 137.5, -137.5, 0, 0, 0],   # J2=2.4rad≈137.5°, J3=-2.4rad≈-137.5°
        "J2半展":           [0, 68.8, -137.5, 0, 0, 0],    # J2≈1.2rad
        "J3半展":           [0, 137.5, -68.8, 0, 0, 0],    # J3≈-1.2rad
        "半展开":           [0, 68.8, -68.8, 0, 0, 0],     # 中间姿态
    }

    print(f"\n{'姿态':<18} ", end="")
    for jn in JOINT_NAMES:
        print(f"{jn:>8}", end="")
    print(f"  {'合计|τ|':>10}")
    print("-" * 76)

    for name, angles in postures.items():
        q = np.deg2rad(angles)
        G = gravity_at_point(model, q)
        total = np.sum(np.abs(G))
        print(f"  {name:<16} ", end="")
        for g in G:
            print(f"{g:>8.3f}", end="")
        print(f"  {total:>10.3f}")

    # ── 轨迹 demo: home → 半展开 ──
    q_home = np.deg2rad([0, 137.5, -137.5, 0, 0, 0])
    q_target = np.deg2rad([0, 68.8, -68.8, 0, 0, 0])
    print(f"\n▶ 轨迹: home(折叠) → 半展开 (2秒)")
    traj = plan_trajectory(q_home, q_target, duration=2.0)
    tau = compute_gravity_compensation(model, traj)
    print_summary(traj, tau)

    if HAS_MPL:
        d = os.path.dirname(os.path.abspath(__file__))
        plot_result(traj, tau, os.path.join(d, "demo_gravity.png"))


def print_static_result(model, q_deg, label=None, manual=False):
    """打印单点静态重力补偿力矩"""
    q = np.deg2rad(q_deg)
    n = len(q_deg)

    if label:
        print(f"\n{'='*60}")
        print(f"  静态重力补偿 — {label}")
        print(f"{'='*60}")
    else:
        print(f"\n{'='*60}")
        print(f"  静态重力补偿力矩 G(q)")
        print(f"{'='*60}")

    print(f"  关节角度(deg): {np.array2string(np.array(q_deg), precision=1, separator=', ')}")
    print(f"  关节角度(rad): {np.array2string(np.array(q), precision=4, separator=', ')}")

    # ── 手动计算模式: 展示每一步 ──
    if manual:
        compute_gravity_manual(model, q, verbose=True)
        return

    # ── Pinocchio 快速模式 ──
    G = gravity_at_point(model, q)
    print()
    print(f"  {'关节':<8} {'力矩 (Nm)':>12} {'方向':>8}")
    print("  " + "-" * 32)

    total = 0.0
    for j in range(n):
        direction = "↑ 抵抗重力" if G[j] > 0.001 else ("↓ 顺应重力" if G[j] < -0.001 else "≈ 零")
        print(f"  {JOINT_NAMES[j]:<8} {G[j]:>12.4f}   {direction}")
        total += abs(G[j])

    print("  " + "-" * 32)
    print(f"  {'合计|τ|':<8} {total:>12.4f} Nm")
    print()
    return G


def main():
    args = parse_args()
    model = load_model(find_urdf())

    # ── 模式1: 静态单点测试 ──
    if args["q_static"] is not None:
        n = model.nv
        q_static_deg = args["q_static"]
        if len(q_static_deg) < n:
            q_static_deg = np.pad(q_static_deg, (0, n - len(q_static_deg)))[:n]
        else:
            q_static_deg = q_static_deg[:n]
        print_static_result(model, q_static_deg, manual=args["manual"])

    # ── 模式2: 轨迹测试 ──
    elif args["q_target"] is not None:
        n = model.nv
        q_start = np.deg2rad(args["q_start"]) if args["q_start"] is not None else np.zeros(n)
        q_target_deg = args["q_target"]
        if len(q_target_deg) != n:
            q_target_deg = np.pad(q_target_deg[:n], (0, max(0, n - len(q_target_deg))))[:n]

        q_target = np.deg2rad(q_target_deg)
        print(f"\n目标: {np.rad2deg(q_start)}° → {np.rad2deg(q_target)}° "
              f"| 时长: {args['duration']}s")

        traj = plan_trajectory(q_start, q_target, duration=args["duration"], dt=args["dt"])
        tau = compute_gravity_compensation(model, traj)
        print_summary(traj, tau)

        if args["plot"]:
            plot_result(traj, tau)

    # ── 模式3: 默认 Demo ──
    else:
        run_demo(model)


if __name__ == "__main__":
    main()
