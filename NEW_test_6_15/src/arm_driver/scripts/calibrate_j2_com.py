#!/usr/bin/env python3
"""
J2 重力补偿标定工具
通过多组实测数据反推 Link2 CoM 的修正量

用法:
  python3 calibrate_j2_com.py
"""

import numpy as np
import os

try:
    import pinocchio
except ImportError:
    print("请先安装 pinocchio: pip install pinocchio")
    exit(1)


def find_urdf():
    paths = [
        "/home/hj/NEW_test_6_15(1)/NEW_test_6_15/src/new_hj_arm2/urdf/new_hj_arm2.urdf",
    ]
    try:
        from ament_index_python import get_package_share_directory
        p = os.path.join(get_package_share_directory("new_hj_arm2"), "urdf", "new_hj_arm2.urdf")
        paths.insert(0, p)
    except Exception:
        pass
    for p in paths:
        if os.path.exists(p):
            return p
    raise FileNotFoundError("找不到 URDF")


# ═══════════════════════════════════════════════════════════
# 实测数据 — 把新的数据加在这里
# ═══════════════════════════════════════════════════════════
DATA = [
    {
        "name": "折叠",
        "q": [-0.1688, 2.4637, -2.9076, 0.0185, -0.2993, -0.1543],
        "tau_actual": -17.76,   # 实际稳住需要的 J2 总力矩
    },
    {
        "name": "竖直靠后",
        "q": [-0.1303, 1.8759, -2.8095, 0.2176, 0.0277, 0.2840],
        "tau_actual": -4.48,
    },
    {
        "name": "竖直前倾",
        "q": [0.0639, 1.5108, -2.3125, -0.1276, -0.5407, -0.4278],
        "tau_actual": 8.72,
    },
]


def compute_tau_j2(model, q):
    """计算 J2 的重力补偿力矩"""
    data = model.createData()
    G = pinocchio.computeGeneralizedGravity(model, data, np.array(q, dtype=np.float64))
    return G[1]


def com_sensitivity(model, q, link2_joint_id):
    """
    用有限差分计算 J2 重力矩对 Link2 CoM (x,y,z) 的灵敏度
    返回 [dtau/dx, dtau/dy, dtau/dz]
    """
    eps = 0.001  # 1mm 扰动
    G0 = compute_tau_j2(model, q)
    sens = []
    for axis in range(3):
        delta = np.zeros(3)
        delta[axis] = eps

        model_tmp = model.copy()
        old = model_tmp.inertias[link2_joint_id]
        new_lever = old.lever + delta
        model_tmp.inertias[link2_joint_id] = pinocchio.Inertia(
            old.mass, new_lever, old.inertia
        )
        G1 = compute_tau_j2(model_tmp, q)
        sens.append((G1 - G0) / eps)
    return np.array(sens)


def main():
    urdf_path = find_urdf()
    print(f"加载 URDF: {urdf_path}")

    model = pinocchio.buildModelFromUrdf(urdf_path)
    print(f"模型: {model.name}, {model.nq} DOF, {model.njoints} joints")

    # 找到 J2 joint 的 ID（它的 inertia 就是 Link2 的 inertia）
    link2_joint_id = model.getJointId("J2")
    print(f"J2 joint ID = {link2_joint_id}")
    old_inertia = model.inertias[link2_joint_id]
    print(f"Link2 当前 mass={old_inertia.mass:.4f}, "
          f"CoM=({old_inertia.lever[0]:.4f}, {old_inertia.lever[1]:.4f}, {old_inertia.lever[2]:.4f})")

    # ── 计算误差 ──
    print("\n" + "=" * 70)
    print(f"{'位置':<12} {'Pinocchio τ₂':>12} {'实际 τ₂':>12} {'误差(Nm)':>12} {'比值':>8}")
    print("-" * 70)

    errors = []
    for d in DATA:
        tau_p = compute_tau_j2(model, d["q"])
        tau_a = d["tau_actual"]
        err = tau_a - tau_p
        ratio = tau_a / tau_p if abs(tau_p) > 0.01 else float("nan")
        print(f"{d['name']:<12} {tau_p:>+12.4f} {tau_a:>+12.4f} {err:>+12.4f} {ratio:>8.2f}x")
        errors.append(err)

    print("-" * 70)

    # ── 各位置的 CoM 灵敏度 ──
    print("\n--- CoM 灵敏度 (每米 CoM 偏移导致的 τ₂ 变化) ---")
    print(f"{'位置':<12} {'∂τ/∂x (Nm/m)':>16} {'∂τ/∂y (Nm/m)':>16} {'∂τ/∂z (Nm/m)':>16}")
    print("-" * 60)

    A = np.zeros((len(DATA), 3))
    for i, d in enumerate(DATA):
        s = com_sensitivity(model, d["q"], link2_joint_id)
        A[i] = s
        print(f"{d['name']:<12} {s[0]:>+16.4f} {s[1]:>+16.4f} {s[2]:>+16.4f}")

    b = np.array(errors)
    print(f"\n误差向量 b (实际 - Pinocchio): {np.array2string(b, precision=4)}")

    # ── 最小二乘求解 ──
    print("\n" + "=" * 70)
    print("求解 A · [dx, dy, dz]^T = b  (最小二乘)")
    print("=" * 70)

    try:
        correction, residuals, rank, sv = np.linalg.lstsq(A, b, rcond=None)

        print(f"\n  ★★★ 推荐的 Link2 CoM 修正量 ★★★")
        print(f"  dx = {correction[0]:+.4f} m  ({correction[0]*100:+.1f} cm)")
        print(f"  dy = {correction[1]:+.4f} m  ({correction[1]*100:+.1f} cm)")
        print(f"  dz = {correction[2]:+.4f} m  ({correction[2]*100:+.1f} cm)")

        old_x, old_y, old_z = old_inertia.lever
        print(f"\n  当前 URDF:  xyz=\"{old_x:.4f} {old_y:.4f} {old_z:.4f}\"")
        print(f"  建议改为:    xyz=\"{old_x+correction[0]:.4f} {old_y+correction[1]:.4f} {old_z+correction[2]:.4f}\"")

        # ── 验证 ──
        print(f"\n--- 拟合验证 ---")
        print(f"{'位置':<12} {'实际误差':>10} {'拟合值':>10} {'残差':>10}")
        fitted = A @ correction
        for i, d in enumerate(DATA):
            print(f"{d['name']:<12} {b[i]:>+10.4f} {fitted[i]:>+10.4f} {b[i]-fitted[i]:>+10.4f}")

        rmse = np.sqrt(np.mean((b - fitted) ** 2))
        print(f"\n  RMSE = {rmse:.4f} Nm  (拟合后剩余误差)")

        if rmse > 2.0:
            print("  ⚠ 残差较大，可能还有其他连杆的参数也需要调整，或需要更多数据点")
        else:
            print("  ✓ 拟合效果良好")

    except np.linalg.LinAlgError as e:
        print(f"求解失败: {e}")


if __name__ == "__main__":
    main()