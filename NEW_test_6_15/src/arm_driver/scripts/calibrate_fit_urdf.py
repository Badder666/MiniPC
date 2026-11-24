#!/usr/bin/env python3
"""
URDF 重力补偿标定 — 参数拟合工具。

读取 calibrate_collect.py 采集的 (q, tau_offset) 数据，
用 scipy 优化 Link2/Link3/Link4 的 mass 和 com_x，
输出修正后的 URDF 文件。

核心关系 (以 J2 为例):
  平衡时: G_original(q)[1] * scale * gain + offset_balance = τ_true_gravity
  目标:   G_optimized(q)[1] * scale * gain ≈ τ_true_gravity

即让优化后的 G(q) 逼近 G_original(q) + offset_balance / (scale * gain)

用法:
  # 基本用法（scale/gain 用默认 1.0）
  python3 calibrate_fit_urdf.py calibration_data.json

  # 指定 scale/gain（与采集时 bridge 中实际值一致）
  python3 calibrate_fit_urdf.py calibration_data.json \
      --j2-scale 1.0 --j2-gain 1.0 --j3-gain 1.0
"""

import json
import os
import argparse
import sys
import copy
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import minimize

import pinocchio

# ── 常量 ────────────────────────────────────────────────────────────

# 要优化的 link 名及参数
OPT_LINKS = ["Link2", "Link3", "Link4"]
JOINT_NAMES = ["J1", "J2", "J3", "J4", "J5", "J6"]

DEFAULT_URDF = (
    "/home/hj/NEW_test_6_15(1)/NEW_test_6_15/src/new_hj_arm2/urdf/new_hj_arm2.urdf"
)


# ── Pinocchio 模型操作 ──────────────────────────────────────────────


def load_model(urdf_path: str):
    """加载 Pinocchio 模型和 data"""
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"URDF 不存在: {urdf_path}")
    model = pinocchio.buildModelFromUrdf(urdf_path)
    data = model.createData()
    return model, data


def get_joint_id(model, link_name: str) -> int:
    """
    Pinocchio 中 link 作为其 parent joint 的 child，
    model.inertias[joint_id] = child link 的惯性参数。
    此函数返回 link_name 对应的 joint_id。
    """
    for jid in range(model.njoints):
        if model.names[jid] == link_name:
            return jid
    raise ValueError(f"在模型中找不到 link: {link_name}")


def compute_gravity(model, data, q: list[float]) -> np.ndarray:
    """计算重力矩 G(q)，返回 (6,) numpy array"""
    q_np = np.array(q, dtype=np.float64)
    return pinocchio.computeGeneralizedGravity(model, data, q_np)


def get_original_inertial_params(model):
    """提取 Link2/3/4 的原始 mass 和 com_x"""
    params = {}
    for link_name in OPT_LINKS:
        jid = get_joint_id(model, link_name)
        inertia = model.inertias[jid]
        params[f"{link_name}_mass"] = float(inertia.mass)
        params[f"{link_name}_com_x"] = float(inertia.lever[0])
    return params


# ── 代价函数 ────────────────────────────────────────────────────────


def make_cost_func(model, data, joint_ids, calib_data):
    """
    构建代价函数。

    calib_data 已预处理，每条包含:
      - q: 关节角度
      - true_j2: 真实 J2 重力矩 (G_orig[1] * s2 * g2 + offset_j2)
      - true_j3: 真实 J3 重力矩 (G_orig[2] * g3 + offset_j3)
    """
    # 保存原始参数用于 restore
    original = {}
    for link_name in OPT_LINKS:
        jid = joint_ids[link_name]
        original[link_name] = {
            "mass": float(model.inertias[jid].mass),
            "com_x": float(model.inertias[jid].lever[0]),
            "com_y": float(model.inertias[jid].lever[1]),
            "com_z": float(model.inertias[jid].lever[2]),
        }

    def set_params(x):
        """将优化变量 x 写入模型"""
        for i, link_name in enumerate(OPT_LINKS):
            jid = joint_ids[link_name]
            model.inertias[jid].mass = float(x[2 * i])
            model.inertias[jid].lever = np.array(
                [float(x[2 * i + 1]),
                 original[link_name]["com_y"],
                 original[link_name]["com_z"]]
            )

    def restore():
        """恢复模型原始参数"""
        set_params([
            original[ln]["mass"] for ln in OPT_LINKS
            for _ in range(2)
        ])
        # 修正: 用正确顺序恢复
        for link_name in OPT_LINKS:
            jid = joint_ids[link_name]
            model.inertias[jid].mass = original[link_name]["mass"]
            model.inertias[jid].lever = np.array([
                original[link_name]["com_x"],
                original[link_name]["com_y"],
                original[link_name]["com_z"],
            ])

    def cost(x):
        set_params(x)
        total = 0.0
        for entry in calib_data:
            G = compute_gravity(model, data, entry["q"])
            total += (float(G[1]) - entry["true_j2"]) ** 2
            total += (float(G[2]) - entry["true_j3"]) ** 2
        return total

    return cost, original, restore


# ── URDF 写入 ────────────────────────────────────────────────────────


def write_calibrated_urdf(urdf_path: str, output_path: str, x: np.ndarray):
    """将优化结果写入新 URDF 文件"""
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    for i, link_name in enumerate(OPT_LINKS):
        new_mass = float(x[2 * i])
        new_com_x = float(x[2 * i + 1])

        for link_elem in root.iter("link"):
            if link_elem.get("name") != link_name:
                continue
            inertial = link_elem.find("inertial")
            if inertial is None:
                continue

            # 更新 mass
            mass_elem = inertial.find("mass")
            if mass_elem is not None:
                mass_elem.set("value", f"{new_mass:.12f}")

            # 更新 origin xyz 中的 x 分量
            origin_elem = inertial.find("origin")
            if origin_elem is not None:
                xyz = origin_elem.get("xyz", "0 0 0")
                parts = xyz.split()
                if len(parts) == 3:
                    parts[0] = f"{new_com_x:.12f}"
                    origin_elem.set("xyz", " ".join(parts))
            break

    tree.write(output_path, encoding="utf-8", xml_declaration=True)
    print(f"已写入: {output_path}")


# ── 主流程 ──────────────────────────────────────────────────────────


def main():
    parser = argparse.ArgumentParser(
        description="URDF 重力补偿参数优化"
    )
    parser.add_argument(
        "data_file", nargs="?", default="calibration_data.json",
        help="标定数据 JSON 文件路径 (默认 calibration_data.json)"
    )
    parser.add_argument(
        "--urdf", default=DEFAULT_URDF,
        help=f"原始 URDF 路径 (默认 {DEFAULT_URDF})"
    )
    parser.add_argument(
        "--output", default=None,
        help="输出 URDF 路径 (默认同目录 new_hj_arm2_calibrated.urdf)"
    )
    parser.add_argument(
        "--j2-scale", type=float, default=1.0,
        help="采集时 bridge 的 j2_tau_scale 值 (默认 1.0)"
    )
    parser.add_argument(
        "--j2-gain", type=float, default=1.0,
        help="采集时 bridge 的 j2_tau_gain 值 (默认 1.0)"
    )
    parser.add_argument(
        "--j3-gain", type=float, default=1.0,
        help="采集时 bridge 的 j3_tau_gain 值 (默认 1.0)"
    )
    parser.add_argument(
        "--bounds-pct", type=float, default=50.0,
        help="参数搜索边界, 原始值 ±% (默认 50)"
    )
    args = parser.parse_args()

    # ── 加载数据 ──────────────────────────────────────────────────
    with open(args.data_file) as f:
        raw_data = json.load(f)

    if not raw_data:
        print("错误: 标定数据为空")
        sys.exit(1)

    print(f"已加载 {len(raw_data)} 组标定数据")
    print(f"  j2_scale={args.j2_scale}  j2_gain={args.j2_gain}  "
          f"j3_gain={args.j3_gain}")

    # ── 加载原始模型 ──────────────────────────────────────────────
    model, data = load_model(args.urdf)
    print(f"已加载 URDF: {model.name}  ({model.nq} DOF)")

    joint_ids = {}
    for link_name in OPT_LINKS:
        joint_ids[link_name] = get_joint_id(model, link_name)
        print(f"  {link_name} → joint_id={joint_ids[link_name]}")

    original_params = get_original_inertial_params(model)
    print("\n原始 inertial 参数:")
    for link_name in OPT_LINKS:
        m = original_params[f"{link_name}_mass"]
        cx = original_params[f"{link_name}_com_x"]
        print(f"  {link_name}:  mass={m:.4f}  com_x={cx:.6f}")

    # ── 预计算真实重力矩 ──────────────────────────────────────────
    j2_sg = args.j2_scale * args.j2_gain
    j3_g = args.j3_gain

    for entry in raw_data:
        q = entry["q"]
        G_orig = compute_gravity(model, data, q)
        # true = G_original * scale * gain + offset_balance
        entry["true_j2"] = float(G_orig[1]) * j2_sg + float(entry["tau_j2"])
        entry["true_j3"] = float(G_orig[2]) * j3_g + float(entry["tau_j3"])

    # 打印真实重力矩概要
    print("\n真实重力矩 (τ_true = G_orig * scale * gain + offset):")
    for i, entry in enumerate(raw_data):
        print(f"  [{i+1}] J2_true={entry['true_j2']:+.4f}  "
              f"J3_true={entry['true_j3']:+.4f}")

    # ── 构建优化问题 ──────────────────────────────────────────────
    cost_func, original, restore = make_cost_func(
        model, data, joint_ids, raw_data
    )

    # 初始值 x0 = [m2, cx2, m3, cx3, m4, cx4]
    x0 = []
    bounds = []
    pct = args.bounds_pct / 100.0
    for link_name in OPT_LINKS:
        m = original[link_name]["mass"]
        cx = original[link_name]["com_x"]
        x0.extend([m, cx])
        bounds.extend([
            (m * (1 - pct), m * (1 + pct)),
            (cx * (1 - pct) if cx >= 0 else cx * (1 + pct),
             cx * (1 + pct) if cx >= 0 else cx * (1 - pct)),
        ])

    err_before = cost_func(x0)
    rmse_before = np.sqrt(err_before / (len(raw_data) * 2))
    print(f"\n优化前:  total_err={err_before:.4f}  RMSE={rmse_before:.4f} Nm")

    # ── 运行优化 ──────────────────────────────────────────────────
    print("\n优化中 (L-BFGS-B)...")
    result = minimize(
        cost_func, x0,
        method="L-BFGS-B",
        bounds=bounds,
        options={"maxiter": 500, "disp": False},
    )

    # ── 结果 ──────────────────────────────────────────────────────
    restore()  # 先恢复原始

    x_opt = result.x
    err_after = result.fun
    rmse_after = np.sqrt(err_after / (len(raw_data) * 2))

    print(f"\n{'='*60}")
    print(f"优化结果: {'收敛' if result.success else '未收敛'}")
    print(f"  total_err: {err_before:.4f} → {err_after:.4f}")
    print(f"  RMSE:      {rmse_before:.4f} → {rmse_after:.4f} Nm")
    if err_before > 1e-12:
        print(f"  改善:      {(1 - err_after / err_before) * 100:.1f}%")
    print(f"{'='*60}")

    print("\n优化后 inertial 参数:")
    for i, link_name in enumerate(OPT_LINKS):
        old_m = original[link_name]["mass"]
        old_cx = original[link_name]["com_x"]
        new_m = float(x_opt[2 * i])
        new_cx = float(x_opt[2 * i + 1])
        dm = (new_m - old_m) / old_m * 100
        dcx = ((new_cx - old_cx) / abs(old_cx) * 100) if abs(old_cx) > 1e-9 else 0.0
        print(f"  {link_name}:")
        print(f"    mass:   {old_m:.4f} → {new_m:.4f}  ({dm:+.1f}%)")
        print(f"    com_x:  {old_cx:.6f} → {new_cx:.6f}  ({dcx:+.1f}%)")

    # ── 逐姿态误差对比 ────────────────────────────────────────────
    print("\n逐姿态误差 (优化前 → 优化后):")
    print(f"  {'#':<4} {'J2 err_before':>14} {'J2 err_after':>14} "
          f"{'J3 err_before':>14} {'J3 err_after':>14}")

    for i, entry in enumerate(raw_data):
        # 优化前的 G
        G_orig = compute_gravity(model, data, entry["q"])
        err_j2_b = float(G_orig[1]) * j2_sg - entry["true_j2"]
        err_j3_b = float(G_orig[2]) * j3_g - entry["true_j3"]

        # 优化后的 G — 暂时写 x_opt 到模型
        for j, link_name in enumerate(OPT_LINKS):
            jid = joint_ids[link_name]
            model.inertias[jid].mass = float(x_opt[2 * j])
            model.inertias[jid].lever = np.array([
                float(x_opt[2 * j + 1]),
                original[link_name]["com_y"],
                original[link_name]["com_z"],
            ])
        G_new = compute_gravity(model, data, entry["q"])
        err_j2_a = float(G_new[1]) * j2_sg - entry["true_j2"]
        err_j3_a = float(G_new[2]) * j3_g - entry["true_j3"]

        print(f"  {i+1:<4} {err_j2_b:+.4f} → {err_j2_a:+.4f}    "
              f"{err_j3_b:+.4f} → {err_j3_a:+.4f}")

    # 恢复
    restore()

    # ── 输出 URDF ─────────────────────────────────────────────────
    output_path = args.output
    if output_path is None:
        dir_name = os.path.dirname(args.urdf)
        output_path = os.path.join(dir_name, "new_hj_arm2_calibrated.urdf")

    write_calibrated_urdf(args.urdf, output_path, x_opt)

    print(f"\n使用方法:")
    print(f"  1. 将 bridge 的 urdf_path 改为: {output_path}")
    print(f"  2. 将 j2_tau_offset 和 j3_tau_offset 归零")
    print(f"  3. 重启 bridge 节点，验证各姿态下不再漂移")


if __name__ == "__main__":
    main()
