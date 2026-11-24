import numpy as np


def compute_global_yaw(q1, q2, q3, q4, q5, q6):
    """
    正解：根据6个关节角度计算全局偏航角（文档公式）
    """
    pitch_sum = q2 + q3 + q4

    R_yaw1 = np.array([
        [np.cos(q1), -np.sin(q1), 0],
        [np.sin(q1),  np.cos(q1), 0],
        [0, 0, 1]
    ])

    R_pitch = np.array([
        [np.cos(pitch_sum),  0, np.sin(pitch_sum)],
        [0,                  1, 0],
        [-np.sin(pitch_sum), 0, np.cos(pitch_sum)]
    ])

    R_roll = np.array([
        [1, 0, 0],
        [0, np.cos(q5), -np.sin(q5)],
        [0, np.sin(q5),  np.cos(q5)]
    ])

    R_yaw2 = np.array([
        [np.cos(q6), -np.sin(q6), 0],
        [np.sin(q6),  np.cos(q6), 0],
        [0, 0, 1]
    ])

    R_total = R_yaw1 @ R_pitch @ R_roll @ R_yaw2
    world_yaw = np.arctan2(R_total[1, 0], R_total[0, 0])
    return world_yaw


def compute_q6_from_global_yaw(target_yaw, q1, q2, q3, q4, q5):
    """
    逆解：给定期望的全局偏航角 target_yaw 及其他5个关节角，求解 q6（j6的最终值）
    """
    pitch_sum = q2 + q3 + q4

    # 系数 A, B, C, D
    A = np.cos(q1) * np.cos(pitch_sum)
    B = np.cos(q1) * np.sin(pitch_sum) * np.sin(q5) - np.sin(q1) * np.cos(q5)
    C = np.sin(q1) * np.cos(pitch_sum)
    D = np.sin(q1) * np.sin(pitch_sum) * np.sin(q5) + np.cos(q1) * np.cos(q5)

    # q6 = atan2(A*sinθ - C*cosθ, D*cosθ - B*sinθ)
    numerator = A * np.sin(target_yaw) - C * np.cos(target_yaw)
    denominator = D * np.cos(target_yaw) - B * np.sin(target_yaw)

    q6 = np.arctan2(numerator, denominator)
    return q6


if __name__ == "__main__":
    # --- 示例：给定前5个关节，反算所需的 q6 ---
    q1 = np.radians(30)   # Yaw1 = 30°
    q2 = np.radians(10)   # Pitch1 = 10°
    q3 = np.radians(20)   # Pitch2 = 20°
    q4 = np.radians(-5)   # Pitch3 = -5°
    q5 = np.radians(15)   # Roll = 15°

    target_yaw = np.radians(45)  # 期望全局偏航角 = 45°

    q6 = compute_q6_from_global_yaw(target_yaw, q1, q2, q3, q4, q5)
    print(f"=== j6 逆解结果 ===")
    print(f"输入: q1={np.degrees(q1):.2f}°, pitch_sum={np.degrees(q2+q3+q4):.2f}°, q5={np.degrees(q5):.2f}°")
    print(f"期望全局偏航角: {np.degrees(target_yaw):.4f}°")
    print(f"解得的 j6 (q6): {np.degrees(q6):.4f}°  ({q6:.6f} rad)")
    print()

    # --- 验证：将解出的 q6 代回正解，确认全局偏航角一致 ---
    yaw_check = compute_global_yaw(q1, q2, q3, q4, q5, q6)
    print(f"=== 正解验证 ===")
    print(f"代回正解得全局偏航角: {np.degrees(yaw_check):.4f}°")
    print(f"与期望值偏差: {abs(np.degrees(yaw_check) - np.degrees(target_yaw)):.10f}°")
    print()

    # --- 多组测试 ---
    print(f"=== 多组随机测试 ===")
    np.random.seed(42)
    for i in range(5):
        r_q1 = np.radians(np.random.uniform(-60, 60))
        r_q2 = np.radians(np.random.uniform(-30, 30))
        r_q3 = np.radians(np.random.uniform(-30, 30))
        r_q4 = np.radians(np.random.uniform(-30, 30))
        r_q5 = np.radians(np.random.uniform(-45, 45))
        r_target = np.radians(np.random.uniform(-180, 180))

        r_q6 = compute_q6_from_global_yaw(r_target, r_q1, r_q2, r_q3, r_q4, r_q5)
        r_actual = compute_global_yaw(r_q1, r_q2, r_q3, r_q4, r_q5, r_q6)

        # 注意 atan2 的周期性：差值应在 0 或 ±2π 附近
        diff = np.arctan2(np.sin(r_actual - r_target), np.cos(r_actual - r_target))
        print(f"  测试{i+1}: target={np.degrees(r_target):7.2f}°, actual={np.degrees(r_actual):7.2f}°, "
              f"q6={np.degrees(r_q6):7.2f}°, 偏差={np.degrees(diff):.6e}°")
