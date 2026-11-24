#!/usr/bin/env python3
"""
CCtrl 关节角度处理节点 — 双模式：CCtrl（编码器遥控）+ GOTO（预设轨迹往返）。

CCtrl 模式: 订阅 /ccctrl_data → 解绕/偏置/滤波/包裹 → /joint_states + /ccctrl_positions
GOTO  模式: main_buttons K1-K4 上升沿触发 → 记录原位(home) → 去程插值到目标
         → 回程插值返回 home → 自动切回 CCtrl 模式

用法:
  ros2 run arm_serial ccctrl_joint_processor.py
  ros2 run arm_serial ccctrl_joint_processor.py --ros-args -p filter_alpha:=0.4
  ros2 param set /ccctrl_joint_processor goto_target_k1 [0.0,-0.5,1.2,0.0,-0.8,0.0]
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node

from arm_serial.msg import CCtrlPayload, CCtrlJoint
from sensor_msgs.msg import JointState


class CCtrlJointProcessor(Node):
    """订阅 /ccctrl_data → 关节角度处理 → 发布 /joint_states + /ccctrl_positions"""

    def __init__(self):
        super().__init__("ccctrl_joint_processor")

        # ── 参数 ──────────────────────────────────────────────
        self.declare_parameter("filter_alpha", 0.4)
        self._filter_alpha = (
            self.get_parameter("filter_alpha").get_parameter_value().double_value
        )

        # ── 关节偏置 (编码器零位 → 机械零位) ──────────────────
        self._offsets = {
            1: math.radians(-170),   # J2
            2: math.radians(-170),   # J3
        }

        # ── 编码器解绕 / 滤波状态 ─────────────────────────────
        self._filtered_unwrapped = None  # np.ndarray, EMA 后的连续角度
        self._enc_unwrapped = None       # np.ndarray, 编码器解绕累积值
        self._prev_enc = None            # np.ndarray, 上一帧原始编码器值

        # ── J6 flip toggle (handle_buttons S2=0x02 触发 180° 翻转) ──
        self._j6_offset_current = 0.0   # 当前平滑插值中的 J6 偏置 (rad)
        self._j6_flip_target = 0.0      # 目标偏置 (0 或 π)
        self._j5_offset = 0.0           # J5 偏置累积 (rad), K3+1/K4-1
        self.declare_parameter("j5_offset_speed", 0.45)
        self._j5_offset_speed = (
            self.get_parameter("j5_offset_speed").get_parameter_value().double_value
        )
        self._j6_flip_speed = math.pi   # 翻转速度 π rad/s → 180° 耗时 1s
        self._prev_handle_buttons = 0   # 上一帧按键状态，用于上升沿检测
        self._prev_main_buttons = 0     # 上一帧 main_buttons，用于 GOTO 上升沿检测
        self._last_cb_time = None       # 上一次回调时间，用于 dt 计算

        # ── K1/K2 双击检测 (上升沿 + 释放跟踪) ──────────────
        self.declare_parameter("double_click_timeout", 0.5)
        self._double_click_timeout = (
            self.get_parameter("double_click_timeout").get_parameter_value().double_value
        )
        self._k1_press_time: float | None = None   # K1 首次按下时刻 (非 None=等待中)
        self._k1_seen_release: bool = False         # 首次按下后是否已检测到释放
        self._k2_press_time: float | None = None   # K2 首次按下时刻
        self._k2_seen_release: bool = False         # 首次按下后是否已检测到释放

        # ── GOTO 模式状态 ─────────────────────────────────────
        self._mode = "ccctrl"                        # "ccctrl" | "goto"
        self._real_position = np.zeros(6, dtype=np.float64)  # 由 /real_positions 更新
        self._real_pos_received = False
        self._goto_home: np.ndarray = None           # 进入 GOTO 时记录的编码器原位 (6,)
        self._goto_waypoints: list = []              # 途经点列表 [home, wp0, wp1, ..., home]
        self._goto_wp_index: int = 0                 # 当前段: waypoints[i] → waypoints[i+1]
        self._goto_state = "moving"                  # "moving" | "dwell"
        self._goto_start_time = None                 # 当前段/停顿 起始时刻
        self._goto_dwell_target = 0.0                # 当前停顿目标时长
        self._goto_turnaround: int = -1              # 折返点索引 (在此设 handle_buttons=0x01, 长停顿)
        self._goto_button = ""                       # 触发 GOTO 的按键名

        # ── S3 安全位姿 ─────────────────────────────────────
        self._s3_target = np.array(
            [0.0219, 2.6007, -2.5915, 0.1055, -0.8623, 0.2039], dtype=np.float64
        )
        self._s3_start_pos: np.ndarray = None
        self._s3_start_time = None
        self.declare_parameter("s3_duration", 1.0)
        self._s3_duration = (
            self.get_parameter("s3_duration").get_parameter_value().double_value
        )

        # ── /ccctrl_data 按键/摇杆缓存 (GOTO 期间用于 CCtrlJoint 面板字段) ──
        self._latest_main_buttons = 0
        self._latest_handle_buttons = 0
        self._latest_joystick_x = 2048
        self._latest_joystick_y = 2048

        # ── GOTO 参数 ─────────────────────────────────────────
        self.declare_parameter("goto_duration", 0.8)
        self._goto_duration = (
            self.get_parameter("goto_duration").get_parameter_value().double_value
        )
        # 梯形速度曲线: 加速度段占比 (0<accel_frac≤0.5, 越大越柔和)
        self.declare_parameter("goto_accel_frac", 0.3)
        self._goto_accel_frac = (
            self.get_parameter("goto_accel_frac").get_parameter_value().double_value
        )
        if not self.has_parameter("goto_duration_k3"):
            self.declare_parameter("goto_duration_k3", 1.3)
        self._goto_duration_k3 = (
            self.get_parameter("goto_duration_k3").get_parameter_value().double_value
        )
        if not self.has_parameter("goto_duration_k2"):
            self.declare_parameter("goto_duration_k2", 0.7)
        self._goto_duration_k2 = (
            self.get_parameter("goto_duration_k2").get_parameter_value().double_value
        )
        self.declare_parameter("goto_dwell", 1.0)
        self._goto_dwell = (
            self.get_parameter("goto_dwell").get_parameter_value().double_value
        )
        self.declare_parameter("goto_dwell_delay", 0.5)  # 折返点触发 S1 前的延迟
        self._goto_dwell_delay = (
            self.get_parameter("goto_dwell_delay").get_parameter_value().double_value
        )
        self.declare_parameter("goto_dwell_short", 0.8)   
        self._goto_dwell_short = (
            self.get_parameter("goto_dwell_short").get_parameter_value().double_value
        )
        if not self.has_parameter("goto_dwell_short_k3"):
            self.declare_parameter("goto_dwell_short_k3", 1.3)
        self._goto_dwell_short_k3 = (
            self.get_parameter("goto_dwell_short_k3").get_parameter_value().double_value
        )
        # K1: zuo (单击)
        _k1_wp0 = np.array([ 0.0383, 2.8719, -2.2831, -0.0998, -0.8429, -0.1787], dtype=np.float64)
        _k1_wp1 = np.array([ 0.0414, 2.9558, -2.5490, -1.2575, -1.3205, -1.2568], dtype=np.float64)
        _k1_wp2 = np.array([ 0.0551, 3.0302, -2.9298, -1.5215, -1.4193, -1.6012], dtype=np.float64)
        _k1_wp3 = np.array([ 0.4200, 2.4061, -2.8274, -1.9205, -1.6329, -2.2597], dtype=np.float64)
        _k1_wp4 = np.array([ 0.4200, 2.4210, -2.4590, -1.9205, -1.5544, -2.0117], dtype=np.float64)
        # K1 双击: 连续轨迹 (home → 3点 → home, wp1 触发 S1)
        _k1d_wp0 = np.array([ 0.6014, 2.5373, -2.5094, -1.6966, -1.4132, -1.8324], dtype=np.float64)
        _k1d_wp1 = np.array([ 0.6079, 2.7372, -2.9094, -1.6520, -1.4155, -1.9030], dtype=np.float64)
        _k1d_wp2 = np.array([ 0.6079, 2.7010, -2.2568, -1.6425, -1.4075, -1.5921], dtype=np.float64)
        _k1d_wp3 = np.array([ -0.2000, 2.7010, -2.2568, -1.6425, -1.4075, -1.5921], dtype=np.float64)

        # K2:you
        _k2_wp0 = np.array([-0.0029, 3.1382, -2.6216,  0.0116, -0.7170, -0.1013], dtype=np.float64)
        _k2_wp1 = np.array([-0.0032, 3.1382, -2.6306,  1.3891, -1.2137,  0.8883], dtype=np.float64)
        _k2_wp2 = np.array([-0.0029, 3.1382, -2.9024,  1.4830, -1.5203,  1.2102], dtype=np.float64)
        _k2_wp3 = np.array([-0.4200, 2.3344, -2.7098,  1.6703, -1.5897,  1.76], dtype=np.float64)
        _k2_wp4 = np.array([-0.4200, 2.3874, -2.4053,  1.5677, -1.5440,  1.3979], dtype=np.float64)
        # K2 双击: 连续无停顿轨迹 (home → 4点 → home)
        _k2d_wp0 = np.array([-0.4200, 2.4290, -2.3150,  1.5425, -1.5932,  1.2579], dtype=np.float64)
        _k2d_wp1 = np.array([-0.4200, 2.7147, -2.9501,  1.5471, -1.5558,  1.6489], dtype=np.float64)
        _k2d_wp2 = np.array([-0.4200, 3.1382, -3.0114,  1.4487, -1.4532,  1.0713], dtype=np.float64)
        _k2d_wp3 = np.array([-0.4200, 3.1382, -2.6717,  0.7750, -1.5150,  1.0713], dtype=np.float64)
        self._goto_chains = {
            # (chain, turnaround_wp_idx, direct_return)2
            "k2": ([_k2_wp0, _k2_wp1, _k2_wp2, _k2_wp3, _k2_wp4], 3, True),
            "k1": ([_k1_wp0, _k1_wp1, _k1_wp2, _k1_wp3, _k1_wp4], 3, True),
            "k2_double": ([_k2d_wp0, _k2d_wp1, _k2d_wp2, _k2d_wp3], 1, True),
            "k1_double": ([_k1d_wp0, _k1d_wp1, _k1d_wp2, _k1d_wp3], 1, True),
        }
        self._goto_targets: dict = {}

        # ── 订阅 /ccctrl_data + /real_positions ────────────────
        self._sub = self.create_subscription(
            CCtrlPayload, "ccctrl_data", self._payload_cb, 10
        )
        self._real_pos_sub = self.create_subscription(
            JointState, "/real_positions", self._real_pos_cb, 10
        )

        # ── 发布 /joint_states + /ccctrl_positions ─────────────
        self._joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self._ccctrl_pos_pub = self.create_publisher(CCtrlJoint, "/ccctrl_positions", 10)

        self.get_logger().info(
            "CCtrl 关节处理器启动 (CCtrl 模式), 等待 /ccctrl_data 数据…"
        )
        # 诊断: 回调计数
        self._payload_cb_count = 0
        self._publish_count = 0
        self._last_diag_time = self.get_clock().now()

    # ── /real_positions 回调 ────────────────────────────────────
    def _real_pos_cb(self, msg: JointState):
        """缓存串口桥接返回的真实关节位置"""
        if len(msg.position) >= 6:
            self._real_position = np.array(msg.position[:6], dtype=np.float64)
            self._real_pos_received = True

    # ── GOTO 模式进入 ──────────────────────────────────────────
    def _enter_goto(self, button: str):
        """进入 GOTO 模式：记录原位 → 途经点链路 → 回到原位 → 切回 CCtrl"""
        if self._filtered_unwrapped is None:
            self.get_logger().warn("GOTO 请求但尚无编码器数据，保持 CCtrl 模式")
            return

        if self._goto_duration <= 0.0:
            self._goto_duration = 1.0

        # 清除 K1/K2 双击检测状态，防止残留
        self._k1_press_time = None
        self._k1_seen_release = False
        self._k2_press_time = None
        self._k2_seen_release = False

        # 记录当前编码器位置作为 home
        self._goto_home = self._filtered_unwrapped.copy()

        # 构建途经点链: home → wp0 → wp1 ⇃ dwell ⇂ wp1 → wp0 → home
        chain_info = self._goto_chains.get(button)
        if chain_info is not None:
            chain, ta_wp_idx, direct_return = chain_info
            fwd = [self._goto_home.copy()]
            for wp in chain:
                fwd.append(wp.copy())
            turnaround = ta_wp_idx  # segment: fwd[ta_wp_idx] → fwd[ta_wp_idx+1]
            if direct_return:
                wps = fwd + [self._goto_home.copy()]  # home→...→c→home 直达
            else:
                rev = list(reversed(fwd[:-1]))
                wps = fwd + rev  # home→...→c→...→home 原路折返
        else:
            # K3/K4: 单目标往返
            target = self._goto_targets.get(button)
            if target is None or len(target) < 6:
                self.get_logger().error(f"GOTO 目标参数 {button} 无效")
                return
            wps = [self._goto_home.copy(), target.copy(), self._goto_home.copy()]
            turnaround = 1

        # ── 最近 2π 等效路径 ──
        for k in range(1, len(wps)):
            prev = wps[k - 1]
            cur = wps[k]
            for j in range(6):
                diff = cur[j] - prev[j]
                cur[j] -= round(diff / math.tau) * math.tau

        self._goto_waypoints = wps
        self._goto_wp_index = 0
        self._goto_state = "moving"
        self._goto_start_time = self.get_clock().now()
        self._goto_turnaround = turnaround if chain is not None else -1
        self._goto_button = button
        self._mode = "goto"

        wp_names = " → ".join(
            f"[{', '.join(f'{math.degrees(v):.0f}' for v in wp)}]"
            for wp in wps
        )
        self.get_logger().info(
            f"GOTO 进入 (btn={button}, {len(wps)-1} 段, "
            f"dur={self._goto_duration:.1f}s/段, dwell={self._goto_dwell:.1f}s)\n"
            f"  turnaround=wp[{self._goto_turnaround}]  {wp_names}"
        )

    # ── 发布辅助 (公用) ────────────────────────────────────────
    def _publish_positions(self, pos: np.ndarray, main_buttons: int,
                           handle_buttons: int, joystick_x: int, joystick_y: int):
        """将 6 关节角度发布到 /joint_states 和 /ccctrl_positions，并写 CSV"""
        # J5 偏置 (K3/K4 累积)
        pos = pos.copy()
        pos[4] += self._j5_offset

        # JointState
        js_out = pos.copy()
        js_out[5] += self._j6_offset_current  # J6 flip offset
        for i in (0, 1, 2, 4):  # J1, J2, J3, J5
            js_out[i] = math.atan2(math.sin(js_out[i]), math.cos(js_out[i]))
        for i in (0, 1, 2):
            js_out[i] = max(-math.pi, min(math.pi, js_out[i]))

        jmsg = JointState()
        jmsg.header.stamp = self.get_clock().now().to_msg()
        jmsg.name = ["J1", "J2", "J3", "J4", "J5", "J6"]
        jmsg.position = js_out.tolist()
        self._joint_pub.publish(jmsg)

        # CCtrlJoint
        cj_out = pos.copy()
        cj_out[5] += self._j6_offset_current  # J6 flip offset
        for i in (0, 1, 2):
            cj_out[i] = math.atan2(math.sin(cj_out[i]), math.cos(cj_out[i]))
            cj_out[i] = max(-math.pi, min(math.pi, cj_out[i]))
        cj_out[4] = math.atan2(math.sin(cj_out[4]), math.cos(cj_out[4]))
        cj_out[4] = max(-math.pi / 2.0, min(math.pi / 2.0, cj_out[4]))

        cjmsg = CCtrlJoint()
        cjmsg.position = cj_out.tolist()
        cjmsg.main_buttons = main_buttons
        cjmsg.handle_buttons = handle_buttons
        cjmsg.joystick_x = joystick_x
        cjmsg.joystick_y = joystick_y
        self._ccctrl_pos_pub.publish(cjmsg)

    # ── 数据回调 ───────────────────────────────────────────────
    def _payload_cb(self, latest: CCtrlPayload):
        """接收原始 CCtrlPayload，处理并发布关节角度"""
        self._payload_cb_count += 1

        # 首次收到 /ccctrl_data 时打印确认
        if self._payload_cb_count == 1:
            self.get_logger().info(
                f"[诊断] 首次收到 /ccctrl_data: "
                f"main_btn=0x{latest.main_buttons:02X}, "
                f"enc={list(latest.encoder_value)}"
            )

        # ── 1. 缓存按键/摇杆 (两种模式都需要) ────────────
        self._latest_main_buttons = latest.main_buttons
        self._latest_handle_buttons = latest.handle_buttons
        self._latest_joystick_x = latest.joystick_x
        self._latest_joystick_y = latest.joystick_y

        # ── 2. 编码器后台处理 (始终运行，保持解绕状态连续) ──
        enc = np.array(latest.encoder_value, dtype=np.float64)
        if self._enc_unwrapped is None:
            self._prev_enc = enc.copy()
            self._enc_unwrapped = enc.copy()
        else:
            delta_enc = enc - self._prev_enc
            delta_enc = np.where(delta_enc > 2048, delta_enc - 4096, delta_enc)
            delta_enc = np.where(delta_enc < -2048, delta_enc + 4096, delta_enc)
            self._enc_unwrapped += delta_enc
            self._prev_enc = enc.copy()

        pos = self._enc_unwrapped * math.tau / 4096.0
        for i, off in self._offsets.items():
            pos[i] += off
        pos[0] *= -1; pos[1] *= -1; pos[3] *= -1; pos[4] *= -1
        # pos[0] += math.radians(-5.8)
        # pos[4] += math.radians(-10)

        if self._filtered_unwrapped is None:
            self._filtered_unwrapped = pos.copy()
        else:
            self._filtered_unwrapped += self._filter_alpha * (pos - self._filtered_unwrapped)

        # ── 3. J6 flip 平滑插值 (始终运行) ──────────────
        now = self.get_clock().now()
        if self._last_cb_time is not None:
            dt = (now - self._last_cb_time).nanoseconds * 1e-9
            diff = self._j6_flip_target - self._j6_offset_current
            if abs(diff) > 1e-9:
                max_step = self._j6_flip_speed * dt
                if abs(diff) <= max_step:
                    self._j6_offset_current = self._j6_flip_target
                else:
                    self._j6_offset_current += max_step if diff > 0 else -max_step
        # ── 3.5 K3/K4: J5 偏置持续调整 (按住时以恒定速度增减) ──
        if self._last_cb_time is not None:
            mbtn = latest.main_buttons
            if mbtn & 0x04:  # K3 按住 → J5 偏置持续增加
                self._j5_offset += self._j5_offset_speed * dt
            if mbtn & 0x08:  # K4 按住 → J5 偏置持续减少
                self._j5_offset -= self._j5_offset_speed * dt

        self._last_cb_time = now

        # ── 4. 模式转换判断 ──────────────────────────────
        hbtn = latest.handle_buttons
        mbtn = latest.main_buttons

        # S2 J6 flip toggle (handle_buttons, CCtrl 模式专用)
        prev_hbtn = self._prev_handle_buttons
        if (hbtn & 0x02) and not (prev_hbtn & 0x02):
            if abs(self._j6_flip_target) < 0.5:
                j6_pos = self._filtered_unwrapped[5] if self._filtered_unwrapped is not None else 0.0
                self._j6_flip_target = -math.pi if j6_pos > 0.0 else math.pi
            else:
                self._j6_flip_target = 0.0
            self.get_logger().info(
                f"J6 flip toggled → target={math.degrees(self._j6_flip_target):.0f}°"
            )

        # S3 安全位姿 (handle_buttons, 电平触发, 任意时刻按下 → 逐帧逼近目标 → 切回 CCtrl)
        s3_now = bool(hbtn & 0x08)
        if s3_now and self._mode != "s3_goto":
            if self._real_pos_received:
                self._mode = "s3_goto"
                self._s3_start_pos = None   # 闭环模式不需要，仅 fallback 用
                self._s3_start_time = None
                # 复位 J5/J6 偏置
                self._j5_offset = 0.0
                self._j6_flip_target = 0.0
                self._j6_offset_current = 0.0
                # 清除 K1/K2 双击状态
                self._k1_press_time = None
                self._k1_seen_release = False
                self._k2_press_time = None
                self._k2_seen_release = False
                # 清除 GOTO 状态
                self._goto_home = None
                self._goto_waypoints = []
                self._goto_wp_index = 0
                self._goto_state = "moving"
                self._goto_turnaround = -1
                self._goto_start_time = None
                self.get_logger().info(
                    "S3 触发 → 逐帧逼近安全位置 "
                    f"[{', '.join(f'{v:.4f}' for v in self._s3_target)}] "
                    f"(τ={self._s3_duration:.1f}s)"
                )
            else:
                self.get_logger().warn("S3 按下但尚未收到 /real_positions 数据，忽略")

        # GOTO 触发 (main_buttons 上升沿)
        rising_m = mbtn & ~self._prev_main_buttons

        # K1: bit0 (0x01) 上升沿 → k1
        if rising_m & 0x01:
            self.get_logger().info("[K1] bit0 上升沿 → k1")
            if self._mode == "goto":
                self.get_logger().info("GOTO 重触发 → 新目标 k1")
            self._enter_goto("k1")

        # K2: bit1 (0x02) 上升沿 → k2
        if rising_m & 0x02:
            self.get_logger().info("[K2] bit1 上升沿 → k2")
            if self._mode == "goto":
                self.get_logger().info("GOTO 重触发 → 新目标 k2")
            self._enter_goto("k2")

        # K5: bit4 (0x10) 上升沿 → k1_double
        if rising_m & 0x10:
            self.get_logger().info("[K5] bit4 上升沿 → k1_double")
            if self._mode == "goto":
                self.get_logger().info("GOTO 重触发 → 新目标 k1_double")
            self._enter_goto("k1_double")

        # K6: bit5 (0x20) 上升沿 → k2_double
        if rising_m & 0x20:
            self.get_logger().info("[K6] bit5 上升沿 → k2_double")
            if self._mode == "goto":
                self.get_logger().info("GOTO 重触发 → 新目标 k2_double")
            self._enter_goto("k2_double")

        self._prev_handle_buttons = hbtn
        self._prev_main_buttons = mbtn

        # ── 6. 按模式发布 ────────────────────────────────
        if self._mode == "s3_goto":
            # S3 安全位姿: 从 real_position 线性插值到目标 (1s 内完成)
            if self._s3_start_pos is None:
                if self._real_pos_received:
                    self._s3_start_pos = self._real_position.copy()
                else:
                    self._s3_start_pos = self._filtered_unwrapped.copy()
                self._s3_start_time = now
                # 最近 2π 等效路径
                for j in range(6):
                    diff = self._s3_target[j] - self._s3_start_pos[j]
                    self._s3_start_pos[j] += round(diff / math.tau) * math.tau

            elapsed = (now - self._s3_start_time).nanoseconds * 1e-9
            dur = self._s3_duration
            if elapsed >= dur:
                self._publish_positions(
                    self._s3_target.copy(),
                    latest.main_buttons,
                    latest.handle_buttons,
                    latest.joystick_x,
                    latest.joystick_y,
                )
                self.get_logger().info("S3 安全位置到达，切回 CCtrl 模式")
                self._mode = "ccctrl"
                self._s3_start_pos = None
                self._s3_start_time = None
            else:
                s = elapsed / dur
                # 梯形速度曲线
                r = 0.3
                Ta = dur * r
                v_max = 1.0 / (dur - Ta)
                a = v_max / Ta
                if elapsed < Ta:
                    s = 0.5 * a * elapsed * elapsed
                elif elapsed < dur - Ta:
                    s = 0.5 * a * Ta * Ta + v_max * (elapsed - Ta)
                else:
                    t_rem = dur - elapsed
                    s = 1.0 - 0.5 * a * t_rem * t_rem
                interp = self._s3_start_pos + s * (self._s3_target - self._s3_start_pos)
                self._publish_positions(
                    interp,
                    latest.main_buttons,
                    latest.handle_buttons,
                    latest.joystick_x,
                    latest.joystick_y,
                )
        elif self._mode == "ccctrl":
            # CCtrl 模式：发布编码器解算的关节位置
            self._publish_positions(
                self._filtered_unwrapped.copy(),
                latest.main_buttons,
                latest.handle_buttons,
                latest.joystick_x,
                latest.joystick_y,
            )
            # 诊断: 每 10s 打印一次发布统计
            self._publish_count += 1
            now_diag = self.get_clock().now()
            if (now_diag - self._last_diag_time).nanoseconds * 1e-9 >= 10.0:
                degs = [f"{math.degrees(v):.1f}°" for v in self._filtered_unwrapped]
                self.get_logger().info(
                    f"[诊断] ccctrl 模式统计(10s): "
                    f"payload_cb={self._payload_cb_count}, "
                    f"publish={self._publish_count}, "
                    f"pos=[{', '.join(degs)}]"
                )
                self._payload_cb_count = 0
                self._publish_count = 0
                self._last_diag_time = now_diag
        else:
            # GOTO 模式：moving → dwell → moving → ...
            wps = self._goto_waypoints
            i = self._goto_wp_index
            elapsed = (now - self._goto_start_time).nanoseconds * 1e-9

            if self._goto_state == "moving":
                seg_start = wps[i]
                seg_end = wps[i + 1]

                # 各按键运动段时长独立
                if self._goto_button == "k3":
                    dur = self._goto_duration_k3
                elif self._goto_button == "k2":
                    dur = self._goto_duration_k2
                else:
                    dur = self._goto_duration

                if elapsed >= dur:
                    # 运动段完成
                    is_turn = (self._goto_turnaround >= 0 and i == self._goto_turnaround)
                    if is_turn:
                        # 折返点：进入长停顿，延迟后触发 S1
                        hold_pos = seg_end.copy()
                        self._goto_state = "dwell"
                        self._goto_start_time = now
                        self._goto_dwell_target = self._goto_dwell
                        self._publish_positions(
                            hold_pos,
                            self._latest_main_buttons,
                            self._latest_handle_buttons,
                            self._latest_joystick_x,
                            self._latest_joystick_y,
                        )
                        self.get_logger().info(
                            f"GOTO 段 {i+1}/{len(wps)-1} 到达 → 停顿 {self._goto_dwell_target:.1f}s"
                            f" [S1 after {self._goto_dwell_delay:.1f}s]"
                        )
                    else:
                        # 非折返点：直接进入下一段，不停顿
                        hold_pos = seg_end.copy()
                        self._publish_positions(
                            hold_pos,
                            self._latest_main_buttons,
                            self._latest_handle_buttons,
                            self._latest_joystick_x,
                            self._latest_joystick_y,
                        )
                        next_idx = i + 1
                        if next_idx + 1 < len(wps):
                            self._goto_wp_index = next_idx
                            self._goto_start_time = now
                            self.get_logger().info(
                                f"GOTO 段 {i+1}/{len(wps)-1} 到达 → 段 {next_idx+1}/{len(wps)-1}"
                            )
                        else:
                            self.get_logger().info(
                                f"GOTO 全部完成 (btn={self._goto_button})，切回 CCtrl 模式"
                            )
                            self._mode = "ccctrl"
                            self._goto_home = None
                            self._goto_waypoints = []
                            self._goto_wp_index = 0
                            self._goto_state = "moving"
                            self._goto_turnaround = -1
                            self._goto_start_time = None
                else:
                    # 梯形速度曲线 (trapezoidal velocity profile)
                    # 加速 → 巡航 → 减速，加速度恒定，运动更平滑可控
                    r = self._goto_accel_frac
                    if r <= 0.0 or r >= 0.5:
                        r = 0.3
                    Ta = dur * r           # 加速段时间
                    v_max = 1.0 / (dur - Ta)  # 巡航速度
                    a = v_max / Ta          # 恒定加速度

                    if elapsed < Ta:
                        s = 0.5 * a * elapsed * elapsed
                    elif elapsed < dur - Ta:
                        # 巡航段：起始位移 + 匀速
                        s = 0.5 * a * Ta * Ta + v_max * (elapsed - Ta)
                    else:
                        t_rem = dur - elapsed
                        s = 1.0 - 0.5 * a * t_rem * t_rem

                    interp = seg_start + s * (seg_end - seg_start)
                    self._publish_positions(
                        interp,
                        self._latest_main_buttons,
                        self._latest_handle_buttons,
                        self._latest_joystick_x,
                        self._latest_joystick_y,
                    )

            else:  # dwell
                # 停在当前段终点
                hold_pos = wps[i + 1].copy()
                is_turn = (self._goto_turnaround >= 0 and i == self._goto_turnaround)
                is_double = self._goto_button in ("k1_double", "k2_double")

                if is_turn and is_double:
                    # ── 双击折返点: 等待外部 S1 触发，不自动发送 S1 ──
                    # 读取 S1 电平 (handle_buttons bit 0x01), 高电平即触发
                    s1_active = bool(hbtn & 0x01)
                    # 检测 K1/K2 按下 → 重来
                    restart_btn = None
                    if rising_m & 0x01:
                        restart_btn = "k1_double"
                    elif rising_m & 0x02:
                        restart_btn = "k2_double"

                    if s1_active:
                        self.get_logger().info(
                            "GOTO 双击折返点: 检测到 S1 触发 → 继续执行"
                        )
                        next_idx = i + 1
                        if next_idx + 1 < len(wps):
                            self._goto_wp_index = next_idx
                            self._goto_state = "moving"
                            self._goto_start_time = now
                            self.get_logger().info(
                                f"GOTO 段 {i+1}/{len(wps)-1} S1触发 "
                                f"→ 段 {next_idx+1}/{len(wps)-1}"
                            )
                        else:
                            self.get_logger().info(
                                f"GOTO 全部完成 (btn={self._goto_button})，切回 CCtrl 模式"
                            )
                            self._mode = "ccctrl"
                            self._goto_home = None
                            self._goto_waypoints = []
                            self._goto_wp_index = 0
                            self._goto_state = "moving"
                            self._goto_turnaround = -1
                            self._goto_start_time = None
                    elif restart_btn is not None:
                        self.get_logger().info(
                            f"GOTO 双击折返点: {restart_btn} 按下 → 重新触发"
                        )
                        self._enter_goto(restart_btn)
                    else:
                        # 等待中, 透传 handle_buttons (不自动置 S1)
                        self._publish_positions(
                            hold_pos,
                            self._latest_main_buttons,
                            hbtn,
                            self._latest_joystick_x,
                            self._latest_joystick_y,
                        )
                else:
                    # ── 单击折返点或非折返点停顿: 原有逻辑 ──
                    if is_turn and elapsed >= self._goto_dwell_delay:
                        hbtn_pub = 0x01  # 折返点延迟后自动触发 S1
                    else:
                        hbtn_pub = self._latest_handle_buttons
                    self._publish_positions(
                        hold_pos,
                        self._latest_main_buttons,
                        hbtn_pub,
                        self._latest_joystick_x,
                        self._latest_joystick_y,
                    )

                    if elapsed >= self._goto_dwell_target:
                        next_idx = i + 1
                        if next_idx + 1 < len(wps):
                            self._goto_wp_index = next_idx
                            self._goto_state = "moving"
                            self._goto_start_time = now
                            self.get_logger().info(
                                f"GOTO 停顿结束 → 段 {next_idx+1}/{len(wps)-1}"
                            )
                        else:
                            self.get_logger().info(
                                f"GOTO 全部完成 (btn={self._goto_button})，切回 CCtrl 模式"
                            )
                            self._mode = "ccctrl"
                            self._goto_home = None
                            self._goto_waypoints = []
                            self._goto_wp_index = 0
                            self._goto_state = "moving"
                            self._goto_turnaround = -1
                            self._goto_start_time = None

def main(args=None):
    rclpy.init(args=args)
    node = CCtrlJointProcessor()

    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
