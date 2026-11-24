#!/usr/bin/env python3
"""
FollowJointTrajectory action server → 串口收发 (位置 + 重力前馈力矩)

架构:
  - 接收定时器: 持续读串口 → 解析真实位置 → 立即计算 G(q_real) → 更新 _latest_tau
  - 发送定时器: 持续运行，限幅逼近目标位置 → 打包 cmd + _latest_tau → 串口发送
  - MoveIt Action: 只负责更新目标位置 _goal_cmd，不直接操作串口

发送协议 (51 字节):
  0xAA | 6×float32 positions | 6×float32 tau_ff | XOR(48B) | 0x55

接收协议 (27 字节):
  0xAA | 6×float32 positions | SUM(24B) | 0x55

用法:
  ros2 run arm_driver serial_dm_feedforward_bridge.py
  ros2 run arm_driver serial_dm_feedforward_bridge.py --ros-args -p port:=/dev/ttyUSB0
  ros2 run arm_driver serial_dm_feedforward_bridge.py --ros-args -p enable_feedforward:=false
"""

import struct
import time
import os
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
import serial

try:
    import pinocchio

    HAS_PINOCCHIO = True
except ImportError:
    HAS_PINOCCHIO = False

# ═══════════════════════════════════════════════════════════════════
# 协议常量
# ═══════════════════════════════════════════════════════════════════

FRAME_START = 0xAA
FRAME_END = 0x55
NUM_JOINTS = 6
MAX_STEP = 0.5          # 单步最大位置变化 (rad)，安全限幅
JOINT_NAMES = ["J1", "J2", "J3", "J4", "J5", "J6"]

# 发送帧 (51 字节):
#   byte[0]       0xAA
#   byte[1:25]    6×float32  — 关节位置 (rad)
#   byte[25:49]   6×float32  — 前馈力矩 (Nm)
#   byte[49]      XOR(byte[1:49]) & 0xFF
#   byte[50]      0x55

TX_FRAME_LEN = 51

# 接收帧 (27 字节):
#   byte[0]       0xAA
#   byte[1:25]    6×float32  — 真实关节位置 (rad)
#   byte[25]      SUM(byte[1:25]) & 0xFF
#   byte[26]      0x55

RX_FRAME_LEN = 27

SUCCESSFUL = 0
INVALID_GOAL = 1
INVALID_JOINTS = 2


def pack_frame(positions: list[float], torques: list[float]) -> bytes:
    """打包位置 + 前馈力矩为一帧"""
    pos_bytes = struct.pack("<6f", *positions)
    tau_bytes = struct.pack("<6f", *torques)
    payload = pos_bytes + tau_bytes  # 48 bytes
    checksum = sum(payload) & 0xFF
    return bytes([FRAME_START]) + payload + bytes([checksum, FRAME_END])


def find_urdf() -> str:
    """查找 new_hj_arm2 的 URDF 文件"""
    try:
        from ament_index_python import get_package_share_directory

        path = os.path.join(
            get_package_share_directory("new_hj_arm2"), "urdf", "new_hj_arm2.urdf"
        )
        if os.path.exists(path):
            return path
    except Exception:
        pass

    import subprocess

    try:
        r = subprocess.run(
            ["ros2", "pkg", "prefix", "new_hj_arm2"],
            capture_output=True,
            text=True,
        )
        path = os.path.join(
            r.stdout.strip(), "share", "new_hj_arm2", "urdf", "new_hj_arm2.urdf"
        )
        if os.path.exists(path):
            return path
    except Exception:
        pass

    d = os.path.dirname(os.path.abspath(__file__))
    path = os.path.normpath(
        os.path.join(d, "..", "..", "new_hj_arm2", "urdf", "new_hj_arm2.urdf")
    )
    if os.path.exists(path):
        return path

    raise FileNotFoundError("无法找到 URDF 文件。请先 source install/setup.bash")


def load_model(urdf_path: str):
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"URDF 文件不存在: {urdf_path}")
    return pinocchio.buildModelFromUrdf(urdf_path)


# ═══════════════════════════════════════════════════════════════════
# 工具函数
# ═══════════════════════════════════════════════════════════════════


def build_joint_map(traj_joint_names: list[str]) -> dict[int, int] | None:
    mapping = {}
    for our_idx, jn in enumerate(JOINT_NAMES):
        if jn in traj_joint_names:
            mapping[traj_joint_names.index(jn)] = our_idx
        else:
            return None
    return mapping


def slew_toward(current: list[float], target: list[float]) -> list[float]:
    out = []
    for c, t in zip(current, target):
        delta = t - c
        if abs(delta) <= MAX_STEP:
            out.append(t)
        else:
            out.append(c + (1.0 if delta > 0 else -1.0) * MAX_STEP)
    return out


def max_delta(a: list[float], b: list[float]) -> float:
    return max(abs(a[i] - b[i]) for i in range(NUM_JOINTS))


# ═══════════════════════════════════════════════════════════════════
# 主节点
# ═══════════════════════════════════════════════════════════════════


class ArmSerialFeedforwardNode(Node):
    """串口桥接节点 — 接收驱动前馈，发送由 MoveIt 目标位置驱动"""

    def __init__(self):
        super().__init__("arm_serial_feedforward_node")

        # ── 参数 ──────────────────────────────────────────────
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 921600)
        self.declare_parameter("send_rate", 10.0)
        self.declare_parameter("receive_rate", 200.0)
        self.declare_parameter(
            "action_name", "/hj_arm_group_controller/follow_joint_trajectory"
        )
        self.declare_parameter(
            "urdf_path",
            "/home/hj/NEW_test_6_15(1)/NEW_test_6_15/src/new_hj_arm2/urdf/new_hj_arm2.urdf",
        )
        self.declare_parameter("enable_feedforward", True)
        self.declare_parameter("j2_tau_gain", 1.0)   # J2 幅值增益，不影响其他关节
        self.declare_parameter("j2_tau_scale", 1.0)
        self.declare_parameter("j2_tau_offset", 1.0)  # J2 力矩手动偏置 (Nm)，标定用
        self.declare_parameter("j3_tau_gain", 1.0)   # J3 幅值增益
        self.declare_parameter("j3_tau_offset", 1.0)  # J3 力矩手动偏置 (Nm)，标定用
        self.declare_parameter("tau_filter_alpha", 0.05)

        port = self.get_parameter("port").get_parameter_value().string_value
        baudrate = self.get_parameter("baudrate").get_parameter_value().integer_value
        self._send_rate = (
            self.get_parameter("send_rate").get_parameter_value().double_value
        )
        self._receive_rate = (
            self.get_parameter("receive_rate").get_parameter_value().double_value
        )
        action_name = (
            self.get_parameter("action_name").get_parameter_value().string_value
        )
        urdf_path_param = (
            self.get_parameter("urdf_path").get_parameter_value().string_value
        )
        self._enable_ff = (
            self.get_parameter("enable_feedforward").get_parameter_value().bool_value
        )
        self._j2_tau_scale = (
            self.get_parameter("j2_tau_scale").get_parameter_value().double_value
        )
        self._j2_tau_gain = (
            self.get_parameter("j2_tau_gain").get_parameter_value().double_value
        )
        self._j2_tau_offset = (
            self.get_parameter("j2_tau_offset").get_parameter_value().double_value
        )
        self._j3_tau_gain = (
            self.get_parameter("j3_tau_gain").get_parameter_value().double_value
        )
        self._j3_tau_offset = (
            self.get_parameter("j3_tau_offset").get_parameter_value().double_value
        )
        self._tau_filter_alpha = (
            self.get_parameter("tau_filter_alpha").get_parameter_value().double_value
        )

        # ── 加载 URDF + Pinocchio 模型 ─────────────────────────
        self._model = None
        self._pin_data = None

        if self._enable_ff:
            if not HAS_PINOCCHIO:
                self.get_logger().warn(
                    "Pinocchio 未安装，前馈力矩将置零。"
                )
                self._enable_ff = False
            else:
                try:
                    urdf_path = urdf_path_param if urdf_path_param else find_urdf()
                    self._model = load_model(urdf_path)
                    self._pin_data = self._model.createData()
                    self.get_logger().info(
                        f"模型已加载: {self._model.name} "
                        f"({self._model.nq} DOF, {self._model.njoints} joints)"
                    )
                except Exception as e:
                    self.get_logger().error(f"URDF 加载失败: {e}")
                    self._enable_ff = False

        # ── 共享状态 (线程安全) ────────────────────────────────
        self._lock = threading.Lock()
        self._cmd: list[float] = [0.0] * NUM_JOINTS       # 当前发送位置
        self._goal_cmd: list[float] = [0.0] * NUM_JOINTS  # MoveIt 目标位置
        self._latest_tau: list[float] = [0.0] * NUM_JOINTS # 最新前馈力矩
        self._real_positions: list[float] = [0.0] * NUM_JOINTS
        self._filtered_tau: list[float] = [0.0] * NUM_JOINTS  # EMA 滤波后的力矩
        self._initialized = False

        # ── 串口 ──────────────────────────────────────────────
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=0.01,
            write_timeout=0.1,
            dsrdtr=False,
        )
        self.get_logger().info(f"串口已打开: {port} @ {baudrate}")
        self._rx_buf = b""

        # ── Publisher: 真实位置 ────────────────────────────────
        self._real_pos_pub = self.create_publisher(JointState, "real_positions", 10)

        # ── 接收统计 ──────────────────────────────────────────
        self._rx_frame_count = 0
        self._rx_error_count = 0
        self._last_rx_stats = time.time()
        self._last_rx_log = 0.0

        # ── 发送统计 ──────────────────────────────────────────
        self._tx_frame_count = 0
        self._last_tx_log = 0.0

        # ── 接收定时器 (高频，串口读 + 实时前馈) ──────────────
        rx_dt = 1.0 / self._receive_rate if self._receive_rate > 0 else 0.005
        self._rx_timer = self.create_timer(rx_dt, self._read_serial_cb)

        # ── 发送定时器 (持续运行，与 MoveIt 解耦) ─────────────
        tx_dt = 1.0 / self._send_rate if self._send_rate > 0 else 0.1
        self._send_timer = self.create_timer(tx_dt, self._send_cb)

        # ── Action Server ─────────────────────────────────────
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            action_name,
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=ReentrantCallbackGroup(),
        )

        self.get_logger().info(
            f"等待 {action_name} goal ... "
            f"发送: {self._send_rate} Hz, 接收+前馈: {self._receive_rate} Hz"
        )

    # ═══════════════════════════════════════════════════════════════
    # 串口接收 + 实时前馈计算
    # ═══════════════════════════════════════════════════════════════

    def _try_parse_rx(self) -> tuple[list[float], int] | None:
        """从接收缓冲区解析一帧。返回 (positions, consumed_bytes) 或 None。"""
        buf = self._rx_buf

        hdr_idx = buf.find(bytes([FRAME_START]))
        if hdr_idx < 0:
            return None

        if hdr_idx > 0:
            self._rx_buf = buf[hdr_idx:]
            consumed = hdr_idx
            buf = self._rx_buf
        else:
            consumed = 0

        if len(buf) < RX_FRAME_LEN:
            return None

        if buf[RX_FRAME_LEN - 1] != FRAME_END:
            self._rx_buf = buf[1:]
            return None

        payload = buf[1:25]  # 24 bytes = 6 float32

        expected_sum = buf[25]
        actual_sum = sum(payload) & 0xFF
        if expected_sum != actual_sum:
            self._rx_buf = buf[1:]
            self._rx_error_count += 1
            return None

        positions = list(struct.unpack("<6f", payload))
        consumed += RX_FRAME_LEN
        return (positions, consumed)

    def _compute_feedforward(self, q: list[float]) -> list[float]:
        """计算重力补偿力矩 G(q)"""
        if not self._enable_ff or self._model is None:
            return [0.0] * NUM_JOINTS
        q_np = np.array(q, dtype=np.float64)
        G = pinocchio.computeGeneralizedGravity(self._model, self._pin_data, q_np)
        return G.tolist()

    def _read_serial_cb(self):
        """定时读串口 → 解析真实位置 → 立即计算前馈力矩"""
        # 读取
        try:
            waiting = self.ser.in_waiting
            if waiting > 0:
                chunk = self.ser.read(waiting)
                self._rx_buf += chunk
        except serial.SerialException as e:
            self.get_logger().error(f"串口读取错误: {e}")
            return

        # 解析所有可用帧
        latest_pos = None
        while True:
            result = self._try_parse_rx()
            if result is None:
                break
            positions, consumed = result
            self._rx_buf = self._rx_buf[consumed:]
            latest_pos = positions
            self._rx_frame_count += 1

        if latest_pos is None:
            return

        # 更新真实位置
        with self._lock:
            self._real_positions = latest_pos

        # 首次收到数据时，用真实位置初始化 cmd/goal_cmd
        if not self._initialized:
            with self._lock:
                self._cmd = list(latest_pos)
                self._goal_cmd = list(latest_pos)
            self._initialized = True
            self.get_logger().info("已从串口接收到初始位置，开始发送")

        # 发布 real_positions (原始位置 + 对应的力矩，仅用于监控)
        # 注: 发送给 STM32 的 tau 在 _send_cb 中用限幅后的 _cmd 计算，更平滑
        tau_monitor = self._compute_feedforward(latest_pos)
        tau_monitor[1] = tau_monitor[1] * self._j2_tau_scale * self._j2_tau_gain + self._j2_tau_offset
        tau_monitor[1] = max(-10.0, min(10.0, tau_monitor[1]))
        tau_monitor[2] = tau_monitor[2] * self._j3_tau_gain + self._j3_tau_offset
        tau_monitor[2] = max(-10.0, min(10.0, tau_monitor[2]))
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.name = JOINT_NAMES
        msg.position = latest_pos
        msg.effort = tau_monitor
        self._real_pos_pub.publish(msg)

        # 日志 (每 0.5s)
        now = time.time()
        if now - self._last_rx_log >= 0.5:
            self._last_rx_log = now
            pos_str = "  ".join(
                f"{JOINT_NAMES[i]}: {latest_pos[i]:+8.4f}" for i in range(NUM_JOINTS)
            )
            tau_str = "  ".join(
                f"{JOINT_NAMES[i]}: {tau_monitor[i]:+8.4f}" for i in range(NUM_JOINTS)
            )
            self.get_logger().info(
                f"[接收] pos: [{pos_str}]\n"
                f"       raw: [{tau_str}]"
            )

        # 统计 (每 10s)
        if now - self._last_rx_stats >= 10.0:
            self._last_rx_stats = now
            self._rx_frame_count = 0
            self._rx_error_count = 0

    # ═══════════════════════════════════════════════════════════════
    # 串口发送 (持续运行)
    # ═══════════════════════════════════════════════════════════════

    def _send_cb(self):
        """定时发送: 限幅 → 打包 cmd + latest_tau → 串口"""
        if not self._initialized:
            return

        with self._lock:
            nxt = slew_toward(self._cmd, self._goal_cmd)

        # 用限幅后的发送位置计算前馈 (天然平滑，不会突变)
        tau = self._compute_feedforward(nxt)
        tau[1] = tau[1] * self._j2_tau_scale * self._j2_tau_gain + self._j2_tau_offset
        tau[1] = max(-10.0, min(10.0, tau[1]))
        tau[2] = tau[2] * self._j3_tau_gain + self._j3_tau_offset
        tau[2] = max(-10.0, min(10.0, tau[2]))

        frame = pack_frame(nxt, tau)
        try:
            self.ser.write(frame)
            self.ser.flush()
        except serial.SerialException as e:
            self.get_logger().error(f"串口发送错误: {e}")
            return

        with self._lock:
            self._cmd = nxt

        self._tx_frame_count += 1

        # 日志 (每 1s)
        now = time.time()
        if now - self._last_tx_log >= 1.0:
            self._last_tx_log = now
            tau_str = "  ".join(f"{tau[i]:+8.4f}" for i in range(NUM_JOINTS))
            # self.get_logger().info(
            #     f"[发送] pos: [{tau_str}]"  # 实际打印的是 tau
            # )
            # self.get_logger().info(
            #     f"       tau: [{tau_str}]"
            # )

    # ═══════════════════════════════════════════════════════════════
    # MoveIt Action 回调 (只负责更新目标位置)
    # ═══════════════════════════════════════════════════════════════

    def _goal_cb(self, goal_request) -> GoalResponse:
        traj = goal_request.trajectory
        if not traj.joint_names or len(traj.points) == 0:
            self.get_logger().warn("Goal 被拒: 轨迹为空")
            return GoalResponse.REJECT
        if build_joint_map(traj.joint_names) is None:
            self.get_logger().warn(f"Goal 被拒: 关节名不匹配, 需要 {JOINT_NAMES}")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_cb(self, _goal_handle) -> CancelResponse:
        self.get_logger().info("Goal 取消")
        self._cancelled = True
        return CancelResponse.ACCEPT

    def _execute_cb(self, goal_handle):
        """轨迹执行: 只负责按时间插值更新 _goal_cmd，串口I/O 由 _send_cb 处理"""
        traj = goal_handle.request.trajectory
        joint_map = build_joint_map(traj.joint_names)

        # 映射 trajectory points
        points: list[JointTrajectoryPoint] = []
        for pt in traj.points:
            mapped = JointTrajectoryPoint()
            mapped.positions = [0.0] * NUM_JOINTS
            for ti, oi in (joint_map or {}).items():
                if ti < len(pt.positions):
                    mapped.positions[oi] = pt.positions[ti]
            if pt.velocities:
                mapped.velocities = [0.0] * NUM_JOINTS
                for ti, oi in (joint_map or {}).items():
                    if ti < len(pt.velocities):
                        mapped.velocities[oi] = pt.velocities[ti]
            mapped.time_from_start = pt.time_from_start
            points.append(mapped)

        traj_duration = (
            points[-1].time_from_start.sec
            + points[-1].time_from_start.nanosec * 1e-9
        )
        final_target = list(points[-1].positions)

        self._cancelled = False
        t_start = self.get_clock().now().nanoseconds * 1e-9

        self.get_logger().info(
            f"开始执行轨迹: {traj_duration:.2f}s, {len(points)} 个路径点"
        )

        result = FollowJointTrajectory.Result()
        dt = 1.0 / self._send_rate
        next_tick = time.perf_counter()

        while rclpy.ok():
            if self._cancelled:
                result.error_code = INVALID_GOAL
                result.error_string = "cancelled"
                goal_handle.abort()
                self.get_logger().info("轨迹已中止")
                return result

            now = self.get_clock().now().nanoseconds * 1e-9
            elapsed = now - t_start
            t = max(0.0, min(elapsed, traj_duration))

            # 插值 → 更新目标位置 (发送由 _send_cb 独立完成)
            desired = self._interpolate(points, t)
            if desired is not None:
                with self._lock:
                    self._goal_cmd = desired

            # 轨迹时间结束 → 等 cmd 逼近最终点
            if elapsed >= traj_duration:
                if max_delta(self._cmd, final_target) < 1e-6:
                    result.error_code = SUCCESSFUL
                    result.error_string = ""
                    goal_handle.succeed()
                    self.get_logger().info("轨迹执行完成")
                    return result

            # 频率控制
            next_tick += dt
            sleep = next_tick - time.perf_counter()
            if sleep > 0.0:
                time.sleep(sleep)
            elif sleep < -dt:
                next_tick = time.perf_counter()

        result.error_code = INVALID_GOAL
        result.error_string = "shutdown"
        return result

    # ── 轨迹插值 ───────────────────────────────────────────────
    def _interpolate(
        self, points: list[JointTrajectoryPoint], t: float
    ) -> list[float] | None:
        if not points:
            return None

        t0 = points[0].time_from_start.sec + points[0].time_from_start.nanosec * 1e-9
        if t <= t0:
            return list(points[0].positions)

        for i in range(len(points) - 1):
            t0_i = (
                points[i].time_from_start.sec
                + points[i].time_from_start.nanosec * 1e-9
            )
            t1 = (
                points[i + 1].time_from_start.sec
                + points[i + 1].time_from_start.nanosec * 1e-9
            )
            if t <= t1:
                if t1 - t0_i < 1e-9:
                    return list(points[i + 1].positions)
                alpha = (t - t0_i) / (t1 - t0_i)
                alpha = max(0.0, min(1.0, alpha))
                result = [0.0] * NUM_JOINTS
                for j in range(
                    min(
                        NUM_JOINTS,
                        len(points[i].positions),
                        len(points[i + 1].positions),
                    )
                ):
                    result[j] = points[i].positions[j] + alpha * (
                        points[i + 1].positions[j] - points[i].positions[j]
                    )
                return result

        return list(points[-1].positions)


def main(args=None):
    rclpy.init(args=args)
    node = ArmSerialFeedforwardNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()