#!/usr/bin/env python3
"""
FollowJointTrajectory action server → 串口发送。

接收 MoveIt2 发来的轨迹，按时间插值后通过串口发送给下位机。
MAX_STEP 仅作安全上限，正常轨迹不会触发。

协议: 0xAA | 6×float32 LE | XOR(24B) | 0x55  (27 字节)
"""

import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import serial

FRAME_START = 0xAA
FRAME_END = 0x55
NUM_JOINTS = 6
MAX_STEP = 0.5          
JOINT_NAMES = [
    'J1', 'J2', 'J3',
    'J4', 'J5', 'J6',
]

SUCCESSFUL = 0
INVALID_GOAL = 1
INVALID_JOINTS = 2


def pack_frame(positions: list[float]) -> bytes:
    joint_bytes = struct.pack('<6f', *positions)
    checksum = sum(joint_bytes) & 0xFF
    return bytes([FRAME_START]) + joint_bytes + bytes([checksum, FRAME_END])


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


class ArmSerialNode(Node):
    def __init__(self):  
        super().__init__('arm_serial_node')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 921600)
        self.declare_parameter('send_rate', 10.0)
        self.declare_parameter('action_name', '/hj_arm_group_controller/follow_joint_trajectory')

        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self._send_rate = self.get_parameter('send_rate').get_parameter_value().double_value
        action_name = self.get_parameter('action_name').get_parameter_value().string_value

        # 当前位置 / 目标
        self._cmd: list[float] = [0.0] * NUM_JOINTS
        self._goal_cmd: list[float] = [0.0] * NUM_JOINTS
        self._initialized = False
        self._cancelled = False

        # 串口
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=0.01,
            write_timeout=0.1,
        )

        # FollowJointTrajectory action server
        # ReentrantCallbackGroup: cancel_cb 可在 execute_cb sleep 时被调用
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            action_name,
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=ReentrantCallbackGroup(),
        )

        print(f'[arm_serial_node] 等待 {action_name} goal ...')

    # ------------------------------------------------------------------
    # Action 回调
    # ------------------------------------------------------------------
    def _goal_cb(self, goal_request) -> GoalResponse:
        traj = goal_request.trajectory
        if not traj.joint_names or len(traj.points) == 0:
            return GoalResponse.REJECT
        if build_joint_map(traj.joint_names) is None:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_cb(self, _goal_handle) -> CancelResponse:
        self._cancelled = True
        return CancelResponse.ACCEPT

    def _execute_cb(self, goal_handle):
        """执行轨迹：全部发送逻辑在此，无 timer，无竞态。"""
        traj = goal_handle.request.trajectory
        joint_map = build_joint_map(traj.joint_names)

        # ---- 映射 trajectory points ----
        points = []
        for pt in traj.points:
            mapped = JointTrajectoryPoint()
            mapped.positions = [0.0] * NUM_JOINTS
            for ti, oi in (joint_map or {}).items():
                if ti < len(pt.positions):
                    mapped.positions[oi] = pt.positions[ti]
            mapped.time_from_start = pt.time_from_start
            points.append(mapped)

        traj_duration = (
            points[-1].time_from_start.sec
            + points[-1].time_from_start.nanosec * 1e-9
        )
        final_target = list(points[-1].positions)

        if not self._initialized:
            self._cmd = list(points[0].positions)
            self._goal_cmd = list(points[0].positions)
            self._initialized = True

        self._cancelled = False
        t_start = self.get_clock().now().nanoseconds * 1e-9

        # ---- 发送循环 ----
        dt = 1.0 / self._send_rate
        next_tick = time.perf_counter()

        result = FollowJointTrajectory.Result()

        while rclpy.ok():
            if self._cancelled:
                result.error_code = INVALID_GOAL
                result.error_string = 'cancelled'
                goal_handle.abort()
                return result

            now = self.get_clock().now().nanoseconds * 1e-9
            elapsed = now - t_start
            t = max(0.0, min(elapsed, traj_duration))

            # 插值期望位置
            desired = self._interpolate(points, t)
            if desired is not None:
                self._goal_cmd = desired

            # 限幅 → 发送
            nxt = slew_toward(self._cmd, self._goal_cmd)
            self.ser.write(pack_frame(nxt))
            self.ser.flush()
            self._cmd = nxt

            print(
                f'[{nxt[0]:.4f}, {nxt[1]:.4f}, {nxt[2]:.4f}, '
                f'{nxt[3]:.4f}, {nxt[4]:.4f}, {nxt[5]:.4f}]'
            )

            # 轨迹时间结束 → 等 cmd 逼近最终点
            if elapsed >= traj_duration:
                if max_delta(self._cmd, final_target) < 1e-6:
                    result.error_code = SUCCESSFUL
                    result.error_string = ''
                    goal_handle.succeed()
                    return result

            # 帧率控制
            next_tick += dt
            sleep = next_tick - time.perf_counter()
            if sleep > 0.0:
                time.sleep(sleep)
            elif sleep < -dt:
                next_tick = time.perf_counter()

        result.error_code = INVALID_GOAL
        result.error_string = 'shutdown'
        return result

    # ------------------------------------------------------------------
    # 轨迹插值
    # ------------------------------------------------------------------
    def _interpolate(self, points: list[JointTrajectoryPoint], t: float) -> list[float] | None:
        if not points:
            return None

        t0 = points[0].time_from_start.sec + points[0].time_from_start.nanosec * 1e-9
        if t <= t0:
            return list(points[0].positions)

        for i in range(len(points) - 1):
            t0_i = points[i].time_from_start.sec + points[i].time_from_start.nanosec * 1e-9
            t1 = points[i + 1].time_from_start.sec + points[i + 1].time_from_start.nanosec * 1e-9
            if t <= t1:
                if t1 - t0_i < 1e-9:
                    return list(points[i + 1].positions)
                alpha = (t - t0_i) / (t1 - t0_i)
                alpha = max(0.0, min(1.0, alpha))
                result = [0.0] * NUM_JOINTS
                for j in range(min(NUM_JOINTS, len(points[i].positions), len(points[i + 1].positions))):
                    result[j] = points[i].positions[j] + alpha * (points[i + 1].positions[j] - points[i].positions[j])
                return result

        # t >= last point time
        return list(points[-1].positions)


def main(args=None):
    rclpy.init(args=args)
    node = ArmSerialNode()

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


if __name__ == '__main__':
    main()
