#!/usr/bin/env python3
"""
串口接收节点 — 解析下位机上报的 6 关节位置，发布到 real_positions 话题。

帧协议 (28 字节):
  byte[0]       0xAA
  byte[1:25]    6×float32 (little-endian) — 关节位置 (rad)
  byte[25]      XOR(byte[1:25]) & 0xFF     — 校验和
  byte[26]      0x55                       — 帧尾

用法:
  ros2 run arm_driver serial_dm_receiver.py
  ros2 run arm_driver serial_dm_receiver.py --ros-args -p port:=/dev/ttyUSB0
  ros2 run arm_driver serial_dm_receiver.py --ros-args -p publish_rate:=50.0
"""

import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import JointState
import serial

FRAME_HEADER = 0xAA
FRAME_FOOTER = 0x55
FRAME_LEN = 28
NUM_JOINTS = 6
JOINT_NAMES = ["J1", "J2", "J3", "J4", "J5", "J6"]


class SerialDmReceiver(Node):
    """串口读取 → JointState 发布"""

    def __init__(self):
        super().__init__("serial_dm_receiver")

        # ── 参数 ──────────────────────────────────────────────
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 921600)
        self.declare_parameter("publish_rate", 50.0)

        port = self.get_parameter("port").get_parameter_value().string_value
        baudrate = self.get_parameter("baudrate").get_parameter_value().integer_value
        self._pub_rate = (
            self.get_parameter("publish_rate").get_parameter_value().double_value
        )

        # ── 串口 ──────────────────────────────────────────────
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=0.01,
            write_timeout=0.1,
            dsrdtr=False,
        )
        self.get_logger().info(f"串口已打开: {port} @ {baudrate}")
        self._buf = b""  # 接收缓冲区

        # ── Publisher ─────────────────────────────────────────
        self._pub = self.create_publisher(JointState, "real_positions", 10)

        # ── 定时器 — 读取串口并发布 ───────────────────────────
        dt = 1.0 / self._pub_rate if self._pub_rate > 0 else 0.01
        self._timer = self.create_timer(dt, self._read_and_publish)
        self._seq = 0

        # 统计
        self._frame_count = 0
        self._error_count = 0
        self._last_stats = time.time()

        self.get_logger().info(
            f"开始接收, publish_rate={self._pub_rate:.1f} Hz "
            f"→ topic: {self._pub.topic_name}"
        )

    # ── 帧解析 ─────────────────────────────────────────────────
    def _try_parse(self) -> tuple[list[float], int] | None:
        """尝试从缓冲区解析一帧。返回 (positions, consumed) 或 None。"""
        buf = self._buf

        # 找帧头 0xAA
        hdr_idx = buf.find(bytes([FRAME_HEADER]))
        if hdr_idx < 0:
            return None

        # 丢弃帧头之前的字节
        if hdr_idx > 0:
            self._buf = buf[hdr_idx:]
            consumed = hdr_idx
            buf = self._buf
        else:
            consumed = 0

        # 等待完整帧
        if len(buf) < FRAME_LEN:
            return None

        # 检查帧尾
        if buf[FRAME_LEN - 1] != FRAME_FOOTER:
            self._buf = buf[1:]  # 跳过虚假帧头
            return None

        payload = buf[1:25]  # 24 bytes = 6 float32

        # 校验
        expected_xor = buf[25]
        actual_xor = sum(payload) & 0xFF
        if expected_xor != actual_xor:
            self._buf = buf[1:]  # 跳过虚假帧头
            self._error_count += 1
            return None

        # 解析
        positions = list(struct.unpack("<6f", payload))
        consumed += FRAME_LEN
        return (positions, consumed)

    # ── 定时回调 ───────────────────────────────────────────────
    def _read_and_publish(self):
        """读取串口数据，解析帧并发布"""
        # 读取
        try:
            chunk = self.ser.read(self.ser.in_waiting or 1024)
            if chunk:
                self._buf += chunk
        except serial.SerialException as e:
            self.get_logger().error(f"串口读取错误: {e}")
            return

        # 解析所有可用帧，只发布最新一帧
        latest = None
        while True:
            result = self._try_parse()
            if result is None:
                break
            positions, consumed = result
            self._buf = self._buf[consumed:]
            latest = positions
            self._frame_count += 1

        # 发布最新位置
        if latest is not None:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            msg.name = JOINT_NAMES
            msg.position = latest
            self._pub.publish(msg)

        # 统计日志 (每 10s)
        now = time.time()
        if now - self._last_stats >= 10.0:
            self._last_stats = now
            self.get_logger().info(
                f"接收: {self._frame_count} 帧 (最近 10s), "
                f"校验错误: {self._error_count}"
            )
            self._frame_count = 0
            self._error_count = 0


def main(args=None):
    rclpy.init(args=args)
    node = SerialDmReceiver()

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