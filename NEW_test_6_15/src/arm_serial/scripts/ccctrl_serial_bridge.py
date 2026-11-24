#!/usr/bin/env python3
"""
串口收发节点 — 订阅 /ccctrl_positions → 串口发送 + 串口接收 → 解析位置。

发送帧 (51 字节):
  byte[0]       0xAA
  byte[1:25]    6×float32  — 关节位置 (rad)
  byte[25]      uint8      — main_buttons
  byte[26]      uint8      — handle_buttons
  byte[27:29]   uint16 LE  — joystick_x
  byte[29:31]   uint16 LE  — joystick_y
  byte[31:58]   27 字节 TcInfo (3h mouse + 3B btn + 16B keys + 2x pad)
  byte[58]      SUM(byte[1:58]) & 0xFF
  byte[59]      0x55

接收帧 (27 字节):
  byte[0]       0xAA
  byte[1:25]    6×float32  — 真实关节位置 (rad)
  byte[25]      SUM(byte[1:25]) & 0xFF
  byte[26]      0x55

"""

import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from arm_serial.msg import CCtrlJoint, TcInfo
from sensor_msgs.msg import JointState
import serial

# ── 帧常量 ────────────────────────────────────────────────────────
FRAME_START = 0xAA
FRAME_END   = 0x55
NUM_JOINTS  = 6
TX_FRAME_LEN = 60
RX_FRAME_LEN = 27


def compute_sum(data: bytes) -> int:
    """逐字节求和后取低 8 位"""
    return sum(data) & 0xFF


def pack_tx_frame(
    positions: list[float],
    main_buttons: int = 0,
    handle_buttons: int = 0,
    joystick_x: int = 2048,
    joystick_y: int = 2048,
    tc_raw: bytes = b"",
) -> bytes:
    """打包发送帧 — 6×float32 关节位置 + CCtrl 面板数据 + TcInfo 27B（60 字节）"""
    buf = bytearray(TX_FRAME_LEN)
    buf[0] = FRAME_START
    # 6×float32 positions
    for i, p in enumerate(positions[:NUM_JOINTS]):
        struct.pack_into("<f", buf, 1 + i * 4, float(p))
    # CCtrl 面板数据
    buf[25] = main_buttons & 0xFF
    buf[26] = handle_buttons & 0xFF
    struct.pack_into("<H", buf, 27, joystick_x & 0xFFFF)
    struct.pack_into("<H", buf, 29, joystick_y & 0xFFFF)
    # TcInfo 27 字节 (byte[31:58])
    if len(tc_raw) >= 27:
        buf[31:58] = tc_raw[:27]
    # SUM checksum (覆盖 byte[1:58])
    buf[58] = compute_sum(buf[1:58])
    buf[59] = FRAME_END
    return bytes(buf)


def parse_rx_frame(data: bytes) -> list[float] | None:
    """解析接收帧，返回 6 个 float32 位置，失败返回 None"""
    if len(data) < RX_FRAME_LEN:
        return None
    if data[0] != FRAME_START or data[RX_FRAME_LEN - 1] != FRAME_END:
        return None

    payload = data[1:25]
    expected_sum = data[25]
    if compute_sum(payload) != expected_sum:
        return None

    positions = list(struct.unpack("<6f", payload))
    return positions


# ═══════════════════════════════════════════════════════════════════
# 串口桥接节点
# ═══════════════════════════════════════════════════════════════════

class CCtrlSerialBridge(Node):
    """订阅 /ccctrl_positions → 串口发送；串口接收 → 日志输出"""

    def __init__(self):
        super().__init__("ccctrl_serial_bridge")

        # ── 参数 ──────────────────────────────────────────────
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("reconnect_max_delay", 5.0)  # 最大重连间隔 (s)
        self.declare_parameter("watchdog_timeout", 15.0)  # 数据看门狗超时(秒)

        self._port = (
            self.get_parameter("port").get_parameter_value().string_value
        )
        self._baudrate = (
            self.get_parameter("baudrate")
            .get_parameter_value()
            .integer_value
        )
        self._reconnect_max_delay = (
            self.get_parameter("reconnect_max_delay")
            .get_parameter_value()
            .double_value
        )
        self._watchdog_timeout = (
            self.get_parameter("watchdog_timeout")
            .get_parameter_value()
            .double_value
        )

        # ── 串口 ──────────────────────────────────────────────
        self.ser: serial.Serial | None = None
        self._connected = False
        self._reconnect_backoff = 0.0  # 当前重连等待时间
        self._last_reconnect_attempt = 0.0

        # ── 数据看门狗：超过 _watchdog_timeout 秒无收发成功则重连 ──
        self._last_data_time = time.time()
        self._watchdog_triggered = False

        self._open_serial()

        # ── 订阅 /ccctrl_positions ────────────────────────────
        self._sub = self.create_subscription(
            CCtrlJoint, "/ccctrl_positions", self._pos_cb, 10
        )
        # ── 订阅 /tc_info ─────────────────────────────────────
        self._tc_sub = self.create_subscription(
            TcInfo, "/tc_info", self._tc_cb, 10
        )

        # ── 发布 /real_positions ───────────────────────────────
        self._real_pos_pub = self.create_publisher(JointState, "/real_positions", 10)

        # ── 最新目标数据 ──────────────────────────────────────
        self._latest_positions: list[float] | None = None
        self._latest_main_buttons: int = 0
        self._latest_handle_buttons: int = 0
        self._latest_joystick_x: int = 2048
        self._latest_joystick_y: int = 2048
        # TcInfo 缓存 (27 字节 → 串口发送帧保留区 byte[31:58])
        self._latest_tc_raw: bytes = b"\x00" * 27

        # ── 接收缓冲区 ────────────────────────────────────────
        self._rx_buf = b""
        self._latest_rx_positions: list[float] = [0.0, 2.4, -2.4, 0.0, 0.0, 0.0]  # 串口收到的真实关节位置

        # ── 收发定时器 (100 Hz) ───────────────────────────────
        self._timer = self.create_timer(1.0 / 100.0, self._tx_rx_loop)

        # ── 统计 ──────────────────────────────────────────────
        self._tx_count = 0
        self._rx_count = 0
        self._last_stats = time.time()
        self._start_time = time.time()  # 启动时刻，用于前 10s 诊断

        self.get_logger().info("CCtrl 串口桥接启动, 等待 /ccctrl_positions 数据…")

    # ── 串口连接管理 ───────────────────────────────────────────
    def _open_serial(self) -> bool:
        """尝试打开串口，成功返回 True。

        当配置端口为 /dev/ttyACM0 时，自动回退尝试 ACM1、ACM2。
        """
        # 构建回退端口列表
        ports_to_try = [self._port]
        if self._port == "/dev/ttyACM0":
            ports_to_try.extend(["/dev/ttyACM1", "/dev/ttyACM2"])

        for port in ports_to_try:
            try:
                self.ser = serial.Serial(
                    port=port,
                    baudrate=self._baudrate,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=0.005,
                    write_timeout=0.01,
                )
                self._connected = True
                self._reconnect_backoff = 0.0
                self._rx_buf = b""
                self._last_data_time = time.time()  # 重置看门狗计时
                if port != self._port:
                    self.get_logger().info(
                        f"配置端口 {self._port} 不可用，已回退至: {port} @ {self._baudrate}"
                    )
                    self._port = port  # 记住实际使用的端口
                else:
                    self.get_logger().info(
                        f"串口已连接: {self._port} @ {self._baudrate}"
                    )
                return True
            except (serial.SerialException, OSError) as e:
                if port == ports_to_try[-1]:
                    # 所有端口都失败时才记录警告
                    self._connected = False
                    self.ser = None
                    ports_str = ", ".join(ports_to_try)
                    self.get_logger().warn(
                        f"串口打开失败 (已尝试: {ports_str}): {e}"
                    )
                continue

        self._connected = False
        self.ser = None
        return False

    def _try_reconnect(self, now: float) -> bool:
        """根据退避策略尝试重连，返回是否已连接"""
        if self._connected:
            return True

        # 控制重连间隔
        if now - self._last_reconnect_attempt < self._reconnect_backoff:
            return False

        self._last_reconnect_attempt = now

        if self._open_serial():
            return True

        # 指数退避: 0.25 → 0.5 → 1 → 2 → 4 → cap
        if self._reconnect_backoff < 0.01:
            self._reconnect_backoff = 0.25
        else:
            self._reconnect_backoff = min(
                self._reconnect_backoff * 2.0, self._reconnect_max_delay
            )
        self.get_logger().info(
            f"将在 {self._reconnect_backoff:.1f}s 后再次尝试重连…"
        )
        return False

    def _close_serial(self):
        """安全关闭串口"""
        if self.ser is not None:
            try:
                self.ser.close()
            except (serial.SerialException, OSError):
                pass
            self.ser = None
        self._connected = False

    # ── 订阅回调 ───────────────────────────────────────────────
    def _pos_cb(self, msg: CCtrlJoint):
        """缓存最新目标位置及面板数据"""
        if len(msg.position) >= NUM_JOINTS:
            self._latest_positions = list(msg.position[:NUM_JOINTS])
        self._latest_main_buttons = msg.main_buttons
        self._latest_handle_buttons = msg.handle_buttons
        self._latest_joystick_x = msg.joystick_x
        self._latest_joystick_y = msg.joystick_y
        # 诊断: 每次收到 /ccctrl_positions 时计数，首次收到时打印确认
        if not hasattr(self, '_pos_cb_count'):
            self._pos_cb_count = 0
            self.get_logger().info(
                f"[诊断] 首次收到 /ccctrl_positions: "
                f"pos={[f'{p:.3f}' for p in msg.position[:NUM_JOINTS]]}, "
                f"main_btn=0x{msg.main_buttons:02X}, "
                f"handle_btn=0x{msg.handle_buttons:02X}"
            )
        self._pos_cb_count += 1

    def _tc_cb(self, msg: TcInfo):
        """缓存最新 TcInfo → 序列化为 27 字节 (3h + 3B + 16B + 2x 填充)"""
        buf = bytearray(27)
        # 3×int16 LE: mouse_x, mouse_y, mouse_z
        struct.pack_into("<3h", buf, 0, msg.mouse_x, msg.mouse_y, msg.mouse_z)
        # 3×uint8: mouse buttons
        struct.pack_into("<3B", buf, 6,
                         msg.mouse_left, msg.mouse_right, msg.mouse_mid)
        # 16×uint8: keys
        struct.pack_into("<16B", buf, 9,
                         msg.key_w, msg.key_s, msg.key_a, msg.key_d,
                         msg.key_shift, msg.key_ctrl, msg.key_q, msg.key_e,
                         msg.key_r, msg.key_f, msg.key_g, msg.key_z,
                         msg.key_x, msg.key_c, msg.key_v, msg.key_b)
        # byte[25:27] 2 字节填充
        self._latest_tc_raw = bytes(buf)

    # ── 发送帧 ─────────────────────────────────────────────────
    def _tx_frame(self):
        """打包并发送一帧。收到 A 包 (/ccctrl_positions) 或 B 包 (/tc_info)
        任一数据即发送；未收到的字段使用安全默认值。
        """
        if not self._connected or self.ser is None:
            return

        # 未收到 /ccctrl_positions 时使用串口收到的真实关节位置作为默认值
        positions = (
            self._latest_positions
            if self._latest_positions is not None
            else self._latest_rx_positions
        )

        frame = pack_tx_frame(
            positions,
            self._latest_main_buttons,
            self._latest_handle_buttons,
            self._latest_joystick_x,
            self._latest_joystick_y,
            self._latest_tc_raw,
        )
        try:
            self.ser.write(frame)
            self._tx_count += 1
            self._last_data_time = time.time()  # 重置看门狗
        except (serial.SerialException, OSError):
            self.get_logger().warn(f"串口写入失败, 标记为断开: {self._port}")
            self._close_serial()

    # ── 接收解析 ───────────────────────────────────────────────
    def _rx_frame(self):
        """读取串口并尝试解析接收帧"""
        if not self._connected or self.ser is None:
            return
        try:
            chunk = self.ser.read(self.ser.in_waiting or 256)
            if chunk:
                self._rx_buf += chunk
        except (serial.SerialException, OSError):
            self.get_logger().warn(f"串口读取失败, 标记为断开: {self._port}")
            self._close_serial()
            return

        while len(self._rx_buf) >= RX_FRAME_LEN:
            idx = self._rx_buf.find(bytes([FRAME_START]))
            if idx < 0:
                self._rx_buf = b""
                return
            if idx > 0:
                self._rx_buf = self._rx_buf[idx:]

            if len(self._rx_buf) < RX_FRAME_LEN:
                return

            positions = parse_rx_frame(self._rx_buf[:RX_FRAME_LEN])
            if positions is not None:
                self._rx_count += 1
                self._rx_buf = self._rx_buf[RX_FRAME_LEN:]
                self._latest_rx_positions = positions  # 缓存最新真实位置
                self._last_data_time = time.time()  # 重置看门狗

                rads = [f"{p:.4f}" for p in positions]
                # self.get_logger().info(
                #     f"RX: J1={rads[0]} J2={rads[1]} J3={rads[2]} "
                #     f"J4={rads[3]} J5={rads[4]} J6={rads[5]}"
                # )

                # 发布到 /real_positions
                rmsg = JointState()
                rmsg.header.stamp = self.get_clock().now().to_msg()
                rmsg.name = ["J1", "J2", "J3", "J4", "J5", "J6"]
                rmsg.position = positions
                self._real_pos_pub.publish(rmsg)
            else:
                self._rx_buf = self._rx_buf[1:]  # 跳过无效 SOF

    # ── 收发循环 ───────────────────────────────────────────────
    def _tx_rx_loop(self):
        """定时收发 (含断线重连)"""
        try:
            now = time.time()

            # 断线重连
            if not self._connected:
                self._try_reconnect(now)

            # ── 数据看门狗：无有效收发超时则触发重连 ──
            if self._connected and time.time() - self._last_data_time > self._watchdog_timeout:
                self.get_logger().warn(
                    f"数据看门狗超时 ({self._watchdog_timeout:.1f}s 无有效收发)，"
                    "触发重连…"
                )
                self._watchdog_triggered = True
                self._close_serial()

            self._rx_frame()
            self._tx_frame()

            # 统计日志 (每 10s)
            if now - self._last_stats >= 10.0:
                pos_source = "pos_cb" if self._latest_positions is not None else "rx_fallback"
                pos_cb_cnt = getattr(self, '_pos_cb_count', 0)
                p = self._latest_positions if self._latest_positions is not None else self._latest_rx_positions
                degs = [f"{x*57.29578:.1f}" for x in (p[:6] if p else [0]*6)]
                status = "已连接" if self._connected else "断开"
                self.get_logger().info(
                    f"统计(10s): TX={self._tx_count}, RX={self._rx_count} [{status}] "
                    f"pos_src={pos_source} cb_cnt={pos_cb_cnt} "
                    f"send_pos=[{', '.join(degs)}]"
                )
                self._tx_count = 0
                self._rx_count = 0
                self._last_stats = now
        except Exception:
            self.get_logger().error(
                f"收发循环异常, 标记串口断开: {self._port}"
            )
            self._close_serial()


def main(args=None):
    rclpy.init(args=args)
    node = CCtrlSerialBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._close_serial()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
