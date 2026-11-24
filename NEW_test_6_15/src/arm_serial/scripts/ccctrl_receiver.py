#!/usr/bin/env python3
"""
CCtrl 串口接收节点 — 解析 ESP32-S3 主机上报的 CCtrl 协议帧，发布到 ccctrl_data 话题。

帧协议 (37 字节):
  byte[0]       SOF          0xA5
  byte[1:3]     payload_len  uint16 LE (固定 28)
  byte[3]       sequence     uint8 (每帧+1)
  byte[4]       header_crc8  uint8 (覆盖偏移 0-3)
  byte[5:7]     cmd_id       uint16 LE (固定 0x0303)
  byte[7:35]    payload      28 bytes (CCtrlPayload 业务数据)
  byte[35:37]   frame_crc16  uint16 LE (覆盖偏移 0-34)

用法:
  ros2 run arm_serial ccctrl_receiver.py
  ros2 run arm_serial ccctrl_receiver.py --ros-args -p port:=/dev/ttyUSB0
  ros2 run arm_serial ccctrl_receiver.py --ros-args -p baudrate:=115200

参考协议: CCtrl_串口协议说明.md
"""

import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from arm_serial.msg import CCtrlPayload, TcInfo
import serial

# ── 协议 A 帧常量 (CCtrl) ──────────────────────────────────────────
FRAME_SOF       = 0xA5
CMD_ID_CCtrl    = 0x0302
HEADER_LEN      = 5   # SOF + payload_len(2) + sequence + header_crc8
# 实际固件 payload_len=31（比文档 28 多 3 字节）
ACCEPTED_PAYLOAD_LENS = {28, 30, 31}
# body (cmd_id + payload) 之后的 CRC16 占用 2 字节

# ── 协议 B 帧常量 (遥控器键盘 21 字节) ─────────────────────────────
HEADER_B = b"\xA9\x53"
PACKET_B_LEN = 21

# CRC 参数
CRC8_POLY       = 0x8C
CRC8_INIT       = 0xFF
CRC16_POLY      = 0x8408
CRC16_INIT      = 0xFFFF


# ═══════════════════════════════════════════════════════════════════
# CRC 算法（严格按协议文档第 4 节 C 代码翻译）
# ═══════════════════════════════════════════════════════════════════

def cctrl_crc8(data: bytes) -> int:
    """CRC8: 初值 0xFF, 反射多项式 0x8C"""
    value = CRC8_INIT
    for b in data:
        value ^= b
        for _ in range(8):
            if value & 1:
                value = (value >> 1) ^ CRC8_POLY
            else:
                value >>= 1
    return value


def cctrl_crc16(data: bytes) -> int:
    """CRC16: 初值 0xFFFF, 反射多项式 0x8408"""
    value = CRC16_INIT
    for b in data:
        value ^= b
        for _ in range(8):
            if value & 1:
                value = (value >> 1) ^ CRC16_POLY
            else:
                value >>= 1
    return value


# ═══════════════════════════════════════════════════════════════════
# Payload 解析
# ═══════════════════════════════════════════════════════════════════

def parse_payload(payload: bytes, payload_len: int = 31) -> CCtrlPayload:
    """解析 payload → CCtrlPayload 消息对象。

    28 字节布局 (协议文档第 3 节):
        0:  diagnostic_flags  uint8
        1:  node_valid_flags  uint8
        2:  main_buttons      uint8
        3:  handle_buttons    uint8
        4-9:   encoder_status[6]  uint8[6]
        10-21: encoder_value[6]   int16[6] (LE)
        22-23: joystick_x         uint16 (LE)
        24-25: joystick_y         uint16 (LE)
        26-27: trigger            uint16 (LE)

    31 字节布局 (实际固件):
        payload[2:5] 为 3 保留字节，相比文档 28 字节，
        main_buttons/handle_buttons/encoder_status 等字段偏移均 +3。
    """
    msg = CCtrlPayload()

    msg.diagnostic_flags = payload[0]
    msg.node_valid_flags = payload[1]

    if payload_len == 31:
        # payload[2:5] 为 3 保留字节，后续字段偏移 +3
        msg.main_buttons     = payload[5]
        msg.handle_buttons   = payload[6]
        msg.encoder_status   = list(payload[7:13])
        enc_offset = 13
        js_offset  = 25
    elif payload_len == 30:
        # payload[2:4] 为 2 保留字节，后续字段偏移 +2
        msg.main_buttons     = payload[2]
        msg.handle_buttons   = payload[3]
        msg.encoder_status   = list(payload[4:10])
        enc_offset = 10
        js_offset  = 22
    else:
        # 文档规定的 28 字节布局
        msg.main_buttons     = payload[2]
        msg.handle_buttons   = payload[3]
        msg.encoder_status   = list(payload[4:10])
        enc_offset = 10
        js_offset  = 22

    # 6×int16 LE
    encoder_values = struct.unpack("<6h", payload[enc_offset:enc_offset+12])
    msg.encoder_value = list(encoder_values)

    msg.joystick_x, msg.joystick_y, msg.trigger = struct.unpack(
        "<3H", payload[js_offset:js_offset+6]
    )

    return msg
# ═══════════════════════════════════════════════════════════════════
# 接收节点
# ═══════════════════════════════════════════════════════════════════

class CCtrlReceiver(Node):
    """CCtrl 串口接收节点 — 帧解析 → CCtrlPayload 发布"""

    def __init__(self):
        super().__init__("ccctrl_receiver")

        # ── 参数 ──────────────────────────────────────────────
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 961200)
        self.declare_parameter("diagnostic", False)  # 诊断模式：输出原始 hex

        self._port = (
            self.get_parameter("port").get_parameter_value().string_value
        )
        self._baudrate = (
            self.get_parameter("baudrate")
            .get_parameter_value()
            .integer_value
        )
        self._diagnostic = (
            self.get_parameter("diagnostic").get_parameter_value().bool_value
        )

        # ── 串口 ──────────────────────────────────────────────
        self.ser: serial.Serial | None = None
        self._connected = False
        self._buf = b""
        self._last_reconnect_attempt = 0.0
        self._reconnect_backoff = 0.0

        # ── Publisher ─────────────────────────────────────────
        self._pub = self.create_publisher(CCtrlPayload, "ccctrl_data", 10)
        self._tc_info_pub = self.create_publisher(TcInfo, "/tc_info", 10)

        # ── 定时器 (约 60 Hz，快于 30 Hz 数据率，避免丢帧) ────
        self._timer = self.create_timer(1.0 / 60.0, self._read_and_publish)

        # ── 帧序号追踪 ────────────────────────────────────────
        self._last_seq: int | None = None
        self._seq_gaps = 0  # 序号跳跃次数

        # ── 分步错误计数 ────────────────────────────────────
        self._err_payload_len = 0
        self._err_crc8 = 0
        self._err_cmd_id = 0
        self._err_crc16 = 0

        # ── 统计 ──────────────────────────────────────────────
        self._frame_count = 0
        self._last_stats = time.time()
        self._diagnostic_dumped = False  # 诊断 hex 只输出第一次

        # ── 初始连接 ──────────────────────────────────────────
        self._connect_serial()

        self.get_logger().info(
            "CCtrl 接收器启动, 等待数据… "
            f"→ topic: {self._pub.topic_name}"
        )

    # ── 串口连接 ─────────────────────────────────────────────
    def _connect_serial(self) -> bool:
        """打开串口，成功返回 True，失败返回 False。
        当 /dev/ttyUSB0 不可用时，自动回退尝试 USB1、USB2。
        """
        self._close_serial()

        ports_to_try = [self._port]
        if self._port == "/dev/ttyUSB0":
            ports_to_try.extend(["/dev/ttyUSB1", "/dev/ttyUSB2"])

        for port in ports_to_try:
            try:
                self.ser = serial.Serial(
                    port=port,
                    baudrate=self._baudrate,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=0.01,
                    write_timeout=0.1,
                    dsrdtr=False,
                )
                self._connected = True
                self._buf = b""
                self._diagnostic_dumped = False
                if port != self._port:
                    self.get_logger().info(
                        f"配置端口 {self._port} 不可用，已回退至: {port} @ {self._baudrate}"
                    )
                    self._port = port
                else:
                    self.get_logger().info(
                        f"串口已连接: {self._port} @ {self._baudrate}"
                    )
                return True
            except (serial.SerialException, OSError) as e:
                if port == ports_to_try[-1]:
                    self._connected = False
                    self.ser = None
                    self.get_logger().error(
                        f"串口连接失败 (已尝试: {', '.join(ports_to_try)}): {e}"
                    )
                continue

        self._connected = False
        self.ser = None
        return False

    def _close_serial(self):
        """安全关闭串口。"""
        if self.ser is not None:
            try:
                self.ser.close()
            except (serial.SerialException, OSError):
                pass
            self.ser = None
        self._connected = False

    # ── 诊断 hex dump ─────────────────────────────────────────
    def _dump_diagnostic(self, buf: bytes, fail_step: str):
        """当 diagnostic 参数为 True 时，输出 SOF 附近的原始 hex 用于调试。"""
        if not self._diagnostic or self._diagnostic_dumped:
            return
        self._diagnostic_dumped = True

        hdr_idx = buf.find(bytes([FRAME_SOF]))
        if hdr_idx < 0:
            hdr_idx = 0
        start = max(0, hdr_idx - 4)
        end = min(len(buf), hdr_idx + 40)

        self.get_logger().warn(
            f"[DIAG] 首次校验失败 ({fail_step})，buf 总长={len(buf)} 字节"
        )
        chunk = buf[start:end]
        hex_str = " ".join(f"{b:02X}" for b in chunk)
        self.get_logger().warn(f"[DIAG] SOF 附近 hex ({start}:{end}): {hex_str}")

    # ── 帧解析（协议第 5 节接收流程） ────────────────────────────
    def _try_parse(self) -> tuple[CCtrlPayload, int] | None:
        """尝试从缓冲区解析一帧。

        返回 (msg, consumed_bytes) 或 None（数据不足/校验失败）。
        """
        buf = self._buf

        # 1. 定位 SOF 0xA5
        hdr_idx = buf.find(bytes([FRAME_SOF]))
        if hdr_idx < 0:
            return None

        discarded = hdr_idx
        buf = buf[hdr_idx:]

        # 2. 等待至少 HEADER_LEN (5) 字节
        if len(buf) < HEADER_LEN:
            return None

        # 读取 payload_len (LE uint16 at offset 1-2)
        payload_len = struct.unpack("<H", buf[1:3])[0]
        if payload_len not in ACCEPTED_PAYLOAD_LENS:
            self._err_payload_len += 1
            if self._err_payload_len == 1:
                # self.get_logger().warn(
                #     f"payload_len 不匹配: got={payload_len}, "
                #     f"expected={ACCEPTED_PAYLOAD_LENS}"
                # )
                self._dump_diagnostic(buf, f"payload_len={payload_len}")
            return None  # 由外层跳过虚假 SOF

        # 动态计算帧长: SOF(1) + payload_len(2) + seq(1) + crc8(1) + cmd_id(2) + payload(payload_len) + crc16(2)
        self._frame_len = 1 + 2 + 1 + 1 + 2 + payload_len + 2
        self._payload_len = payload_len

        # 3. 校验 header CRC8 (覆盖偏移 0-3)
        expected_crc8 = buf[4]
        actual_crc8 = cctrl_crc8(buf[0:4])
        if expected_crc8 != actual_crc8:
            self._err_crc8 += 1
            if self._err_crc8 == 1:
                self.get_logger().warn(
                    f"Header CRC8 不匹配: expected=0x{expected_crc8:02X}, "
                    f"actual=0x{actual_crc8:02X}"
                )
                self._dump_diagnostic(buf, f"CRC8 mismatch")
            return None  # 由外层跳过虚假 SOF

        # 4. 等待完整帧
        if len(buf) < self._frame_len:
            return None

        # 5. 检查 cmd_id (LE uint16 at offset 5-6)
        cmd_id = struct.unpack("<H", buf[5:7])[0]
        if cmd_id != CMD_ID_CCtrl:
            self._err_cmd_id += 1
            if self._err_cmd_id == 1:
                self.get_logger().warn(
                    f"cmd_id 不匹配: got=0x{cmd_id:04X}, expected=0x{CMD_ID_CCtrl:04X}"
                )
                self._dump_diagnostic(buf, f"cmd_id=0x{cmd_id:04X}")
            return None  # 由外层跳过虚假 SOF

        # 6. 校验 frame CRC16 (覆盖偏移 0 到 frame_len-3)
        crc16_offset = self._frame_len - 2
        expected_crc16 = struct.unpack("<H", buf[crc16_offset:crc16_offset+2])[0]
        actual_crc16 = cctrl_crc16(buf[0:crc16_offset])
        if expected_crc16 != actual_crc16:
            self._err_crc16 += 1
            if self._err_crc16 == 1:
                self.get_logger().warn(
                    f"Frame CRC16 不匹配: expected=0x{expected_crc16:04X}, "
                    f"actual=0x{actual_crc16:04X}"
                )
                self._dump_diagnostic(buf, f"CRC16 mismatch")
            return None  # 由外层跳过虚假 SOF

        # 7. 解析 payload
        payload = buf[7:7 + self._payload_len]
        msg = parse_payload(payload, self._payload_len)
        self._raw_payload = payload  # 保存原始 payload 供日志输出

        # 诊断模式：成功解析第一帧时也输出
        if self._diagnostic and self._frame_count == 0:
            hex_str = " ".join(f"{b:02X}" for b in buf[:self._frame_len])
            self.get_logger().info(f"[DIAG] 首帧 hex ({self._frame_len}B): {hex_str}")

        # 8. 统计帧序号连续性
        seq = buf[3]
        if self._last_seq is not None:
            expected = (self._last_seq + 1) & 0xFF
            if seq != expected:
                self._seq_gaps += 1
                self.get_logger().debug(
                    f"帧序号跳跃: {self._last_seq} → {seq} "
                    f"(期望 {expected})"
                )
        self._last_seq = seq

        consumed = self._frame_len + discarded
        return (msg, consumed)

    # ── 定时回调 ───────────────────────────────────────────────
    def _read_and_publish(self):
        """读取串口数据，解析帧并发布"""
        # 断线时尝试重连（带节流）
        if not self._connected or self.ser is None:
            now = time.time()
            if now - self._last_reconnect_attempt >= self._reconnect_backoff:
                self._last_reconnect_attempt = now
                if self._connect_serial():
                    self._reconnect_backoff = 0.0
                else:
                    self._reconnect_backoff = min(
                        max(self._reconnect_backoff * 2.0, 0.5), 5.0
                    )
            return

        # 读取
        try:
            chunk = self.ser.read(self.ser.in_waiting or 1024)
            if chunk:
                self._buf += chunk
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f"串口读取错误: {e}")
            self._close_serial()
            return

        # 分拣数据包 — 同时寻找两种协议包头
        SOF_A = FRAME_SOF  # 0xA5
        latest = None
        while len(self._buf) >= 2:
            idx_a = self._buf.find(bytes([SOF_A]))
            idx_b = self._buf.find(HEADER_B)

            # 两个包头都没找到
            if idx_a == -1 and idx_b == -1:
                # 保留最后一个字节，防止刚好切断了包头 (0xA5 或 0xA9)
                if self._buf[-1] in (SOF_A, 0xA9):
                    self._buf = self._buf[-1:]
                elif len(self._buf) >= 64:
                    # 缓冲区已累积较多数据却无包头，视为垃圾清空
                    self.get_logger().warn(
                        f"缓冲区 {len(self._buf)}B 无有效包头，已清空: "
                        f"{' '.join(f'{b:02X}' for b in self._buf[:32])}..."
                    )
                    self._buf = b""
                # else: 数据量少，保留等下一波
                break

            # 找出最靠前的包头；若 0xA5 紧挨在 A9 53 之前，优先按协议 B 对齐
            valid_indices = [i for i in (idx_a, idx_b) if i != -1]
            first = min(valid_indices)
            if (
                idx_b != -1
                and idx_a != -1
                and idx_b == idx_a + 1
                and first == idx_a
            ):
                first = idx_b

            # 跳过包头前的无效字节
            if first > 0:
                self._buf = self._buf[first:]

            # ── 分支 B：21 字节遥控器键盘包 (0xA9 0x53) ──────────
            if self._buf[:2] == HEADER_B:
                if len(self._buf) < PACKET_B_LEN:
                    break

                data_packet = bytes(self._buf[:PACKET_B_LEN])

                tc_msg = TcInfo()
                tc_msg.mouse_x, tc_msg.mouse_y, tc_msg.mouse_z = (
                    struct.unpack('<3h', data_packet[10:16])
                )

                mouse_btn_byte = data_packet[16]
                tc_msg.mouse_left = (mouse_btn_byte >> 0) & 0b11
                tc_msg.mouse_right = (mouse_btn_byte >> 2) & 0b11
                tc_msg.mouse_mid = (mouse_btn_byte >> 4) & 0b11

                keys_val = struct.unpack('<H', data_packet[17:19])[0]
                tc_msg.key_w = (keys_val >> 0) & 1
                tc_msg.key_s = (keys_val >> 1) & 1
                tc_msg.key_a = (keys_val >> 2) & 1
                tc_msg.key_d = (keys_val >> 3) & 1
                tc_msg.key_shift = (keys_val >> 4) & 1
                tc_msg.key_ctrl = (keys_val >> 5) & 1
                tc_msg.key_q = (keys_val >> 6) & 1
                tc_msg.key_e = (keys_val >> 7) & 1
                tc_msg.key_r = (keys_val >> 8) & 1
                tc_msg.key_f = (keys_val >> 9) & 1
                tc_msg.key_g = (keys_val >> 10) & 1
                tc_msg.key_z = (keys_val >> 11) & 1
                tc_msg.key_x = (keys_val >> 12) & 1
                tc_msg.key_c = (keys_val >> 13) & 1
                tc_msg.key_v = (keys_val >> 14) & 1
                tc_msg.key_b = (keys_val >> 15) & 1

                self.get_logger().debug(
                    f"key_q={tc_msg.key_q}"
                )

                self._tc_info_pub.publish(tc_msg)

                self._buf = self._buf[PACKET_B_LEN:]
                continue

            # ── 分支 A：CCtrl 帧 (0xA5) ──────────────────────────
            if self._buf[0] != SOF_A:
                break  # 不是预期的 SOF，退出等下一波数据

            result = self._try_parse()
            if result is None:
                # 校验失败，跳过虚假 SOF 后继续搜索
                if len(self._buf) > 1:
                    self._buf = self._buf[1:]
                else:
                    break
                continue
            msg, consumed = result
            self._buf = self._buf[consumed:]
            latest = msg
            self._frame_count += 1

        # 发布最新帧
        if latest is not None:
            self._pub.publish(latest)
            # self.get_logger().info(
            #     f"raw=[{' '.join(f'{b:02X}' for b in self._raw_payload)}] "
            #     f"main_btn=0x{latest.main_buttons:02X}, "
            #     f"handle_btn=0x{latest.handle_buttons:02X}, "
            #     f"encoder={list(latest.encoder_value)}, "
            #     f"joystick=({latest.joystick_x}, {latest.joystick_y}), "
            #     f"trigger={latest.trigger}"
            # )


        # 统计日志 (每 10s)
        now = time.time()
        if now - self._last_stats >= 10.0: 
            self._last_stats = now
            total_err = (
                self._err_payload_len + self._err_crc8
                + self._err_cmd_id + self._err_crc16
            )
            self.get_logger().info(
                f"统计(10s): {self._frame_count} 帧, "
            )
            self._frame_count = 0
            self._err_payload_len = 0
            self._err_crc8 = 0
            self._err_cmd_id = 0
            self._err_crc16 = 0
            self._seq_gaps = 0


def main(args=None):
    rclpy.init(args=args)
    node = CCtrlReceiver()

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
