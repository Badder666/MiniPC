"""Parser and builder for the Custom_Controller USB state frame."""

from __future__ import annotations

from dataclasses import dataclass
import math
import struct
from typing import Iterable


SOF = b"\xA5\x5A"
VERSION = 1
TYPE_STATE = 1
TYPE_HOST_GRAVITY_TORQUE = 2
FRAME_SIZE = 42
AXIS_COUNT = 6


def crc16(data: bytes) -> int:
    """Return the controller CRC16 (initial 0xffff, reflected 0x8408)."""
    value = 0xFFFF
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ 0x8408 if value & 1 else value >> 1
    return value & 0xFFFF


@dataclass(frozen=True)
class StateFrame:
    sequence: int
    timestamp_ms: int
    positions: tuple[float, ...]
    buttons: int
    valid_motor_mask: int


def build_host_torque_frame(
    timestamp_ms: int,
    torques: Iterable[float],
    valid_motor_mask: int = 0x3F,
    enabled: bool = True,
    sequence: int = 0,
) -> bytes:
    """Build the binary Pinocchio torque frame consumed by the controller."""
    values = tuple(float(value) for value in torques)
    if len(values) != AXIS_COUNT or any(not math.isfinite(value) for value in values):
        raise ValueError("torques must contain six finite values")
    if valid_motor_mask & ~0x3F:
        raise ValueError("valid_motor_mask contains reserved bits")
    flags = 1 if enabled else 0
    body = bytearray(SOF)
    body += bytes((VERSION, TYPE_HOST_GRAVITY_TORQUE))
    body += struct.pack("<HHIHH6f", FRAME_SIZE, sequence & 0xFFFF,
                        timestamp_ms & 0xFFFFFFFF, valid_motor_mask & 0x3F,
                        flags, *values)
    if len(body) != FRAME_SIZE - 2:
        raise AssertionError("invalid host torque frame layout")
    body += struct.pack("<H", crc16(body))
    return bytes(body)


def build_state_frame(
    timestamp_ms: int,
    positions: Iterable[float],
    buttons: int,
    valid_motor_mask: int,
    sequence: int = 0,
) -> bytes:
    values = tuple(float(value) for value in positions)
    if len(values) != AXIS_COUNT or any(not math.isfinite(value) for value in values):
        raise ValueError("positions must contain six finite values")
    body = bytearray()
    body += SOF
    body += bytes((VERSION, TYPE_STATE))
    body += struct.pack("<HHI", FRAME_SIZE, sequence & 0xFFFF, timestamp_ms & 0xFFFFFFFF)
    body += struct.pack("<6fHH", *values, buttons & 0xFFFF, valid_motor_mask & 0x3F)
    if len(body) != FRAME_SIZE - 2:
        raise AssertionError("invalid frame layout")
    body += struct.pack("<H", crc16(body))
    return bytes(body)


def parse_state_frame(frame: bytes) -> StateFrame:
    if len(frame) != FRAME_SIZE or frame[:2] != SOF:
        raise ValueError("invalid frame size or header")
    if frame[2] != VERSION or frame[3] != TYPE_STATE:
        raise ValueError("unsupported frame version or type")
    if struct.unpack_from("<H", frame, 4)[0] != FRAME_SIZE:
        raise ValueError("invalid frame length")
    if struct.unpack_from("<H", frame, -2)[0] != crc16(frame[:-2]):
        raise ValueError("CRC mismatch")
    sequence, timestamp_ms = struct.unpack_from("<HI", frame, 6)
    values = struct.unpack_from("<6f", frame, 12)
    buttons, valid_motor_mask = struct.unpack_from("<HH", frame, 36)
    if any(not math.isfinite(value) for value in values):
        raise ValueError("non-finite motor position")
    if valid_motor_mask & ~0x3F:
        raise ValueError("reserved motor bitmap bits are set")
    return StateFrame(sequence, timestamp_ms, values, buttons, valid_motor_mask)


class StateFrameParser:
    """Incremental parser for USB reads containing partial or multiple frames."""

    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, data: bytes) -> list[StateFrame]:
        self._buffer.extend(data)
        frames: list[StateFrame] = []
        while True:
            start = self._buffer.find(SOF)
            if start < 0:
                # Retain a partial start marker split across USB reads.
                trailing_a5 = self._buffer.endswith(SOF[:1])
                self._buffer.clear()
                if trailing_a5:
                    self._buffer.extend(SOF[:1])
                break
            if start:
                del self._buffer[:start]
            if len(self._buffer) < 6:
                break
            length = struct.unpack_from("<H", self._buffer, 4)[0]
            if (length != FRAME_SIZE or self._buffer[2] != VERSION or
                    self._buffer[3] != TYPE_STATE):
                del self._buffer[0]
                continue
            if len(self._buffer) < length:
                break
            candidate = bytes(self._buffer[:length])
            try:
                frames.append(parse_state_frame(candidate))
            except ValueError:
                # A damaged frame may contain the next frame's start marker.
                del self._buffer[0]
                continue
            del self._buffer[:length]
        return frames
