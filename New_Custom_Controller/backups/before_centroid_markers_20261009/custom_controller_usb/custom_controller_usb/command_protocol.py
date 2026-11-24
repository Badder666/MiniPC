"""Version 1 host commands and the existing arm feedback stream."""

from dataclasses import dataclass, field
import math
import struct

from .protocol import AXIS_COUNT, SOF, VERSION, crc16

COMMAND_SIZE = 64
TYPE_COMMAND = 0x10
OPERATOR_VALID = 1
KEY_NAMES = (
    "w", "s", "a", "d", "shift", "ctrl", "q", "e", "r",
    "f", "g", "z", "x", "c", "v", "b", "1", "2",
)


@dataclass(frozen=True)
class OperatorState:
    joystick_x: int = 2048
    joystick_y: int = 2048
    mouse: tuple[int, int, int] = (0, 0, 0)
    mouse_buttons: tuple[int, int, int] = (0, 0, 0)
    keys: int = 0
    main_buttons: int = 0
    handle_buttons: int = 0


@dataclass(frozen=True)
class CommandFrame:
    sequence: int
    timestamp_ms: int
    positions: tuple[float, ...]
    buttons: int = 0
    valid_joint_mask: int = 0
    operator_valid: bool = False
    operator: OperatorState = field(default_factory=OperatorState)


def build_command_frame(command: CommandFrame) -> bytes:
    positions = tuple(float(value) for value in command.positions)
    op = command.operator
    if len(positions) != AXIS_COUNT or any(not math.isfinite(p) for p in positions):
        raise ValueError("positions must contain six finite radians")
    if command.valid_joint_mask & ~0x3F or not 0 <= command.buttons <= 0xFFFF:
        raise ValueError("invalid joint or button bitmap")
    if not (0 <= op.joystick_x <= 4095 and 0 <= op.joystick_y <= 4095):
        raise ValueError("joystick values must be in 0..4095")
    if (len(op.mouse) != 3 or any(not -32768 <= v <= 32767 for v in op.mouse) or
            len(op.mouse_buttons) != 3 or any(not 0 <= v <= 3 for v in op.mouse_buttons)):
        raise ValueError("invalid mouse fields")
    if not 0 <= op.keys <= 0x3FFFF or not all(
            0 <= v <= 255 for v in (op.main_buttons, op.handle_buttons)):
        raise ValueError("invalid operator button or key bitmap")
    body = SOF + bytes((VERSION, TYPE_COMMAND))
    body += struct.pack("<HHI6fHHHH3h3BBIBB2x", COMMAND_SIZE,
                        command.sequence & 0xFFFF, command.timestamp_ms & 0xFFFFFFFF,
                        *positions, command.buttons, command.valid_joint_mask,
                        op.joystick_x, op.joystick_y, *op.mouse, *op.mouse_buttons,
                        OPERATOR_VALID if command.operator_valid else 0,
                        op.keys, op.main_buttons, op.handle_buttons)
    if len(body) != COMMAND_SIZE - 2:
        raise AssertionError("invalid command layout")
    return body + struct.pack("<H", crc16(body))


def parse_command_frame(frame: bytes) -> CommandFrame:
    if (len(frame) != COMMAND_SIZE or frame[:4] != SOF + bytes((VERSION, TYPE_COMMAND)) or
            struct.unpack_from("<H", frame, 4)[0] != COMMAND_SIZE or
            struct.unpack_from("<H", frame, 62)[0] != crc16(frame[:62])):
        raise ValueError("invalid command header or CRC")
    seq, timestamp = struct.unpack_from("<HI", frame, 6)
    positions = struct.unpack_from("<6f", frame, 12)
    buttons, mask, joy_x, joy_y = struct.unpack_from("<4H", frame, 36)
    mouse = struct.unpack_from("<3h", frame, 44)
    mouse_buttons = struct.unpack_from("<3B", frame, 50)
    flags = frame[53]
    keys = struct.unpack_from("<I", frame, 54)[0]
    if flags & ~OPERATOR_VALID or frame[60:62] != b"\x00\x00":
        raise ValueError("reserved flags or bytes are set")
    command = CommandFrame(seq, timestamp, positions, buttons, mask, bool(flags),
                           OperatorState(joy_x, joy_y, mouse, mouse_buttons,
                                         keys, frame[58], frame[59]))
    build_command_frame(command)  # Apply the same payload validation as the sender.
    return command


class ArmFeedbackParser:
    """Parse the existing AA + six float32 + byte sum + 55 feedback (27 B)."""

    def __init__(self):
        self._buffer = bytearray()

    def feed(self, data: bytes) -> list[tuple[float, ...]]:
        self._buffer.extend(data)
        frames = []
        while self._buffer:
            start = self._buffer.find(b"\xAA")
            if start < 0:
                self._buffer.clear()
                break
            del self._buffer[:start]
            if len(self._buffer) < 27:
                break
            if self._buffer[26] != 0x55 or sum(self._buffer[1:25]) & 0xFF != self._buffer[25]:
                del self._buffer[0]
                continue
            positions = struct.unpack_from("<6f", self._buffer, 1)
            if not all(math.isfinite(p) for p in positions):
                del self._buffer[0]
                continue
            frames.append(positions)
            del self._buffer[:27]
        return frames
