"""Command selection and bounded serial writes, independent of ROS2."""

from .command_protocol import CommandFrame, OperatorState, build_command_frame


class PendingCommandWriter:
    """Finish a started frame; keep at most one newest waiting command."""

    def __init__(self):
        self.clear()

    def clear(self):
        self._active = b""
        self._offset = 0
        self._latest = None

    def submit(self, frame: bytes):
        self._latest = frame

    def write_once(self, port):
        if not self._active:
            if self._latest is None:
                return
            self._active, self._latest = self._latest, None
            self._offset = 0
        remaining = self._active[self._offset:]
        count = port.write(remaining)
        if count is None or not 0 <= count <= len(remaining):
            raise OSError("invalid serial write result")
        self._offset += count
        if self._offset == len(self._active):
            self._active = b""
            self._offset = 0


class CommandSelector:
    def __init__(self, mapper, main_button_bits, handle_button_bits):
        self.mapper = mapper
        self.main_button_bits = self._validate_bits(main_button_bits)
        self.handle_button_bits = self._validate_bits(handle_button_bits)
        self.sequence = 0

    @staticmethod
    def _validate_bits(bits):
        if len(bits) != 8 or any(not isinstance(v, int) or not -1 <= v <= 15 for v in bits):
            raise ValueError("button mappings require eight source bits in -1..15")
        return tuple(bits)

    @staticmethod
    def _map_buttons(buttons, bits):
        return sum(((buttons >> source) & 1) << target
                   for target, source in enumerate(bits) if source >= 0)

    def encode(self, positions, valid_mask, timestamp_ms, buttons, operator=None):
        targets, mask = self.mapper.map(positions, valid_mask)
        external = operator or OperatorState()
        op = OperatorState(
            external.joystick_x, external.joystick_y, external.mouse,
            external.mouse_buttons, external.keys,
            external.main_buttons | self._map_buttons(buttons, self.main_button_bits),
            external.handle_buttons | self._map_buttons(buttons, self.handle_button_bits),
        )
        command = CommandFrame(self.sequence, timestamp_ms, targets, buttons,
                               mask, True, op)
        frame = build_command_frame(command)
        self.sequence = (self.sequence + 1) & 0xFFFF
        return frame
