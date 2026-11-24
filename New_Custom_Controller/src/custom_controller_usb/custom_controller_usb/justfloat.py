"""Incremental reader for the firmware's 25-channel JustFloat stream."""

import math
import struct

from .protocol import StateFrame


class JustFloatParser:
    TAIL = b'\x00\x00\x80\x7f'

    def __init__(self):
        self._buffer = bytearray()
        self._sequence = 0

    def feed(self, data):
        self._buffer.extend(data)
        frames = []
        while True:
            end = self._buffer.find(self.TAIL)
            if end < 0:
                # One payload plus a possibly split delimiter; bound noisy input.
                if len(self._buffer) > 103:
                    del self._buffer[:-103]
                break
            payload = bytes(self._buffer[:end])
            del self._buffer[:end + 4]
            if len(payload) != 100:
                continue
            values = struct.unpack('<25f', payload)
            if not all(math.isfinite(v) for v in values):
                continue
            buttons, mask, timestamp = values[18], values[19], values[22]
            if (not buttons.is_integer() or not 0 <= buttons <= 65535 or
                    not mask.is_integer() or not 0 <= mask <= 63 or
                    not timestamp.is_integer() or not 0 <= timestamp <= 4294967296):
                continue
            frames.append(StateFrame(self._sequence, int(timestamp) & 0xFFFFFFFF,
                                     values[:6], int(buttons), int(mask)))
            self._sequence = (self._sequence + 1) & 0xFFFF
        return frames
