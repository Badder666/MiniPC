import math
import struct
import unittest

from custom_controller_usb.protocol import (
    FRAME_SIZE, TYPE_HOST_GRAVITY_TORQUE, StateFrameParser,
    build_host_torque_frame, build_state_frame, crc16, parse_state_frame,
)


class ProtocolTests(unittest.TestCase):
    def setUp(self):
        self.frame = build_state_frame(1234, [0, 1, -2, 3, 4, 5], 0x1234, 0x3F, 7)

    def test_wire_format_and_crc(self):
        self.assertEqual(crc16(b"123456789"), 0x6F91)
        self.assertEqual(self.frame.hex(),
                         "a55a01012a000700d2040000000000000000803f000000c0"
                         "00004040000080400000a04034123f00b0bc")
        decoded = parse_state_frame(self.frame)
        self.assertEqual(decoded.sequence, 7)
        self.assertEqual(decoded.timestamp_ms, 1234)
        self.assertEqual(decoded.positions, (0, 1, -2, 3, 4, 5))
        self.assertEqual(decoded.buttons, 0x1234)
        self.assertEqual(decoded.valid_motor_mask, 0x3F)

    def test_every_split_and_bytewise_input(self):
        for split in range(len(self.frame) + 1):
            parser = StateFrameParser()
            frames = parser.feed(self.frame[:split]) + parser.feed(self.frame[split:])
            self.assertEqual(len(frames), 1, split)
        parser = StateFrameParser()
        frames = []
        for value in self.frame:
            frames.extend(parser.feed(bytes([value])))
        self.assertEqual(len(frames), 1)

    def test_noise_and_multiple_frames(self):
        parser = StateFrameParser()
        self.assertEqual(len(parser.feed(b"noise" + self.frame * 3 + b"noise")), 3)
        self.assertEqual(parser.feed(b"noise\xa5"), [])
        self.assertEqual(len(parser.feed(self.frame[1:])), 1)

    def test_corruption_and_truncation_resynchronize(self):
        corrupt = bytearray(self.frame)
        corrupt[20] ^= 1
        parser = StateFrameParser()
        self.assertEqual(len(parser.feed(bytes(corrupt) + self.frame)), 1)
        # Dropping bytes makes the next frame begin inside a CRC candidate.
        parser = StateFrameParser()
        self.assertEqual(len(parser.feed(self.frame[:-5] + self.frame)), 1)

    def test_bad_header_length_and_version(self):
        for offset in (2, 3, 4):
            corrupt = bytearray(self.frame)
            corrupt[offset] ^= 0xFF
            parser = StateFrameParser()
            self.assertEqual(len(parser.feed(bytes(corrupt) + self.frame)), 1)

    def test_invalid_payload(self):
        for position in (math.nan, math.inf):
            with self.assertRaises(ValueError):
                build_state_frame(0, [position, 0, 0, 0, 0, 0], 0, 0)
        corrupt = bytearray(self.frame)
        struct.pack_into("<f", corrupt, 12, math.nan)
        struct.pack_into("<H", corrupt, 40, crc16(corrupt[:40]))
        with self.assertRaises(ValueError):
            parse_state_frame(corrupt)

    def test_counter_wrap_and_bitmap(self):
        frame = build_state_frame(0x100000000, [0] * 6, 0xFFFF, 0xFFFF, 0x10000)
        decoded = parse_state_frame(frame)
        self.assertEqual((decoded.sequence, decoded.timestamp_ms), (0, 0))
        self.assertEqual((decoded.buttons, decoded.valid_motor_mask), (0xFFFF, 0x3F))

    def test_host_torque_frame_layout_and_crc(self):
        frame = build_host_torque_frame(123, [1, -2, 3, 4, 5, 6], 0x15, True, 7)
        self.assertEqual(len(frame), FRAME_SIZE)
        self.assertEqual((frame[2], frame[3]), (1, TYPE_HOST_GRAVITY_TORQUE))
        self.assertEqual(int.from_bytes(frame[4:6], "little"), FRAME_SIZE)
        self.assertEqual(int.from_bytes(frame[6:8], "little"), 7)
        self.assertEqual(int.from_bytes(frame[12:14], "little"), 0x15)
        self.assertEqual(int.from_bytes(frame[14:16], "little"), 1)
        self.assertEqual(int.from_bytes(frame[-2:], "little"), crc16(frame[:-2]))


if __name__ == "__main__":
    unittest.main()
