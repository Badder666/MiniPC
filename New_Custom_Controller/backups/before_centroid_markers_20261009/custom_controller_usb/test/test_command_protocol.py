import unittest
from pathlib import Path
import shutil
import struct
import subprocess
import tempfile

from custom_controller_usb.bridge_state import CommandSelector, PendingCommandWriter
from custom_controller_usb.joint_mapping import JointMapper
from custom_controller_usb.protocol import crc16

from custom_controller_usb.command_protocol import (
    ArmFeedbackParser, CommandFrame, OperatorState, build_command_frame,
    parse_command_frame,
)


class CommandProtocolTests(unittest.TestCase):
    def test_round_trip(self):
        command = CommandFrame(
            7, 1234, (0.0, 1.0, -2.0, 3.0, 4.0, 5.0), 0x1234, 0x3F, True,
            OperatorState(2000, 2100, (-1, 2, -3), (1, 2, 3), 0x2AAAA, 4, 8),
        )
        frame = build_command_frame(command)
        self.assertEqual(len(frame), 64)
        self.assertEqual(parse_command_frame(frame), command)

    def test_feedback_parser_handles_split_and_noise(self):
        payload = b"\xAA" + b"\x00" * 24
        payload += bytes([sum(payload[1:]) & 0xFF, 0x55])
        parser = ArmFeedbackParser()
        self.assertEqual(parser.feed(b"noise" + payload[:8]), [])
        self.assertEqual(parser.feed(payload[8:]), [(0.0,) * 6])

    def test_invalid_frames(self):
        command = CommandFrame(0, 0, (0.0,) * 6)
        frame = build_command_frame(command)
        for offset in (0, 2, 3, 4, 20, 62):
            damaged = bytearray(frame)
            damaged[offset] ^= 1
            with self.assertRaises(ValueError):
                parse_command_frame(bytes(damaged))
        for offset, value in ((38, 0x80), (53, 2), (60, 1)):
            damaged = bytearray(frame)
            damaged[offset] = value
            struct.pack_into("<H", damaged, 62, crc16(damaged[:62]))
            with self.assertRaises(ValueError):
                parse_command_frame(bytes(damaged))

    def test_mapping_and_valid_axis_permutation(self):
        mapper = JointMapper([2, 1, 0, 3, 4, 5], [-1.0, 2.0, 1.0, 1.0, 1.0, 1.0],
                             [0.5, 0.0, 0.0, 0.0, 0.0, 0.0])
        targets, mask = mapper.map([1, 2, 3, 4, 5, 6], 0x05)
        self.assertEqual(mask, 0x05)
        self.assertEqual(targets, (-2.5, 0, 1, 0, 0, 0))
        targets, mask = mapper.map([1, 2, float("nan"), 4, 5, 6], 0x05)
        self.assertEqual(mask, 0x04)
        with self.assertRaises(ValueError):
            JointMapper([0] * 6, [1.0] * 6, [0.0] * 6)

    def test_button_mapping_and_counter_wrap(self):
        selector = CommandSelector(JointMapper(range(6), [1.0] * 6, [0.0] * 6),
                                   range(8), range(8, 16))
        selector.sequence = 0xFFFF
        command = parse_command_frame(selector.encode([1] * 6, 0x15, 1234, 0x8001))
        self.assertEqual(command.operator.main_buttons, 1)
        self.assertEqual(command.operator.handle_buttons, 0x80)
        self.assertEqual(command.buttons, 0x8001)
        self.assertEqual(selector.sequence, 0)
        self.assertEqual(command.valid_joint_mask, 0x15)
        disabled = CommandSelector(JointMapper(range(6), [1.0] * 6, [0.0] * 6),
                                   [-1] * 8, [-1] * 8)
        command = parse_command_frame(disabled.encode([1] * 6, 0x3F, 0, 0xFFFF))
        self.assertEqual(command.buttons, 0xFFFF)
        self.assertEqual((command.operator.main_buttons, command.operator.handle_buttons), (0, 0))

    def test_short_write_keeps_frame_boundaries_and_drops_backlog(self):
        class Port:
            output = bytearray()

            def write(self, data):
                count = min(len(data), 7)
                self.output.extend(data[:count])
                return count

        writer, port = PendingCommandWriter(), Port()
        first = build_command_frame(CommandFrame(1, 0, (0.0,) * 6))
        skipped = build_command_frame(CommandFrame(2, 0, (0.0,) * 6))
        latest = build_command_frame(CommandFrame(3, 0, (0.0,) * 6))
        writer.submit(first)
        writer.write_once(port)
        writer.submit(skipped)
        writer.submit(latest)
        for _ in range(25):
            writer.write_once(port)
        self.assertEqual(port.output, first + latest)
        writer.submit(skipped)
        writer.clear()
        writer.write_once(port)
        self.assertEqual(port.output, first + latest)

    def test_python_sender_matches_c_receiver_and_usb_pipeline(self):
        engineering = Path(__file__).resolve().parents[5]
        firmware = engineering / "Engineer_2026_H7_Template_new"
        test_dir = firmware / "Tests" / "ControllerProtocol"
        compiler = shutil.which("gcc")
        if not compiler or not test_dir.is_dir():
            self.skipTest("requires sibling firmware source and host gcc")
        command = CommandFrame(7, 1234, (0, 1, -2, 3, 4, 5), 0x1234, 0x15, True,
                               OperatorState(2000, 2100, (-1, 2, -3), (1, 2, 3), 0x2AAAA, 4, 8))
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            frame_path = root / "command.bin"
            frame_path.write_bytes(build_command_frame(command))
            executable = root / "controller_command_test.exe"
            subprocess.run([
                compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
                "-I" + str(test_dir / "stubs"), "-I" + str(firmware / "Device/MiniPC"),
                "-I" + str(firmware / "Algorithm"), "-I" + str(firmware / "System"),
                str(test_dir / "test_controller_command.c"),
                str(firmware / "Device/MiniPC/custom_controller_protocol.c"),
                str(firmware / "Device/MiniPC/MiniPC.c"), str(firmware / "System/usb_comm.c"),
                str(firmware / "Algorithm/CRC.c"), "-o", str(executable),
            ], check=True, capture_output=True, text=True)
            result = subprocess.run([str(executable), str(frame_path)], check=True,
                                    capture_output=True, text=True)
            self.assertIn("integration tests passed", result.stdout)


if __name__ == "__main__":
    unittest.main()
