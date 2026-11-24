"""Read controller USB CDC reports and print one JSON record per valid frame."""

import argparse
from dataclasses import asdict
import json

from usb_report_protocol import StateFrameParser


def main() -> None:
    arg_parser = argparse.ArgumentParser(description=__doc__)
    arg_parser.add_argument("port", help="USB serial port, e.g. COM8 or /dev/ttyACM0")
    args = arg_parser.parse_args()
    try:
        import serial
    except ImportError:
        arg_parser.exit(1, "Install pyserial: python -m pip install -r requirements.txt\n")

    parser = StateFrameParser()
    try:
        # CDC line coding does not set the USB transfer rate.
        with serial.Serial(args.port, baudrate=115200, timeout=0.1) as port:
            while True:
                chunk = port.read(max(1, min(port.in_waiting, 4096)))
                for frame in parser.feed(chunk):
                    print(json.dumps(asdict(frame)), flush=True)
    except serial.SerialException as error:
        arg_parser.exit(1, f"USB serial error: {error}\n")
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
