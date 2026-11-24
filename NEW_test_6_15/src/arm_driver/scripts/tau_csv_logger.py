#!/usr/bin/env python3
"""
订阅 real_positions 话题，将关节位置和力矩记录到 CSV 文件。

用法:
  ros2 run arm_driver tau_csv_logger.py
  ros2 run arm_driver tau_csv_logger.py --ros-args -p output:=/tmp/tau_data.csv
"""

import csv
import os
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class TauCsvLogger(Node):
    def __init__(self):
        super().__init__("tau_csv_logger")

        self.declare_parameter("output", os.path.expanduser("~/tau_data.csv"))
        self.declare_parameter("save_interval", 0.02)  # 50Hz 写入间隔

        output_path = self.get_parameter("output").get_parameter_value().string_value
        self._save_interval = (
            self.get_parameter("save_interval").get_parameter_value().double_value
        )

        self._file = open(output_path, "w", newline="")
        self._writer = csv.writer(self._file)
        self._writer.writerow([
            "timestamp",
            "J1_pos", "J2_pos", "J3_pos", "J4_pos", "J5_pos", "J6_pos",
            "J1_tau", "J2_tau", "J3_tau", "J4_tau", "J5_tau", "J6_tau",
        ])
        self._last_write = 0.0

        self._sub = self.create_subscription(
            JointState, "real_positions", self._cb, 10
        )
        self.get_logger().info(f"记录 tau → {output_path}")

    def _cb(self, msg: JointState):
        now = time.time()
        if now - self._last_write < self._save_interval:
            return
        self._last_write = now

        pos = list(msg.position) if msg.position else [0.0] * 6
        eff = list(msg.effort) if msg.effort else [0.0] * 6

        row = [msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9]
        row.extend(f"{p:.6f}" for p in pos[:6])
        row.extend(f"{e:.6f}" for e in eff[:6])
        self._writer.writerow(row)
        self._file.flush()

    def destroy_node(self):
        self._file.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = TauCsvLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()