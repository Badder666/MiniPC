#!/usr/bin/env python3
"""将 real_positions 话题转发到 /joint_states，供 RViz 可视化。"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateRelay(Node):
    def __init__(self):
        super().__init__("joint_state_relay")
        self._pub = self.create_publisher(JointState, "/joint_states", 10)
        self._sub = self.create_subscription(
            JointState, "real_positions", self._cb, 10
        )
        self.get_logger().info("转发 real_positions → /joint_states")

    def _cb(self, msg: JointState):
        self._pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(JointStateRelay())
    rclpy.shutdown()


if __name__ == "__main__":
    main()