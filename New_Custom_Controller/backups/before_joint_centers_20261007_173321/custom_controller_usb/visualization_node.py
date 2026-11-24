"""RViz target/actual poses and controller diagnostics; read-only."""

from pathlib import Path
import csv
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy
from custom_controller_interfaces.msg import ControllerState
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from .joint_mapping import JointMapper
from .live_trajectory import LiveTrajectory
from .trajectory import TrajectoryData, load_trajectory
from .visualization_state import VisualizationState, read_arm_description


class ControllerVisualization(Node):
    def __init__(self, **kwargs):
        kwargs.setdefault("namespace", "custom_controller")
        super().__init__("visualization", **kwargs)
        self.declare_parameter("urdf_path", "")
        self.declare_parameter("axis_order", list(range(6)))
        self.declare_parameter("scales", [1.0] * 6)
        self.declare_parameter("offsets", [0.0] * 6)
        self.declare_parameter("arm_joint_names", [f"joint_{i + 1}" for i in range(6)])
        self.declare_parameter("reference_positions", [0.0, 2.4, -2.4, 0.0, 0.0, 0.0])
        self.declare_parameter("display_stale_after_s", 0.25)
        self.declare_parameter("display_period_s", 0.05)
        self.declare_parameter("show_status_text", False)
        self.declare_parameter("forward_enabled", False)
        self.declare_parameter("show_trajectory", True)
        self.declare_parameter("trajectory_max_points", 3000)
        self.declare_parameter("show_planned_trajectory", True)
        self.declare_parameter("planned_trajectory_csv", "")
        self.declare_parameter("planned_trajectory_max_points", 3000)
        self.declare_parameter("trajectory_output_csv", "")
        urdf_path = self.get_parameter("urdf_path").value
        urdf_text = Path(urdf_path).read_text(encoding="utf-8")
        self._description = read_arm_description(urdf_text)
        mapper = JointMapper(self.get_parameter("axis_order").value,
                             self.get_parameter("scales").value,
                             self.get_parameter("offsets").value)
        # Encoder-zero reference: q = scale * 0 + offset, independent of USB.
        self._zero_positions, _ = mapper.map([0.0] * 6, 0x3F)
        self._state = VisualizationState(mapper, self.get_parameter("arm_joint_names").value,
                                         self.get_parameter("reference_positions").value,
                                         self.get_parameter("display_stale_after_s").value)
        self._forward = self.get_parameter("forward_enabled").value
        self._show_status_text = self.get_parameter("show_status_text").value
        self._trajectory_frame = "world"
        self._show_trajectory = self.get_parameter("show_trajectory").value
        self._trajectory = LiveTrajectory(
            urdf_text, int(self.get_parameter("trajectory_max_points").value),
            self.get_parameter("display_stale_after_s").value)
        self._planned_trajectory: TrajectoryData | None = None
        self._show_planned_trajectory = self.get_parameter("show_planned_trajectory").value
        planned_path = str(self.get_parameter("planned_trajectory_csv").value or "").strip()
        if self._show_planned_trajectory and planned_path:
            try:
                self._planned_trajectory = load_trajectory(
                    planned_path, urdf_text, mapper,
                    max_points=int(self.get_parameter("planned_trajectory_max_points").value),
                    mode="auto")
                self.get_logger().info(
                    f"规划轨迹已加载: {self._planned_trajectory.source} "
                    f"({self._planned_trajectory.count} 点, "
                    f"读取 {self._planned_trajectory.rows_read} 行, "
                    f"跳过 {self._planned_trajectory.rows_skipped} 行)")
            except (OSError, ValueError) as error:
                self.get_logger().warning(f"规划轨迹未加载: {error}")
        self._trajectory_csv = None
        trajectory_output = str(self.get_parameter("trajectory_output_csv").value or "").strip()
        if trajectory_output:
            output_path = Path(trajectory_output).expanduser()
            output_path.parent.mkdir(parents=True, exist_ok=True)
            self._trajectory_csv = output_path.open("w", encoding="utf-8", newline="")
            self._trajectory_writer = csv.writer(self._trajectory_csv)
            self._trajectory_writer.writerow([
                "sample", "timestamp_ms", *[f"controller_J{i}" for i in range(1, 7)],
                *[f"urdf_J{i}" for i in range(1, 7)],
                "valid_mask", "segment", "end_x", "end_y", "end_z"])
            self._trajectory_csv.flush()
            self.get_logger().info(f"实时六轴轨迹保存到 {output_path}")
        period = self.get_parameter("display_period_s").value
        if not math.isfinite(period) or period <= 0:
            raise ValueError("display_period_s must be positive and finite")
        qos = QoSProfile(depth=1)
        marker_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._target_pub = self.create_publisher(JointState, "target/joint_states", qos)
        self._zero_pub = self.create_publisher(JointState, "zero/joint_states", qos)
        self._actual_pub = self.create_publisher(JointState, "actual/joint_states", qos)
        self._marker_pub = self.create_publisher(MarkerArray, "status_markers", marker_qos)
        self._trajectory_pub = self.create_publisher(MarkerArray, "trajectory_markers", marker_qos)
        self._next_trajectory_publish = 0.0
        self._diagnostic_pub = self.create_publisher(DiagnosticArray, "diagnostics", qos)
        self._controller_sub = self.create_subscription(ControllerState, "state", self._controller_cb, qos)
        self._feedback_sub = self.create_subscription(JointState, "arm_joint_states", self._feedback_cb, qos)
        self._timer = self.create_timer(period, self._refresh)

    def _controller_cb(self, msg):
        try:
            self._state.update_controller(msg.positions, msg.valid_motor_mask, msg.buttons,
                                          msg.sequence, msg.device_time_ms, time.monotonic())
            accepted = self._trajectory.update(self._state.targets, self._state.valid_joint_mask,
                                               time.monotonic(), msg.device_time_ms)
            if accepted and self._trajectory_csv is not None:
                index = self._trajectory.count - 1
                self._trajectory_writer.writerow([
                    index, self._trajectory.device_times[index],
                    *[f"{v:.9f}" for v in msg.positions],
                    *[f"{v:.9f}" for v in self._trajectory.joint_positions[index]],
                    f"0x{msg.valid_motor_mask:02X}", self._trajectory.segments[index],
                    *[f"{v:.9f}" for v in self._trajectory.points[index]],
                ])
                self._trajectory_csv.flush()
        except ValueError as error:
            self.get_logger().warning(f"忽略无效显示状态: {error}")

    def _feedback_cb(self, msg):
        try:
            self._state.update_feedback(msg.name, msg.position, time.monotonic())
        except ValueError as error:
            self.get_logger().warning(f"忽略无效机械臂反馈: {error}")

    def _joints(self, publisher, positions, stamp):
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = list(self._description.joint_names)
        msg.position = list(positions)
        publisher.publish(msg)

    @staticmethod
    def _value(value):
        return "--" if value is None or not math.isfinite(value) else f"{value:+.3f}"

    def _rows(self, now):
        state = self._state
        white = (0.90, 0.93, 0.95)
        green = (0.32, 0.88, 0.57)
        amber = (1.0, 0.72, 0.25)
        red = (1.0, 0.36, 0.36)
        controller_online = state.controller_online(now)
        feedback_online = state.feedback_online(now)
        controller_age = state.age(state.controller_arrival, now)
        feedback_age = state.age(state.feedback_arrival, now)
        age_text = lambda age: "--" if age is None else f"{age * 1000:.0f} ms"
        rows = [
            ("CUSTOM CONTROLLER / ARM", white),
            (f"MODE: {'FORWARDING' if self._forward else 'OBSERVE'}", green),
            (f"USB: {'LIVE' if controller_online else 'WAITING / STALE'}  "
             f"{state.receive_rate(now):.1f} Hz  age {age_text(controller_age)}",
             green if controller_online else red),
            (f"ARM: {'LIVE' if feedback_online else 'WAITING / STALE'}  age {age_text(feedback_age)}",
             green if feedback_online else amber),
            (f"SEQ {state.sequence}   TIME {state.device_time_ms} ms   "
             f"VALID 0x{state.effective_valid_mask(now):02X}", white),
            (f"TARGET POSE: {'REFERENCE' if not state.target_seen_mask else 'LAST VALID VALUES'}", amber),
            ("AXIS  MOTOR    RAW       TARGET     ACTUAL     ERROR", white),
        ]
        for joint in range(6):
            motor = state.mapper.axis_order[joint]
            bit = 1 << joint
            valid = bool(state.effective_valid_mask(now) & bit)
            target = state.targets[joint] if state.target_seen_mask & bit else None
            actual = state.actual[joint] if state.actual_seen_mask & bit else None
            raw = state.raw_positions[motor]
            lower, upper = self._description.limits[joint]
            outside = target is not None and not lower <= target <= upper
            suffix = " LIMIT" if outside else "" if valid else " INVALID"
            rows.append((f"J{joint + 1}     M{motor + 1}   {self._value(raw):>7}   "
                         f"{self._value(target):>7}   {self._value(actual):>7}   "
                         f"{self._value(state.joint_error(joint, now)):>7}{suffix}",
                         amber if outside or not valid else white))
        pressed = " ".join(str(i) for i in range(16) if state.buttons & (1 << i)) or "--"
        rows.extend([
            (f"BUTTONS 0x{state.buttons:04X}   {pressed}", white),
            ("CONTROLLER: LIVE USB POSE", green),
        ])
        return rows

    @staticmethod
    def _point(value):
        point = Point()
        point.x, point.y, point.z = value
        return point

    def _trajectory_markers(self, stamp):
        """Create the planned path and live observed path for one MarkerArray."""
        markers = MarkerArray()
        if self._show_planned_trajectory and self._planned_trajectory is not None:
            planned = self._planned_trajectory
            path = Marker()
            path.header.frame_id = self._trajectory_frame
            path.header.stamp = stamp
            path.ns = "planned_trajectory"
            path.id = 0
            path.type = Marker.LINE_STRIP
            path.action = Marker.ADD
            path.pose.orientation.w = 1.0
            path.scale.x = 0.006
            path.color.r, path.color.g, path.color.b, path.color.a = (0.10, 0.95, 0.95, 0.95)
            path.points = [self._point(point) for point in planned.points]
            markers.markers.append(path)

            samples = Marker()
            samples.header.frame_id = self._trajectory_frame
            samples.header.stamp = stamp
            samples.ns = "planned_trajectory"
            samples.id = 1
            samples.type = Marker.SPHERE_LIST
            samples.action = Marker.ADD
            samples.pose.orientation.w = 1.0
            samples.scale.x = samples.scale.y = samples.scale.z = 0.012
            samples.color.r, samples.color.g, samples.color.b, samples.color.a = (0.10, 0.95, 0.95, 0.80)
            samples.points = [self._point(point) for point in planned.points]
            markers.markers.append(samples)

            lower = planned.minimum
            upper = planned.maximum
            label = Marker()
            label.header.frame_id = self._trajectory_frame
            label.header.stamp = stamp
            label.ns = "planned_trajectory"
            label.id = 2
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x, label.pose.position.y = upper[0], upper[1]
            label.pose.position.z = upper[2] + 0.12
            label.pose.orientation.w = 1.0
            label.scale.z = 0.05
            label.color.r, label.color.g, label.color.b, label.color.a = (0.10, 0.95, 0.95, 0.95)
            label.text = (f"PLANNED COLLISION-FREE PATH  {planned.count} pts\n"
                          f"X {lower[0]:+.3f}..{upper[0]:+.3f}  "
                          f"Y {lower[1]:+.3f}..{upper[1]:+.3f}  "
                          f"Z {lower[2]:+.3f}..{upper[2]:+.3f}")
            markers.markers.append(label)

        if self._show_trajectory and self._trajectory.count:
            path = Marker()
            path.header.frame_id = self._trajectory_frame
            path.header.stamp = stamp
            path.ns = "collected_trajectory"
            path.id = 0
            path.type = Marker.LINE_LIST
            path.action = Marker.ADD
            path.pose.orientation.w = 1.0
            path.scale.x = 0.002
            path.color.r, path.color.g, path.color.b, path.color.a = (1.0, 0.72, 0.10, 0.92)
            path.points = [self._point(point) for point in self._trajectory.line_points()]
            markers.markers.append(path)

            samples = Marker()
            samples.header.frame_id = self._trajectory_frame
            samples.header.stamp = stamp
            samples.ns = "collected_trajectory"
            samples.id = 1
            samples.type = Marker.SPHERE_LIST
            samples.action = Marker.ADD
            samples.pose.orientation.w = 1.0
            samples.scale.x = samples.scale.y = samples.scale.z = 0.004
            samples.color.r, samples.color.g, samples.color.b, samples.color.a = (1.0, 0.88, 0.25, 0.42)
            samples.points = [self._point(point) for point in self._trajectory.points]
            markers.markers.append(samples)

            lower = self._trajectory.minimum
            upper = self._trajectory.maximum
            corners = [(x, y, z) for x in (lower[0], upper[0])
                       for y in (lower[1], upper[1]) for z in (lower[2], upper[2])]
            edges = ((0, 1), (0, 2), (0, 4), (1, 3), (1, 5),
                     (2, 3), (2, 6), (3, 7), (4, 5), (4, 6),
                     (5, 7), (6, 7))
            bounds = Marker()
            bounds.header.frame_id = self._trajectory_frame
            bounds.header.stamp = stamp
            bounds.ns = "collected_trajectory"
            bounds.id = 2
            bounds.type = Marker.LINE_LIST
            bounds.action = Marker.ADD
            bounds.pose.orientation.w = 1.0
            bounds.scale.x = 0.004
            bounds.color.r, bounds.color.g, bounds.color.b, bounds.color.a = (0.15, 0.85, 1.0, 0.62)
            for left, right in edges:
                bounds.points.extend((self._point(corners[left]), self._point(corners[right])))
            markers.markers.append(bounds)

            label = Marker()
            label.header.frame_id = self._trajectory_frame
            label.header.stamp = stamp
            label.ns = "collected_trajectory"
            label.id = 3
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x, label.pose.position.y = upper[0], upper[1]
            label.pose.position.z = upper[2] + 0.10
            label.pose.orientation.w = 1.0
            label.scale.z = 0.045
            label.color.r, label.color.g, label.color.b, label.color.a = (1.0, 0.88, 0.25, 0.95)
            label.text = (f"LIVE OBSERVED RANGE  {self._trajectory.count} pts\n"
                          f"X {lower[0]:+.3f}..{upper[0]:+.3f}  "
                          f"Y {lower[1]:+.3f}..{upper[1]:+.3f}  "
                          f"Z {lower[2]:+.3f}..{upper[2]:+.3f}")
            markers.markers.append(label)
        return markers

    def _refresh(self):
        now = time.monotonic()
        stamp = self.get_clock().now().to_msg()
        # The reference pose is visual only; never send it to the command bridge.
        self._joints(self._zero_pub, self._zero_positions, stamp)
        self._joints(self._target_pub, self._state.targets, stamp)
        if self._state.actual_seen_mask:
            self._joints(self._actual_pub, self._state.actual, stamp)
        if now >= self._next_trajectory_publish:
            self._trajectory_pub.publish(self._trajectory_markers(stamp))
            self._next_trajectory_publish = now + 0.2
        markers = MarkerArray()
        for index, (text, color) in enumerate(self._rows(now)) if self._show_status_text else []:
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = "world"
            marker.ns = "controller_status"
            marker.id = index
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.position.x = 0.90
            marker.pose.position.y = 0.25
            marker.pose.position.z = 1.0 - index * 0.055
            marker.pose.orientation.w = 1.0
            marker.scale.z = 0.027
            marker.color.r, marker.color.g, marker.color.b = color
            marker.color.a = 1.0
            marker.text = text
            markers.markers.append(marker)
        self._marker_pub.publish(markers)

        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = stamp
        for label, online, arrival in (
                ("Controller USB", self._state.controller_online(now), self._state.controller_arrival),
                ("Arm feedback", self._state.feedback_online(now), self._state.feedback_arrival)):
            status = DiagnosticStatus()
            status.name = f"custom_controller/{label}"
            status.hardware_id = "DM4310 controller / controlled arm"
            status.level = DiagnosticStatus.OK if online else DiagnosticStatus.WARN
            status.message = "LIVE" if online else "WAITING / STALE"
            age = self._state.age(arrival, now)
            status.values = [KeyValue(key="age_ms", value="--" if age is None else f"{age * 1000:.0f}"),
                             KeyValue(key="valid_joint_mask", value=f"0x{self._state.effective_valid_mask(now):02X}"),
                             KeyValue(key="buttons", value=f"0x{self._state.buttons:04X}")]
            diagnostics.status.append(status)
        self._diagnostic_pub.publish(diagnostics)

    def destroy_node(self):
        self._timer.cancel()
        if self._trajectory_csv is not None:
            self._trajectory_csv.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ControllerVisualization()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
