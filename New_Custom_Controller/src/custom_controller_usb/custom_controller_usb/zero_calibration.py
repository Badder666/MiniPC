"""RViz menu and persisted encoder-zero calibration shared with gravity."""

from datetime import datetime, timezone
import json
import math
from pathlib import Path
import re
import shutil
import tempfile
import time

import yaml
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParametersAtomically
from std_srvs.srv import Trigger
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker

from .joint_mapping import JointMapper
from .live_trajectory import LiveTrajectory


def save_zero_snapshot(config_path, motors, mapper, offsets):
    """Preserve YAML comments, back up the old config and atomically replace it."""
    path = Path(config_path).expanduser().resolve(strict=True)
    text = path.read_text(encoding="utf-8")
    parameters = yaml.safe_load(text)["/**"]["ros__parameters"]
    if (tuple(parameters["axis_order"]) != mapper.axis_order or
            tuple(parameters["scales"]) != mapper.scales):
        raise ValueError("配置中的轴序/比例已变化，请重启后重新记录零位")
    records = path.parent / "zero_snapshots"
    records.mkdir(exist_ok=True)
    name = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    backup = records / (name + "_before.yaml")
    shutil.copy2(path, backup)
    record = records / (name + ".json")
    record.write_text(json.dumps({
        "time_utc": name, "encoder_positions": list(motors),
        "axis_order": mapper.axis_order, "scales": mapper.scales,
        "previous_offsets": mapper.offsets, "offsets": list(offsets),
        "target_model_angles": [0.0] * 6, "config_backup": str(backup),
    }, ensure_ascii=False, indent=2), encoding="utf-8")
    for key, values in (("offsets", offsets), ("zero_encoder_positions", motors)):
        line = "    " + key + ": " + json.dumps(list(values))
        pattern = r"(?m)^    " + key + r":[^\n]*$"
        if re.search(pattern, text):
            text = re.sub(pattern, lambda _: line, text)
        else:
            text += "\n" + line + "\n"
    yaml.safe_load(text)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent,
                                         delete=False) as output:
            temporary = Path(output.name)
            output.write(text)
        temporary.replace(path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
    return record


class ZeroCalibration:
    def __init__(self, node):
        self.node = node
        self.busy = False
        self.config_path = node.get_parameter("calibration_config_path").value
        group = ReentrantCallbackGroup()
        self.receiver = node.create_client(
            SetParametersAtomically, "/custom_controller/usb_receiver/set_parameters_atomically",
            callback_group=group)
        self.service = node.create_service(Trigger, "capture_zero", self.capture)
        self.client = node.create_client(Trigger, "capture_zero", callback_group=group)
        self.server = InteractiveMarkerServer(node, "zero_calibration")
        self.menu = MenuHandler()
        self.menu.insert("Capture current pose as ZERO", callback=self._menu_capture)
        button = InteractiveMarker()
        button.header.frame_id = "world"
        button.name = "capture_zero"
        button.description = "ZERO SNAPSHOT (right click)"
        button.pose.orientation.w = 1.0
        button.pose.position.x = 0.25
        button.pose.position.y = 0.15
        button.pose.position.z = 0.10
        button.scale = 0.15
        control = InteractiveMarkerControl()
        control.name = "capture_zero_menu"
        control.interaction_mode = InteractiveMarkerControl.MENU
        control.always_visible = True
        cube = Marker()
        cube.type = Marker.CUBE
        cube.pose.orientation.w = 1.0
        cube.scale.x, cube.scale.y, cube.scale.z = 0.07, 0.07, 0.025
        cube.color.r, cube.color.g, cube.color.b, cube.color.a = 0.2, 0.85, 0.45, 1.0
        control.markers.append(cube)
        button.controls.append(control)
        self.button = button
        self.server.insert(button)
        self.menu.apply(self.server, button.name)
        self.server.applyChanges()

    def _menu_capture(self, feedback):
        future = self.client.call_async(Trigger.Request())
        future.add_done_callback(self._report)

    def _report(self, future):
        result = future.result()
        logger = self.node.get_logger()
        (logger.info if result.success else logger.warning)(result.message)

    def _set_receiver_offsets(self, offsets):
        request = SetParametersAtomically.Request()
        request.parameters = [Parameter("offsets", value=list(offsets)).to_parameter_msg()]
        return self.receiver.call_async(request)

    async def capture(self, request, response):
        node = self.node
        state = node._state
        now = time.monotonic()
        if self.busy:
            response.message = "正在记录零位，请等待"
            return response
        if state.effective_valid_mask(now) != 0x3F or not all(
                math.isfinite(v) for v in state.raw_positions):
            response.message = "未记录：需要六轴实时有效反馈，请检查 USB 和电机反馈"
            return response
        if not self.config_path or not self.receiver.service_is_ready():
            response.message = "未记录：缺少配置路径或上位机接收节点未就绪"
            return response
        motors = tuple(float(v) for v in state.raw_positions)
        mapper = state.mapper
        offsets = [-mapper.scales[j] * motors[m] for j, m in enumerate(mapper.axis_order)]
        self.busy = True
        updated = False
        try:
            result = await self._set_receiver_offsets(offsets)
            if not result.result.successful:
                response.message = "未记录：" + result.result.reason
                return response
            updated = True
            record = save_zero_snapshot(self.config_path, motors, mapper, offsets)
            node.set_parameters([Parameter("offsets", value=offsets),
                                 Parameter("zero_encoder_positions", value=list(motors))])
            state.mapper = JointMapper(mapper.axis_order, mapper.scales, offsets)
            state.targets, _ = state.mapper.map(state.raw_positions, state.valid_joint_mask)
            state.targets = list(state.targets)
            node._zero_positions, _ = state.mapper.map(motors, 0x3F)
            # Old paths used different coordinates; start a new calibration segment.
            node._trajectory = LiveTrajectory(
                node._urdf_text, int(node.get_parameter("trajectory_max_points").value),
                node.get_parameter("display_stale_after_s").value)
            node._planned_trajectory = None
            from visualization_msgs.msg import MarkerArray
            deletion = Marker()
            deletion.action = Marker.DELETEALL
            node._trajectory_pub.publish(MarkerArray(markers=[deletion]))
            response.success = True
            response.message = f"零位已记录，offsets={offsets}；快照：{record}"
            self.button.description = "ZERO SAVED (right click to recapture)"
            self.server.insert(self.button)
            self.menu.apply(self.server, self.button.name)
            self.server.applyChanges()
        except (OSError, TypeError, ValueError, KeyError, yaml.YAMLError) as error:
            if updated:
                await self._set_receiver_offsets(mapper.offsets)
            response.message = "零位保存失败：" + str(error)
        finally:
            self.busy = False
        return response

    def shutdown(self):
        self.server.shutdown()
