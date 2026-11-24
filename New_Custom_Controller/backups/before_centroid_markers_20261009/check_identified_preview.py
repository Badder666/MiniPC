import json
import time
from pathlib import Path
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from visualization_msgs.msg import MarkerArray
from custom_controller_interfaces.msg import ControllerState

rclpy.init()
node = rclpy.create_node('identified_preview_audit')
states, markers = [], []
node.create_subscription(ControllerState, '/custom_controller/state', states.append, 10)
node.create_subscription(MarkerArray, '/custom_controller/center_of_mass_markers', markers.append,
                         QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
deadline = time.monotonic() + 3.0
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
result = {'state_frames_3s': len(states), 'marker_arrays_3s': len(markers),
          'marker_count': len(markers[-1].markers) if markers else 0}
if states:
    result.update({'last_positions': [float(v) for v in states[-1].positions],
                   'valid_motor_mask': int(states[-1].valid_motor_mask)})
if markers:
    result['marker_frames'] = sorted({m.header.frame_id for m in markers[-1].markers})
Path('/tmp/controller_identified_preview_check.json').write_text(json.dumps(result, indent=2))
print(json.dumps(result))
node.destroy_node()
rclpy.shutdown()
path = Path(Path('/tmp/controller_identified_preview.logpath').read_text())
with path.open('rb') as stream:
    stream.seek(max(0, path.stat().st_size - 3000))
    print(stream.read().replace(bytes([0]), b'').decode(errors='replace'))
