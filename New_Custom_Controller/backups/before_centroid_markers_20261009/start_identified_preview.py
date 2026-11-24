"""Restart the known offset-calibration display as an identified COM preview."""
import os
import signal
import subprocess
import time
from pathlib import Path

old_pid_file = Path('/tmp/controller_offset_calibration.pid')
if old_pid_file.exists():
    pid = int(old_pid_file.read_text())
    proc = Path(f'/proc/{pid}/cmdline')
    if proc.exists():
        cmd = proc.read_bytes().replace(b'\0', b' ').decode()
        if 'ros2 launch custom_controller_usb visualization.launch.py' not in cmd or os.getpgid(pid) != pid:
            raise RuntimeError(f'Refuse to stop unrelated process: {pid}: {cmd}')
        os.killpg(pid, signal.SIGINT)
        for _ in range(40):
            if not proc.exists():
                break
            time.sleep(0.2)
        else:
            raise RuntimeError('Previous visualization did not stop')

env = os.environ.copy()
env['DISPLAY'] = ':0'
env['XDG_RUNTIME_DIR'] = '/run/user/1000'
auth = sorted(Path('/run/user/1000').glob('.mutter-Xwaylandauth.*'))
if auth:
    env['XAUTHORITY'] = str(auth[-1])
log_path = f'/tmp/controller_identified_preview_{time.strftime("%Y%m%d_%H%M%S")}.log'
command = ('source /opt/ros/humble/setup.bash && '
           'source /home/badder/minipc/New_Custom_Controller/install/setup.bash && '
           'exec ros2 launch custom_controller_usb identified_gravity.launch.py '
           'port:=/dev/ttyACM0 gravity_dry_run:=true torque_scale:=1.0')
with open(log_path, 'w') as log:
    process = subprocess.Popen(['bash', '-c', command], env=env, stdout=log,
                               stderr=subprocess.STDOUT, start_new_session=True)
Path('/tmp/controller_identified_preview.pid').write_text(str(process.pid))
Path('/tmp/controller_identified_preview.logpath').write_text(log_path)
print('PID:', process.pid, 'LOG:', log_path)
