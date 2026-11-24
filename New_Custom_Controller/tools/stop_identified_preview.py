"""停止本次后台启动的辨识模型预览，不影响其它 ROS 程序。"""
import os
from pathlib import Path
import signal
import time

pid_file = Path("/tmp/controller_identified_preview.pid")
if pid_file.exists():
    pid = int(pid_file.read_text())
    proc = Path(f"/proc/{pid}/cmdline")
    if proc.exists():
        command = proc.read_bytes().replace(b"\0", b" ").decode()
        if "ros2 launch custom_controller_usb identified_gravity.launch.py" not in command or os.getpgid(pid) != pid:
            raise RuntimeError(f"PID {pid} 已属于其它程序，未停止")
        os.killpg(pid, signal.SIGINT)
        for _ in range(40):
            if not proc.exists():
                break
            time.sleep(0.2)
        else:
            raise RuntimeError("旧预览仍未退出，请检查后再打开串口")
        print(f"辨识模型后台预览已停止：{pid}")
    else:
        print("后台预览已经退出")
else:
    print("没有本次后台预览的 PID 记录")
