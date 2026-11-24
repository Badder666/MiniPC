#!/usr/bin/env python3
"""控制器模型、MCU 同源重力补偿及模拟人手外力；独立于 ROS 和 USB。"""

import argparse
import csv
import ctypes
import hashlib
from pathlib import Path
import queue
import subprocess
import tempfile
import threading
import time
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / "src/custom_controller_usb/config/controller_model_urdf2/controller_identified.urdf"
MODEL_SHA256 = "0b717261f6c01a3a1392398b1b5ccb77986d4be4c72fd7c6cdfe7cddb9da1422"
GRAVITY = np.array([0.0, 0.0, -9.7883], dtype=np.float32)
JOINT_NAMES = [f"joint{i}" for i in range(1, 7)]
CONTROL_PERIOD = 0.01


def vector(text, count):
    values = np.array([float(part) for part in text.split(",")])
    if values.shape != (count,) or not np.all(np.isfinite(values)):
        raise argparse.ArgumentTypeError(f"需要 {count} 个有限数值，以逗号分隔")
    return values


class FirmwareGravity:
    """编译并调用从 MCU 工程复制的算法，输入/输出均为 URDF 关节坐标。"""

    def __init__(self):
        source = Path(__file__).parent / "gravity_c"
        names = ["gravity_compensation.c", "gravity_compensation.h", "gravity_model.c", "gravity_model.h"]
        digest = hashlib.sha256(b"".join((source / name).read_bytes() for name in names)).hexdigest()[:16]
        cache = Path.home() / ".cache/custom_controller_sim" / digest
        cache.mkdir(parents=True, exist_ok=True)
        library = cache / "libcontroller_gravity.so"
        if not library.exists():
            with tempfile.TemporaryDirectory(dir=cache) as directory:
                output = Path(directory) / library.name
                subprocess.run(["gcc", "-std=c11", "-O2", "-fPIC", "-shared",
                                str(source / "gravity_compensation.c"), str(source / "gravity_model.c"),
                                "-lm", "-o", str(output)], check=True)
                output.replace(library)
        self.library = ctypes.CDLL(str(library))
        self.calculate = self.library.GravityCompensation_Calculate
        array = np.ctypeslib.ndpointer(dtype=np.float32, flags="C_CONTIGUOUS")
        self.calculate.argtypes = [array, array, array]
        self.calculate.restype = ctypes.c_bool

    def torque(self, q):
        result = np.empty(6, dtype=np.float32)
        if not self.calculate(np.asarray(q, dtype=np.float32), GRAVITY, result):
            raise ValueError("MCU 重力算法拒绝了输入")
        return result.astype(float)


def load_model():
    """保留原质量、质心、惯量及安装变换；适配网格路径和无限位关节。"""
    if hashlib.sha256(MODEL.read_bytes()).hexdigest() != MODEL_SHA256:
        raise ValueError("URDF 已变化，需要同步 gravity_c 模型常量后再运行")
    robot = ET.parse(MODEL).getroot()
    for link in robot.findall("link"):
        for collision in link.findall("collision"):
            link.remove(collision)
    for mesh in robot.findall(".//mesh"):
        mesh.set("filename", str(MODEL.parent / "meshes" / Path(mesh.get("filename")).name))
    for joint in robot.findall("joint"):
        if joint.get("type") == "revolute":
            joint.set("type", "continuous")
            limit = joint.find("limit")
            if limit is not None:
                joint.remove(limit)
    extension = ET.SubElement(robot, "mujoco")
    ET.SubElement(extension, "compiler", discardvisual="false", fusestatic="false", strippath="false")
    imported = mujoco.MjModel.from_xml_string(ET.tostring(robot, encoding="unicode"))
    with tempfile.TemporaryDirectory() as directory:
        xml_path = Path(directory) / "controller.xml"
        mujoco.mj_saveLastXML(str(xml_path), imported)
        scene = ET.parse(xml_path).getroot()
    option = scene.find("option")
    if option is None:
        option = ET.SubElement(scene, "option")
    option.set("gravity", "0 0 -9.7883")
    option.set("timestep", "0.001")
    option.set("integrator", "implicitfast")
    world = scene.find("worldbody")
    ET.SubElement(world, "light", pos="0 -1 2", dir="0 0 -1", directional="true")
    ET.SubElement(world, "geom", name="ground_visual", type="plane", pos="0 0 -0.08",
                  size="2 2 0.1", rgba="0.18 0.20 0.23 1", contype="0", conaffinity="0")
    tip = scene.find(".//body[@name='link6']")
    ET.SubElement(tip, "site", name="hand_point", pos="0 0 0", size="0.012", rgba="1 0.5 0.1 1")
    for geom in scene.findall(".//geom"):
        geom.set("contype", "0")
        geom.set("conaffinity", "0")
    actuators = ET.SubElement(scene, "actuator")
    for name in JOINT_NAMES:
        ET.SubElement(actuators, "motor", name=f"motor_{name}", joint=name, gear="1", ctrllimited="false")
    model = mujoco.MjModel.from_xml_string(ET.tostring(scene, encoding="unicode"))
    names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(model.njnt)]
    if model.nq != 6 or model.nv != 6 or names != JOINT_NAMES:
        raise ValueError(f"关节顺序不匹配：{names}")
    return model


class ControllerSimulation:
    def __init__(self, args):
        self.args = args
        self.model = load_model()
        self.data = mujoco.MjData(self.model)
        self.gravity = FirmwareGravity()
        self.body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "link6")
        self.site = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "hand_point")
        self.commands = queue.SimpleQueue()
        self.paused = False
        self.compensation = True
        self.pulse_end = 0.0
        self.pulse_force = np.zeros(3)
        self.feedforward = np.zeros(6)
        self.external = np.zeros(6)
        self.input_force = np.zeros(3)
        self.next_control = 0.0
        self.next_log = 0.0
        self.csv_file = None
        self.writer = None
        if args.csv:
            output = Path(args.csv).expanduser()
            output.parent.mkdir(parents=True, exist_ok=True)
            self.csv_file = output.open("x", encoding="utf-8", newline="")
            self.writer = csv.writer(self.csv_file)
            self.writer.writerow(["time_s", *[f"q_J{i}" for i in range(1, 7)],
                                  *[f"dq_J{i}" for i in range(1, 7)],
                                  *[f"gravity_J{i}" for i in range(1, 7)],
                                  *[f"motor_J{i}" for i in range(1, 7)],
                                  *[f"external_J{i}" for i in range(1, 7)],
                                  "force_x_N", "force_y_N", "force_z_N"])
        self.reset()

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:] = self.args.initial
        mujoco.mj_forward(self.model, self.data)
        self.next_control = self.data.time
        self.next_log = self.data.time
        self.pulse_end = 0.0
        self.feedforward[:] = 0
        self.external[:] = 0

    def handle_keys(self):
        while not self.commands.empty():
            code = self.commands.get()
            if code == ord("R"):
                self.reset()
                print("已重置初始姿态", flush=True)
            elif code == ord("G"):
                self.compensation = not self.compensation
                self.next_control = self.data.time
                print(f"重力补偿：{'开启' if self.compensation else '关闭'}", flush=True)
            elif code == ord("P"):
                self.paused = not self.paused
            elif code in (ord("U"), ord("J"), ord("I"), ord("K")):
                self.pulse_force = np.array({ord("U"): [0, 0, self.args.hand_force],
                                            ord("J"): [0, 0, -self.args.hand_force],
                                            ord("I"): [0, self.args.hand_force, 0],
                                            ord("K"): [0, -self.args.hand_force, 0]}[code])
                self.pulse_end = self.data.time + self.args.pulse_duration
                print(f"末端施力 {self.pulse_force.tolist()} N，持续 {self.args.pulse_duration} s", flush=True)

    def step(self):
        data, model = self.data, self.model
        mujoco.mj_forward(model, data)
        if data.time + 1e-9 >= self.next_control:
            self.feedforward = self.gravity.torque(data.qpos) * self.args.gravity_scale if self.compensation else np.zeros(6)
            self.next_control = data.time + CONTROL_PERIOD
        # 重力前馈每 10 ms 更新；速度阻尼每个物理步执行，模拟电机内部反馈。
        data.ctrl[:] = self.feedforward - self.args.damping * data.qvel
        self.input_force[:] = 0
        if self.args.force_start <= data.time < self.args.force_start + self.args.force_duration:
            self.input_force += self.args.force
        if data.time < self.pulse_end:
            self.input_force += self.pulse_force
        data.qfrc_applied[:] = 0
        mujoco.mj_applyFT(model, data, self.input_force, np.zeros(3), data.site_xpos[self.site],
                         self.body, data.qfrc_applied)
        # 鼠标扰动由 viewer.sync 写入 xfrc_applied，保留它并投影到关节坐标供记录。
        self.external[:] = data.qfrc_applied
        for body in range(1, model.nbody):
            wrench = data.xfrc_applied[body]
            if np.any(wrench):
                mujoco.mj_applyFT(model, data, wrench[:3], wrench[3:], data.xipos[body], body, self.external)
        if self.writer and data.time + 1e-9 >= self.next_log:
            self.writer.writerow([data.time, *data.qpos, *data.qvel, *self.feedforward,
                                  *data.ctrl, *self.external, *self.input_force])
            self.next_log = data.time + CONTROL_PERIOD
        mujoco.mj_step(model, data)
        if not np.all(np.isfinite(data.qpos)) or not np.all(np.isfinite(data.qvel)):
            raise RuntimeError("仿真状态出现非有限值")

    def close(self):
        if self.csv_file:
            self.csv_file.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true", help="不打开图形窗口，按仿真时间运行")
    parser.add_argument("--duration", type=float, default=0.0, help="运行秒数；GUI 的 0 表示持续运行")
    parser.add_argument("--initial", type=lambda s: vector(s, 6), default=vector("0,-0.8,1.1,0,0,0", 6), help="六轴 URDF 角度，rad")
    parser.add_argument("--damping", type=lambda s: vector(s, 6), default=vector("0,0.3,0.3,0,0,0", 6))
    parser.add_argument("--gravity-scale", type=float, default=1.0)
    parser.add_argument("--force", type=lambda s: vector(s, 3), default=np.zeros(3), help="末端世界坐标外力，N")
    parser.add_argument("--force-start", type=float, default=1.0)
    parser.add_argument("--force-duration", type=float, default=0.3)
    parser.add_argument("--hand-force", type=float, default=0.5, help="快捷键施加的力，N")
    parser.add_argument("--pulse-duration", type=float, default=0.3)
    parser.add_argument("--csv", default="", help="保存 100 Hz 仿真数据；拒绝覆盖同名文件")
    args = parser.parse_args()
    values = [args.duration, args.gravity_scale, args.force_start, args.force_duration, args.hand_force, args.pulse_duration]
    if not all(np.isfinite(values)) or min(values) < 0 or np.any(args.damping < 0):
        parser.error("时间、比例、力大小和阻尼需要非负有限值")
    if args.headless and args.duration <= 0:
        parser.error("无窗口运行需要 --duration 大于 0")
    sim = ControllerSimulation(args)
    print("控制器物理仿真：六轴无角度限位；接触碰撞关闭；不连接真实硬件。", flush=True)
    print("力矩及角度均使用 URDF 关节坐标；理想电机模型尚未模拟电机电流环、摩擦或 CAN 延迟。", flush=True)
    print("按键：U/J=世界±Z施力，I/K=世界±Y施力，G=切换重力补偿，R=重置，P=暂停。", flush=True)
    print("鼠标：双击选择 Link，Ctrl+右键拖动施力，Ctrl+左键拖动施加力矩。", flush=True)
    render_threads = []
    try:
        if args.headless:
            while sim.data.time < args.duration:
                sim.step()
        else:
            import mujoco.viewer
            existing_threads = set(threading.enumerate())
            with mujoco.viewer.launch_passive(sim.model, sim.data, key_callback=sim.commands.put) as viewer:
                render_threads = [thread for thread in threading.enumerate()
                                  if thread not in existing_threads and "_launch_internal" in thread.name]
                with viewer.lock():
                    viewer.cam.lookat[:] = [0, 0, 0.15]
                    viewer.cam.distance = 1.3
                    viewer.cam.azimuth = 135
                    viewer.cam.elevation = -20
                    viewer.opt.geomgroup[0] = 1
                next_status = 0.0
                while viewer.is_running() and (args.duration == 0 or sim.data.time < args.duration):
                    start = time.monotonic()
                    with viewer.lock():
                        sim.handle_keys()
                        if not sim.paused:
                            for _ in range(10):
                                sim.step()
                    viewer.sync()
                    if time.monotonic() >= next_status:
                        print(f"t={sim.data.time:.2f}s q={np.round(sim.data.qpos, 3)} "
                              f"g={np.round(sim.feedforward, 3)} ext={np.round(sim.external, 3)}", flush=True)
                        next_status = time.monotonic() + 1
                    time.sleep(max(0.0, CONTROL_PERIOD - (time.monotonic() - start)))
        print(f"结束：t={sim.data.time:.3f}s q={sim.data.qpos.tolist()}", flush=True)
    except KeyboardInterrupt:
        print("仿真已停止", flush=True)
    finally:
        # passive viewer 的关闭请求是异步的，退出 Python 前等待渲染线程释放 OpenGL。
        for thread in render_threads:
            thread.join(timeout=5)
            if thread.is_alive():
                raise RuntimeError("MuJoCo 渲染线程未完成关闭")
        sim.close()


if __name__ == "__main__":
    main()
