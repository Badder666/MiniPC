"""Publish standalone 3D workspace artifacts from offline computation results."""

import argparse
import json
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
import plotly.graph_objects as go
from scipy.spatial.transform import Rotation


def pose_preview(fig, urdf):
    if urdf is None:
        return ""
    root = ET.parse(urdf).getroot()
    joints = []
    for joint in root.findall("joint"):
        if joint.attrib["type"] == "fixed":
            continue
        origin = joint.find("origin")
        matrix = np.eye(4)
        matrix[:3, :3] = Rotation.from_euler("xyz", np.fromstring(origin.attrib["rpy"], sep=" ")).as_matrix()
        matrix[:3, 3] = np.fromstring(origin.attrib["xyz"], sep=" ")
        joints.append({"origin": matrix.tolist(), "axis": np.fromstring(joint.find("axis").attrib["xyz"], sep=" ").tolist()})
    payload = {"joints": joints, "meshes": [], "trace_ids": []}
    for index, name in enumerate(["base_link"] + [f"Link{i}" for i in range(1, 7)]):
        with (urdf.parent.parent / "meshes" / (name + ".STL")).open("rb") as stream:
            stream.seek(80)
            count = struct.unpack("<I", stream.read(4))[0]
            dtype = np.dtype([("normal", "<f4", (3,)), ("v", "<f4", (3, 3)), ("attribute", "<u2")])
            vertices = np.fromfile(stream, dtype=dtype, count=count)["v"].reshape(-1, 3)
        triangles = surface_subset(vertices, 2600, index + 10)
        payload["meshes"].append(np.round(triangles.reshape(-1, 3), 6).tolist())
        label = "选中姿态整臂（基座/YAW 为金色）" if index == 0 else name
        trace = mesh_trace(triangles, label, "#e3b34d" if index < 2 else "#a4daf4", .85, "legendonly")
        trace.legendgroup = "selected_pose"
        trace.showlegend = index == 0
        payload["trace_ids"].append(len(fig.data))
        fig.add_trace(trace)
    serialized = json.dumps(payload, separators=(",", ":"))
    return """
const robot = ROBOT_PAYLOAD;
const plot = document.getElementById('workspace-plot');
const identity = () => [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]];
function multiply(a,b) {
  const c = Array.from({length:4},()=>[0,0,0,0]);
  for(let i=0;i<4;i++) for(let j=0;j<4;j++) for(let k=0;k<4;k++) c[i][j]+=a[i][k]*b[k][j];
  return c;
}
function rotation(axis,q) {
  const [x,y,z]=axis, c=Math.cos(q), s=Math.sin(q), d=1-c;
  return [[c+x*x*d,x*y*d-z*s,x*z*d+y*s,0],
          [y*x*d+z*s,c+y*y*d,y*z*d-x*s,0],
          [z*x*d-y*s,z*y*d+x*s,c+z*z*d,0],[0,0,0,1]];
}
plot.on('plotly_click', function(event) {
  const degrees=Array.from(event.points[0].customdata||[]);
  if(degrees.length<6) return;
  const q=degrees.slice(0,6).map(x=>x*Math.PI/180), poses=[identity()];
  for(let i=0;i<6;i++) poses.push(multiply(multiply(poses[i],robot.joints[i].origin),rotation(robot.joints[i].axis,q[i])));
  const xs=[],ys=[],zs=[];
  for(let i=0;i<7;i++) {
    const t=poses[i], vx=[],vy=[],vz=[];
    for(const p of robot.meshes[i]) {
      vx.push(t[0][0]*p[0]+t[0][1]*p[1]+t[0][2]*p[2]+t[0][3]);
      vy.push(t[1][0]*p[0]+t[1][1]*p[1]+t[1][2]*p[2]+t[1][3]);
      vz.push(t[2][0]*p[0]+t[2][1]*p[1]+t[2][2]*p[2]+t[2][3]);
    }
    xs.push(vx);ys.push(vy);zs.push(vz);
  }
  Plotly.restyle(plot,{x:xs,y:ys,z:zs,visible:true},robot.trace_ids);
  Plotly.restyle(plot,{visible:false},[1]);
  document.getElementById('selection-info').textContent='选中姿态：'+degrees.slice(0,6).map((x,i)=>'J'+(i+1)+'='+x.toFixed(1)+'°').join('，')+'；浅蓝色为检查车体碰撞的 Link2～Link6，金色为基座/YAW 安装部分；使用 URDF 几何。';
});
""".replace("ROBOT_PAYLOAD", serialized)


def surface_subset(vertices, count, seed):
    triangles = vertices.reshape(-1, 3, 3)
    if len(triangles) <= count:
        return triangles
    area = np.linalg.norm(np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0]), axis=1)
    rng = np.random.default_rng(seed)
    score = np.maximum(area, 1e-18) / rng.exponential(size=len(area))
    selected = np.argpartition(score, -count)[-count:]
    return triangles[selected]


def mesh_trace(triangles, name, color, opacity=1., visible=True):
    vertices = triangles.reshape(-1, 3)
    faces = np.arange(len(vertices)).reshape(-1, 3)
    return go.Mesh3d(x=vertices[:, 0], y=vertices[:, 1], z=vertices[:, 2],
                     i=faces[:, 0], j=faces[:, 1], k=faces[:, 2], name=name,
                     color=color, opacity=opacity, flatshading=True, showscale=False,
                     hoverinfo="name", visible=visible)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--scene-only", action="store_true")
    parser.add_argument("--urdf", type=Path)
    args = parser.parse_args()
    cfg = json.loads((args.scene / "scene_config.json").read_text(encoding="utf-8"))
    meshes = np.load(args.scene / "scene_meshes.npz")
    environment = meshes["environment"]
    carriers = []
    if "driving_zero" in cfg:
        body, start = [], 0
        for component in cfg["obstacles"]:
            end = start + component["triangles"] * 3
            if "坦克拖链" in component["name"]:
                carriers.append((component["name"].rsplit("-", 1)[-1], environment[start:end]))
            else:
                body.append(environment[start:end])
            start = end
        chassis = surface_subset(np.concatenate(body), 45000, 1)
    else:
        chassis = surface_subset(environment, 45000, 1)
    arm = surface_subset(meshes["cad_arm"], 22000, 2)
    fig = go.Figure()
    fig.add_trace(mesh_trace(chassis, "车体：四轮正常行驶零位" if carriers else "车体 CAD 当前机构状态", "#8293a5", .85))
    fig.add_trace(mesh_trace(arm, "机械臂 CAD 当前装配姿态", "#d19145", .95))
    for number, vertices in sorted(carriers):
        fig.add_trace(mesh_trace(vertices.reshape(-1, 3, 3), "拖链 " + number, "#b854bb", .95))
    xyz = np.empty((0, 3))
    connected = np.empty((0, 3))
    summary = None
    if not args.scene_only:
        summary = json.loads((args.scene / "workspace_summary.json").read_text(encoding="utf-8"))
        voxel_file = args.scene / "workspace_voxels.npz"
        samples = np.load(voxel_file if voxel_file.exists() else args.scene / "free_configurations.npz")
        xyz, q = samples["tcp"], np.rad2deg(samples["q"])
        if "external_clearance_m" in samples:
            hover_data = np.column_stack([q, samples["external_clearance_m"] * 1000])
            distance_hover = "<br>车体/地面最小间隙=%{customdata[6]:.1f} mm"
            if "chassis_clearance_by_link_m" in samples:
                hover_data = np.column_stack([hover_data, samples["chassis_clearance_by_link_m"] * 1000])
                distance_hover += "<br>各连杆到车体：" + " / ".join(
                    f"{name}=%{{customdata[{7+i}]:.1f}} mm"
                    for i, name in enumerate(samples["chassis_clearance_link_names"]))
        else:
            hover_data, distance_hover = q, ""
        fig.add_trace(go.Scatter3d(x=xyz[:, 0], y=xyz[:, 1], z=xyz[:, 2], mode="markers", name="通过姿态碰撞检查的参考端点（每格一姿态）",
                                  marker=dict(size=2, color="#48a9ce", opacity=.33), customdata=hover_data,
                                  hovertemplate="XYZ=(%{x:.3f},%{y:.3f},%{z:.3f}) m<br>J1=%{customdata[0]:.1f}° J2=%{customdata[1]:.1f}° J3=%{customdata[2]:.1f}°<br>J4=%{customdata[3]:.1f}° J5=%{customdata[4]:.1f}° J6=%{customdata[5]:.1f}°" + distance_hover + "<extra>该朝向通过检查；路径未必连通</extra>"))
        road = np.load(args.scene / "seed_connected.npz")
        connected = road["tcp"]
        seed_label = "固件归位" if summary["connected_seed"] == "firmware_home" else "CAD 总装"
        fig.add_trace(go.Scatter3d(x=connected[:, 0], y=connected[:, 1], z=connected[:, 2], mode="markers", name="与" + seed_label + "姿态路径连通的已检查点",
                                  marker=dict(size=3.5, color="#2dbe87", opacity=.85), customdata=np.rad2deg(road["q"]),
                                  hovertemplate="XYZ=(%{x:.3f},%{y:.3f},%{z:.3f}) m<br>J1=%{customdata[0]:.1f}° J2=%{customdata[1]:.1f}° J3=%{customdata[2]:.1f}°<br>J4=%{customdata[3]:.1f}° J5=%{customdata[4]:.1f}° J6=%{customdata[5]:.1f}°<extra>路径经离散检查；以模型假设为前提</extra>"))
        directions = Rotation.from_quat(samples["quaternion_xyzw"]).apply(np.tile([0., 0., 1.], (len(xyz), 1)))
        downward = directions[:, 2] < -np.cos(np.deg2rad(20.))
        fig.add_trace(go.Scatter3d(x=xyz[downward, 0], y=xyz[downward, 1], z=xyz[downward, 2], mode="markers", name="工具 +Z 朝下 ±20° 的姿态子集",
                                  marker=dict(size=3, color="#a47ed9", opacity=.8), visible="legendonly"))
        edges = []
        for index, parent in enumerate(road["parent"]):
            if parent >= 0:
                edges.extend([connected[int(parent)], connected[index], [np.nan, np.nan, np.nan]])
        if edges:
            e = np.asarray(edges)
            fig.add_trace(go.Scatter3d(x=e[:, 0], y=e[:, 1], z=e[:, 2], mode="lines", name="已检查路径树（端点投影示意）",
                                      line=dict(color="#2dbe87", width=2), visible="legendonly", hoverinfo="skip"))
        seed = connected[0]
        fig.add_trace(go.Scatter3d(x=[seed[0]], y=[seed[1]], z=[seed[2]], mode="markers+text", name="计算种子",
                                  text=[seed_label + "种子"], marker=dict(size=7, color="#f4ce70"), textposition="top center"))
    fig.add_trace(go.Scatter3d(x=[0.], y=[0.], z=[0.], mode="markers+text", text=["J1 / base_link"],
                              name="坐标原点", marker=dict(color="#ffffff", size=5)))
    preview_script = pose_preview(fig, args.urdf) if summary else ""
    fig.update_layout(title="机器人上的机械臂工作空间 · 模型预览" if summary else "整车与机械臂 CAD · 安装坐标核对",
                      template="plotly_dark", margin=dict(l=0, r=0, b=0, t=55),
                      scene=dict(aspectmode="data", xaxis_title="base X / m", yaxis_title="base Y / m", zaxis_title="base Z / m",
                                 camera=dict(eye=dict(x=1.6, y=1.8, z=1.1))),
                      legend=dict(x=.01, y=.99, bgcolor="rgba(15,23,42,.7)"), height=840)
    if summary:
        status = (f"六轴采样 {summary['samples']:,} 组；通过 {summary['accepted']:,} 组；"
                  f"20 mm 网格占用 {summary['occupied_voxels']:,} 格；"
                  f"连通树 {summary['seed_connected_nodes']:,} 个姿态。")
    else:
        status = f"车体 {cfg['environment_instances']} 个零件实例；机械臂 {cfg['cad_arm_instances']} 个实例。"
    warning = "首版离线模型结果：旧 URDF 与当前 CAD 的 Link2 和附件存在差异；关节实限、可动杆和线束包络尚未核定。"
    policy = ""
    if summary:
        home_pairs = summary["diagnostics"]["firmware_home"]["colliding_pairs"]
        warning += (" 固件归位姿态未通过间隙检查。" if home_pairs else " 固件归位姿态通过本模型的间隙检查。")
        warning += (" 路径从固件归位姿态开始。" if summary["connected_seed"] == "firmware_home"
                    else " 路径从 CAD 总装姿态开始。")
        warning += (f" 外部间隙 {summary['external_clearance_threshold_m'] * 1000:g} mm，"
                    f"自碰撞间隙 {summary['self_clearance_threshold_m'] * 1000:g} mm。")
        if "collision_policy" in summary:
            policy = ("<p>碰撞规则：基座 base_link 与 J1/YAW（Link1）豁免车体接触；"
                      "Link2～Link6 的整个网格均检查与车体的碰撞，包括模型中的腕部和夹爪。"
                      "仍检查机械臂自身碰撞和地面间隙；路径中间姿态使用同一规则。"
                      "点云显示通过整臂检查的姿态所对应的末端位置，每个点只代表记录的关节姿态。</p>")
    refresh = "<meta http-equiv='refresh' content='30'>" if summary and summary.get("status") == "computing_connectivity" else ""
    phase = "<p>关节姿态采样已完成，路径连通检查仍在运行；页面会自动刷新。</p>" if refresh else ""
    driving_state = ("<p>车体状态：四个轮组均处于用户确认的正常行驶零位；紫色为四条拖链。"
                     "后侧拖链采用随安装端平移的名义形状，实际弯曲形状尚需核对。</p>" if carriers else "")
    header = ("<!doctype html><html lang='zh-CN'><meta charset='utf-8'>" + refresh + "<title>机械臂工作空间</title>"
              "<body style='margin:0;background:#111827;color:#e5e7eb;font:16px sans-serif'>"
              f"<div style='padding:18px 24px'><b>{status}</b>{phase}{driving_state}{policy}<p style='color:#f6c77d'>{warning}</p>"
              "<span>拖动旋转，滚轮缩放；单击图例显隐，双击图例独显。坐标以 J1 基座为原点。端点使用 Link6 +Z 方向 192 mm 的几何参考，尚未标定抓取中心。</span>"
              "<p id='selection-info'>点击采样点可查看对应关节姿态。</p></div>")
    filename = "workspace.html" if summary else "scene_alignment.html"
    (args.scene / filename).write_text(header + fig.to_html(full_html=False, include_plotlyjs=True,
                                                          div_id="workspace-plot", post_script=preview_script) + "</body></html>", encoding="utf-8")
    # A portable static view accompanies the interactive artifact.
    chassis_png = surface_subset(meshes["environment"], 14000, 3)
    arm_png = surface_subset(meshes["cad_arm"], 8000, 4)
    plt.rcParams.update({"figure.facecolor": "#101824", "axes.facecolor": "#101824", "text.color": "#e7edf5",
                         "axes.labelcolor": "#e7edf5", "xtick.color": "#c0cbda", "ytick.color": "#c0cbda"})
    output, axes = plt.subplots(1, 2, figsize=(17, 8), subplot_kw={"projection": "3d"})
    for index, axis in enumerate(axes):
        axis.add_collection3d(Poly3DCollection(chassis_png, facecolors="#8192a4", edgecolors="none", alpha=.7, zsort="average"))
        axis.add_collection3d(Poly3DCollection(arm_png, facecolors="#d19145", edgecolors="none", alpha=.95))
        if len(xyz):
            axis.scatter(*xyz.T, s=1., c="#48a9ce", alpha=.23, depthshade=False, label="Pose-valid samples")
            axis.scatter(*connected.T, s=6., c="#2dbe87", alpha=.8, depthshade=False,
                         label="Connected from home" if summary["connected_seed"] == "firmware_home" else "Connected from CAD seed")
        all_points = np.concatenate([chassis_png.reshape(-1, 3), arm_png.reshape(-1, 3), xyz])
        lower, upper = all_points.min(0), all_points.max(0)
        center = (lower + upper) / 2.
        extent = max(upper - lower) * .53
        axis.set_xlim(center[0] - extent, center[0] + extent)
        axis.set_ylim(center[1] - extent, center[1] + extent)
        axis.set_zlim(center[2] - extent, center[2] + extent)
        axis.set_box_aspect([1, 1, 1])
        axis.set_xlabel("base X / m")
        axis.set_ylabel("base Y / m")
        axis.set_zlabel("base Z / m", color="#e7edf5")
        axis.tick_params(axis="z", colors="#c0cbda")
        for dimension in [axis.xaxis, axis.yaxis, axis.zaxis]:
            dimension.set_pane_color((.07, .10, .15, 1.))
            dimension._axinfo["grid"]["color"] = (.45, .55, .65, .18)
        axis.view_init(elev=24 if index == 0 else 10, azim=42 if index == 0 else -5)
        axis.set_title("Robot and arm" if index == 0 else "Side view", color="#e7edf5")
    title = "Full-arm collision checks: Link2-Link6 vs chassis; base/YAW mount exempt" if summary and "collision_policy" in summary else (
        "PRELIMINARY MODEL: sampled collision-free workspace" if summary else "SolidWorks assembly in URDF base frame")
    output.suptitle(title, fontsize=17)
    output.text(.03, .035, "CAD chassis pose fixed | provisional joint limits / tool reference | URDF Link2 and CAD differ", fontsize=11, color="#f6c77d")
    if summary:
        output.text(.03, .075, f"{summary['samples']:,} joint samples | {summary['accepted']:,} pose-valid | {summary['seed_connected_nodes']:,} checked connected nodes", fontsize=12)
    output.subplots_adjust(left=0., right=1., bottom=.13, top=.90, wspace=0.)
    output.savefig(args.scene / ("workspace.png" if summary else "scene_alignment.png"), dpi=150, facecolor=output.get_facecolor())
    plt.close(output)
    print("Published", args.scene / filename, flush=True)


if __name__ == "__main__":
    main()
