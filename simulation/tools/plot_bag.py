#!/usr/bin/env python3
"""
Картинка по записи rosbag2 (tools/record.sh): телеметрия PX4 и лидар вместе.

    source ~/simulation/tools/ros_env.sh
    python3 ~/simulation/tools/plot_bag.py              # последняя запись в ~/simulation/bags
    python3 ~/simulation/tools/plot_bag.py путь/к/записи

Сохраняет docs/bag_<имя записи>.png:
  - вид сверху: трек дрона по телеметрии, точки лидара в трёх моментах полёта,
    препятствия из мира test_field;
  - высота по телеметрии и число лучей лидара с данными по времени;
и печатает сводку: длительность, число сообщений и частоты основных тем.
"""
import glob
import math
import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError:
    sys.exit("Нет ROS 2. Сначала: source ~/simulation/tools/ros_env.sh")

SIM = Path(__file__).resolve().parent.parent
BAGS = SIM / "bags"
WORLD = SIM / "worlds" / "test_field.sdf"


def latest_bag():
    bags = [p for p in glob.glob(str(BAGS / "flight_*")) if os.path.isdir(p)]
    if not bags:
        sys.exit(f"Записей нет в {BAGS}. Сначала: bash ~/simulation/tools/record.sh")
    return max(bags, key=os.path.getmtime)


def read_bag(path, wanted):
    """{тема: [(время_с, сообщение), ...]} для тем из wanted (по началу имени)."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id="mcap"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    topics = [t for t in types if any(t.startswith(w) for w in wanted)]
    classes = {t: get_message(types[t]) for t in topics}
    out = {t: [] for t in topics}
    counts = {}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        counts[topic] = counts.get(topic, 0) + 1
        if topic in classes:
            out[topic].append((t_ns * 1e-9, deserialize_message(data, classes[topic])))
    return out, counts, types


def world_obstacles():
    """Препятствия мира с коллизией: [(имя, вид, восток, север, размер_x, размер_y, yaw)]."""
    obs = []
    try:
        root = ET.parse(WORLD).getroot()
    except (OSError, ET.ParseError):
        return obs
    for model in root.iter("model"):
        name = model.get("name", "")
        if name in ("ground_plane", "landing_pad", "project_quad"):
            continue
        pose = model.find("pose")
        coll = model.find("link/collision/geometry")
        if pose is None or coll is None:
            continue
        x, y, _, _, _, yaw = (list(map(float, pose.text.split())) + [0] * 6)[:6]
        box = coll.find("box/size")
        cyl = coll.find("cylinder/radius")
        if box is not None:
            sx, sy, _ = map(float, box.text.split())
            obs.append((name, "box", x, y, sx, sy, yaw))
        elif cyl is not None:
            r = float(cyl.text)
            obs.append((name, "cyl", x, y, 2 * r, 2 * r, 0.0))
    return obs


def scan_to_world(scan, north, east, heading):
    """Точки скана (лидар, оси FLU) -> локальные координаты PX4 (север, восток)."""
    r = np.array(scan.ranges, dtype=float)
    a = scan.angle_min + np.arange(len(r)) * scan.angle_increment
    ok = np.isfinite(r) & (r >= scan.range_min) & (r <= scan.range_max)
    xb, yb = r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])   # вперёд, влево
    c, s = math.cos(heading), math.sin(heading)              # heading: от севера по часовой
    pn = north + xb * c + yb * s
    pe = east + xb * s - yb * c
    return pn, pe


def nearest(t_arr, t):
    return int(np.clip(np.searchsorted(t_arr, t), 0, len(t_arr) - 1))


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else latest_bag()
    data, counts, types = read_bag(path, ["/fmu/out/vehicle_local_position", "/lidar/scan"])
    lp_topic = next((t for t in data if t.startswith("/fmu/out/vehicle_local_position")), None)
    if lp_topic is None or not data[lp_topic]:
        sys.exit("В записи нет /fmu/out/vehicle_local_position*.")
    scans = data.get("/lidar/scan", [])
    if not scans:
        sys.exit("В записи нет /lidar/scan.")

    t_all = [t for v in data.values() for t, _ in v]
    t0 = min(t_all)
    lp = data[lp_topic]
    t_lp = np.array([t - t0 for t, _ in lp])
    n = np.array([m.x for _, m in lp])
    e = np.array([m.y for _, m in lp])
    z = np.array([m.z for _, m in lp])
    hd = np.array([m.heading for _, m in lp])
    alt = -(z - z[0])
    t_sc = np.array([t - t0 for t, _ in scans])
    valid = np.array([sum(1 for r in m.ranges if math.isfinite(r) and m.range_min <= r <= m.range_max)
                      for _, m in scans])

    # три момента: старт, наибольшая высота, середина между ними и концом
    i_top = int(np.argmax(alt))
    moments = [("на земле", t_lp[0] + 1.0), ("в полёте", t_lp[i_top]),
               ("перед посадкой", t_lp[-1] - 5.0)]
    colors = ["#2ca02c", "#d62728", "#9467bd"]

    fig = plt.figure(figsize=(13, 7.5))
    gs = fig.add_gridspec(2, 2, width_ratios=[1.25, 1])
    ax = fig.add_subplot(gs[:, 0])
    for name, kind, x, y, sx, sy, yaw in world_obstacles():
        # мир Gazebo: X — восток, Y — север
        if kind == "cyl":
            ax.add_patch(plt.Circle((x, y), sx / 2, fc="#bbbbbb", ec="#666", lw=0.8))
        else:
            rect = plt.Rectangle((x - sx / 2, y - sy / 2), sx, sy, fc="#bbbbbb", ec="#666", lw=0.8)
            rect.set_transform(matplotlib.transforms.Affine2D().rotate_around(x, y, yaw) + ax.transData)
            ax.add_patch(rect)
    ax.plot(e, n, color="#1f77b4", lw=1.5, label="трек дрона (телеметрия PX4)")
    for (label, tm), col in zip(moments, colors):
        k = nearest(t_sc, tm)
        j = nearest(t_lp, t_sc[k])
        pn, pe = scan_to_world(scans[k][1], n[j], e[j], hd[j])
        ax.scatter(pe, pn, s=6, color=col, label=f"лидар {label} (t = {t_sc[k]:.0f} с, {len(pn)} точек)")
        ax.plot(e[j], n[j], "^", color=col, ms=10, mec="k")
    ax.set_aspect("equal")
    ax.set_xlabel("восток, м")
    ax.set_ylabel("север, м")
    ax.grid(alpha=0.3)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.08), fontsize=8, ncol=2)
    ax.set_title("Вид сверху: серое — препятствия мира test_field")

    a1 = fig.add_subplot(gs[0, 1])
    a1.plot(t_lp, alt, color="#1f77b4")
    a1.set_ylabel("высота, м (телеметрия)")
    a1.grid(alpha=0.3)
    for (_, tm), col in zip(moments, colors):
        a1.axvline(tm, color=col, ls=":")
    a2 = fig.add_subplot(gs[1, 1], sharex=a1)
    a2.plot(t_sc, valid, color="#ff7f0e")
    a2.set_ylabel("лучей лидара с данными")
    a2.set_xlabel("время от начала записи, с")
    a2.grid(alpha=0.3)
    name = Path(path).name
    fig.suptitle(f"Запись rosbag2 {name}: телеметрия PX4 и лидар", fontsize=13)
    fig.tight_layout()
    out = SIM / "docs" / f"bag_{name}.png"
    fig.savefig(out, dpi=110)

    dur = max(t_all) - t0
    print(f"Запись:  {path}")
    print(f"График:  {out}")
    print(f"Длительность {dur:.1f} с, наибольшая высота {alt.max():.1f} м")
    print("Сообщений по темам (частота):")
    for t in sorted(counts):
        if counts[t]:
            print(f"  {t:<46} {counts[t]:7d}  ({counts[t] / dur:6.1f} Гц)")


if __name__ == "__main__":
    main()
