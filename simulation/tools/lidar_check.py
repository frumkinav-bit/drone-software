#!/usr/bin/env python3
"""
Проверка данных лидара в ROS 2 (тема /lidar/scan, sensor_msgs/LaserScan).

Подписывается на скан, раз в секунду печатает: сколько лучей с данными, сколько
«ближе минимума» и «дальше максимума», ближайшее препятствие и его направление,
а также сектора, где лидар закрыт деталями самого дрона (мачта GPS спереди).

    source ~/simulation/tools/ros_env.sh
    python3 ~/simulation/tools/lidar_check.py            # 10 секунд
    python3 ~/simulation/tools/lidar_check.py --seconds 30

Углы: 0° — вперёд (+X дрона), против часовой стрелки положительно (как в ROS).
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def sectors(flags, angles):
    """Непрерывные участки, где flags == True: [(угол_от, угол_до), ...] в градусах."""
    out, start = [], None
    n = len(flags)
    for i in range(n + 1):
        f = flags[i % n] if i < n else False
        if f and start is None:
            start = i
        if not f and start is not None:
            out.append((math.degrees(angles[start]), math.degrees(angles[i - 1])))
            start = None
    # склеить участок через ±180°
    if len(out) > 1 and flags[0] and flags[-1]:
        first, last = out.pop(0), out.pop()
        out.append((last[0], first[1] + 360))
    return out


class LidarCheck(Node):
    def __init__(self, topic):
        super().__init__("lidar_check")
        self.sub = self.create_subscription(LaserScan, topic, self.on_scan, qos_profile_sensor_data)
        self.count = 0
        self.last_print = 0.0
        self.close_sectors = None

    def on_scan(self, m):
        self.count += 1
        now = time.time()
        if now - self.last_print < 1.0:
            return
        self.last_print = now
        n = len(m.ranges)
        angles = [m.angle_min + i * m.angle_increment for i in range(n)]
        valid = [m.range_min <= r <= m.range_max and math.isfinite(r) for r in m.ranges]
        too_close = [(r < m.range_min) or (r == -math.inf) for r in m.ranges]
        too_far = [(r > m.range_max) or (r == math.inf) for r in m.ranges]
        txt = (f"лучей {n}: с данными {sum(valid)}, ближе {m.range_min:.2f} м — {sum(too_close)}, "
               f"дальше {m.range_max:.0f} м — {sum(too_far)}")
        if any(valid):
            i = min((i for i in range(n) if valid[i]), key=lambda i: m.ranges[i])
            txt += f"; ближайшее {m.ranges[i]:.2f} м под углом {math.degrees(angles[i]):+.0f}°"
        print(txt, flush=True)
        self.close_sectors = sectors(too_close, angles)
        self.info = (n, m.range_min, m.range_max, math.degrees(m.angle_increment), m.header.frame_id)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/lidar/scan")
    ap.add_argument("--seconds", type=float, default=10.0)
    args = ap.parse_args()
    rclpy.init()
    node = LidarCheck(args.topic)
    print(f"Жду {args.topic} ...", flush=True)
    t0 = time.time()
    try:
        while rclpy.ok() and time.time() - t0 < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    dt = time.time() - t0
    print()
    if node.count == 0:
        print(f"Сканов нет. Запущены ли Gazebo (▶) и ros_run.sh?")
    else:
        n, rmin, rmax, step, frame = node.info
        print(f"Итого: {node.count} сканов за {dt:.0f} с ({node.count / dt:.1f} Гц), "
              f"{n} лучей с шагом {step:.1f}°, дальность {rmin:.2f}–{rmax:.0f} м, frame_id '{frame}'")
        if node.close_sectors:
            print("Сектора «ближе минимума» (закрыты деталями дрона):")
            for a, b in node.close_sectors:
                print(f"  от {a:+.0f}° до {b:+.0f}° (ширина {b - a + step:.0f}°)")
        else:
            print("Секторов, закрытых деталями дрона, нет.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
