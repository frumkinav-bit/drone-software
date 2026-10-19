"""Узел telemetry_logger: записывает телеметрию из /uav/telemetry в файл CSV.

Формат файла описан в docs/telemetry_format.md.
"""
import csv
import os
from datetime import datetime, timezone

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from uav_interfaces.msg import Telemetry

# Заголовки колонок CSV, порядок как в docs/telemetry_format.md
COLUMNS = [
    'stamp_iso', 'uav_id', 'latitude', 'longitude', 'altitude_m', 'speed_mps',
    'roll_deg', 'pitch_deg', 'yaw_deg', 'battery_voltage_v', 'battery_percent',
    'link_state', 'flight_mode', 'mission_status', 'alerts',
]


def stamp_to_iso(stamp):
    """Время ROS (секунды + наносекунды) -> строка ISO 8601 в UTC, например 2026-10-18T12:00:01.500Z."""
    seconds = stamp.sec + stamp.nanosec * 1e-9
    moment = datetime.fromtimestamp(seconds, tz=timezone.utc)
    return moment.isoformat(timespec='milliseconds').replace('+00:00', 'Z')


class TelemetryLogger(Node):
    def __init__(self):
        super().__init__('telemetry_logger')

        self.declare_parameter('log_dir', 'telemetry_logs')  # папка для журналов
        log_dir = self.get_parameter('log_dir').get_parameter_value().string_value
        os.makedirs(log_dir, exist_ok=True)  # создаём папку, если её нет

        # Имя файла содержит время запуска, чтобы разные запуски не перезаписывали друг друга
        name = datetime.now(timezone.utc).strftime('telemetry_%Y%m%d_%H%M%S.csv')
        self.path = os.path.join(log_dir, name)
        self.file = open(self.path, 'w', newline='', encoding='utf-8')
        self.writer = csv.writer(self.file)
        self.writer.writerow(COLUMNS)  # первая строка — заголовки
        self.file.flush()              # сразу сбрасываем на диск
        self.count = 0                 # сколько строк записано

        self.subscription = self.create_subscription(
            Telemetry, '/uav/telemetry', self.on_telemetry, 10)
        self.get_logger().info(f'Журнал телеметрии: {self.path}')

    def on_telemetry(self, msg):
        self.writer.writerow([
            stamp_to_iso(msg.header.stamp),
            msg.uav_id,
            f'{msg.latitude:.6f}',            # широта и долгота — 6 знаков после точки
            f'{msg.longitude:.6f}',
            f'{msg.altitude_m:.2f}',          # остальные числа — 2 знака
            f'{msg.speed_mps:.2f}',
            f'{msg.roll_deg:.2f}',
            f'{msg.pitch_deg:.2f}',
            f'{msg.yaw_deg:.2f}',
            f'{msg.battery_voltage_v:.2f}',
            msg.battery_percent,
            msg.link_state,
            msg.flight_mode,
            msg.mission_status,
            ';'.join(msg.alerts),             # список аварий -> одна ячейка через «;»
        ])
        self.file.flush()  # сбрасываем на диск: при аварийной остановке данные не потеряются
        self.count += 1

    def close(self):
        self.file.close()
        self.get_logger().info(f'Журнал закрыт, записано строк: {self.count}')


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryLogger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
