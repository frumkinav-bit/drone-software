"""Узел diagnostics_node: следит за телеметрией и сообщает о проблемах.

Подписан на /uav/telemetry, публикует состояние в /diagnostics
и пишет понятные сообщения в терминал, когда проблема появляется или исчезает.
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus  # стандартные сообщения диагностики ROS
from uav_interfaces.msg import Telemetry

from uav_telemetry.diagnostics_logic import ERROR, OK, WARN, evaluate, worst_level

# Перевод наших уровней в уровни стандартного сообщения ROS
DIAG_LEVELS = {
    OK: DiagnosticStatus.OK,
    WARN: DiagnosticStatus.WARN,
    ERROR: DiagnosticStatus.ERROR,
}


class DiagnosticsNode(Node):
    def __init__(self):
        super().__init__('diagnostics_node')

        self.active = {}  # проблемы, которые уже были замечены: {код: уровень}

        self.publisher = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.subscription = self.create_subscription(
            Telemetry, '/uav/telemetry', self.on_telemetry, 10)
        self.get_logger().info('diagnostics_node запущен')

    def on_telemetry(self, msg):
        # Проверяем текущие данные по правилам из diagnostics_logic.py
        problems = evaluate(msg.battery_percent, msg.link_state, msg.flight_mode, list(msg.alerts))

        # Сообщаем, когда проблема появилась или стала серьёзнее (было WARN, стало ERROR),
        # а не на каждое сообщение: иначе терминал заполнится одинаковыми строками
        for code, (level, text) in problems.items():
            if code not in self.active or level > self.active[code]:
                if level == ERROR:
                    self.get_logger().error(f'[{msg.uav_id}] {text}')
                else:
                    self.get_logger().warning(f'[{msg.uav_id}] {text}')

        # Проблемы, которые исчезли: сообщаем об этом
        for code in list(self.active):
            if code not in problems:
                self.get_logger().info(f'[{msg.uav_id}] Проблема устранена: {code}')

        # Запоминаем актуальный набор проблем
        self.active = {code: level for code, (level, _text) in problems.items()}

        # Публикуем итоговое состояние в стандартную тему /diagnostics
        status = DiagnosticStatus()
        status.level = DIAG_LEVELS[worst_level(problems)]
        status.name = f'{msg.uav_id}/telemetry'
        status.hardware_id = msg.uav_id
        status.message = '; '.join(text for _level, text in problems.values()) or 'Всё в норме'
        array = DiagnosticArray()
        array.header = msg.header
        array.status = [status]
        self.publisher.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticsNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
