"""Узел test_data_player: воспроизводит тестовые данные из CSV как будто это автопилот.

Публикует сообщения RawState в тему /uav/raw_state.
"""
import rclpy  # главная библиотека ROS 2 для Python
from rclpy.executors import ExternalShutdownException  # исключение при остановке через launch
from rclpy.node import Node  # базовый класс для всех узлов

from uav_interfaces.msg import RawState  # наше сообщение из пакета uav_interfaces
from uav_autopilot.csv_reader import read_rows


class TestDataPlayer(Node):
    def __init__(self):
        super().__init__('test_data_player')  # имя узла в системе ROS

        # Параметры узла: значения по умолчанию, их можно менять при запуске
        self.declare_parameter('csv_path', 'data/test/sample_telemetry.csv')
        self.declare_parameter('rate_hz', 1.0)  # сколько строк в секунду отправлять
        self.declare_parameter('loop', False)   # повторять ли данные по кругу

        csv_path = self.get_parameter('csv_path').get_parameter_value().string_value
        rate_hz = self.get_parameter('rate_hz').get_parameter_value().double_value
        self.loop = self.get_parameter('loop').get_parameter_value().bool_value

        self.rows = read_rows(csv_path)  # читаем весь файл сразу; при ошибке узел не запустится
        self.index = 0                   # номер строки, которую отправим следующей

        # Издатель: тип сообщения, имя темы, длина очереди
        self.publisher = self.create_publisher(RawState, '/uav/raw_state', 10)
        # Таймер вызывает publish_next с заданной частотой
        self.timer = self.create_timer(1.0 / rate_hz, self.publish_next)
        self.get_logger().info(f'Загружено строк: {len(self.rows)} из {csv_path}')

    def publish_next(self):
        if self.index >= len(self.rows):  # данные закончились
            if self.loop:
                self.index = 0            # начинаем сначала
            else:
                self.get_logger().info('Данные закончились, воспроизведение остановлено')
                self.timer.cancel()       # выключаем таймер
                return

        row = self.rows[self.index]
        msg = RawState()
        msg.header.stamp = self.get_clock().now().to_msg()  # метка «сейчас»
        msg.latitude = row['latitude']
        msg.longitude = row['longitude']
        msg.altitude_m = row['altitude_m']
        msg.speed_mps = row['speed_mps']
        msg.roll_deg = row['roll_deg']
        msg.pitch_deg = row['pitch_deg']
        msg.yaw_deg = row['yaw_deg']
        msg.battery_voltage_v = row['battery_voltage_v']
        msg.battery_percent = row['battery_percent']
        msg.link_state = row['link_state']
        msg.flight_mode = row['flight_mode']
        msg.alerts = row['alerts']
        self.publisher.publish(msg)  # отправляем сообщение всем подписчикам
        self.index += 1


def main(args=None):
    rclpy.init(args=args)       # запуск ROS 2 в этой программе
    node = TestDataPlayer()
    try:
        rclpy.spin(node)        # узел работает, пока его не остановят
    except (KeyboardInterrupt, ExternalShutdownException):
        pass                    # остановка по Ctrl+C — это нормально
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
