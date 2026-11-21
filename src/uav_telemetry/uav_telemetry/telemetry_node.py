"""Узел telemetry_node: превращает сырое состояние (RawState) в телеметрию (Telemetry).

Подписан на /uav/raw_state, публикует в /uav/telemetry.
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from uav_interfaces.msg import LidarStatus, RawState, Telemetry
from uav_telemetry.alerts_logic import merge_alerts


class TelemetryNode(Node):
    def __init__(self):
        super().__init__('telemetry_node')

        self.declare_parameter('uav_id', 'uav_01')  # идентификатор БПЛА
        self.uav_id = self.get_parameter('uav_id').get_parameter_value().string_value
        # Пока заглушка: позже статус миссии будет приходить от mission_manager
        self.mission_status = 'IDLE'
        self.lidar_alerts = []        # коды тревоги из последнего статуса лидара
        self.lidar_time = None        # когда этот статус пришёл (секунды по часам узла)

        self.publisher = self.create_publisher(Telemetry, '/uav/telemetry', 10)
        # Подписка: тип сообщения, тема, функция, которая вызовется на каждое сообщение
        self.subscription = self.create_subscription(
            RawState, '/uav/raw_state', self.on_raw_state, 10)
        # Статус лидара от lidar_processor: его коды (OBSTACLE_NEAR и др.) добавляем в alerts
        self.create_subscription(LidarStatus, '/uav/lidar_status', self.on_lidar_status, 10)
        self.get_logger().info(f'telemetry_node запущен, uav_id={self.uav_id}')

    def now_s(self):
        # Текущее время узла в секундах (учитывает use_sim_time)
        return self.get_clock().now().nanoseconds * 1e-9

    def on_lidar_status(self, status):
        self.lidar_alerts = list(status.alerts)
        self.lidar_time = self.now_s()

    def on_raw_state(self, raw):
        msg = Telemetry()
        msg.header = raw.header                 # время берём из исходного сообщения
        msg.uav_id = self.uav_id
        msg.latitude = raw.latitude
        msg.longitude = raw.longitude
        msg.altitude_m = raw.altitude_m
        msg.speed_mps = raw.speed_mps
        msg.roll_deg = raw.roll_deg
        msg.pitch_deg = raw.pitch_deg
        msg.yaw_deg = raw.yaw_deg
        msg.battery_voltage_v = raw.battery_voltage_v
        msg.battery_percent = raw.battery_percent
        msg.link_state = raw.link_state
        msg.flight_mode = raw.flight_mode
        msg.mission_status = self.mission_status
        # Коды автопилота + коды лидара (если статус лидара свежий)
        age = None if self.lidar_time is None else self.now_s() - self.lidar_time
        msg.alerts = merge_alerts(list(raw.alerts), self.lidar_alerts, age)
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryNode()
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
