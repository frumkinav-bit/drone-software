"""Узел lidar_scan_player: публикует искусственный скан лидара в /lidar/scan для демо без симулятора.

Сценарий описан в lidar_logic.synthetic_ranges: стена спереди приближается и уходит,
а после stop_after_s секунд скан перестаёт приходить (проверка LIDAR_LOST).
"""
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan

from uav_lidar.lidar_logic import laserscan_order, synthetic_ranges


class LidarScanPlayer(Node):
    def __init__(self):
        super().__init__('lidar_scan_player')
        self.declare_parameter('rate_hz', 8.0)           # как у лидара в симуляции
        self.declare_parameter('stop_after_s', 25.0)     # через сколько секунд «лидар отключается»
        rate = self.get_parameter('rate_hz').get_parameter_value().double_value
        self.stop_after = self.get_parameter('stop_after_s').get_parameter_value().double_value
        self.start = self.get_clock().now()
        self.publisher = self.create_publisher(LaserScan, '/lidar/scan', qos_profile_sensor_data)
        self.create_timer(1.0 / rate, self.tick)
        self.stopped = False
        self.get_logger().info(f'lidar_scan_player: искусственный скан {rate} Гц, остановка через {self.stop_after} с')

    def tick(self):
        t = (self.get_clock().now() - self.start).nanoseconds * 1e-9
        if t >= self.stop_after:
            if not self.stopped:
                self.stopped = True
                self.get_logger().info('Скан остановлен (имитация потери лидара)')
            return
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'lidar'
        msg.angle_min = -math.pi                  # как в симуляции: 360 лучей с шагом 1°
        msg.angle_max = math.pi
        msg.angle_increment = math.radians(1.0)
        msg.range_min = 0.15
        msg.range_max = 12.0
        # synthetic_ranges нумерует лучи от 0° (вперёд), а в LaserScan первый луч стоит на -180°
        msg.ranges = [float(r) for r in laserscan_order(synthetic_ranges(t))]
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LidarScanPlayer()
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
