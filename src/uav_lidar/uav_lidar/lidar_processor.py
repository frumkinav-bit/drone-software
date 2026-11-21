"""Узел lidar_processor: обрабатывает скан лидара и сообщает об опасных препятствиях.

Подписан на /lidar/scan (sensor_msgs/LaserScan) и /uav/raw_state (крен и тангаж дрона),
публикует /uav/lidar_status (uav_interfaces/LidarStatus) 5 раз в секунду.
Правила — в lidar_logic.py. Описание — в docs/lidar.md.
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data  # QoS для датчиков: best effort, без повторов

from sensor_msgs.msg import LaserScan
from uav_interfaces.msg import LidarStatus, RawState

from uav_lidar import lidar_logic as logic


class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')

        # Параметры: их можно менять при запуске, например --ros-args -p obstacle_warn_m:=4.0
        self.declare_parameter('obstacle_warn_m', logic.OBSTACLE_WARN_M)
        self.declare_parameter('obstacle_error_m', logic.OBSTACLE_ERROR_M)
        self.declare_parameter('max_tilt_deg', logic.MAX_TILT_DEG)
        self.declare_parameter('scan_timeout_s', logic.SCAN_TIMEOUT_S)
        self.declare_parameter('blind_width_deg', logic.BLIND_WIDTH_DEG)
        self.declare_parameter('blind_center_deg', 0.0)
        self.declare_parameter('topic_scan', '/lidar/scan')
        get = self.get_parameter
        self.warn_m = get('obstacle_warn_m').get_parameter_value().double_value
        self.error_m = get('obstacle_error_m').get_parameter_value().double_value
        self.max_tilt = get('max_tilt_deg').get_parameter_value().double_value
        self.timeout = get('scan_timeout_s').get_parameter_value().double_value
        self.blind_width = get('blind_width_deg').get_parameter_value().double_value
        self.blind_center = get('blind_center_deg').get_parameter_value().double_value
        topic_scan = get('topic_scan').get_parameter_value().string_value

        # Когда пришёл последний скан. До первого вызова publish_status это None: при use_sim_time
        # часы узла «прыгают» к времени записи, и брать время в момент запуска нельзя
        self.last_scan_s = None
        self.analysis = None               # результат разбора последнего скана
        self.roll = 0.0                    # последние крен и тангаж из /uav/raw_state
        self.pitch = 0.0
        self.last_level = logic.OK         # прошлый уровень, чтобы писать в терминал только изменения
        self.was_lost = False

        self.publisher = self.create_publisher(LidarStatus, '/uav/lidar_status', 10)
        self.create_subscription(LaserScan, topic_scan, self.on_scan, qos_profile_sensor_data)
        self.create_subscription(RawState, '/uav/raw_state', self.on_raw_state, 10)
        self.create_timer(0.2, self.publish_status)   # 5 раз в секунду
        self.get_logger().info(
            f'lidar_processor запущен: предупреждение ближе {self.warn_m} м, опасность ближе {self.error_m} м, '
            f'слепой сектор {self.blind_width}°, максимальный наклон {self.max_tilt}°')

    def now_s(self):
        # Текущее время узла в секундах (учитывает use_sim_time, т.е. время из записи rosbag2)
        return self.get_clock().now().nanoseconds * 1e-9

    def on_raw_state(self, raw):
        self.roll = raw.roll_deg
        self.pitch = raw.pitch_deg

    def on_scan(self, scan):
        self.last_scan_s = self.now_s()
        self.analysis = logic.analyze_scan(
            scan.ranges, scan.angle_min, scan.angle_increment, scan.range_min, scan.range_max,
            self.blind_width, self.blind_center)

    def publish_status(self):
        if self.last_scan_s is None:
            # Первый вызов таймера: от этого момента считаем, что лидар «так и не заработал», если сканов нет
            self.last_scan_s = self.now_s()
        lost = logic.scan_lost(self.last_scan_s, self.now_s(), self.timeout)
        tilted = logic.tilt_too_large(self.roll, self.pitch, self.max_tilt)
        msg = LidarStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'lidar'
        msg.scan_ok = not lost and self.analysis is not None
        msg.scan_used = msg.scan_ok and not tilted
        # -1.0 означает «препятствий нет»
        msg.nearest_m = msg.front_m = msg.left_m = msg.back_m = msg.right_m = -1.0
        msg.nearest_angle_deg = 0.0
        level, alerts = logic.OK, []

        if lost:
            level, alerts = logic.ERROR, ['LIDAR_LOST']
        elif self.analysis is not None:
            msg.total_rays = self.analysis['total']
            msg.valid_rays = self.analysis['valid']
            if msg.scan_used:   # при большом наклоне цифры о препятствиях не публикуем
                a = self.analysis
                if a['nearest'] is not None:
                    msg.nearest_m = float(a['nearest'])
                    msg.nearest_angle_deg = float(a['nearest_angle'])
                s = a['sectors']
                msg.front_m = -1.0 if s['front'] is None else float(s['front'])
                msg.left_m = -1.0 if s['left'] is None else float(s['left'])
                msg.back_m = -1.0 if s['back'] is None else float(s['back'])
                msg.right_m = -1.0 if s['right'] is None else float(s['right'])
                level, alerts = logic.obstacle_level(a['nearest'], self.warn_m, self.error_m)
        msg.level = level
        msg.alerts = alerts
        self.publisher.publish(msg)
        self.report_changes(level, alerts, lost, msg)

    def report_changes(self, level, alerts, lost, msg):
        # В терминал пишем только когда состояние изменилось, иначе строки будут повторяться 5 раз в секунду
        if lost and not self.was_lost:
            self.get_logger().error(f'LIDAR_LOST: нет данных лидара больше {self.timeout:.1f} с')
        elif self.was_lost and not lost:
            self.get_logger().info('Данные лидара снова приходят')
        self.was_lost = lost
        if not lost and level != self.last_level:
            if level == logic.ERROR:
                self.get_logger().error(
                    f'{alerts[0]}: препятствие в {msg.nearest_m:.2f} м, направление {msg.nearest_angle_deg:+.0f}°')
            elif level == logic.WARN:
                self.get_logger().warning(
                    f'{alerts[0]}: препятствие в {msg.nearest_m:.2f} м, направление {msg.nearest_angle_deg:+.0f}°')
            else:
                self.get_logger().info('Препятствий рядом нет')
        if not lost:
            self.last_level = level
        else:
            self.last_level = logic.OK


def main(args=None):
    rclpy.init(args=args)
    node = LidarProcessor()
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
