"""Узел autopilot_bridge: связь с автопилотом.

Два режима работы (параметр source):
  stub — заглушка. Принимает команды из /uav/command и пишет их в журнал. Автопилот не нужен.
         Этот режим используется для проверки в контейнере.
  px4  — настоящая связь с PX4 через uXRCE-DDS. Читает темы /fmu/out/..., переводит их в наше
         сообщение RawState (тема /uav/raw_state) и отправляет команды в /fmu/in/vehicle_command.
         Нужны пакет px4_msgs (ветка release/1.17) и запущенный Micro XRCE-DDS Agent.
         Запускается на ноутбуке группы моделирования рядом с симуляцией.

Режим px4 написан по документации PX4, но ещё не проверен с настоящим PX4 SITL.
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy, qos_profile_sensor_data)

from uav_interfaces.msg import Command, RawState
from uav_autopilot import px4_mapping as mapping


class AutopilotBridge(Node):
    def __init__(self):
        super().__init__('autopilot_bridge')

        # Параметры узла (значения по умолчанию можно менять при запуске)
        self.declare_parameter('source', 'stub')            # 'stub' или 'px4'
        self.declare_parameter('rate_hz', 10.0)             # как часто публиковать /uav/raw_state
        self.declare_parameter('consider_rc_loss', True)    # учитывать потерю пульта как потерю связи
        self.declare_parameter('weak_rssi_percent', 30)     # ниже этого уровня сигнала связь «слабая»
        # Имена тем PX4. У разных версий PX4 к ним могут добавляться суффиксы (_v1, _v2):
        # настоящие имена показывает команда `ros2 topic list`
        self.declare_parameter('topic_vehicle_status', '/fmu/out/vehicle_status')
        self.declare_parameter('topic_battery', '/fmu/out/battery_status')
        self.declare_parameter('topic_local_position', '/fmu/out/vehicle_local_position')
        self.declare_parameter('topic_global_position', '/fmu/out/vehicle_global_position')
        self.declare_parameter('topic_attitude', '/fmu/out/vehicle_attitude')
        self.declare_parameter('topic_failsafe', '/fmu/out/failsafe_flags')
        self.declare_parameter('topic_command', '/fmu/in/vehicle_command')

        self.source = self.get_parameter('source').get_parameter_value().string_value
        if self.source not in ('stub', 'px4'):
            raise ValueError(f'Параметр source должен быть stub или px4, а не "{self.source}"')

        # Команды принимаем в обоих режимах
        self.command_subscription = self.create_subscription(
            Command, '/uav/command', self.on_command, 10)

        if self.source == 'px4':
            self.setup_px4()
            self.get_logger().info('autopilot_bridge запущен в режиме px4')
        else:
            self.get_logger().info('autopilot_bridge запущен в режиме stub (заглушка, автопилот не нужен)')

    # ------------------------------------------------------------------
    # Настройка режима px4
    # ------------------------------------------------------------------
    def setup_px4(self):
        # px4_msgs импортируем здесь, а не вверху файла: в режиме stub пакет не нужен
        from px4_msgs.msg import (BatteryStatus, FailsafeFlags, VehicleAttitude,
                                  VehicleCommand, VehicleGlobalPosition,
                                  VehicleLocalPosition, VehicleStatus)
        self.VehicleCommand = VehicleCommand

        self.consider_rc_loss = self.get_parameter('consider_rc_loss').get_parameter_value().bool_value
        self.weak_rssi = self.get_parameter('weak_rssi_percent').get_parameter_value().integer_value
        self.state = {}  # последние полученные данные: {'status': msg, 'battery': msg, ...}

        def topic(name):
            return self.get_parameter(name).get_parameter_value().string_value

        # PX4 публикует данные с QoS best effort: с обычным QoS сообщения не придут
        for key, msg_type, name in [
            ('status', VehicleStatus, 'topic_vehicle_status'),
            ('battery', BatteryStatus, 'topic_battery'),
            ('local', VehicleLocalPosition, 'topic_local_position'),
            ('global', VehicleGlobalPosition, 'topic_global_position'),
            ('attitude', VehicleAttitude, 'topic_attitude'),
            ('failsafe', FailsafeFlags, 'topic_failsafe'),
        ]:
            self.create_subscription(
                msg_type, topic(name),
                lambda msg, key=key: self.state.__setitem__(key, msg),  # запоминаем последнее сообщение
                qos_profile_sensor_data)

        # Профиль для команд, как в примерах PX4 для Python
        command_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)
        self.px4_command_publisher = self.create_publisher(
            VehicleCommand, topic('topic_command'), command_qos)

        self.raw_publisher = self.create_publisher(RawState, '/uav/raw_state', 10)
        rate_hz = self.get_parameter('rate_hz').get_parameter_value().double_value
        self.timer = self.create_timer(1.0 / rate_hz, self.publish_raw_state)
        self.waiting_logged = False

    # ------------------------------------------------------------------
    # PX4 -> RawState
    # ------------------------------------------------------------------
    def publish_raw_state(self):
        s = self.state
        # Без позиции, ориентации и статуса сообщение собрать нельзя: ждём
        if not all(key in s for key in ('status', 'global', 'local', 'attitude')):
            if not self.waiting_logged:
                self.get_logger().info('Жду данные от PX4 (статус, позиция, ориентация)...')
                self.waiting_logged = True
            return

        roll, pitch, yaw = mapping.quaternion_to_euler_deg(*s['attitude'].q)  # q = [w, x, y, z]
        failsafe = s.get('failsafe')
        rc_lost = bool(failsafe.manual_control_signal_lost) if failsafe and self.consider_rc_loss else False
        gcs_lost = bool(failsafe.gcs_connection_lost) if failsafe else False
        battery = s.get('battery')

        msg = RawState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.latitude = float(s['global'].lat)
        msg.longitude = float(s['global'].lon)
        msg.altitude_m = mapping.altitude_from_ned_z(s['local'].z)
        msg.speed_mps = mapping.ground_speed(s['local'].vx, s['local'].vy)
        msg.roll_deg = roll
        msg.pitch_deg = pitch
        msg.yaw_deg = yaw
        msg.battery_voltage_v = float(battery.voltage_v) if battery else 0.0
        msg.battery_percent = mapping.percent_from_remaining(battery.remaining) if battery else -1
        msg.link_state = mapping.link_state_from_flags(rc_lost, gcs_lost, None, self.weak_rssi)
        msg.flight_mode = mapping.mode_from_nav_state(s['status'].nav_state)
        msg.alerts = []
        self.raw_publisher.publish(msg)

    # ------------------------------------------------------------------
    # Команды: /uav/command -> автопилот
    # ------------------------------------------------------------------
    def on_command(self, cmd):
        description = mapping.describe_command(cmd.command, cmd.mode)
        try:
            params = mapping.command_to_px4(cmd.command, cmd.mode)  # проверяем команду в любом режиме
        except ValueError as error:
            self.get_logger().error(f'Команда отклонена: {error}')
            return

        if self.source == 'stub':
            self.get_logger().info(f'Команда принята (заглушка, автопилоту не отправлена): {description}')
            return

        out = self.VehicleCommand()
        out.timestamp = int(self.get_clock().now().nanoseconds / 1000)  # микросекунды
        out.command = params['command']
        out.param1 = params['param1']
        out.param2 = params['param2']
        out.param3 = params['param3']
        out.param4 = params['param4']
        out.param5 = params['param5']
        out.param6 = params['param6']
        out.param7 = params['param7']
        out.target_system = 1
        out.target_component = 1
        out.source_system = 1
        out.source_component = 1
        out.from_external = True
        self.px4_command_publisher.publish(out)
        self.get_logger().info(f'Команда отправлена автопилоту: {description}')


def main(args=None):
    rclpy.init(args=args)
    node = AutopilotBridge()
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
