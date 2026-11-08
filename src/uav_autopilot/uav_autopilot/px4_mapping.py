"""Перевод данных PX4 в наши сообщения и наших команд в команды PX4.

Здесь нет ROS и нет px4_msgs, поэтому всё проверяется обычными тестами.
Узел autopilot_bridge.py только достаёт числа из сообщений PX4 и вызывает функции отсюда.

ВАЖНО: номера режимов и команд ниже взяты из документации PX4. Перед первым запуском
с настоящим PX4 v1.17.0 их нужно сверить с файлами px4_msgs (ветка release/1.17),
см. раздел «Что проверить» в docs/autopilot_interface.md.
"""
import math

# ---------------------------------------------------------------------------
# Режимы полёта
# ---------------------------------------------------------------------------

# Номер режима nav_state из сообщения VehicleStatus -> наше название режима.
# Режимы Manual и Stabilized у мультикоптера PX4 считаем одним: STABILIZED.
NAV_STATE_TO_MODE = {
    0: 'STABILIZED',   # Manual
    1: 'ALTITUDE',     # удержание высоты
    2: 'POSITION',     # удержание позиции вручную
    3: 'MISSION',      # полёт по миссии
    4: 'HOLD',         # зависание на месте (Loiter)
    5: 'RTL',          # возврат на точку взлёта
    8: 'LAND',         # аварийная посадка (отказ двигателя)
    9: 'LAND',         # аварийная посадка (потеря GPS)
    12: 'LAND',        # аварийное снижение
    14: 'OFFBOARD',    # управление из ROS 2
    15: 'STABILIZED',  # Stabilized
    17: 'TAKEOFF',     # взлёт
    18: 'LAND',        # посадка
}


def mode_from_nav_state(nav_state):
    """Номер режима PX4 -> название. Неизвестный номер даёт 'UNKNOWN'."""
    return NAV_STATE_TO_MODE.get(int(nav_state), 'UNKNOWN')


# ---------------------------------------------------------------------------
# Данные для телеметрии
# ---------------------------------------------------------------------------

def percent_from_remaining(remaining):
    """Заряд PX4 (доля от 0 до 1) -> проценты 0..100; -1, если заряд неизвестен."""
    if remaining is None or math.isnan(remaining) or remaining < 0:
        return -1
    return int(round(min(remaining, 1.0) * 100))


def altitude_from_ned_z(z_down):
    """Высота над точкой взлёта. В PX4 ось z направлена вниз (NED), поэтому меняем знак.

    Если данных нет (NaN), возвращаем NaN: так видно, что высота неизвестна.
    """
    if z_down is None or math.isnan(z_down):
        return float('nan')
    return -float(z_down)


def ground_speed(vx, vy):
    """Скорость относительно земли в м/с по горизонтальным составляющим."""
    if vx is None or vy is None or math.isnan(vx) or math.isnan(vy):
        return 0.0
    return math.hypot(vx, vy)


def quaternion_to_euler_deg(w, x, y, z):
    """Кватернион ориентации (как в VehicleAttitude) -> (крен, тангаж, курс) в градусах.

    Курс считается в системе NED: 0 = север, положительное направление по часовой
    стрелке, результат от 0 до 360.
    """
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sin_pitch = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))  # ограничиваем, чтобы asin не упал
    pitch = math.asin(sin_pitch)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw) % 360.0


def link_state_from_flags(rc_signal_lost, gcs_connection_lost, rssi=None, weak_rssi_percent=30):
    """Состояние связи для телеметрии: 0 нет, 1 слабая, 2 нормальная.

    Потерю определяет сам PX4 (флаги failsafe_flags). Порога «слабый сигнал» в PX4 нет,
    поэтому слабой считаем связь, если уровень сигнала rssi (0..100) ниже заданного порога.
    Значения rssi вне диапазона 0..100 считаем «неизвестно» и слабой связь не называем.
    """
    if rc_signal_lost or gcs_connection_lost:
        return 0
    if rssi is not None and 0 <= rssi <= 100 and rssi < weak_rssi_percent:
        return 1
    return 2


# ---------------------------------------------------------------------------
# Команды автопилоту
# ---------------------------------------------------------------------------

# Коды команд, как в uav_interfaces/msg/Command.msg
CMD_ARM = 0
CMD_DISARM = 1
CMD_TAKEOFF = 2
CMD_LAND = 3
CMD_RTL = 4
CMD_SET_MODE = 5

COMMAND_NAMES = {
    CMD_ARM: 'ARM',
    CMD_DISARM: 'DISARM',
    CMD_TAKEOFF: 'TAKEOFF',
    CMD_LAND: 'LAND',
    CMD_RTL: 'RTL',
    CMD_SET_MODE: 'SET_MODE',
}

# Номера команд PX4 (VehicleCommand.command, совпадают с MAVLink)
VEHICLE_CMD_NAV_RETURN_TO_LAUNCH = 20
VEHICLE_CMD_NAV_LAND = 21
VEHICLE_CMD_NAV_TAKEOFF = 22
VEHICLE_CMD_DO_SET_MODE = 176
VEHICLE_CMD_COMPONENT_ARM_DISARM = 400

# Наш режим -> (основной режим PX4, подрежим) для команды DO_SET_MODE
MODE_TO_PX4 = {
    'STABILIZED': (7, 0),
    'ALTITUDE': (2, 0),
    'POSITION': (3, 0),
    'OFFBOARD': (6, 0),
    'TAKEOFF': (4, 2),
    'HOLD': (4, 3),
    'MISSION': (4, 4),
    'RTL': (4, 5),
    'LAND': (4, 6),
}


def describe_command(command, mode=''):
    """Короткое описание команды для журнала: 'TAKEOFF' или 'SET_MODE (HOLD)'."""
    name = COMMAND_NAMES.get(command, f'НЕИЗВЕСТНАЯ({command})')
    return f'{name} ({mode})' if command == CMD_SET_MODE else name


def command_to_px4(command, mode=''):
    """Наша команда -> словарь для сообщения VehicleCommand: command и param1..param7.

    Параметры, которые не используются, ставим в NaN: так PX4 берёт значения по умолчанию.
    Для неизвестной команды или режима вызывает ValueError.
    """
    nan = float('nan')
    params = {'param1': nan, 'param2': nan, 'param3': nan, 'param4': nan,
              'param5': nan, 'param6': nan, 'param7': nan}

    if command == CMD_ARM:
        params['command'] = VEHICLE_CMD_COMPONENT_ARM_DISARM
        params['param1'] = 1.0   # 1 = включить
    elif command == CMD_DISARM:
        params['command'] = VEHICLE_CMD_COMPONENT_ARM_DISARM
        params['param1'] = 0.0   # 0 = выключить
    elif command == CMD_TAKEOFF:
        params['command'] = VEHICLE_CMD_NAV_TAKEOFF   # высота — по умолчанию из PX4
    elif command == CMD_LAND:
        params['command'] = VEHICLE_CMD_NAV_LAND
    elif command == CMD_RTL:
        params['command'] = VEHICLE_CMD_NAV_RETURN_TO_LAUNCH
    elif command == CMD_SET_MODE:
        if mode not in MODE_TO_PX4:
            raise ValueError(f'Неизвестный режим для SET_MODE: "{mode}"')
        main_mode, sub_mode = MODE_TO_PX4[mode]
        params['command'] = VEHICLE_CMD_DO_SET_MODE
        params['param1'] = 1.0               # режим задаётся пользовательскими номерами PX4
        params['param2'] = float(main_mode)
        params['param3'] = float(sub_mode)
    else:
        raise ValueError(f'Неизвестная команда: {command}')
    return params
