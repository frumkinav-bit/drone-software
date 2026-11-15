"""Формат журнала телеметрии (CSV). Здесь нет ROS, поэтому файл можно проверить обычными тестами.

Его используют узел telemetry_logger (запись) и проверка журнала check_telemetry_log (чтение).
Описание формата для людей — в docs/telemetry_format.md.
"""
from datetime import datetime, timezone

# Заголовки колонок CSV, порядок как в docs/telemetry_format.md
COLUMNS = [
    'stamp_iso', 'uav_id', 'latitude', 'longitude', 'altitude_m', 'speed_mps',
    'roll_deg', 'pitch_deg', 'yaw_deg', 'battery_voltage_v', 'battery_percent',
    'link_state', 'flight_mode', 'mission_status', 'alerts',
]


def stamp_to_iso(stamp):
    """Время ROS (секунды + наносекунды) -> строка ISO 8601 в UTC, например 2026-10-18T12:00:01.500Z.

    stamp — любой объект с полями sec и nanosec (в ROS это builtin_interfaces/Time).
    """
    seconds = stamp.sec + stamp.nanosec * 1e-9
    moment = datetime.fromtimestamp(seconds, tz=timezone.utc)
    return moment.isoformat(timespec='milliseconds').replace('+00:00', 'Z')


def parse_iso(text):
    """Строка ISO 8601 (с Z на конце или без миллисекунд) -> datetime в UTC."""
    return datetime.fromisoformat(text.replace('Z', '+00:00'))
