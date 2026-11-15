"""Делает тестовый файл data/test/sample_mission.csv: полёт по маршруту группы моделирования.

Маршрут (WP1...WP4, высота, скорость, начало координат) взят из simulation/tools/route.py (v0.5).
Файл записан в формате журнала телеметрии, поэтому его можно подать в test_data_player.
Запуск из корня репозитория:  python3 tools/make_mission_sample.py
"""
import csv
import math
import sys
from datetime import datetime, timedelta, timezone

# Путь к simulation/tools, чтобы взять route.py как есть и не копировать числа
sys.path.insert(0, 'simulation/tools')
import route  # noqa: E402  (импорт после правки пути — так и задумано)

SPEED = route.SPEED          # скорость по маршруту, м/с
ALT = route.ALT              # высота, м
STEP_S = 1.0                 # шаг записи, с
START = datetime(2026, 11, 16, 12, 0, 0, tzinfo=timezone.utc)

COLUMNS = ['stamp_iso', 'uav_id', 'latitude', 'longitude', 'altitude_m', 'speed_mps',
           'roll_deg', 'pitch_deg', 'yaw_deg', 'battery_voltage_v', 'battery_percent',
           'link_state', 'flight_mode', 'mission_status', 'alerts']


def yaw_px4(east, north):
    """Курс по правилам PX4: 0 = север, по часовой. Из ENU: 90° − угол от востока."""
    return (90.0 - math.degrees(math.atan2(north, east))) % 360.0


def build_points():
    """Список точек (восток, север, высота, скорость, режим, статус, курс) с шагом STEP_S."""
    points = []
    x = y = 0.0
    yaw = 90.0  # дрон стоит носом на восток
    # Взлёт: набор высоты 1 м/с
    for i in range(int(ALT) + 1):
        points.append((x, y, float(i), 0.0 if i == 0 else 1.0, 'TAKEOFF', 'RUNNING', yaw))
    # Полёт по точкам маршрута
    for _name, wx, wy in route.ROUTE:
        dist = math.hypot(wx - x, wy - y)
        steps = max(1, round(dist / (SPEED * STEP_S)))
        yaw = yaw_px4(wx - x, wy - y)
        for k in range(1, steps + 1):
            points.append((x + (wx - x) * k / steps, y + (wy - y) * k / steps, ALT, SPEED, 'MISSION', 'RUNNING', yaw))
        x, y = wx, wy
        for _ in range(int(route.HOLD)):  # задержка в точке
            points.append((x, y, ALT, 0.0, 'MISSION', 'RUNNING', yaw))
    # Посадка: снижение 1 м/с
    for i in range(int(ALT) - 1, -1, -1):
        points.append((x, y, float(i), 1.0 if i else 0.0, 'LAND', 'RUNNING' if i else 'DONE', yaw))
    return points


def main():
    points = build_points()
    with open('data/test/sample_mission.csv', 'w', newline='', encoding='utf-8') as file:
        writer = csv.writer(file, lineterminator='\n')
        writer.writerow(COLUMNS)
        total = len(points)
        for n, (x, y, alt, speed, mode, status, yaw) in enumerate(points):
            lat, lon = route.world_to_latlon(x, y)
            stamp = (START + timedelta(seconds=n * STEP_S)).strftime('%Y-%m-%dT%H:%M:%SZ')
            percent = round(100 - 12 * n / total)           # условный разряд за полёт
            volts = round(16.8 - 1.2 * n / total, 2)
            writer.writerow([stamp, 'uav_01', f'{lat:.6f}', f'{lon:.6f}', f'{alt:.1f}', f'{speed:.1f}',
                             '0.0', '0.0', f'{yaw:.1f}', f'{volts:.2f}', percent, 2, mode, status, ''])
    print(f'Записано строк: {total}')


if __name__ == '__main__':
    main()
