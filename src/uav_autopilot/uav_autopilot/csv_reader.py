"""Чтение тестовых данных из CSV.

Здесь нет ROS, поэтому код можно проверить обычными тестами.
"""
import csv  # встроенный модуль Python для работы с таблицами CSV

# Колонки, без которых файл нельзя использовать
REQUIRED_COLUMNS = [
    'latitude', 'longitude', 'altitude_m', 'speed_mps',
    'roll_deg', 'pitch_deg', 'yaw_deg',
    'battery_voltage_v', 'battery_percent',
    'link_state', 'flight_mode', 'alerts',
]


def split_alerts(text):
    """'LINK_WEAK;LOW_BATTERY' -> ['LINK_WEAK', 'LOW_BATTERY']; пустая строка -> []."""
    # split(';') режет строку по точкам с запятой, if item отбрасывает пустые куски
    return [item for item in text.split(';') if item]


def read_rows(path):
    """Читает CSV и возвращает список словарей с уже приведёнными типами."""
    # newline='' и encoding нужны, чтобы файл читался одинаково на Mac и Windows
    with open(path, newline='', encoding='utf-8') as file:
        reader = csv.DictReader(file)  # каждая строка станет словарём {колонка: значение}
        columns = reader.fieldnames or []
        missing = [name for name in REQUIRED_COLUMNS if name not in columns]
        if missing:
            raise ValueError(f'В файле {path} нет колонок: {", ".join(missing)}')

        rows = []
        # start=2: первая строка файла — заголовки, данные начинаются со второй
        for line_number, row in enumerate(reader, start=2):
            try:
                rows.append({
                    'latitude': float(row['latitude']),
                    'longitude': float(row['longitude']),
                    'altitude_m': float(row['altitude_m']),
                    'speed_mps': float(row['speed_mps']),
                    'roll_deg': float(row['roll_deg']),
                    'pitch_deg': float(row['pitch_deg']),
                    'yaw_deg': float(row['yaw_deg']),
                    'battery_voltage_v': float(row['battery_voltage_v']),
                    'battery_percent': int(row['battery_percent']),
                    'link_state': int(row['link_state']),
                    'flight_mode': row['flight_mode'],
                    'alerts': split_alerts(row['alerts']),
                })
            except (ValueError, TypeError) as error:
                # Понятное сообщение: в какой строке какого файла ошибка
                raise ValueError(f'Ошибка в строке {line_number} файла {path}: {error}') from error
    return rows
