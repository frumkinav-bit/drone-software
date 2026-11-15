"""Проверка журнала телеметрии: «записалось ли всё, что принято, и без искажений».

Запуск (в контейнере, после сборки):
    ros2 run uav_telemetry check_telemetry_log telemetry_logs/telemetry_XXXX.csv
    ros2 run uav_telemetry check_telemetry_log telemetry_logs/telemetry_XXXX.csv --source data/test/sample_telemetry.csv

Здесь нет ROS — только стандартный Python, поэтому модуль проверяется обычными тестами.
"""
import argparse
import csv
import sys

from uav_telemetry.log_format import COLUMNS, parse_iso

# Допустимые диапазоны числовых полей: (минимум, максимум)
RANGES = {
    'latitude': (-90.0, 90.0),
    'longitude': (-180.0, 180.0),
    'speed_mps': (0.0, 100.0),
    'battery_percent': (0.0, 100.0),
    'link_state': (0, 2),
}
# Поля, которые обязаны быть числами
NUMERIC = [
    'latitude', 'longitude', 'altitude_m', 'speed_mps', 'roll_deg', 'pitch_deg',
    'yaw_deg', 'battery_voltage_v', 'battery_percent', 'link_state',
]


def check_log(path, max_gap_s=2.0):
    """Проверяет один файл журнала. Возвращает (число_строк, список_проблем)."""
    problems = []
    rows = 0
    previous_time = None
    with open(path, newline='', encoding='utf-8') as file:
        reader = csv.DictReader(file)
        # 1. Заголовки должны совпадать с форматом из docs/telemetry_format.md
        if reader.fieldnames != COLUMNS:
            problems.append(f'Заголовки не совпадают с форматом: {reader.fieldnames}')
            return 0, problems
        # start=2: первая строка файла — заголовки
        for line, row in enumerate(reader, start=2):
            rows += 1
            # 2. Время должно разбираться
            try:
                moment = parse_iso(row['stamp_iso'])
            except ValueError:
                problems.append(f'Строка {line}: не разобрать время «{row["stamp_iso"]}»')
                moment = None
            # 3. Время не должно идти назад, и не должно быть больших пропусков
            if moment is not None and previous_time is not None:
                gap = (moment - previous_time).total_seconds()
                if gap < 0:
                    problems.append(f'Строка {line}: время идёт назад ({gap:.2f} с)')
                elif gap > max_gap_s:
                    problems.append(f'Строка {line}: пропуск {gap:.2f} с (допустимо {max_gap_s} с)')
            if moment is not None:
                previous_time = moment
            # 4. Числа и диапазоны
            for name in NUMERIC:
                try:
                    value = float(row[name])
                except ValueError:
                    problems.append(f'Строка {line}: поле {name} не число («{row[name]}»)')
                    continue
                if name in RANGES:
                    low, high = RANGES[name]
                    if not low <= value <= high:
                        problems.append(f'Строка {line}: {name}={value} вне диапазона {low}..{high}')
    if rows == 0:
        problems.append('В журнале нет ни одной строки данных')
    return rows, problems


def compare_with_source(log_path, source_path, tolerance=0.01):
    """Сравнивает журнал с исходным тестовым CSV по числу строк и значениям.

    Журнал округляет числа до 2–6 знаков, поэтому сравнение идёт с допуском tolerance.
    Если журнал короче исходника (запись начали позже), сравниваются последние строки.
    Возвращает список проблем.
    """
    problems = []
    with open(log_path, newline='', encoding='utf-8') as file:
        log_rows = list(csv.DictReader(file))
    with open(source_path, newline='', encoding='utf-8') as file:
        source_rows = list(csv.DictReader(file))
    if len(log_rows) < len(source_rows):
        problems.append(f'В журнале {len(log_rows)} строк, в исходнике {len(source_rows)}: часть данных потеряна')
    # Сравниваем столько строк с начала, сколько есть и там и там
    for index, (got, want) in enumerate(zip(log_rows, source_rows), start=1):
        for name in NUMERIC:
            try:
                if abs(float(got[name]) - float(want[name])) > tolerance:
                    problems.append(f'Строка данных {index}: {name} в журнале {got[name]}, в исходнике {want[name]}')
            except (ValueError, KeyError):
                problems.append(f'Строка данных {index}: поле {name} не удалось сравнить')
        for name in ('flight_mode', 'alerts'):
            if got.get(name) != want.get(name):
                problems.append(f'Строка данных {index}: {name} в журнале «{got.get(name)}», в исходнике «{want.get(name)}»')
    return problems


def main(argv=None):
    parser = argparse.ArgumentParser(description='Проверка журнала телеметрии БПЛА')
    parser.add_argument('log', help='файл журнала telemetry_*.csv')
    parser.add_argument('--source', help='исходный тестовый CSV для сравнения')
    parser.add_argument('--max-gap', type=float, default=2.0, help='допустимый пропуск между строками, с')
    args = parser.parse_args(argv)

    rows, problems = check_log(args.log, args.max_gap)
    if args.source:
        problems += compare_with_source(args.log, args.source)

    print(f'Строк в журнале: {rows}')
    if problems:
        print(f'НАЙДЕНО ПРОБЛЕМ: {len(problems)}')
        for text in problems:
            print('  -', text)
        return 1
    print('Журнал в порядке: формат, время, диапазоны' + (' и сравнение с исходником' if args.source else ''))
    return 0


if __name__ == '__main__':
    sys.exit(main())
