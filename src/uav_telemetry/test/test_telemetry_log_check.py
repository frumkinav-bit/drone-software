"""Тесты проверки журнала. ROS не нужен."""
import csv
import shutil
from pathlib import Path

from uav_telemetry.log_format import COLUMNS
from uav_telemetry.telemetry_log_check import check_log, compare_with_source, main

SAMPLE = Path(__file__).resolve().parents[3] / 'data' / 'test' / 'sample_telemetry.csv'


def write_log(path, rows):
    """Пишет журнал с нужными заголовками; rows — список словарей (недостающие поля берутся из образца)."""
    base = {
        'stamp_iso': '2026-10-18T12:00:00.000Z', 'uav_id': 'uav_01', 'latitude': '55.7',
        'longitude': '37.6', 'altitude_m': '1', 'speed_mps': '1', 'roll_deg': '0',
        'pitch_deg': '0', 'yaw_deg': '0', 'battery_voltage_v': '16', 'battery_percent': '90',
        'link_state': '2', 'flight_mode': 'AUTO', 'mission_status': 'RUNNING', 'alerts': '',
    }
    with open(path, 'w', newline='', encoding='utf-8') as file:
        writer = csv.DictWriter(file, fieldnames=COLUMNS)
        writer.writeheader()
        for row in rows:
            writer.writerow({**base, **row})


def test_sample_is_valid():
    # Образец тестовых данных сам должен проходить проверку
    rows, problems = check_log(SAMPLE)
    assert rows == 12 and problems == []


def test_copy_matches_source(tmp_path):
    copy = tmp_path / 'copy.csv'
    shutil.copy(SAMPLE, copy)
    assert compare_with_source(copy, SAMPLE) == []


def test_lost_rows_detected(tmp_path):
    lines = SAMPLE.read_text(encoding='utf-8').splitlines()
    short = tmp_path / 'short.csv'
    short.write_text('\n'.join(lines[:-3]) + '\n', encoding='utf-8')  # убрали 3 строки данных
    assert any('потеряна' in p for p in compare_with_source(short, SAMPLE))


def test_time_going_back(tmp_path):
    path = tmp_path / 'a.csv'
    write_log(path, [{'stamp_iso': '2026-10-18T12:00:05.000Z'}, {'stamp_iso': '2026-10-18T12:00:04.000Z'}])
    _, problems = check_log(path)
    assert any('назад' in p for p in problems)


def test_gap_detected(tmp_path):
    path = tmp_path / 'a.csv'
    write_log(path, [{'stamp_iso': '2026-10-18T12:00:00.000Z'}, {'stamp_iso': '2026-10-18T12:00:09.000Z'}])
    _, problems = check_log(path, max_gap_s=2.0)
    assert any('пропуск' in p for p in problems)


def test_out_of_range_and_not_number(tmp_path):
    path = tmp_path / 'a.csv'
    write_log(path, [{'battery_percent': '150'}, {'latitude': 'abc'}])
    _, problems = check_log(path)
    assert any('battery_percent' in p for p in problems)
    assert any('latitude' in p and 'не число' in p for p in problems)


def test_wrong_header(tmp_path):
    path = tmp_path / 'a.csv'
    path.write_text('a,b\n1,2\n', encoding='utf-8')
    rows, problems = check_log(path)
    assert rows == 0 and problems


def test_empty_log(tmp_path):
    path = tmp_path / 'a.csv'
    write_log(path, [])
    _, problems = check_log(path)
    assert any('ни одной' in p for p in problems)


def test_main_exit_codes(tmp_path):
    assert main([str(SAMPLE)]) == 0
    bad = tmp_path / 'bad.csv'
    write_log(bad, [{'link_state': '7'}])
    assert main([str(bad)]) == 1
