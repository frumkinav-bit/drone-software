from pathlib import Path

import pytest

from uav_autopilot.csv_reader import read_rows, split_alerts

# Путь к примеру данных: test -> uav_autopilot -> src -> корень репозитория
SAMPLE = Path(__file__).resolve().parents[3] / 'data' / 'test' / 'sample_telemetry.csv'


def test_split_alerts():
    assert split_alerts('') == []
    assert split_alerts('LINK_WEAK') == ['LINK_WEAK']
    assert split_alerts('LINK_WEAK;LOW_BATTERY') == ['LINK_WEAK', 'LOW_BATTERY']


def test_read_sample():
    rows = read_rows(SAMPLE)
    assert len(rows) == 12
    assert rows[0]['flight_mode'] == 'MANUAL'
    assert rows[6]['alerts'] == ['LINK_WEAK']
    assert isinstance(rows[0]['battery_percent'], int)


def test_missing_column(tmp_path):
    bad = tmp_path / 'bad.csv'
    bad.write_text('latitude,longitude\n55.0,37.0\n', encoding='utf-8')
    with pytest.raises(ValueError, match='нет колонок'):
        read_rows(bad)


def test_bad_number(tmp_path):
    header = open(SAMPLE, encoding='utf-8').readline()
    bad = tmp_path / 'bad.csv'
    bad.write_text(header + 'x,uav_01,abc,37,1,1,1,1,1,16,90,2,AUTO,RUNNING,\n', encoding='utf-8')
    with pytest.raises(ValueError, match='строке 2'):
        read_rows(bad)
