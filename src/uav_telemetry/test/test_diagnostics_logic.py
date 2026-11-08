from uav_telemetry.diagnostics_logic import ERROR, OK, WARN, evaluate, worst_level


def test_all_ok():
    assert evaluate(80, 2, 'AUTO', []) == {}


def test_low_battery_warn_and_error():
    assert evaluate(14, 2, 'AUTO', [])['LOW_BATTERY'][0] == WARN
    assert evaluate(6, 2, 'AUTO', [])['LOW_BATTERY'][0] == ERROR
    assert evaluate(15, 2, 'AUTO', []) == {}  # граница: 15% ещё не предупреждение
    assert evaluate(7, 2, 'AUTO', [])['LOW_BATTERY'][0] == WARN  # граница: 7% ещё не критично


def test_battery_thresholds_are_parameters():
    # Свои пороги: предупреждение ниже 30%, ошибка ниже 10%
    assert evaluate(25, 2, 'AUTO', [], 30, 10)['LOW_BATTERY'][0] == WARN
    assert evaluate(9, 2, 'AUTO', [], 30, 10)['LOW_BATTERY'][0] == ERROR


def test_unknown_battery_is_skipped():
    assert evaluate(-1, 2, 'AUTO', []) == {}


def test_link_states():
    assert evaluate(80, 1, 'AUTO', [])['LINK_WEAK'][0] == WARN
    assert evaluate(80, 0, 'AUTO', [])['LINK_LOST'][0] == ERROR


def test_rtl_mode():
    assert 'MODE_RTL' in evaluate(80, 2, 'RTL', [])


def test_land_and_unknown_modes():
    assert evaluate(80, 2, 'LAND', [])['MODE_LAND'][0] == WARN
    assert evaluate(80, 2, 'UNKNOWN', [])['MODE_UNKNOWN'][0] == WARN


def test_normal_modes_have_no_problems():
    for mode in ['STABILIZED', 'ALTITUDE', 'POSITION', 'HOLD', 'MISSION', 'TAKEOFF', 'OFFBOARD']:
        assert evaluate(80, 2, mode, []) == {}


def test_alert_not_duplicated():
    problems = evaluate(80, 0, 'AUTO', ['LINK_LOST', 'SENSOR_FAIL'])
    assert list(problems).count('LINK_LOST') == 1
    assert 'SENSOR_FAIL' in problems


def test_worst_level():
    assert worst_level({}) == OK
    assert worst_level(evaluate(15, 0, 'AUTO', [])) == ERROR
