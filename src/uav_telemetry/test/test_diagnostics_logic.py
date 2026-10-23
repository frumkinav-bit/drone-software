from uav_telemetry.diagnostics_logic import ERROR, OK, WARN, evaluate, worst_level


def test_all_ok():
    assert evaluate(80, 2, 'AUTO', []) == {}


def test_low_battery_warn_and_error():
    assert evaluate(19, 2, 'AUTO', [])['LOW_BATTERY'][0] == WARN
    assert evaluate(9, 2, 'AUTO', [])['LOW_BATTERY'][0] == ERROR
    assert evaluate(20, 2, 'AUTO', []) == {}  # граница: 20% ещё не предупреждение


def test_unknown_battery_is_skipped():
    assert evaluate(-1, 2, 'AUTO', []) == {}


def test_link_states():
    assert evaluate(80, 1, 'AUTO', [])['LINK_WEAK'][0] == WARN
    assert evaluate(80, 0, 'AUTO', [])['LINK_LOST'][0] == ERROR


def test_rtl_mode():
    assert 'MODE_RTL' in evaluate(80, 2, 'RTL', [])


def test_alert_not_duplicated():
    problems = evaluate(80, 0, 'AUTO', ['LINK_LOST', 'SENSOR_FAIL'])
    assert list(problems).count('LINK_LOST') == 1
    assert 'SENSOR_FAIL' in problems


def test_worst_level():
    assert worst_level({}) == OK
    assert worst_level(evaluate(15, 0, 'AUTO', [])) == ERROR
