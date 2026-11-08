import math

import pytest

from uav_autopilot import px4_mapping as m


def test_mode_from_nav_state():
    assert m.mode_from_nav_state(0) == 'STABILIZED'
    assert m.mode_from_nav_state(15) == 'STABILIZED'  # Manual и Stabilized — один режим
    assert m.mode_from_nav_state(2) == 'POSITION'
    assert m.mode_from_nav_state(3) == 'MISSION'
    assert m.mode_from_nav_state(4) == 'HOLD'
    assert m.mode_from_nav_state(5) == 'RTL'
    assert m.mode_from_nav_state(18) == 'LAND'
    assert m.mode_from_nav_state(999) == 'UNKNOWN'


def test_percent_from_remaining():
    assert m.percent_from_remaining(0.5) == 50
    assert m.percent_from_remaining(1.0) == 100
    assert m.percent_from_remaining(1.2) == 100          # больше 100% не бывает
    assert m.percent_from_remaining(0.0) == 0
    assert m.percent_from_remaining(-1.0) == -1          # неизвестно
    assert m.percent_from_remaining(float('nan')) == -1
    assert m.percent_from_remaining(None) == -1


def test_altitude_from_ned_z():
    assert m.altitude_from_ned_z(-10.0) == 10.0          # z вниз: -10 значит 10 м над точкой взлёта
    assert m.altitude_from_ned_z(0.0) == 0.0
    assert math.isnan(m.altitude_from_ned_z(float('nan')))


def test_ground_speed():
    assert m.ground_speed(3.0, 4.0) == pytest.approx(5.0)
    assert m.ground_speed(float('nan'), 1.0) == 0.0


def test_quaternion_identity_is_north_level():
    roll, pitch, yaw = m.quaternion_to_euler_deg(1.0, 0.0, 0.0, 0.0)
    assert (roll, pitch, yaw) == pytest.approx((0.0, 0.0, 0.0))


def test_quaternion_yaw_east_is_90():
    # Поворот на 90 градусов вокруг вертикальной оси: курс на восток
    s = math.sqrt(0.5)
    roll, pitch, yaw = m.quaternion_to_euler_deg(s, 0.0, 0.0, s)
    assert yaw == pytest.approx(90.0)
    assert roll == pytest.approx(0.0)
    assert pitch == pytest.approx(0.0)


def test_quaternion_yaw_is_0_to_360():
    s = math.sqrt(0.5)
    _roll, _pitch, yaw = m.quaternion_to_euler_deg(s, 0.0, 0.0, -s)  # поворот на -90 градусов
    assert yaw == pytest.approx(270.0)


def test_quaternion_roll_and_pitch():
    s = math.sqrt(0.5)
    roll, _pitch, _yaw = m.quaternion_to_euler_deg(s, s, 0.0, 0.0)   # крен 90 градусов
    assert roll == pytest.approx(90.0)
    _roll, pitch, _yaw = m.quaternion_to_euler_deg(math.cos(math.radians(10)), 0.0,
                                                   math.sin(math.radians(10)), 0.0)  # тангаж 20 градусов
    assert pitch == pytest.approx(20.0)


def test_link_state():
    assert m.link_state_from_flags(False, False) == 2
    assert m.link_state_from_flags(True, False) == 0
    assert m.link_state_from_flags(False, True) == 0
    assert m.link_state_from_flags(False, False, rssi=20, weak_rssi_percent=30) == 1
    assert m.link_state_from_flags(False, False, rssi=80, weak_rssi_percent=30) == 2
    assert m.link_state_from_flags(False, False, rssi=-1) == 2      # rssi неизвестен
    assert m.link_state_from_flags(True, False, rssi=80) == 0       # потеря сильнее rssi


def test_command_codes_match_message():
    # Коды в px4_mapping должны совпадать с Command.msg
    assert (m.CMD_ARM, m.CMD_DISARM, m.CMD_TAKEOFF, m.CMD_LAND, m.CMD_RTL, m.CMD_SET_MODE) == (0, 1, 2, 3, 4, 5)


def test_command_to_px4_arm_disarm():
    arm = m.command_to_px4(m.CMD_ARM)
    assert arm['command'] == 400 and arm['param1'] == 1.0
    disarm = m.command_to_px4(m.CMD_DISARM)
    assert disarm['command'] == 400 and disarm['param1'] == 0.0


def test_command_to_px4_simple_commands():
    assert m.command_to_px4(m.CMD_TAKEOFF)['command'] == 22
    assert m.command_to_px4(m.CMD_LAND)['command'] == 21
    assert m.command_to_px4(m.CMD_RTL)['command'] == 20
    assert math.isnan(m.command_to_px4(m.CMD_LAND)['param1'])  # лишние параметры — NaN


def test_command_to_px4_set_mode():
    hold = m.command_to_px4(m.CMD_SET_MODE, 'HOLD')
    assert hold['command'] == 176
    assert (hold['param1'], hold['param2'], hold['param3']) == (1.0, 4.0, 3.0)
    pos = m.command_to_px4(m.CMD_SET_MODE, 'POSITION')
    assert (pos['param2'], pos['param3']) == (3.0, 0.0)


def test_command_to_px4_errors():
    with pytest.raises(ValueError):
        m.command_to_px4(m.CMD_SET_MODE, 'FLY_AWAY')
    with pytest.raises(ValueError):
        m.command_to_px4(99)


def test_describe_command():
    assert m.describe_command(m.CMD_TAKEOFF) == 'TAKEOFF'
    assert m.describe_command(m.CMD_SET_MODE, 'HOLD') == 'SET_MODE (HOLD)'
    assert 'НЕИЗВЕСТНАЯ' in m.describe_command(42)


def test_all_modes_in_mapping_are_known_to_set_mode():
    # Каждый режим, который мы отдаём в телеметрию, можно и включить командой SET_MODE
    for mode in set(m.NAV_STATE_TO_MODE.values()):
        assert mode in m.MODE_TO_PX4, mode
