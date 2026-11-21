"""Тесты обработки лидара. ROS не нужен."""
import math

from uav_lidar import lidar_logic as L

INC = math.radians(1.0)
MIN_A = -math.pi


def scan_with(distance_by_angle, count=360):
    """Скан из 360 лучей (-180°…179°): все inf, кроме указанных {угол_в_градусах: расстояние}."""
    ranges = [math.inf] * count
    for angle, distance in distance_by_angle.items():
        ranges[int(round(angle + 180)) % count] = distance
    return ranges


def analyze(ranges, **kwargs):
    return L.analyze_scan(ranges, MIN_A, INC, 0.15, 12.0, **kwargs)


def test_empty_scan_has_no_obstacles():
    result = analyze([math.inf] * 360)
    assert result['valid'] == 0 and result['nearest'] is None and result['total'] == 360


def test_nearest_and_direction():
    result = analyze(scan_with({30: 4.0, -90: 2.5, 180: 7.0}))
    assert result['valid'] == 3
    assert result['nearest'] == 2.5 and round(result['nearest_angle']) == -90


def test_sectors():
    # спереди берём 10°, а не 0°: луч прямо по курсу попадает в слепой сектор по умолчанию
    result = analyze(scan_with({10: 5.0, 90: 4.0, -90: 3.0, 180: 2.0}))
    assert result['sectors'] == {'front': 5.0, 'left': 4.0, 'right': 3.0, 'back': 2.0}


def test_invalid_values_are_skipped():
    # слишком близко (ниже range_min), слишком далеко, nan и inf не считаются препятствиями
    result = analyze(scan_with({10: 0.05, 20: 15.0, 30: math.nan, 40: math.inf, 50: 3.0}))
    assert result['valid'] == 1 and result['nearest'] == 3.0


def test_blind_sector_ignored():
    # луч в 2° прямо по курсу попадает в слепой сектор шириной 6°, луч в 5° — уже нет
    result = analyze(scan_with({2: 0.3, 5: 4.0}), blind_width_deg=6.0)
    assert result['nearest'] == 4.0
    result = analyze(scan_with({2: 0.3}), blind_width_deg=0.0)   # слепой сектор выключен
    assert result['nearest'] == 0.3


def test_blind_sector_center():
    result = analyze(scan_with({90: 0.5, 0: 2.0}), blind_width_deg=10.0, blind_center_deg=90.0)
    assert result['nearest'] == 2.0


def test_normalize_deg():
    assert L.normalize_deg(190) == -170 and L.normalize_deg(-190) == 170 and L.normalize_deg(180) == -180


def test_obstacle_level():
    assert L.obstacle_level(None) == (L.OK, [])
    assert L.obstacle_level(5.0) == (L.OK, [])
    assert L.obstacle_level(2.0) == (L.WARN, ['OBSTACLE_NEAR'])
    assert L.obstacle_level(0.5) == (L.ERROR, ['OBSTACLE_CLOSE'])
    assert L.obstacle_level(3.5, warn_m=4.0) == (L.WARN, ['OBSTACLE_NEAR'])   # свой порог
    assert L.obstacle_level(1.0) == (L.WARN, ['OBSTACLE_NEAR'])               # граница: ровно 1 м — ещё не ERROR


def test_tilt():
    assert L.tilt_too_large(5.0, -5.0) is False
    assert L.tilt_too_large(20.0, 0.0) is True
    assert L.tilt_too_large(0.0, -16.0) is True


def test_scan_lost():
    assert L.scan_lost(10.0, 10.5) is False
    assert L.scan_lost(10.0, 11.5) is True
    assert L.scan_lost(0.0, 5.0, timeout_s=10.0) is False


def test_synthetic_scenario_covers_all_levels():
    levels = []
    for t in (0.0, 8.0, 12.0, 15.0, 18.0, 22.0):
        nearest = analyze(L.laserscan_order(L.synthetic_ranges(t)), blind_width_deg=0.0)['nearest']
        levels.append(L.obstacle_level(nearest)[0])
    # t=0 — далеко (OK), t=8 — 5,8 м (OK), t=12 — 2,9 м (WARN), t=15 — 0,8 м (ERROR), t=18 — 4,2 м... слева стена 6 м
    assert levels[0] == L.OK and L.WARN in levels and L.ERROR in levels and levels[-1] == L.OK


def test_synthetic_scan_front_distance():
    result = analyze(L.laserscan_order(L.synthetic_ranges(15.0)), blind_width_deg=0.0)
    assert abs(result['nearest'] - 0.8) < 1e-6 and abs(result['nearest_angle']) < 1e-6
