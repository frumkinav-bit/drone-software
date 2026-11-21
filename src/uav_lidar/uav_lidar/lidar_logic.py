"""Правила обработки скана лидара. Здесь нет ROS, поэтому всё проверяется обычными тестами.

Скан — это список расстояний по лучам. Луч i смотрит под углом angle_min + i * angle_increment.
Углы: 0 = прямо по курсу дрона, положительные — влево (как в ROS), в градусах здесь и далее.
"""
import math

# Уровни серьёзности (те же числа, что в diagnostics_logic и LidarStatus.level)
OK = 0
WARN = 1
ERROR = 2

# Пороги по умолчанию — наше предложение, их должен уточнить инженер по автопилоту.
# Дрон летит до 3 м/с, лидар видит от 0,15 до 12 м.
OBSTACLE_WARN_M = 3.0    # ближе этого расстояния — предупреждение OBSTACLE_NEAR
OBSTACLE_ERROR_M = 1.0   # ближе этого расстояния — серьёзная проблема OBSTACLE_CLOSE
MAX_TILT_DEG = 15.0      # при большем крене или тангаже скан не используем (лучи попадают в землю)
SCAN_TIMEOUT_S = 1.0     # через сколько секунд без сканов считаем, что лидар пропал (LIDAR_LOST)
BLIND_WIDTH_DEG = 6.0    # ширина сектора по курсу, который закрывает мачта GPS (ждём подтверждения автопилота)

# Секторы вокруг дрона: имя -> (от, до) в градусах, «до» не входит. «back» — всё остальное.
SECTOR_LIMITS = {
    'front': (-45.0, 45.0),
    'left': (45.0, 135.0),
    'right': (-135.0, -45.0),
}


def normalize_deg(angle):
    """Приводит угол к диапазону от -180 до +180 градусов."""
    return (angle + 180.0) % 360.0 - 180.0


def sector_of(angle_deg):
    """В каком секторе лежит угол: 'front', 'left', 'right' или 'back'."""
    for name, (low, high) in SECTOR_LIMITS.items():
        if low <= angle_deg < high:
            return name
    return 'back'


def analyze_scan(ranges, angle_min, angle_increment, range_min, range_max,
                 blind_width_deg=BLIND_WIDTH_DEG, blind_center_deg=0.0):
    """Разбирает один скан.

    Возвращает словарь:
      total, valid — число лучей всего и лучей с корректным расстоянием;
      nearest, nearest_angle — ближайшее препятствие (м) и его угол (°) или None, None;
      sectors — {'front': м или None, 'left': ..., 'right': ..., 'back': ...}.
    Корректное расстояние — число в диапазоне [range_min, range_max]. «inf» («нет отражения»),
    «nan» и значения вне диапазона пропускаются. Лучи внутри «слепого» сектора тоже пропускаются.
    """
    sectors = {'front': None, 'left': None, 'right': None, 'back': None}
    nearest = None
    nearest_angle = None
    valid = 0
    half_blind = blind_width_deg / 2.0
    for i, distance in enumerate(ranges):
        # Пропускаем «нет отражения» (inf), nan и значения вне рабочего диапазона лидара
        if not math.isfinite(distance) or distance < range_min or distance > range_max:
            continue
        angle = normalize_deg(math.degrees(angle_min + i * angle_increment))
        # Слепой сектор: этот луч упирается в деталь самого дрона (мачту GPS), а не в препятствие
        if blind_width_deg > 0 and abs(normalize_deg(angle - blind_center_deg)) <= half_blind:
            continue
        valid += 1
        name = sector_of(angle)
        if sectors[name] is None or distance < sectors[name]:
            sectors[name] = distance
        if nearest is None or distance < nearest:
            nearest = distance
            nearest_angle = angle
    return {'total': len(ranges), 'valid': valid, 'nearest': nearest,
            'nearest_angle': nearest_angle, 'sectors': sectors}


def obstacle_level(nearest, warn_m=OBSTACLE_WARN_M, error_m=OBSTACLE_ERROR_M):
    """По расстоянию до ближайшего препятствия: (уровень, список кодов тревоги)."""
    if nearest is None:
        return OK, []
    if nearest < error_m:
        return ERROR, ['OBSTACLE_CLOSE']
    if nearest < warn_m:
        return WARN, ['OBSTACLE_NEAR']
    return OK, []


def tilt_too_large(roll_deg, pitch_deg, max_tilt_deg=MAX_TILT_DEG):
    """True, если дрон наклонён сильнее допустимого и скан использовать нельзя."""
    return abs(roll_deg) > max_tilt_deg or abs(pitch_deg) > max_tilt_deg


def scan_lost(last_received_s, now_s, timeout_s=SCAN_TIMEOUT_S):
    """True, если сканов нет дольше timeout_s секунд.

    last_received_s — время последнего скана; пока сканов не было, передаём время запуска узла,
    чтобы лидар, который так и не заработал, тоже считался потерянным.
    """
    return now_s - last_received_s > timeout_s


def synthetic_ranges(t, count=360, range_min=0.15, range_max=12.0):
    """Искусственный скан для демо без симулятора: стена спереди то приближается, то уходит.

    t — время в секундах от начала. Стена прямо по курсу: до 5 с на расстоянии 8 м,
    с 5 до 15 с приближается до 0,8 м, с 15 до 20 с отходит обратно. Слева постоянная стена на 6 м.
    Лучи без отражения получают значение inf, как в настоящем LaserScan.
    """
    if t < 5.0:
        front = 8.0
    elif t < 15.0:
        front = 8.0 - (t - 5.0) / 10.0 * 7.2    # от 8 м до 0,8 м
    elif t < 20.0:
        front = 0.8 + (t - 15.0) / 5.0 * 7.2    # от 0,8 м до 8 м
    else:
        front = 8.0
    ranges = []
    for i in range(count):
        angle = normalize_deg(i * 360.0 / count)    # луч 0 — вперёд, шаг 1°
        radians = math.radians(angle)
        best = math.inf
        if abs(angle) < 60.0:                       # стена спереди: плоскость на расстоянии front
            best = min(best, front / math.cos(radians))
        if 20.0 < angle < 160.0:                    # стена слева: плоскость на расстоянии 6 м
            best = min(best, 6.0 / math.sin(radians))
        ranges.append(best if range_min <= best <= range_max else math.inf)
    return ranges


def laserscan_order(ranges):
    """Переставляет лучи из порядка «от 0° (вперёд)» в порядок LaserScan «от -180°».

    В нашем скане 360 лучей с шагом 1°: в LaserScan первый луч стоит на -180°, а в synthetic_ranges — на 0°,
    поэтому список сдвигается на половину (180 лучей).
    """
    half = len(ranges) // 2
    return ranges[half:] + ranges[:half]
