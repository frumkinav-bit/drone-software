"""
Тестовый маршрут — общие данные для tools/mission.py и tools/plot_mission.py.
Чтобы изменить маршрут, правьте этот файл.
"""
import math

# Начало мира test_field (<spherical_coordinates> в worlds/test_field.sdf)
LAT0, LON0 = 55.7818, 37.7108

# Точки в координатах мира Gazebo: X — восток, Y — север (м).
# Совпадают с маркерами wp1...wp4 в worlds/test_field.sdf.
ROUTE = [
    ("WP1", 10.0, 0.0),
    ("WP2", 10.0, 10.0),
    ("WP3", -3.0, 10.0),
    ("WP4", -3.0, 0.0),
]
ALT = 5.0          # высота маршрута над точкой взлёта, м
SPEED = 3.0        # скорость по маршруту, м/с
HOLD = 2.0         # задержка в каждой точке, с
ACC_RADIUS = 1.0   # радиус, в котором точка считается достигнутой, м


def meters_per_degree(lat_deg):
    """Длина градуса широты и долготы на эллипсоиде WGS84, м."""
    p = math.radians(lat_deg)
    m_lat = 111132.92 - 559.82 * math.cos(2 * p) + 1.175 * math.cos(4 * p)
    m_lon = 111412.84 * math.cos(p) - 93.5 * math.cos(3 * p)
    return m_lat, m_lon


def world_to_latlon(east, north):
    m_lat, m_lon = meters_per_degree(LAT0)
    return LAT0 + north / m_lat, LON0 + east / m_lon
