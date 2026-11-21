#!/usr/bin/env python3
"""
Генератор SDF-модели квадрокоптера project_quad (версия 0.4).

Геометрия — CAD-модель рамы группы конструкторов (меши в models/project_quad/meshes).
Здесь задаются масса, инерция, моторы и сенсоры. Чтобы что-то поменять:

    1. исправьте значения в блоке ПАРАМЕТРЫ;
    2. python3 tools/generate_model.py
    3. перезапустите Gazebo.

Скрипт перезапишет:
    models/project_quad/model.sdf        — модель;
    worlds/test_field_stab.sdf           — мир со встроенным стабилизатором Gazebo;
    px4/4050_gz_project_quad             — настройки (airframe) PX4 SITL под эту раму;
    tools/hover_speeds.txt               — обороты висения;
и выведет сводку (масса, центр масс, инерция, обороты висения).
"""
import math
from pathlib import Path

import numpy as np

# ============================== ПАРАМЕТРЫ ===================================
# Все величины в СИ: м, кг, с, рад. Оси модели: X вперёд, Y влево, Z вверх.
# Начало координат модели — центр между осями моторов, на нижней плоскости рамы.

MODEL_NAME = "project_quad"

# --- Компоновка от 8.11.2026, в 16.11 не менялась (отчёт конструкторов, этапы 4–5).
# «Остальная часть дрона без аккумулятора»: рама, моторы-статоры, регулятор
# 4-в-1, FPV-камера в пазах спереди, VTX сзади, приёмник в кармане основания.
# Масса и ЦМ — от конструкторов. Инерция — из CAD-модели g4zo_x2 без
# аккумулятора, развёрнутой на 180° (новое «вперёд» — сторона камеры).
BASE_MASS = 0.244
BASE_COM = (0.0, 0.0, 0.0118)
BASE_INERTIA = np.array([      # относительно BASE_COM
    [1.357881e-03, 0.0,          -1.469086e-04],
    [0.0,          1.709809e-03,  0.0],
    [-1.469086e-04, 0.0,          1.896295e-03],
])

# --- Опоры (16.11): под каждым мотором фланец 22x22x2 + труба Ø12.
#     Дрон стоит на опорах, нижняя точка Z = -LEG_HEIGHT. Конструкторы обсуждают
#     удлинение до 40 мм под бортовой компьютер — тогда LEG_HEIGHT = 0.040.
LEG_HEIGHT = 0.030
LEGS_MASS = 0.0148            # все 4 опоры

# --- Бортовые компоненты и крепления (компоновка конструкторов от 16.11).
#     (имя, масса кг, размеры box м, положение центра м). * — модель не выбрана.
EXTRA_COMPONENTS = [
    ("аккумулятор 6S 1250",            0.220,  (0.076, 0.036, 0.033),   (-0.0080, 0.0, 0.0525)),
    ("Pixhawk 6C Mini",                0.0392, (0.0533, 0.039, 0.0162), ( 0.0145, 0.0, 0.0226)),
    ("бортовой компьютер *",           0.045,  (0.065, 0.030, 0.010),   ( 0.0445, 0.0, -0.0100)),
    ("радиомодем SiK 433 МГц",         0.025,  (0.050, 0.025, 0.010),   (-0.0450, 0.0, -0.0100)),
    ("модуль питания и провода",       0.030,  (0.040, 0.030, 0.010),   (-0.0410, 0.0, 0.0270)),
    ("мачта GPS",                      0.015,  (0.008, 0.008, 0.065),   ( 0.0650, 0.0, 0.0775)),
    ("GPS Holybro M9N",                0.025,  (0.050, 0.050, 0.015),   ( 0.0650, 0.0, 0.1175)),
    ("лидар на кронштейне *",          0.050,  (0.040, 0.040, 0.030),   (-0.0240, 0.0, 0.0885)),
    ("кронштейн лидара со стойками",   0.0146, (0.0925, 0.039, 0.0025), (-0.0082, 0.0, 0.0722)),
    ("кронштейн мачты GPS",            0.0050, (0.031, 0.034, 0.009),   ( 0.0565, 0.0, 0.0405)),
    ("ремень, втулки, стойки плат",    0.0027, (0.020, 0.036, 0.010),   (-0.0080, 0.0, 0.0300)),
] + [
    (f"опора {i + 1}", LEGS_MASS / 4, (0.012, 0.012, LEG_HEIGHT),
     (sx * 0.0750, sy * 0.0875, -LEG_HEIGHT / 2))
    for i, (sx, sy) in enumerate(((1, -1), (-1, 1), (1, 1), (-1, -1)))
]
ESC_POSE, ESC_SIZE = (0.0097, 0.0, 0.0105), (0.036, 0.036, 0.007)   # регулятор 4-в-1 (только вид)

# --- Роторы (колокол мотора + пропеллер 5.1", 3 лопасти) — из CAD-модели
ROTOR_MASS = 0.0199
ROTOR_COM_Z = 0.01904
ROTOR_INERTIA = (4.569430e-06, 4.569430e-06, 6.882481e-06)   # Ixx, Iyy, Izz
ROTOR_Z = 0.005               # посадочная плоскость мотора над низом рамы

# Оси моторов: 150 мм вдоль (X) и 175 мм поперёк (Y) — из отчёта конструкторов
HALF_X = 0.0750
HALF_Y = 0.0875

# Порядок и направление — стандарт PX4 «Quadrotor X» (НЕ Betaflight!):
#   мотор 1 (rotor_0) — передний правый, CCW
#   мотор 2 (rotor_1) — задний левый,    CCW
#   мотор 3 (rotor_2) — передний левый,  CW
#   мотор 4 (rotor_3) — задний правый,   CW
ROTORS = [
    (0, +HALF_X, -HALF_Y, "ccw"),
    (1, -HALF_X, +HALF_Y, "ccw"),
    (2, +HALF_X, +HALF_Y, "cw"),
    (3, -HALF_X, -HALF_Y, "cw"),
]

# --- Модель мотора AE2207 V2 1960KV + Gemfan 51499 на 6S (оценка конструкторов).
# Физика: тяга F = k * omega^2, максимум 16,7 Н (~1700 г) на мотор при
# реальных ~3200 рад/с.
#
# !!! PX4 SITL v1.17 не может послать мотору команду больше 1000
#     (параметр SIM_GZ_EC_MAX ограничен 0…1000). Поэтому в симуляции
#     обороты МАСШТАБИРОВАНЫ: 1000 «условных» рад/с = 3200 настоящих.
#     Тяга и моменты те же, меняются только числа в командах моторам.
REAL_MAX_ROT_VELOCITY = 3200.0     # рад/с, настоящие
REAL_MOTOR_CONSTANT = 1.63e-06     # Н/(рад/с)², настоящий
SPEED_SCALE = REAL_MAX_ROT_VELOCITY / 1000.0

MAX_ROT_VELOCITY = 1000.0                                  # условные рад/с
MOTOR_CONSTANT = REAL_MOTOR_CONSTANT * SPEED_SCALE ** 2    # тяга при условных рад/с
MOMENT_CONSTANT = 0.016                                    # момент/тяга, не масштабируется
TIME_CONSTANT_UP = 0.0125
TIME_CONSTANT_DOWN = 0.025
ROTOR_DRAG_COEFF = 1.0e-06 * SPEED_SCALE                   # ∝ omega — тоже масштабируется
ROLLING_MOMENT_COEFF = 1.0e-08 * SPEED_SCALE
ROTOR_SLOWDOWN_SIM = 10.0
JOINT_DAMPING = 2.0e-06

# --- PX4 SITL (файл px4/4050_gz_project_quad генерируется отсюда)
PX4_AIRFRAME_ID = 4050
PX4_IDLE_SPEED = 70        # условные рад/с на холостом ходу (тяга ~4% веса, как у x500)
# Общие коэффициенты регуляторов угловой скорости PX4 (MC_*RATE_K).
# Проверено расчётом (tools/analysis/px4_stability.py, docs/px4_stability.png):
# со стандартными K = 1 наш лёгкий дрон раскачивается по крену уже при
# K ≈ 2,5 (запас всего 2,5 раза). K = 0,5 даёт запас в 5 раз при почти той же
# скорости реакции. Рыскание — стандартное.
PX4_ROLLPITCH_RATE_K = 0.5
PX4_YAW_RATE_K = 1.0

# --- Встроенный стабилизатор Gazebo (запасной вариант без PX4)
#     Коэффициенты из примера Gazebo для X3; внутри делятся на инерцию модели,
#     поэтому подходят и нашему дрону.
STAB_VELOCITY_GAIN = (2.7, 2.7, 2.7)
STAB_ATTITUDE_GAIN = (2.0, 3.0, 0.15)
STAB_RATE_GAIN = (0.4, 0.52, 0.18)
STAB_MAX_ACCEL = (2.0, 2.0, 2.0)

# --- Сенсоры. True/False — включить/выключить.
ENABLE_IMU = True
ENABLE_BARO = True
ENABLE_MAG = True
ENABLE_GPS = True
ENABLE_LIDAR = True
LIDAR_MODE = "2d"            # "2d" — 360° в одной плоскости, "3d" — 16 лучей ±15°
ENABLE_DOWN_CAMERA = True    # камера вниз для группы компьютерного зрения
ENABLE_FPV_CAMERA = False    # FPV-камера спереди (из CAD), выключена для скорости

IMU_RATE = 250               # Гц
BARO_RATE = 50
MAG_RATE = 100
GPS_RATE = 10
LIDAR_RATE = 10
DOWN_CAMERA_RATE = 15

# Шумы (СКО). Источник: типовые значения моделей PX4 / даташиты, см. docs.
GYRO_NOISE = 0.0003394       # рад/с
GYRO_BIAS = 3.8785e-05       # рад/с
ACCEL_NOISE = 0.004          # м/с²
ACCEL_BIAS = 0.006           # м/с²
BARO_NOISE = 1.5             # Па (~0,12 м высоты)
WORLD_ELEVATION = 150.0      # м, высота начала мира (= <elevation> в test_field.sdf)
MAG_NOISE = 0.003            # Гс (= 0,3 мкТл). Gazebo выдаёт поле в гауссах, как PX4
GPS_POS_NOISE_H_M = 0.3      # м. Gazebo применяет этот шум к широте/долготе
GPS_POS_NOISE_H = GPS_POS_NOISE_H_M / 111320.0   # в ГРАДУСАХ — поэтому пересчёт
GPS_POS_NOISE_V = 0.4        # м
GPS_VEL_NOISE_H = 0.05       # м/с
GPS_VEL_NOISE_V = 0.10       # м/с
LIDAR_NOISE = 0.01           # м
CAMERA_NOISE = 0.007

# Положение сенсоров (x, y, z, roll, pitch, yaw)
GPS_POSE = (0.065, 0.0, 0.1175, 0, 0, 0)      # центр модуля GPS (16.11)
LIDAR_POSE = (-0.024, 0.0, 0.0995, 0, 0, 0)   # плоскость сканирования 99,5 мм (16.11)
# Камера вниз — под основанием, место X -20…12 мм, Y ±15 мм (16.11). Условно
# Raspberry Pi Camera Module 3 (25x24x12 мм, угол ~66°). Закреплена жёстко:
# строго вниз смотрит при висении, в наклоне — вместе с корпусом.
DOWN_CAMERA_SIZE = (0.025, 0.024, 0.012)
DOWN_CAMERA_POSE = (-0.004, 0.0, -0.013, 0, math.pi / 2, 0)   # объектив под низом камеры
FPV_CAMERA_POSE = (0.086, 0.0, 0.0235, 0, -0.436, 0)

TOP_PLATE_Z = 0.036          # верх верхней пластины рамы (из CAD)
G = 9.80665
# ===========================================================================


def box_I(m, s):
    x, y, z = s
    return np.diag([m / 12 * (y*y + z*z), m / 12 * (x*x + z*z), m / 12 * (x*x + y*y)])


def shift(I, m, d):
    """Перенос тензора инерции (теорема Штейнера)."""
    d = np.asarray(d, float)
    return I + m * (d @ d * np.eye(3) - np.outer(d, d))


def combine():
    parts = [(BASE_MASS, np.array(BASE_COM), BASE_INERTIA)]
    for _, m, s, p in EXTRA_COMPONENTS:
        parts.append((m, np.array(p), box_I(m, s)))
    M = sum(p[0] for p in parts)
    com = sum(p[0] * p[1] for p in parts) / M
    I = sum(shift(p[2], p[0], p[1] - com) for p in parts)
    return M, com, I


def inertial_xml(m, com, I, ind="      "):
    return f"""{ind}<inertial>
{ind}  <pose>{com[0]:.5f} {com[1]:.5f} {com[2]:.5f} 0 0 0</pose>
{ind}  <mass>{m:.5f}</mass>
{ind}  <inertia>
{ind}    <ixx>{I[0,0]:.6e}</ixx><ixy>{I[0,1]:.6e}</ixy><ixz>{I[0,2]:.6e}</ixz>
{ind}    <iyy>{I[1,1]:.6e}</iyy><iyz>{I[1,2]:.6e}</iyz>
{ind}    <izz>{I[2,2]:.6e}</izz>
{ind}  </inertia>
{ind}</inertial>"""


def mesh_visual(name, mesh, rgba):
    return f"""      <visual name="{name}">
        <geometry><mesh><uri>model://{MODEL_NAME}/meshes/{mesh}</uri></mesh></geometry>
        <material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse><specular>0.2 0.2 0.2 1</specular></material>
      </visual>"""


def noise(sd, bias=None):
    b = f"<bias_mean>0</bias_mean><bias_stddev>{bias}</bias_stddev>" if bias else ""
    return f'<noise type="gaussian"><mean>0</mean><stddev>{sd}</stddev>{b}</noise>'


def pose(p):
    return " ".join(f"{v:.4f}" for v in p)


def sensors_xml():
    out = []
    if ENABLE_IMU:
        g = noise(GYRO_NOISE, GYRO_BIAS)
        a = noise(ACCEL_NOISE, ACCEL_BIAS)
        out.append(f"""
      <!-- IMU (на Pixhawk 6C Mini). Имя imu_sensor — как ждёт PX4 SITL. -->
      <sensor name="imu_sensor" type="imu">
        <always_on>1</always_on>
        <update_rate>{IMU_RATE}</update_rate>
        <imu>
          <angular_velocity><x>{g}</x><y>{g}</y><z>{g}</z></angular_velocity>
          <linear_acceleration><x>{a}</x><y>{a}</y><z>{a}</z></linear_acceleration>
        </imu>
      </sensor>""")
    if ENABLE_BARO:
        out.append(f"""
      <!-- Барометр (на Pixhawk 6C Mini) -->
      <sensor name="air_pressure_sensor" type="air_pressure">
        <always_on>1</always_on>
        <update_rate>{BARO_RATE}</update_rate>
        <air_pressure>
          <reference_altitude>{WORLD_ELEVATION:g}</reference_altitude>
          <pressure>{noise(BARO_NOISE)}</pressure>
        </air_pressure>
      </sensor>""")
    if ENABLE_MAG:
        n = noise(MAG_NOISE)
        out.append(f"""
      <!-- Магнитометр (компас в модуле GPS) -->
      <sensor name="magnetometer_sensor" type="magnetometer">
        <pose>{pose(GPS_POSE)}</pose>
        <always_on>1</always_on>
        <update_rate>{MAG_RATE}</update_rate>
        <magnetometer><x>{n}</x><y>{n}</y><z>{n}</z></magnetometer>
      </sensor>""")
    if ENABLE_GPS:
        out.append(f"""
      <!-- GPS Holybro M9N -->
      <sensor name="navsat_sensor" type="navsat">
        <pose>{pose(GPS_POSE)}</pose>
        <always_on>1</always_on>
        <update_rate>{GPS_RATE}</update_rate>
        <navsat>
          <position_sensing>
            <horizontal>{noise(f'{GPS_POS_NOISE_H:.3e}')}</horizontal>
            <vertical>{noise(GPS_POS_NOISE_V)}</vertical>
          </position_sensing>
          <velocity_sensing>
            <horizontal>{noise(GPS_VEL_NOISE_H)}</horizontal>
            <vertical>{noise(GPS_VEL_NOISE_V)}</vertical>
          </velocity_sensing>
        </navsat>
      </sensor>""")
    if ENABLE_LIDAR:
        if LIDAR_MODE == "3d":
            vert = "<samples>16</samples><resolution>1</resolution><min_angle>-0.2618</min_angle><max_angle>0.2618</max_angle>"
            hs, rmax = 900, 30.0
        else:
            vert = "<samples>1</samples><resolution>1</resolution><min_angle>0</min_angle><max_angle>0</max_angle>"
            hs, rmax = 360, 12.0
        out.append(f"""
      <!-- Тестовый лидар ({LIDAR_MODE.upper()}). Модель лидара ещё не выбрана электронщиками. -->
      <sensor name="lidar_sensor" type="gpu_lidar">
        <pose>{pose(LIDAR_POSE)}</pose>
        <topic>{MODEL_NAME}/lidar</topic>
        <always_on>1</always_on>
        <update_rate>{LIDAR_RATE}</update_rate>
        <visualize>true</visualize>
        <lidar>
          <scan>
            <horizontal><samples>{hs}</samples><resolution>1</resolution><min_angle>-3.14159</min_angle><max_angle>3.14159</max_angle></horizontal>
            <vertical>{vert}</vertical>
          </scan>
          <range><min>0.15</min><max>{rmax}</max><resolution>0.01</resolution></range>
          <noise><type>gaussian</type><mean>0</mean><stddev>{LIDAR_NOISE}</stddev></noise>
        </lidar>
      </sensor>""")
    if ENABLE_DOWN_CAMERA:
        out.append(f"""
      <!-- Камера вниз 1280x720 (формат тестовых данных группы CV) -->
      <sensor name="down_camera" type="camera">
        <pose>{pose(DOWN_CAMERA_POSE)}</pose>
        <topic>{MODEL_NAME}/camera_down/image</topic>
        <always_on>1</always_on>
        <update_rate>{DOWN_CAMERA_RATE}</update_rate>
        <camera>
          <horizontal_fov>1.2043</horizontal_fov>
          <image><width>1280</width><height>720</height><format>R8G8B8</format></image>
          <clip><near>0.05</near><far>300</far></clip>
          <noise><type>gaussian</type><mean>0</mean><stddev>{CAMERA_NOISE}</stddev></noise>
        </camera>
      </sensor>""")
    if ENABLE_FPV_CAMERA:
        out.append(f"""
      <!-- FPV-камера (из CAD-модели) -->
      <sensor name="fpv_camera" type="camera">
        <pose>{pose(FPV_CAMERA_POSE)}</pose>
        <topic>{MODEL_NAME}/camera_fpv/image</topic>
        <always_on>1</always_on>
        <update_rate>30</update_rate>
        <camera>
          <horizontal_fov>2.0</horizontal_fov>
          <image><width>640</width><height>480</height></image>
          <clip><near>0.05</near><far>300</far></clip>
        </camera>
      </sensor>""")
    return "\n".join(out)


def build():
    M, com, I = combine()

    visuals = [
        mesh_visual("frame_base", "frame_base.stl", "0.14 0.14 0.16 1"),
        mesh_visual("frame_top", "frame_top.stl", "0.22 0.22 0.25 1"),
        mesh_visual("spacers", "spacers.stl", "0.82 0.20 0.16 1"),
        mesh_visual("cam_plates", "cam_plates.stl", "0.31 0.31 0.63 1"),
        mesh_visual("esc_cap", "esc_cap.stl", "0.08 0.08 0.08 1"),
        mesh_visual("rx", "rx.stl", "0.08 0.08 0.08 1"),
        mesh_visual("camera", "camera.stl", "0.06 0.06 0.06 1"),
        mesh_visual("motor_stators", "motor_stators.stl", "0.63 0.63 0.67 1"),
    ]
    # 8.11: VTX перенесён назад (меш разворачиваем на 180°), стек F405 заменён
    # на регулятор 4-в-1 + Pixhawk 6C Mini, аккумулятор сдвинут на X = -24 мм.
    batt = next(c for c in EXTRA_COMPONENTS if c[0].startswith("аккумулятор"))
    pix = next(c for c in EXTRA_COMPONENTS if c[0].startswith("Pixhawk"))
    lid = next(c for c in EXTRA_COMPONENTS if c[0].startswith("лидар"))
    mast = next(c for c in EXTRA_COMPONENTS if c[0].startswith("мачта"))
    visuals.append(f"""      <visual name="vtx">
        <pose>0 0 0 0 0 3.14159</pose>
        <geometry><mesh><uri>model://{MODEL_NAME}/meshes/vtx.stl</uri></mesh></geometry>
        <material><ambient>0.12 0.12 0.12 1</ambient><diffuse>0.12 0.12 0.12 1</diffuse></material>
      </visual>
      <visual name="battery">
        <pose>{pose(batt[3])} 0 0 0</pose>
        <geometry><box><size>{pose(batt[2])}</size></box></geometry>
        <material><ambient>0.12 0.24 0.55 1</ambient><diffuse>0.12 0.24 0.55 1</diffuse></material>
      </visual>
      <visual name="esc_4in1">
        <pose>{pose(ESC_POSE)} 0 0 0</pose>
        <geometry><box><size>{pose(ESC_SIZE)}</size></box></geometry>
        <material><ambient>0.10 0.35 0.20 1</ambient><diffuse>0.10 0.35 0.20 1</diffuse></material>
      </visual>
      <visual name="pixhawk_6c_mini">
        <pose>{pose(pix[3])} 0 0 0</pose>
        <geometry><box><size>{pose(pix[2])}</size></box></geometry>
        <material><ambient>0.85 0.85 0.85 1</ambient><diffuse>0.85 0.85 0.85 1</diffuse></material>
      </visual>""")
    gx, gy, gz = GPS_POSE[:3]
    visuals.append(f"""      <!-- Заглушки компонентов, которых пока нет в CAD -->
      <visual name="gps_mast">
        <pose>{pose(mast[3])} 0 0 0</pose>
        <geometry><cylinder><radius>0.004</radius><length>{mast[2][2]:.4f}</length></cylinder></geometry>
        <material><ambient>0.3 0.3 0.3 1</ambient><diffuse>0.3 0.3 0.3 1</diffuse></material>
      </visual>
      <visual name="gps_puck">
        <pose>{gx:.4f} {gy:.4f} {gz:.4f} 0 0 0</pose>
        <geometry><cylinder><radius>0.025</radius><length>0.015</length></cylinder></geometry>
        <material><ambient>0.9 0.9 0.9 1</ambient><diffuse>0.9 0.9 0.9 1</diffuse></material>
      </visual>""")
    if ENABLE_LIDAR:
        visuals.append(f"""      <visual name="lidar_body">
        <pose>{pose(lid[3])} 0 0 0</pose>
        <geometry><box><size>{pose(lid[2])}</size></box></geometry>
        <material><ambient>0.9 0.75 0.1 1</ambient><diffuse>0.9 0.75 0.1 1</diffuse></material>
      </visual>""")

    # 16.11: крепления, опоры, компоненты под основанием, камера вниз
    box_colors = {
        "бортовой компьютер": "0.15 0.45 0.20 1",
        "радиомодем": "0.25 0.25 0.55 1",
        "модуль питания": "0.55 0.30 0.15 1",
        "кронштейн лидара": "0.85 0.85 0.85 1",
        "кронштейн мачты": "0.85 0.85 0.85 1",
    }
    for name, m, size, p in EXTRA_COMPONENTS:
        key = next((k for k in box_colors if name.startswith(k)), None)
        if key:
            vname = "".join(ch if ch.isascii() and ch.isalnum() else "_" for ch in
                            {"бортовой компьютер": "companion", "радиомодем": "sik_modem",
                             "модуль питания": "power_module", "кронштейн лидара": "lidar_bridge",
                             "кронштейн мачты": "gps_bracket"}[key])
            visuals.append(f"""      <visual name="{vname}">
        <pose>{pose(p)} 0 0 0</pose>
        <geometry><box><size>{pose(size)}</size></box></geometry>
        <material><ambient>{box_colors[key]}</ambient><diffuse>{box_colors[key]}</diffuse></material>
      </visual>""")
    for i, (name, m, size, (lx, ly, lz)) in enumerate(c for c in EXTRA_COMPONENTS if c[0].startswith("опора")):
        visuals.append(f"""      <visual name="leg_{i}">
        <pose>{lx:.4f} {ly:.4f} {lz:.4f} 0 0 0</pose>
        <geometry><cylinder><radius>0.006</radius><length>{LEG_HEIGHT:.4f}</length></cylinder></geometry>
        <material><ambient>0.85 0.85 0.85 1</ambient><diffuse>0.85 0.85 0.85 1</diffuse></material>
      </visual>""")
    if ENABLE_DOWN_CAMERA:
        cx, cy, cz = DOWN_CAMERA_POSE[0], DOWN_CAMERA_POSE[1], -DOWN_CAMERA_SIZE[2] / 2
        visuals.append(f"""      <visual name="down_camera_body">
        <pose>{cx:.4f} {cy:.4f} {cz:.4f} 0 0 0</pose>
        <geometry><box><size>{pose(DOWN_CAMERA_SIZE)}</size></box></geometry>
        <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material>
      </visual>""")

    rotor_I = np.diag(ROTOR_INERTIA)
    rotors, plugins = [], []
    pos_names = {(1, -1): "передний правый", (-1, 1): "задний левый",
                 (1, 1): "передний левый", (-1, -1): "задний правый"}
    for idx, x, y, d in ROTORS:
        where = pos_names[(int(math.copysign(1, x)), int(math.copysign(1, y)))]
        rotors.append(f"""
    <!-- Мотор {idx + 1} PX4 (rotor_{idx}): {where}, {d.upper()} -->
    <link name="rotor_{idx}">
      <pose>{x:.4f} {y:.4f} {ROTOR_Z:.4f} 0 0 0</pose>
{inertial_xml(ROTOR_MASS, (0, 0, ROTOR_COM_Z), rotor_I)}
      <visual name="rotor_{idx}_bell">
        <geometry><mesh><uri>model://{MODEL_NAME}/meshes/rotor_bell_{d}.stl</uri></mesh></geometry>
        <material><ambient>0.12 0.12 0.14 1</ambient><diffuse>0.12 0.12 0.14 1</diffuse></material>
      </visual>
      <visual name="rotor_{idx}_prop">
        <geometry><mesh><uri>model://{MODEL_NAME}/meshes/rotor_prop_{d}.stl</uri></mesh></geometry>
        <material><ambient>0.9 0.35 0.05 1</ambient><diffuse>0.9 0.35 0.05 1</diffuse></material>
      </visual>
    </link>
    <joint name="rotor_{idx}_joint" type="revolute">
      <parent>base_link</parent>
      <child>rotor_{idx}</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit><lower>-1e+16</lower><upper>1e+16</upper></limit>
        <dynamics><damping>{JOINT_DAMPING}</damping></dynamics>
      </axis>
    </joint>""")
        plugins.append(f"""
    <plugin filename="gz-sim-multicopter-motor-model-system"
            name="gz::sim::systems::MulticopterMotorModel">
      <robotNamespace>{MODEL_NAME}</robotNamespace>
      <jointName>rotor_{idx}_joint</jointName>
      <linkName>rotor_{idx}</linkName>
      <turningDirection>{d}</turningDirection>
      <timeConstantUp>{TIME_CONSTANT_UP}</timeConstantUp>
      <timeConstantDown>{TIME_CONSTANT_DOWN}</timeConstantDown>
      <maxRotVelocity>{MAX_ROT_VELOCITY}</maxRotVelocity>
      <motorConstant>{MOTOR_CONSTANT}</motorConstant>
      <momentConstant>{MOMENT_CONSTANT}</momentConstant>
      <commandSubTopic>command/motor_speed</commandSubTopic>
      <motorNumber>{idx}</motorNumber>
      <rotorDragCoefficient>{ROTOR_DRAG_COEFF}</rotorDragCoefficient>
      <rollingMomentCoefficient>{ROLLING_MOMENT_COEFF}</rollingMomentCoefficient>
      <rotorVelocitySlowdownSim>{ROTOR_SLOWDOWN_SIM}</rotorVelocitySlowdownSim>
      <motorType>velocity</motorType>
    </plugin>""")

    leg_collisions = "\n".join(f"""      <collision name="leg_{i}_collision">
        <pose>{lx:.4f} {ly:.4f} {lz:.4f} 0 0 0</pose>
        <geometry><cylinder><radius>0.006</radius><length>{LEG_HEIGHT:.4f}</length></cylinder></geometry>
      </collision>""" for i, (_, _, _, (lx, ly, lz)) in
        enumerate(c for c in EXTRA_COMPONENTS if c[0].startswith("опора")))
    extras = "\n".join(f"       - {n}: {m * 1000:.1f} г" for n, m, _, _ in EXTRA_COMPONENTS)
    total = M + 4 * ROTOR_MASS
    sdf = f"""<?xml version="1.0"?>
<!--
  project_quad v0.6 — цифровая модель квадрокоптера проекта.
  ФАЙЛ СГЕНЕРИРОВАН скриптом tools/generate_model.py — правьте параметры там.

  Геометрия: CAD-модель группы конструкторов (рама G4ZO x2, 5", диагональ 230,5 мм).
  Единицы СИ. Оси: X вперёд, Y влево, Z вверх. Начало — низ рамы, центр моторов.
  Порядок моторов: PX4 Quadrotor X. Компоновка: отчёт конструкторов от 16.11.2026.
  Обороты моторов в УСЛОВНЫХ рад/с (1000 = 3200 настоящих), см. генератор.

  Общая масса: {total:.3f} кг. Бортовые компоненты:
{extras}
-->
<sdf version="1.9">
  <model name="{MODEL_NAME}">
    <pose>0 0 {LEG_HEIGHT + 0.005:.3f} 0 0 0</pose>

    <link name="base_link">
{inertial_xml(M, com, I)}
{chr(10).join(visuals)}

      <!-- Коллизия: пластина рамы + аккумулятор + опоры (дрон стоит на опорах) -->
      <collision name="frame_collision">
        <pose>0 0 0.0035 0 0 0</pose>
        <geometry><box><size>0.195 0.223 0.007</size></box></geometry>
      </collision>
      <collision name="battery_collision">
        <pose>{pose(batt[3])} 0 0 0</pose>
        <geometry><box><size>0.076 0.036 0.033</size></box></geometry>
      </collision>
{leg_collisions}
{sensors_xml()}
    </link>
{''.join(rotors)}

    <!-- Моторы. Команды: топик /{MODEL_NAME}/command/motor_speed (см. tools/motors.sh) -->{''.join(plugins)}
  </model>
</sdf>
"""
    return sdf, M, com, I, total


# ------------------------- PX4 и стабилизатор --------------------------------
def vehicle_inertia(M, com):
    """Масса, ЦМ и тензор инерции всего дрона (корпус + роторы) относительно общего ЦМ."""
    _, _, I_body = combine()
    parts = [(M, np.asarray(com), I_body)]
    for _, x, y, _ in ROTORS:
        parts.append((ROTOR_MASS, np.array([x, y, ROTOR_Z + ROTOR_COM_Z]), np.diag(ROTOR_INERTIA)))
    mt = sum(m for m, _, _ in parts)
    cg = sum(m * c for m, c, _ in parts) / mt
    It = sum(shift(I, m, c - cg) for m, c, I in parts)
    return mt, cg, It


def px4_airframe(mt, K):
    w_hover = math.sqrt(mt * G / (4 * MOTOR_CONSTANT))
    thr_hover = (w_hover - PX4_IDLE_SPEED) / (MAX_ROT_VELOCITY - PX4_IDLE_SPEED)
    lines = [f"""#!/bin/sh
#
# @name project_quad (G4ZO x2 5-inch frame)
#
# @type Quadrotor
#
# GENERATED by simulation/tools/generate_model.py - edit parameters there.
# Mass {mt:.3f} kg, AE2207 1960KV motors, 5.1-inch props, PX4 Quad X.
# Install into PX4: bash simulation/tools/px4_setup.sh
# (file is ASCII-only on purpose: PX4 build parses airframe metadata)
#

. ${{R}}etc/init.d/rc.mc_defaults

PX4_SIMULATOR=${{PX4_SIMULATOR:=gz}}
PX4_GZ_WORLD=${{PX4_GZ_WORLD:=test_field}}
PX4_SIM_MODEL=${{PX4_SIM_MODEL:={MODEL_NAME}}}

param set-default SIM_GZ_EN 1

param set-default CA_AIRFRAME 0
param set-default CA_ROTOR_COUNT {len(ROTORS)}
"""]
    # PX4: оси FRD (X вперёд, Y вправо) -> PY = -y модели. KM > 0 для CCW.
    lines.append("# Rotor geometry, PX4 body frame FRD (PY = -y of the SDF model)\n")
    for idx, x, y, d in ROTORS:
        km = 0.05 if d == "ccw" else -0.05
        lines.append(f"param set-default CA_ROTOR{idx}_PX {x:.4f}\n"
                     f"param set-default CA_ROTOR{idx}_PY {-y:.4f}\n"
                     f"param set-default CA_ROTOR{idx}_KM {km:.2f}\n")
    for i in range(1, len(ROTORS) + 1):
        lines.append(f"param set-default SIM_GZ_EC_FUNC{i} {100 + i}\n"
                     f"param set-default SIM_GZ_EC_MIN{i} {PX4_IDLE_SPEED}\n"
                     f"param set-default SIM_GZ_EC_MAX{i} {MAX_ROT_VELOCITY:.0f}\n")
    lines.append(f"""
# Hover: motors at {w_hover:.0f} of {MAX_ROT_VELOCITY:.0f} scaled rad/s
# (sim speeds are scaled: 1000 = {REAL_MAX_ROT_VELOCITY:.0f} real rad/s).
# PX4 refines this in flight (hover thrust estimator).
param set-default MPC_THR_HOVER {thr_hover:.2f}

# Rate controller global gains (PX4 defaults otherwise).
# Light 5-inch frame: with K=1 roll oscillation margin is only ~2.5x
# (offline check, tools/analysis/px4_stability.py); K=0.5 gives ~5x.
# If it oscillates - decrease K, if sluggish - increase.
param set-default MC_ROLLRATE_K {K['roll']}
param set-default MC_PITCHRATE_K {K['pitch']}
param set-default MC_YAWRATE_K {K['yaw']}

param set-default NAV_DLL_ACT 2

# Simulation only: altitude from barometer, GPS altitude not fused
# (bit 1 of EKF2_GPS_CTRL off; 7 -> 5). With white GPS altitude noise in
# Gazebo the EKF shifts its global altitude reference, and mission altitude
# setpoints drifted 4.1-4.8 m instead of 5 m (flight 14.11). PX4 default is 7.
param set-default EKF2_GPS_CTRL 5
""")
    return "\n".join(lines)


def spawn_pose_in_world(root):
    """Высота появления дрона в test_field.sdf: над площадкой (верх 0,01 м) на опорах.
    <pose> в <include> мира перекрывает <pose> модели, поэтому правим его здесь."""
    import re
    path = root / "worlds" / "test_field.sdf"
    src = path.read_text(encoding="utf-8")
    z = 0.010 + LEG_HEIGHT + 0.005
    new, n = re.subn(r"(<include>\s*<uri>model://" + MODEL_NAME + r"</uri>.*?<pose>)([^<]*)(</pose>)",
                     lambda m: f"{m.group(1)}0 0 {z:.3f} 0 0 0{m.group(3)}", src, count=1, flags=re.S)
    if n and new != src:
        path.write_text(new, encoding="utf-8")
    return z


def stab_world(root):
    """Мир test_field_stab.sdf: тот же полигон + встроенный стабилизатор Gazebo."""
    spawn_pose_in_world(root)
    src = (root / "worlds" / "test_field.sdf").read_text(encoding="utf-8")
    rotors = "".join(f"""
          <rotor>
            <jointName>rotor_{idx}_joint</jointName>
            <forceConstant>{MOTOR_CONSTANT:.6e}</forceConstant>
            <momentConstant>{MOMENT_CONSTANT}</momentConstant>
            <direction>{1 if d == "ccw" else -1}</direction>
          </rotor>""" for idx, _, _, d in ROTORS)
    v3 = lambda t: " ".join(f"{x:g}" for x in t)
    plugin = f"""
      <!-- Встроенный стабилизатор Gazebo (БЕЗ PX4). Управление: tools/stab_flight.sh
           Скорость: топик /{MODEL_NAME}/cmd_vel (gz.msgs.Twist),
           включение: /{MODEL_NAME}/enable (gz.msgs.Boolean). -->
      <plugin filename="gz-sim-multicopter-control-system"
              name="gz::sim::systems::MulticopterVelocityControl">
        <robotNamespace>{MODEL_NAME}</robotNamespace>
        <commandSubTopic>cmd_vel</commandSubTopic>
        <enableSubTopic>enable</enableSubTopic>
        <comLinkName>base_link</comLinkName>
        <velocityGain>{v3(STAB_VELOCITY_GAIN)}</velocityGain>
        <attitudeGain>{v3(STAB_ATTITUDE_GAIN)}</attitudeGain>
        <angularRateGain>{v3(STAB_RATE_GAIN)}</angularRateGain>
        <maximumLinearAcceleration>{v3(STAB_MAX_ACCEL)}</maximumLinearAcceleration>
        <maximumLinearVelocity>2 2 2</maximumLinearVelocity>
        <rotorConfiguration>{rotors}
        </rotorConfiguration>
      </plugin>
    </include>"""
    marker = f"<uri>model://{MODEL_NAME}</uri>"
    i = src.index(marker)
    j = src.index("</include>", i)
    out = src[:j].rstrip() + plugin + src[j + len("</include>"):]
    out = out.replace("<sdf version=\"1.9\">",
                      "<!-- СГЕНЕРИРОВАН tools/generate_model.py из test_field.sdf + стабилизатор Gazebo -->\n"
                      "<sdf version=\"1.9\">", 1)
    (root / "worlds" / "test_field_stab.sdf").write_text(out, encoding="utf-8")


def main():
    root = Path(__file__).resolve().parent.parent
    out = root / "models" / MODEL_NAME / "model.sdf"
    sdf, M, com, I, total = build()
    out.write_text(sdf, encoding="utf-8")

    # Обороты висения с учётом смещения центра масс: передняя и задняя пары
    # моторов дают разную тягу, чтобы дрон не заваливался (без автопилота).
    cg = (M * com + 4 * ROTOR_MASS * np.array([0, 0, ROTOR_Z + ROTOR_COM_Z])) / total
    w = total * G
    f_front = w * (HALF_X + cg[0]) / (2 * HALF_X) / 2     # на один передний мотор
    f_rear = w * (HALF_X - cg[0]) / (2 * HALF_X) / 2
    speeds = [math.sqrt((f_front if x > 0 else f_rear) / MOTOR_CONSTANT) for _, x, _, _ in ROTORS]
    (root / "tools" / "hover_speeds.txt").write_text(",".join(f"{s:.0f}" for s in speeds) + "\n")

    hover = math.sqrt(total * G / (4 * MOTOR_CONSTANT))
    fmax = 4 * MOTOR_CONSTANT * MAX_ROT_VELOCITY ** 2
    print(f"Записано: {out}")
    print(f"Общая масса:                {total:.3f} кг  (корпус {M:.3f} + роторы {4 * ROTOR_MASS:.3f})")
    print(f"Центр масс корпуса:         x={com[0] * 1000:.1f}  y={com[1] * 1000:.1f}  z={com[2] * 1000:.1f} мм")
    print(f"Инерция Ixx / Iyy / Izz:    {I[0, 0]:.3e} / {I[1, 1]:.3e} / {I[2, 2]:.3e} кг·м²")
    print(f"Обороты висения:            {hover:.0f} рад/с  ({hover / MAX_ROT_VELOCITY:.0%} от максимума)")
    print(f"Висение по моторам 1-4:     {', '.join(f'{s:.0f}' for s in speeds)} рад/с (tools/hover_speeds.txt)")
    print(f"Максимальная тяга:          {fmax:.1f} Н ({fmax / G * 1000:.0f} г)")
    print(f"Тяговооружённость:          {fmax / (total * G):.1f}")
    on = [n for n, f in [("IMU", ENABLE_IMU), ("барометр", ENABLE_BARO), ("магнитометр", ENABLE_MAG),
                         ("GPS", ENABLE_GPS), (f"лидар {LIDAR_MODE}", ENABLE_LIDAR),
                         ("камера вниз", ENABLE_DOWN_CAMERA), ("FPV-камера", ENABLE_FPV_CAMERA)] if f]
    print(f"Сенсоры:                    {', '.join(on)}")

    mt, cg, It = vehicle_inertia(M, com)
    K = dict(roll=PX4_ROLLPITCH_RATE_K, pitch=PX4_ROLLPITCH_RATE_K, yaw=PX4_YAW_RATE_K)
    (root / "px4").mkdir(exist_ok=True)
    af = root / "px4" / f"{PX4_AIRFRAME_ID}_gz_{MODEL_NAME}"
    af.write_text(px4_airframe(mt, K), encoding="utf-8")
    stab_world(root)
    print(f"ЦМ всего дрона:             x={cg[0] * 1000:.1f}  y={cg[1] * 1000:.1f}  z={cg[2] * 1000:.1f} мм")
    print(f"Инерция всего дрона:        {It[0, 0]:.3e} / {It[1, 1]:.3e} / {It[2, 2]:.3e} кг·м²")
    print(f"PX4: MC_ROLL/PITCH/YAWRATE_K {K['roll']} / {K['pitch']} / {K['yaw']}  -> {af.relative_to(root)}")
    print(f"Мир со стабилизатором Gazebo: worlds/test_field_stab.sdf")


if __name__ == "__main__":
    main()
