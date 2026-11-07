#!/usr/bin/env python3
"""
Генератор SDF-модели квадрокоптера project_quad (версия 0.2).

Геометрия — CAD-модель рамы группы конструкторов (меши в models/project_quad/meshes).
Здесь задаются масса, инерция, моторы и сенсоры. Чтобы что-то поменять:

    1. исправьте значения в блоке ПАРАМЕТРЫ;
    2. python3 tools/generate_model.py
    3. перезапустите Gazebo.

Скрипт перезапишет models/project_quad/model.sdf и выведет сводку
(масса, центр масс, инерция, обороты висения, тяговооружённость).
"""
import math
from pathlib import Path

import numpy as np

# ============================== ПАРАМЕТРЫ ===================================
# Все величины в СИ: м, кг, с, рад. Оси модели: X вперёд, Y влево, Z вверх.
# Начало координат модели — центр между осями моторов, на нижней плоскости рамы.

MODEL_NAME = "project_quad"

# --- Базовая сборка из CAD (рама, моторы-статоры, 4in1 ESC, VTX, приёмник,
#     аккумулятор 6S 1250 мА·ч, FPV-камера). Масса и инерция посчитаны
#     конструкторами по мешам (модель g4zo_x2_fpv5, В. Артём).
BASE_MASS = 0.444
BASE_COM = (-0.00529, 0.0, 0.03015)
BASE_INERTIA = np.array([      # относительно BASE_COM
    [1.579424e-03, 1.868506e-08, 2.868936e-04],
    [1.868506e-08, 2.113811e-03, 1.835437e-09],
    [2.868936e-04, 1.835437e-09, 2.121953e-03],
])

# --- Компоненты, которых нет в CAD-сборке (оценки! уточнить у электронщиков).
#     (имя, масса кг, размеры box м, положение центра м)
EXTRA_COMPONENTS = [
    ("Pixhawk 6C Mini",                0.0392, (0.0533, 0.039, 0.0162), (-0.010, 0.0, 0.020)),
    ("бортовой компьютер (оценка)",    0.045, (0.065, 0.030, 0.010), (-0.040, 0.0, 0.016)),
    ("радиомодем SiK 433 МГц",         0.025, (0.050, 0.025, 0.010), (-0.060, 0.0, 0.020)),
    ("модуль питания и провода",       0.030, (0.040, 0.030, 0.010), ( 0.000, 0.0, 0.012)),
    ("GPS Holybro M9N + мачта",        0.040, (0.050, 0.050, 0.015), (-0.065, 0.0, 0.095)),
    ("лидар (тестовый, модель не выбрана)", 0.050, (0.040, 0.040, 0.030), (0.045, 0.0, 0.070)),
]

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

# --- Модель мотора AE2207 V2 1960KV + Gemfan 51499 на 6S (оценка конструкторов):
#     тяга F = MOTOR_CONSTANT * omega^2; 3200 рад/с -> 16,7 Н (~1700 г)
MOTOR_CONSTANT = 1.63e-06
MOMENT_CONSTANT = 0.016
MAX_ROT_VELOCITY = 3200.0
TIME_CONSTANT_UP = 0.0125
TIME_CONSTANT_DOWN = 0.025
ROTOR_DRAG_COEFF = 1.0e-06
ROLLING_MOMENT_COEFF = 1.0e-08
ROTOR_SLOWDOWN_SIM = 10.0
JOINT_DAMPING = 2.0e-06

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
GPS_POS_NOISE_H_M = 0.7      # м. Gazebo применяет этот шум к широте/долготе
GPS_POS_NOISE_H = GPS_POS_NOISE_H_M / 111320.0   # в ГРАДУСАХ — поэтому пересчёт
GPS_POS_NOISE_V = 1.2        # м
GPS_VEL_NOISE_H = 0.05       # м/с
GPS_VEL_NOISE_V = 0.10       # м/с
LIDAR_NOISE = 0.01           # м
CAMERA_NOISE = 0.007

# Положение сенсоров (x, y, z, roll, pitch, yaw)
GPS_POSE = (-0.065, 0.0, 0.103, 0, 0, 0)
LIDAR_POSE = (0.045, 0.0, 0.085, 0, 0, 0)
DOWN_CAMERA_POSE = (0.030, 0.0, 0.001, 0, math.pi / 2, 0)   # смотрит вниз
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
        mesh_visual("esc_fc", "esc_fc.stl", "0.10 0.35 0.20 1"),
        mesh_visual("esc_cap", "esc_cap.stl", "0.08 0.08 0.08 1"),
        mesh_visual("vtx", "vtx.stl", "0.12 0.12 0.12 1"),
        mesh_visual("rx", "rx.stl", "0.08 0.08 0.08 1"),
        mesh_visual("battery", "battery.stl", "0.12 0.24 0.55 1"),
        mesh_visual("camera", "camera.stl", "0.06 0.06 0.06 1"),
        mesh_visual("motor_stators", "motor_stators.stl", "0.63 0.63 0.67 1"),
    ]
    gx, gy, gz = GPS_POSE[:3]
    visuals.append(f"""      <!-- Заглушки компонентов, которых пока нет в CAD -->
      <visual name="gps_mast">
        <pose>{gx:.4f} {gy:.4f} {(TOP_PLATE_Z + gz - 0.0075) / 2:.4f} 0 0 0</pose>
        <geometry><cylinder><radius>0.004</radius><length>{gz - 0.0075 - TOP_PLATE_Z:.4f}</length></cylinder></geometry>
        <material><ambient>0.3 0.3 0.3 1</ambient><diffuse>0.3 0.3 0.3 1</diffuse></material>
      </visual>
      <visual name="gps_puck">
        <pose>{gx:.4f} {gy:.4f} {gz:.4f} 0 0 0</pose>
        <geometry><cylinder><radius>0.025</radius><length>0.015</length></cylinder></geometry>
        <material><ambient>0.9 0.9 0.9 1</ambient><diffuse>0.9 0.9 0.9 1</diffuse></material>
      </visual>""")
    if ENABLE_LIDAR:
        lx, ly, lz = LIDAR_POSE[:3]
        visuals.append(f"""      <visual name="lidar_body">
        <pose>{lx:.4f} {ly:.4f} {(TOP_PLATE_Z + lz - 0.004) / 2:.4f} 0 0 0</pose>
        <geometry><cylinder><radius>0.018</radius><length>{lz - 0.004 - TOP_PLATE_Z:.4f}</length></cylinder></geometry>
        <material><ambient>0.9 0.75 0.1 1</ambient><diffuse>0.9 0.75 0.1 1</diffuse></material>
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

    extras = "\n".join(f"       - {n}: {m * 1000:.0f} г" for n, m, _, _ in EXTRA_COMPONENTS)
    total = M + 4 * ROTOR_MASS
    sdf = f"""<?xml version="1.0"?>
<!--
  project_quad v0.2 — цифровая модель квадрокоптера проекта.
  ФАЙЛ СГЕНЕРИРОВАН скриптом tools/generate_model.py — правьте параметры там.

  Геометрия: CAD-модель группы конструкторов (рама G4ZO x2, 5", диагональ 230,5 мм).
  Единицы СИ. Оси: X вперёд, Y влево, Z вверх. Начало — низ рамы, центр моторов.
  Порядок моторов: PX4 Quadrotor X.

  Общая масса: {total:.3f} кг. Из них оценки (нет в CAD):
{extras}
-->
<sdf version="1.9">
  <model name="{MODEL_NAME}">
    <pose>0 0 0.02 0 0 0</pose>

    <link name="base_link">
{inertial_xml(M, com, I)}
{chr(10).join(visuals)}

      <!-- Коллизия: пластина рамы + аккумулятор -->
      <collision name="frame_collision">
        <pose>0 0 0.0035 0 0 0</pose>
        <geometry><box><size>0.195 0.223 0.007</size></box></geometry>
      </collision>
      <collision name="battery_collision">
        <pose>-0.0225 0 0.0525 0 0 0</pose>
        <geometry><box><size>0.076 0.036 0.033</size></box></geometry>
      </collision>
{sensors_xml()}
    </link>
{''.join(rotors)}

    <!-- Моторы. Команды: топик /{MODEL_NAME}/command/motor_speed (см. tools/motors.sh) -->{''.join(plugins)}
  </model>
</sdf>
"""
    return sdf, M, com, I, total


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


if __name__ == "__main__":
    main()
