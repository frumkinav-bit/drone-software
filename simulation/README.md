# Цифровая модель квадрокоптера — симуляция

Группа моделирования: Кузнецова Марина, Крюков Константин
Версия 0.6 (к контрольной точке 23 ноября 2026: телеметрия и данные лидара в ROS 2;
модель по компоновке конструкторов от 16.11)

## Окружение

| Компонент | Версия |
|---|---|
| ОС | Windows 11 + WSL2, Ubuntu 24.04 |
| Симулятор | Gazebo Harmonic (`gz-harmonic`) |
| Формат описания | SDF 1.9 |
| Автопилот | PX4 v1.17.0 (Pixhawk 6C Mini), в симуляции — PX4 SITL |
| ROS 2 | Jazzy (как в репозитории drone-software) |

## Состав

```
simulation/
├── models/project_quad/        модель квадрокоптера
│   ├── model.config
│   ├── model.sdf               (генерируется скриптом, руками не править)
│   └── meshes/                 CAD-геометрия от конструкторов (STL)
├── worlds/
│   ├── test_field.sdf          тестовый полигон (для PX4 и ручной проверки)
│   └── test_field_stab.sdf     тот же полигон + стабилизатор Gazebo (генерируется)
├── px4/
│   └── 4050_gz_project_quad    настройки PX4 под нашу раму (генерируется)
├── tools/
│   ├── generate_model.py       генератор: ВСЕ параметры здесь
│   ├── install_px4.sh          установка PX4 v1.17.0 (один раз)
│   ├── px4_setup.sh            подключить настройки рамы к PX4
│   ├── px4_run.sh              запустить PX4, подключённый к Gazebo
│   ├── plot_flight.py          графики полёта из лога PX4 → docs/
│   ├── route.py                тестовый маршрут: точки, высота, скорость
│   ├── mission.py              загрузить маршрут в PX4 и пролететь его
│   ├── plot_mission.py         траектория маршрута и точность → docs/
│   ├── install_ros.sh          установка ROS 2 Jazzy, ros_gz, Micro XRCE-DDS Agent, px4_msgs
│   ├── ros_env.sh              окружение ROS 2 (source)
│   ├── ros_run.sh              мосты PX4 → ROS 2 и Gazebo → ROS 2
│   ├── check_ros.sh            проверка тем и частот в ROS 2
│   ├── lidar_check.py          проверка данных лидара в ROS 2
│   ├── record.sh               запись в rosbag2 (MCAP)
│   ├── plot_bag.py             картинка по записи: трек, лидар на карте → docs/
│   ├── stab_flight.sh          полёт со стабилизатором Gazebo (без PX4)
│   ├── motors.sh               ручное управление моторами
│   ├── check_sensors.sh        проверка всех сенсоров
│   ├── hover_speeds.txt        обороты висения (пишет генератор)
│   └── analysis/               расчётные проверки стабилизации (Python, без Gazebo)
├── ros/ros_gz_bridge.yaml      мост Gazebo → ROS 2
├── COLCON_IGNORE               папка не является ROS-пакетом
└── docs/
    ├── telemetry_lidar.md      отчёт: телеметрия и лидар в ROS 2 (23 ноября)
    ├── bag_flight_*.png        запись полёта: трек и лидар на карте
    ├── mission.md              отчёт: тестовый маршрут (16 ноября)
    ├── mission_17_23_56.png    маршрут, полёт 1
    ├── mission_18_39_59.png    маршрут, полёт 2 (итоговый)
    ├── gazebo_mission.png      дрон у точки маршрута
    ├── stabilization.md        отчёт: запуск и стабилизация (9 ноября)
    ├── physics_and_sensors.md  отчёт: физика и сенсоры (2 ноября)
    ├── conventions.md          соглашения по цифровой модели
    ├── flight_19_41_59.png     полёт 1 с PX4 (старый шум GPS)
    ├── flight_20_00_08.png     полёт 2 с PX4 (итоговый)
    ├── gazebo_hover.png        дрон в висении (скриншот Gazebo)
    ├── px4_stability.png       расчёт: стабилизация PX4
    ├── gz_stabilizer.png       расчёт: стабилизатор Gazebo
    └── project_quad_layout.png схема модели (компоновка 8.11)
```

## Запуск модели

```bash
cd ~/simulation
gz sim worlds/test_field.sdf
```

(переменная `GZ_SIM_RESOURCE_PATH=$HOME/simulation/models` уже прописана в `~/.bashrc`)

Нажмите ▶. Дрон стоит на площадке с буквой H, вокруг видны лучи лидара.
Флаг `-r` (`gz sim -r ...`) запускает симуляцию сразу, без ▶.

## Полёт с PX4 (основной вариант)

### Установка — один раз, 40–90 минут

```bash
bash ~/simulation/tools/install_px4.sh
```

Скачивает PX4 v1.17.0 в `~/PX4-Autopilot`, ставит зависимости (спросит пароль),
собирает PX4 и подключает настройки нашей рамы. После — закрыть все окна Ubuntu
и открыть заново.

### Полёт

Окно 1 — Gazebo:

```bash
cd ~/simulation && gz sim -r worlds/test_field.sdf
```

Окно 2 — PX4:

```bash
bash ~/simulation/tools/px4_run.sh
```

Без QGroundControl PX4 откажется взлетать (`Preflight Fail: No connection to the GCS`):
в airframe задан возврат домой при потере связи с наземной станцией. Для полёта без QGC
в консоли `pxh>` один раз за запуск введите:

```
param set NAV_DLL_ACT 0
```

Дождитесь строки `INFO [commander] Ready for takeoff!` и в консоли `pxh>` введите:

```
commander takeoff
```

Дрон взлетает на 2,5 м (над высотой, которую PX4 считал на земле) и висит. Через 30–60 секунд:

```
commander land
```

После посадки — графики для отчёта (окно 3 или после `shutdown` в окне 2):

```bash
python3 ~/simulation/tools/plot_flight.py
```

Картинка сохраняется в `docs/flight_<время>.png`, в терминале — сводка: высота
висения и её разброс, максимальные крен и тангаж, уход в сторону.

Если параметры модели поменялись (`generate_model.py`) — `bash tools/px4_setup.sh`.
После обновления до версии 0.6 (масса 810 г) это нужно сделать один раз.

## Тестовый маршрут WP1–WP4

Маршрут: взлёт на 5 м → WP1 (10; 0) → WP2 (10; 10) → WP3 (−3; 10) → WP4 (−3; 0) →
домой → посадка. Координаты — в метрах мира Gazebo (X — восток, Y — север), точки
совпадают с оранжевыми маркерами wp1…wp4. Скорость 3 м/с, в каждой точке задержка 2 с.
Всё задаётся в `tools/route.py`.

Один раз установить библиотеку MAVLink:

```bash
pip install --break-system-packages pymavlink
```

Полёт — три окна Ubuntu:

```bash
cd ~/simulation && gz sim -r worlds/test_field.sdf      # окно 1
bash ~/simulation/tools/px4_run.sh                      # окно 2, затем в pxh>:
                                                        #   param set NAV_DLL_ACT 0
python3 ~/simulation/tools/mission.py                   # окно 3, после "Ready for takeoff!"
```

`mission.py` загружает маршрут в PX4, включает режим Mission и моторы, печатает
достигнутые точки и заканчивает работу после посадки. Затем графики:

```bash
python3 ~/simulation/tools/plot_mission.py
```

Картинка `docs/mission_<время>.png` (вид сверху с маршрутом, высота и уставка PX4,
скорость), в терминале — время маршрута, отклонение от линии маршрута, промах по точкам,
высота. Результаты 14 ноября — `docs/mission.md`.

В симуляции высота берётся только с барометра (`EKF2_GPS_CTRL = 5` в настройках рамы):
с высотой по GPS уставка маршрута плавала на 0,2–0,9 м (см. `docs/mission.md`). Если
PX4 был установлен до версии 0.5, обновите настройки рамы: `bash tools/px4_setup.sh`.

`python3 tools/mission.py --upload-only` только загружает маршрут; запустить можно из
QGroundControl или в pxh: `commander mode auto:mission`, затем `commander arm`.

**QGroundControl** (по желанию — карта с маршрутом и треком): установить на Windows с
qgroundcontrol.com и запустить, пока работает PX4. Он подключится сам (UDP 14550; нужен
режим сети WSL `networkingMode=mirrored`). Когда QGC подключён, `NAV_DLL_ACT 0` не нужен.

## Полёт со стабилизатором Gazebo (запасной вариант, без PX4)

Встроенный регулятор Gazebo держит заданную скорость и ровное положение.
PX4 для него не нужен.

```bash
cd ~/simulation && gz sim -r worlds/test_field_stab.sdf     # окно 1
bash ~/simulation/tools/stab_flight.sh                       # окно 2
```

Сценарий: подъём ~2 м → висение 20 с → посадка. По шагам: `stab_flight.sh up`,
`hover`, `down`, `off`. В этом мире моторами управляет стабилизатор, поэтому
`motors.sh` и PX4 там не работают — для PX4 нужен `test_field.sdf`.

## Проверка сенсоров

Во **втором** окне Ubuntu, пока симуляция идёт:

```bash
cd ~/simulation && bash tools/check_sensors.sh
```

Напротив каждого сенсора должно быть `OK`.

Изображение камеры: в Gazebo меню ⋮ (справа вверху) → **Image Display** → выбрать
топик `/project_quad/camera_down/image`.

## Проверка моторов (без автопилота)

```bash
bash tools/motors.sh hover   # обороты висения
bash tools/motors.sh up      # взлёт
bash tools/motors.sh stop    # остановить
```

Стабилизации здесь нет: дрон со временем уплывает и может перевернуться.
Сброс сцены — кнопка ↻ внизу слева.

### Обороты моторов — «условные»

PX4 v1.17 не может послать мотору в Gazebo команду больше 1000
(`SIM_GZ_EC_MAX` ограничен 0…1000), а настоящий AE2207 на 6S крутится до
~3200 рад/с. Поэтому в модели обороты масштабированы: **1000 условных рад/с =
3200 настоящих**, коэффициент тяги увеличен в 3,2² раза. Тяга, моменты и
поведение дрона те же, меняются только числа в командах. Висение — 337 условных
(≈ 1080 настоящих) рад/с.

## Телеметрия и лидар в ROS 2

Установка — один раз, 30–60 минут (спросит пароль):

```bash
bash ~/simulation/tools/install_ros.sh
```

Ставит ROS 2 Jazzy (`ros-base`), мост `ros_gz`, Micro XRCE-DDS Agent v2.4.3 и собирает
`px4_msgs` (ветка `release/1.17`) в `~/ros2_ws`. ROS 2 в `~/.bashrc` не прописывается:
окна с Gazebo и PX4 работают как раньше, ROS 2 подключается командой
`source ~/simulation/tools/ros_env.sh` или внутри скриптов.

Запуск — четыре окна Ubuntu:

```bash
cd ~/simulation && gz sim -r worlds/test_field.sdf   # окно 1: Gazebo
bash ~/simulation/tools/px4_run.sh                   # окно 2: PX4 (pxh> param set NAV_DLL_ACT 0)
bash ~/simulation/tools/ros_run.sh                   # окно 3: мосты в ROS 2
bash ~/simulation/tools/check_ros.sh                 # окно 4: проверка
```

`ros_run.sh` запускает Micro XRCE-DDS Agent (PX4 → темы `/fmu/out/...`) и `ros_gz_bridge`
(лидар `/lidar/scan`, `/lidar/points`, камера `/camera/down/image`, `/camera/down/camera_info`,
`/clock`). `check_ros.sh` печатает список тем PX4 и частоты основных тем.
`python3 ~/simulation/tools/lidar_check.py` (после `source ~/simulation/tools/ros_env.sh`)
показывает данные лидара и сектора, закрытые деталями дрона.

Запись полёта: в окне 4 `bash ~/simulation/tools/record.sh`, в окне 5 — маршрут
`python3 ~/simulation/tools/mission.py`, после посадки Ctrl+C в окне 4. Запись — в
`~/simulation/bags/flight_<время>/` (MCAP): `/clock`, `/lidar/scan`, `/lidar/points` и все
`/fmu/out/...`; с `--camera` — ещё изображение камеры (около 40 МБ/с).

Картинка по записи (трек по телеметрии, точки лидара на карте полигона, высота и число
лучей с данными): `source ~/simulation/tools/ros_env.sh && python3 ~/simulation/tools/plot_bag.py`.

## Связь с ROS 2

Файл `ros/ros_gz_bridge.yaml` — мост Gazebo → ROS 2 для лидара, камеры и часов.
Мост запускается на том же компьютере, что и Gazebo (нужен пакет `ros-jazzy-ros-gz`).
Файл `COLCON_IGNORE` нужен, чтобы `colcon build` в репозитории drone-software не трогал
эту папку. Подключение `autopilot_bridge` программистов к PX4 SITL — по
`docs/autopilot_interface.md` в drone-software (нужен Micro XRCE-DDS Agent).

## Изменение параметров модели

1. Откройте `tools/generate_model.py`, поменяйте значения в блоке ПАРАМЕТРЫ
   (массы компонентов, моторы, сенсоры, шумы, коэффициенты PX4).
2. `python3 tools/generate_model.py` — перезапишет модель, мир со стабилизатором
   и настройки PX4, выведет массу, инерцию, обороты висения.
3. Если используете PX4: `bash tools/px4_setup.sh`.
4. Перезапустите Gazebo.

Расчётные проверки после изменений (по 10 секунд, без Gazebo):

```bash
python3 tools/analysis/px4_stability.py    # стабилизация PX4 → docs/px4_stability.png
python3 tools/analysis/gz_stabilizer.py    # стабилизатор Gazebo → docs/gz_stabilizer.png
```

## Модель project_quad v0.6

- Геометрия — CAD-модель конструкторов: рама G4ZO x2 (5", диагональ 230,5 мм,
  Thingiverse 7014680, лицензия CC BY-NC-SA), моторы AE2207, винты 5,1".
- Компоновка — отчёт конструкторов от 16.11.2026: «вперёд» — сторона FPV-камеры,
  VTX сзади, Pixhawk 6C Mini над регулятором 4-в-1, аккумулятор на X = −8 мм,
  лидар на кронштейне над ним (X = −24 мм), GPS на мачте спереди (117,5 мм),
  модуль питания под верхней пластиной сзади, бортовой компьютер и радиомодем
  под основанием, камера вниз между ними.
- **Опоры 30 мм** под моторами: дрон стоит на опорах, нижняя точка Z = −30 мм
  (параметр `LEG_HEIGHT` в `generate_model.py`; если опоры удлинят до 40 мм — 0.040).
- Масса **0,810 кг** (в том числе 37 г креплений), центр масс X ≈ 0 (совпадает с
  центром тяги), Z = 33,3 мм. Инерция всего дрона: 2,94 / 3,64 / 3,82 ·10⁻³ кг·м².
- Порядок моторов **PX4 Quad X**:

  | Мотор PX4 | Звено | Положение | Вращение |
  |---|---|---|---|
  | 1 | `rotor_0` | передний правый | CCW |
  | 2 | `rotor_1` | задний левый | CCW |
  | 3 | `rotor_2` | передний левый | CW |
  | 4 | `rotor_3` | задний правый | CW |

- Сенсоры: IMU, барометр, магнитометр, GPS, тестовый 2D-лидар (плоскость 99,5 мм),
  камера вниз 1280×720 под основанием. Подробности, шумы и источники — в `docs/physics_and_sensors.md`.

## Мир test_field

- Площадка 200×200 м, зона взлёта 2×2 м в начале координат.
- Препятствия для лидара: стена, здание, две колонны.
- Маркеры контрольных точек WP1–WP4 на высоте 5 м.
- «Машины» и «люди» под маршрутом — для проверки камеры.
- Магнитное поле и географическая привязка — Москва.

## План

| Дата | Что добавляется |
|---|---|
| ~~26 октября~~ | базовая модель и мир ✔ |
| ~~2 ноября~~ | CAD-геометрия, физика, тестовые сенсоры ✔ |
| ~~9 ноября~~ | компоновка 8.11, PX4 SITL, стабилизация (PX4 и запасной стабилизатор Gazebo) ✔ |
| ~~16 ноября~~ | тестовый маршрут по WP1–WP4 (`mission.py`, режим Mission) ✔ |
| 23 ноября | телеметрия PX4 и лидар в ROS 2, запись rosbag2; компоновка 16.11 |
