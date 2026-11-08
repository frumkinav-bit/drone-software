# Интерфейс с автопилотом

Описание того, как наше ПО обменивается данными с автопилотом PX4 (контроллер Pixhawk 6C Mini).
Версии для всей команды: Ubuntu 24.04, ROS 2 Jazzy, PX4 v1.17.0, `px4_msgs` из ветки `release/1.17`,
Micro XRCE-DDS Agent v2.4.3 (решение от 6 ноября, см. [decisions.md](decisions.md)).
Настройка самого автопилота описана в документе группы автопилота.

## Схема

```
PX4 (автопилот или SITL)
   │  uXRCE-DDS (UDP, порт 8888 в симуляции; TELEM2, 921600 на реальном борте)
   ▼
Micro XRCE-DDS Agent
   │  темы /fmu/out/...                      темы /fmu/in/...
   ▼                                              ▲
┌──────────────────── autopilot_bridge ───────────────────┐
│  PX4 → наши сообщения        наши команды → PX4         │
└──────┬──────────────────────────────┬───────────────────┘
       │ /uav/raw_state (RawState)    │ /uav/command (Command)
       ▼                              │
 telemetry_node → /uav/telemetry → telemetry_logger, diagnostics_node
```

Мост — единственное место, где наше ПО знает про PX4. Остальные узлы работают только с нашими
сообщениями `RawState`, `Telemetry` и `Command`, поэтому их можно проверять и без автопилота
(тестовые данные из `test_data_player`).

## Два режима работы моста

Режим задаётся параметром `source`.

| Режим | Что делает | Когда использовать |
| --- | --- | --- |
| `stub` (по умолчанию) | принимает команды из `/uav/command`, проверяет их и пишет в журнал; автопилот не нужен | проверка в контейнере |
| `px4` | читает темы PX4, публикует `/uav/raw_state`, отправляет команды в `/fmu/in/vehicle_command` | симуляция и реальный борт (ноутбук группы моделирования) |

Режим `px4` написан по документации PX4, но ещё не проверен с настоящим PX4: см. раздел «Что проверить».

## Чтение: темы PX4 → поля RawState

Подписка на темы PX4 идёт с QoS `best effort` (`qos_profile_sensor_data`), иначе данные не приходят.
Имена тем можно поменять параметрами узла (у разных версий PX4 к именам добавляются суффиксы `_v1`, `_v2`:
настоящие имена показывает `ros2 topic list`).

| Поле RawState | Откуда берём | Как считаем |
| --- | --- | --- |
| `latitude`, `longitude` | `/fmu/out/vehicle_global_position` (`lat`, `lon`) | как есть, градусы WGS84 |
| `altitude_m` | `/fmu/out/vehicle_local_position` (`z`) | `−z`: в PX4 ось z вниз, высота над точкой взлёта |
| `speed_mps` | `/fmu/out/vehicle_local_position` (`vx`, `vy`) | `√(vx² + vy²)`, скорость над землёй |
| `roll_deg`, `pitch_deg`, `yaw_deg` | `/fmu/out/vehicle_attitude` (`q`) | кватернион → углы, градусы |
| `battery_voltage_v` | `/fmu/out/battery_status` (`voltage_v`) | как есть |
| `battery_percent` | `/fmu/out/battery_status` (`remaining`, 0…1) | ×100, округляем; `-1`, если неизвестно |
| `link_state` | `/fmu/out/failsafe_flags` | см. ниже |
| `flight_mode` | `/fmu/out/vehicle_status` (`nav_state`) | см. таблицу ниже |
| `alerts` | — | пока пустой список |

Сообщение `RawState` публикуется с частотой `rate_hz` (по умолчанию 10 Гц), когда уже получены
статус, позиция и ориентация. Пока их нет, мост пишет «Жду данные от PX4».

### Режимы полёта

Manual и Stabilized у мультикоптера PX4 считаем одним режимом `STABILIZED`.

| `nav_state` PX4 | `flight_mode` у нас |
| --- | --- |
| 0 (Manual), 15 (Stabilized) | `STABILIZED` |
| 1 | `ALTITUDE` |
| 2 | `POSITION` |
| 3 | `MISSION` |
| 4 | `HOLD` |
| 5 | `RTL` |
| 8, 9, 12, 18 | `LAND` (посадка и аварийная посадка) |
| 14 | `OFFBOARD` |
| 17 | `TAKEOFF` |
| любой другой | `UNKNOWN` |

### Состояние связи (`link_state`)

| Значение | Условие |
| --- | --- |
| 0, нет связи | PX4 поднял флаг `manual_control_signal_lost` (потеря пульта) или `gcs_connection_lost` (потеря наземной станции). Время срабатывания задают параметры PX4 `COM_RC_LOSS_T` (0,5 с) и `COM_DL_LOSS_T` (10 с) |
| 1, слабая | потери нет, но уровень сигнала `rssi` ниже порога `weak_rssi_percent` (по умолчанию 30%) |
| 2, нормальная | во всех остальных случаях |

Порога «слабый сигнал» в PX4 нет, его задаём сами. Пока уровень `rssi` в мост не подключён, состояние 1 не выдаётся.
В симуляции без пульта PX4 считает пульт потерянным, тогда запускайте с `consider_rc_loss:=false`:
флаг пульта игнорируется, а потеря наземной станции учитывается.

### Координаты, курс и высота

Совпадает с `simulation/docs/conventions.md` (раздел 2а).

- Курс `yaw_deg`: 0 = север, по часовой стрелке, диапазон 0…360. Так же считает PX4, поэтому берём как есть.
  Если курс считается из данных Gazebo или ROS (система ENU), то `yaw_deg = 90° − yaw_ENU`.
- Высота `altitude_m`: над точкой взлёта. Абсолютная высота в симуляции = 150 м + `altitude_m`.
- Домашняя точка мира `test_field`: 55,7818° с.ш., 37,7108° в.д.; на старте дрон смотрит на восток
  (в PX4 курс на старте 90°).

## Команды: `/uav/command` → PX4

Сообщение `uav_interfaces/msg/Command`: поле `command` (код) и, для `SET_MODE`, поле `mode`.

| `command` | Код | Команда PX4 (`VehicleCommand.command`) | Параметры |
| --- | --- | --- | --- |
| `ARM` | 0 | 400, `COMPONENT_ARM_DISARM` | `param1 = 1` |
| `DISARM` | 1 | 400 | `param1 = 0` |
| `TAKEOFF` | 2 | 22, `NAV_TAKEOFF` | высота по умолчанию из PX4 (`MIS_TAKEOFF_ALT`) |
| `LAND` | 3 | 21, `NAV_LAND` | — |
| `RTL` | 4 | 20, `NAV_RETURN_TO_LAUNCH` | — |
| `SET_MODE` | 5 | 176, `DO_SET_MODE` | `param1 = 1`, `param2` = основной режим, `param3` = подрежим |

Режимы для `SET_MODE` (поле `mode`):

| `mode` | Основной режим PX4 | Подрежим |
| --- | --- | --- |
| `STABILIZED` | 7 | 0 |
| `ALTITUDE` | 2 | 0 |
| `POSITION` | 3 | 0 |
| `OFFBOARD` | 6 | 0 |
| `TAKEOFF` | 4 | 2 |
| `HOLD` | 4 | 3 |
| `MISSION` | 4 | 4 |
| `RTL` | 4 | 5 |
| `LAND` | 4 | 6 |

Неиспользуемые параметры команды ставятся в `NaN`, тогда PX4 берёт значения по умолчанию.
Неизвестная команда или режим отклоняются с сообщением об ошибке в журнале, автопилоту они не уходят.
Команды в PX4 уходят с `target_system = 1`, `target_component = 1`.

Примеры (из терминала, после `source install/setup.bash`):

```bash
ros2 topic pub --once /uav/command uav_interfaces/msg/Command "{command: 2}"                 # взлёт
ros2 topic pub --once /uav/command uav_interfaces/msg/Command "{command: 5, mode: 'HOLD'}"   # режим HOLD
```

### Что не делается через ROS 2

- **Загрузка маршрута.** Миссия по uXRCE-DDS не загружается, она идёт по MAVLink. Маршрут загружаем в
  QGroundControl и запускаем режимом `MISSION` (командой `SET_MODE` с `mode: 'MISSION'`).
- **Движение из ROS 2 (Offboard).** Нужны сообщения `offboard_control_mode` и `trajectory_setpoint`
  не реже 2 Гц до переключения в режим Offboard. Пока не реализовано, входит в этап 30 ноября.
- **Высота взлёта.** Командой задаётся только факт взлёта; высота берётся из параметра PX4 `MIS_TAKEOFF_ALT`.

## Параметры узла `autopilot_bridge`

| Параметр | По умолчанию | Назначение |
| --- | --- | --- |
| `source` | `stub` | `stub` или `px4` |
| `rate_hz` | `10.0` | частота публикации `/uav/raw_state` |
| `consider_rc_loss` | `true` | учитывать потерю пульта как потерю связи |
| `weak_rssi_percent` | `30` | ниже этого уровня сигнала связь считается слабой |
| `topic_vehicle_status`, `topic_battery`, `topic_local_position`, `topic_global_position`, `topic_attitude`, `topic_failsafe`, `topic_command` | `/fmu/out/...`, `/fmu/in/vehicle_command` | имена тем PX4 |

Узел `diagnostics_node` принимает параметры `battery_warn_percent` (15) и `battery_error_percent` (7).
Значения по умолчанию совпадают с настройками PX4 по умолчанию (`BAT_LOW_THR` 15%, `BAT_CRIT_THR` 7%).
После выбора аккумулятора их нужно уточнить у инженера по автопилоту.

## Как запускать

### Заглушка (в контейнере)

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch uav_bringup telemetry_demo.launch.py
```

Во втором терминале можно послать команду; в первом появится строка «Команда принята (заглушка…)»:

```bash
source install/setup.bash
ros2 topic pub --once /uav/command uav_interfaces/msg/Command "{command: 3}"
```

### С PX4 (ноутбук группы моделирования, WSL2)

1. Клонировать репозиторий и `px4_msgs` строго нужной ветки (папка `src/px4_msgs` исключена из Git):

   ```bash
   git clone https://github.com/frumkinav-bit/drone-software.git && cd drone-software
   git clone -b release/1.17 https://github.com/PX4/px4_msgs.git src/px4_msgs
   colcon build --symlink-install
   source install/setup.bash
   ```

2. Запустить симуляцию (Gazebo + PX4 SITL v1.17.0) и `MicroXRCEAgent udp4 -p 8888` по инструкции групп
   моделирования и автопилота. Проверить, что `ros2 topic list` показывает темы `/fmu/out/...`.
3. Запустить мост и остальные узлы:

   ```bash
   ros2 launch uav_bringup autopilot_px4.launch.py use_sim_time:=true consider_rc_loss:=false
   ```

   `use_sim_time:=true` нужен, если мост Gazebo → ROS 2 (`simulation/ros/ros_gz_bridge.yaml`) публикует `/clock`.
   В журнале телеметрии тогда будет время симуляции, а не настоящее.

## Что проверить с настоящим PX4

Режим `px4` и перевод данных написаны по документации и не проверены с PX4 v1.17.0. При первом запуске нужно:

1. Сверить имена тем: `ros2 topic list | grep fmu`. Если есть суффиксы (`_v1` и т. п.), передать имена параметрами.
2. Сверить номера `nav_state` с файлом `VehicleStatus.msg` из `px4_msgs` (ветка `release/1.17`) и поправить `NAV_STATE_TO_MODE` в `px4_mapping.py`.
3. Сверить имена полей `FailsafeFlags` (`manual_control_signal_lost`, `gcs_connection_lost`) и `BatteryStatus` (`remaining`, `voltage_v`).
4. Проверить команды: `ARM`, `TAKEOFF`, `LAND`, `RTL`, `SET_MODE` на SITL.
5. Проверить, что `link_state` не залипает в 0 в симуляции без пульта (при необходимости `consider_rc_loss:=false`).
6. Убедиться, что пакет `px4_msgs` собирается в WSL2 одной версией с прошивкой PX4.

## Что проверено

- Перевод данных и команд (`px4_mapping.py`) проверен автоматическими тестами: `src/uav_autopilot/test/test_px4_mapping.py`.
- Режим `stub` и диагностика проверяются запуском `telemetry_demo.launch.py` в контейнере.
