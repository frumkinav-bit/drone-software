# drone-software

Программная часть проекта «Модульная беспилотная авиационная система» (ROS 2 Jazzy, Ubuntu 24.04, Gazebo Harmonic).

## Структура

- `src/uav_interfaces/` — сообщения и сервисы (Telemetry.msg)
- `src/uav_telemetry/`, `uav_autopilot/`, `uav_lidar/`, `uav_mission/`, `uav_bringup/` — пакеты узлов (создаются по плану)
- `docs/` — документация
- `data/test/` — тестовые данные (CSV)
- `.devcontainer/` — окружение для VS Code (Docker)

## Как запустить окружение

1. Установить Docker Desktop и расширение VS Code «Dev Containers».
2. Открыть папку проекта и выбрать «Dev Containers: Reopen in Container».
3. В терминале внутри контейнера:

```bash
colcon build            # собрать все пакеты
source install/setup.bash   # подключить собранное
ros2 interface show uav_interfaces/msg/Telemetry   # проверить сообщение
```

## Правила работы

- Ветки: `main` (стабильная, напрямую не пишем) и `feature/имя-задачи`.
- В `main` только через pull request с проверкой одним из программистов.
- Сообщения коммитов короткие и по делу.
- Пакеты и узлы называем в snake_case, темы начинаются с `/uav/`.
