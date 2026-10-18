# Архитектура ПО БПЛА

Программа делится на независимые ROS 2-узлы. Узлы общаются только через темы (topics) и сервисы (services).

| Узел | Назначение | Публикует | Подписан на | Нужен к |
| --- | --- | --- | --- | --- |
| `test_data_player` | читает тестовые данные (CSV), имитирует автопилот | `/uav/raw_state` | — | 2 ноября |
| `autopilot_bridge` | связь с автопилотом, перевод данных в наши сообщения | `/uav/raw_state` | `/uav/command` | 9 ноября |
| `telemetry_node` | собирает данные в одно сообщение телеметрии | `/uav/telemetry` | `/uav/raw_state`, `/uav/lidar_status` | 2–6 ноября |
| `telemetry_logger` | пишет журнал (CSV, rosbag2) | — | `/uav/telemetry` | 16 ноября |
| `lidar_processor` | обработка данных лидара, диагностика | `/uav/lidar_status` | `/lidar/points` | 23 ноября |
| `mission_manager` | принимает маршрут, ведёт миссию | `/uav/mission_status` | `/uav/telemetry` | 30 ноября |
| `diagnostics_node` | ошибки, предупреждения, понятные логи | `/diagnostics` | темы статуса | 7 декабря |

Сервисы: `/mission/upload` (загрузить маршрут), `/mission/start`, `/mission/abort`.

Формат сообщений — в [telemetry_format.md](telemetry_format.md), решения — в [decisions.md](decisions.md), сроки — в [plan.md](plan.md).
