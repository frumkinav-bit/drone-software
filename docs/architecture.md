# Архитектура ПО БПЛА

Узлы, темы и сервисы описаны в отчёте по первому заданию.
Здесь будет актуальная версия после согласования с командой.

| Узел | Назначение | Публикует | Подписан на |
| --- | --- | --- | --- |
| test_data_player | чтение тестовых данных, имитация автопилота | /uav/raw_state | — |
| autopilot_bridge | связь с автопилотом | /uav/raw_state | /uav/command |
| telemetry_node | сбор телеметрии | /uav/telemetry | /uav/raw_state, /uav/lidar_status |
| telemetry_logger | запись журнала (CSV, rosbag2) | — | /uav/telemetry |
| lidar_processor | обработка данных лидара | /uav/lidar_status | /lidar/points |
| mission_manager | ведение миссии | /uav/mission_status | /uav/telemetry |
| diagnostics_node | ошибки и логи | /diagnostics | темы статуса |
