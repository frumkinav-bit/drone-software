# Запуск демо: тестовые данные → телеметрия → журнал

Все команды выполняются внутри Dev Container, из корня репозитория.

```bash
colcon build                                        # собрать все пакеты
source install/setup.bash                           # подключить собранное
ros2 launch uav_bringup telemetry_demo.launch.py    # запустить три узла
```

Во втором терминале (после `source install/setup.bash`):

```bash
ros2 topic echo /uav/telemetry     # смотреть телеметрию
ros2 topic hz /uav/telemetry       # проверить частоту
```

Результат: файл `telemetry_logs/telemetry_ГГГГММДД_ЧЧММСС.csv` со строками телеметрии.
Узел `test_data_player` отправляет 1 строку в секунду (параметр `rate_hz`) и останавливается, когда данные закончились.

Другой файл или скорость: `ros2 launch uav_bringup telemetry_demo.launch.py csv_path:=data/test/my.csv rate_hz:=10.0`
(число для `rate_hz` пишем с точкой, например `10.0`).

## Сообщение RawState

`/uav/raw_state` — состояние от автопилота. Поля те же, что в `Telemetry`, но без `uav_id` и `mission_status`:
их добавляет `telemetry_node`. Пока `mission_status` всегда `IDLE`, позже его будет задавать `mission_manager`.

## Ограничения

- QoS сейчас по умолчанию (reliable), на best effort перейдём при интеграции с автопилотом.
- Данные в `data/test/` синтетические.
- Тесты: `python3 -m pytest src/uav_autopilot/test` (проверяют чтение CSV).
