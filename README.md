# drone-software

Программная часть проекта «Модульная беспилотная авиационная система» (квадрокоптер).
Стек: ROS 2 Jazzy, Ubuntu 24.04, Python; симуляция — Gazebo Harmonic.

Сейчас в репозитории работает цепочка обработки телеметрии на тестовых данных:

```text
test_data_player  →  /uav/raw_state  →  telemetry_node  →  /uav/telemetry  →  telemetry_logger  →  файл CSV
(читает CSV)                            (собирает сообщение)                    (пишет журнал)
```

## Что где лежит

| Папка или файл | Что это |
| --- | --- |
| `src/uav_interfaces/` | описание сообщений (`Telemetry.msg`, `RawState.msg`) |
| `src/uav_autopilot/` | узел `test_data_player`: воспроизводит тестовые данные как автопилот |
| `src/uav_telemetry/` | узлы `telemetry_node` и `telemetry_logger` |
| `src/uav_bringup/` | launch-файл: запуск всех узлов одной командой |
| `data/test/` | тестовые данные (CSV, синтетические) |
| `docs/` | документация (список ниже) |
| `.devcontainer/` | описание окружения: Ubuntu 24.04 + ROS 2 Jazzy в Docker |

## Состояние работ

| Узел | Статус |
| --- | --- |
| `test_data_player`, `telemetry_node`, `telemetry_logger` | готово, проверено на тестовых данных |
| `diagnostics_node`, `autopilot_bridge`, `lidar_processor`, `mission_manager` | запланированы, сроки в [docs/plan.md](docs/plan.md) |

## Быстрый старт

ROS 2 Jazzy работает на Ubuntu, поэтому на Windows и Mac мы запускаем его внутри Docker-контейнера. Ставить ROS 2 на свой компьютер отдельно **не нужно**: окружение описано в папке `.devcontainer` и у всех получается одинаковым.

Что понадобится на компьютере: около 15 ГБ свободного места и 8 ГБ оперативной памяти (Docker Desktop должен получить хотя бы 4 ГБ).

### Windows

1. **WSL2.** Откройте PowerShell от имени администратора и выполните `wsl --install`, затем перезагрузите компьютер.
2. **Docker Desktop.** Скачайте с docker.com и установите. На вопрос о WSL 2 ответьте «да» (использовать WSL 2). Запустите Docker Desktop и дождитесь, пока он загрузится.
3. **Git и VS Code.** Установите [Git для Windows](https://git-scm.com/download/win) и [VS Code](https://code.visualstudio.com/). В VS Code поставьте расширение **Dev Containers** (издатель Microsoft).
4. **Настройка Git.** В PowerShell:

   ```powershell
   git config --global core.autocrlf input
   git config --global user.name "Ваше Имя"
   git config --global user.email "почта@..."
   ```

   Имя лучше писать латиницей. Почта должна быть привязана к вашему GitHub.
5. **Скачайте проект.** Выберите папку **без кириллицы и пробелов в пути**, например `C:\dev`:

   ```powershell
   cd C:\dev
   git clone https://github.com/frumkinav-bit/drone-software.git
   cd drone-software
   code .
   ```
6. **Откройте в контейнере.** В VS Code появится сообщение «Папка содержит файл конфигурации контейнера разработки»: нажмите **«Повторно открыть в контейнере»**. Если сообщения нет: `Ctrl+Shift+P` → `Dev Containers: Reopen in Container`. При первом запуске образ скачивается 10–20 минут, дальше запуск быстрый.
7. Когда внизу слева появится `Dev Container: drone-ros2-jazzy`, откройте терминал VS Code и перейдите к разделу «Запуск демо».

### Mac

1. Установите [Docker Desktop](https://www.docker.com/products/docker-desktop/) (версия «Apple chip», если у вас M1–M4) и [VS Code](https://code.visualstudio.com/). В VS Code поставьте расширение **Dev Containers**.
2. В Терминале:

   ```bash
   git config --global user.name "Ваше Имя"
   git config --global user.email "почта@..."
   git clone https://github.com/frumkinav-bit/drone-software.git
   cd drone-software
   code .
   ```
3. В VS Code нажмите «Повторно открыть в контейнере» (или `Cmd+Shift+P` → `Dev Containers: Reopen in Container`).

### Gazebo

Gazebo Harmonic запускается отдельно, на Windows-компьютере через WSL2. Инструкцию добавим сюда после подтверждения среды симуляции с инженерами по моделированию.

## Запуск демо

Все команды выполняются в терминале VS Code **внутри контейнера** (в приглашении видно `ubuntu@...:/workspaces/drone-software$`), из корня репозитория:

```bash
colcon build                                       # собрать все пакеты
source install/setup.bash                          # подключить собранное (в каждом новом терминале)
ros2 launch uav_bringup telemetry_demo.launch.py   # запустить цепочку
```

Во втором терминале (после `source install/setup.bash`) можно смотреть телеметрию:

```bash
ros2 topic echo /uav/telemetry
```

Узел `test_data_player` отправляет 12 строк тестовых данных и останавливается. Остановить запуск: `Ctrl+C`. Результат — файл `telemetry_logs/telemetry_ГГГГММДД_ЧЧММСС.csv`. Подробнее: [docs/run_demo.md](docs/run_demo.md).

## Как работать командой

Главное правило: **в `main` напрямую не пишем**. Вся работа идёт в отдельных ветках и попадает в `main` через pull request.

1. Перед началом: `git checkout main`, затем `git pull`.
2. Создайте ветку под задачу: `git checkout -b feature/короткое-название`.
3. Пишите код, проверяйте его в контейнере (`colcon build`, `ros2 launch ...`).
4. Сохраните: `git add .`, `git commit -m "что сделано"`.
5. Отправьте: `git push -u origin feature/короткое-название`.
6. На GitHub нажмите «Compare & pull request», опишите, что сделано и как проверить, и попросите коллегу посмотреть (поле Reviewers).
7. После проверки нажмите «Merge pull request» и удалите ветку.

Договорённости:

- Один пакет в `src/` — один ответственный: так не будет конфликтов при слиянии. Распределите пакеты между собой и запишите в чате.
- Сообщения коммитов короткие и по делу: «Добавлен узел diagnostics_node».
- Папки `build/`, `install/`, `log/`, `telemetry_logs/` в репозиторий не попадают (они в `.gitignore`).
- Названия пакетов и узлов — в `snake_case`, темы начинаются с `/uav/`.
- Если `git push` отклонён с текстом `fetch first`, выполните `git pull --rebase`, затем снова `git push`.

## Частые проблемы

| Проблема | Решение |
| --- | --- |
| Контейнер долго «грузится» | убедиться, что Docker Desktop запущен; первый запуск — 10–20 минут; журнал: «show log» внизу VS Code |
| `package '...' not found` | выполнить `colcon build`, затем `source install/setup.bash` |
| `cannot import name ...` после добавления сообщения | `rm -rf build install log`, затем `colcon build` и `source install/setup.bash` |
| Правки в Python-коде не видны | выполнить `colcon build` (или собирать с `--symlink-install`) |
| Ошибки сборки после распаковки архива | `rm -rf build install log`, затем `colcon build` |

## Документация

- [Архитектура ПО](docs/architecture.md): узлы, темы, сервисы
- [Формат телеметрии](docs/telemetry_format.md): поля, единицы, журнал CSV
- [Запуск демо](docs/run_demo.md)
- [Принятые решения](docs/decisions.md)
- [План работ программистов](docs/plan.md)
