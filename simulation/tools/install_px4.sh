#!/usr/bin/env bash
# Установка PX4 v1.17.0 для симуляции (Ubuntu 24.04 в WSL2). Один раз.
#
#     bash ~/simulation/tools/install_px4.sh
#
# Что делает:
#   1. скачивает PX4-Autopilot v1.17.0 в ~/PX4-Autopilot (~1,5 ГБ);
#   2. ставит зависимости официальным скриптом PX4 (спросит пароль sudo);
#   3. собирает PX4 SITL;
#   4. подключает настройки нашей рамы (tools/px4_setup.sh).
# Займёт 40–90 минут, в основном ожидание загрузки и сборки.
set -e

PX4_DIR=${PX4_DIR:-$HOME/PX4-Autopilot}
PX4_VERSION=v1.17.0
SIM_DIR="$(cd "$(dirname "$0")/.." && pwd)"

echo "=== 1/4 Скачивание PX4 $PX4_VERSION в $PX4_DIR ==="
if [ -d "$PX4_DIR/.git" ]; then
  echo "Папка уже есть, пропускаю скачивание."
else
  git clone --recursive --depth 1 --shallow-submodules -b "$PX4_VERSION" \
      https://github.com/PX4/PX4-Autopilot.git "$PX4_DIR" || {
    echo
    echo "!!! Скачивание прервалось. Удалите папку и запустите скрипт ещё раз:"
    echo "    rm -rf $PX4_DIR && bash $0"
    exit 1
  }
fi

echo "=== 2/4 Зависимости (скрипт PX4, без прошивок для реальных плат) ==="
bash "$PX4_DIR/Tools/setup/ubuntu.sh" --no-nuttx

echo "=== 3/4 Сборка PX4 SITL ==="
cd "$PX4_DIR"
make px4_sitl

echo "=== 4/4 Настройки рамы project_quad ==="
bash "$SIM_DIR/tools/px4_setup.sh"

echo
echo "Готово. Закройте это окно и откройте Ubuntu заново (чтобы подхватились"
echo "новые переменные), затем запуск — см. README, раздел «Полёт с PX4»."
