#!/usr/bin/env bash
# Подключает настройки (airframe) нашей рамы к PX4 и пересобирает PX4 SITL.
# Запускать после install_px4.sh и каждый раз после изменения generate_model.py:
#
#     bash ~/simulation/tools/px4_setup.sh
set -e

PX4_DIR=${PX4_DIR:-$HOME/PX4-Autopilot}
SIM_DIR="$(cd "$(dirname "$0")/.." && pwd)"
AIRFRAME_DIR="$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes"

if [ ! -d "$AIRFRAME_DIR" ]; then
  echo "Не найден PX4 в $PX4_DIR. Сначала: bash $SIM_DIR/tools/install_px4.sh"
  exit 1
fi

# Свежие файлы модели и airframe
python3 "$SIM_DIR/tools/generate_model.py" > /dev/null

for f in "$SIM_DIR"/px4/*_gz_*; do
  name=$(basename "$f")
  cp "$f" "$AIRFRAME_DIR/$name"
  if ! grep -q "^\s*$name\s*$" "$AIRFRAME_DIR/CMakeLists.txt"; then
    # добавить в список airframe сразу после 4001_gz_x500
    sed -i "s/^\(\s*\)4001_gz_x500\s*$/&\n\1$name/" "$AIRFRAME_DIR/CMakeLists.txt"
  fi
  grep -q "^\s*$name\s*$" "$AIRFRAME_DIR/CMakeLists.txt" \
    && echo "airframe $name подключён" \
    || { echo "!!! не удалось добавить $name в CMakeLists.txt"; exit 1; }
done

cd "$PX4_DIR"
make px4_sitl
echo "PX4 пересобран с настройками project_quad."
