#!/usr/bin/env bash
# Запуск PX4 SITL, подключённого к нашей модели в уже открытом Gazebo.
#
# Окно 1:  cd ~/simulation && gz sim -r worlds/test_field.sdf
# Окно 2:  bash ~/simulation/tools/px4_run.sh
#
# Появится консоль PX4 (pxh>). Команды:
#   commander takeoff     взлёт на 2,5 м и висение
#   commander land        посадка
#   listener vehicle_local_position   текущие координаты (z — вниз, т. е. -высота)
#   shutdown              выйти
set -e

PX4_DIR=${PX4_DIR:-$HOME/PX4-Autopilot}
PX4_BIN="$PX4_DIR/build/px4_sitl_default/bin/px4"
WORLD=test_field
MODEL=project_quad

if [ ! -x "$PX4_BIN" ]; then
  echo "PX4 не собран. Сначала: bash ~/simulation/tools/install_px4.sh"
  exit 1
fi
if ! gz topic -l | grep -q "^/world/$WORLD/clock"; then
  echo "Gazebo с миром $WORLD не найден."
  echo "Сначала в другом окне: cd ~/simulation && gz sim -r worlds/$WORLD.sdf"
  exit 1
fi
if ! gz topic -l | grep -q "^/$MODEL/command/motor_speed"; then
  echo "В Gazebo нет модели $MODEL (или открыт мир со стабилизатором Gazebo)."
  echo "Для PX4 нужен мир worlds/$WORLD.sdf, а не ${WORLD}_stab.sdf."
  exit 1
fi

cd "$PX4_DIR"
PX4_GZ_STANDALONE=1 \
PX4_SYS_AUTOSTART=4050 \
PX4_GZ_WORLD=$WORLD \
PX4_GZ_MODEL_NAME=$MODEL \
  "$PX4_BIN"
