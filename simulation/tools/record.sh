#!/usr/bin/env bash
# Запись телеметрии PX4 и данных лидара в rosbag2 (формат MCAP).
# Запускать, когда работают Gazebo, PX4 и ros_run.sh. Остановить — Ctrl+C.
#
#     bash ~/simulation/tools/record.sh            # телеметрия + лидар + /clock
#     bash ~/simulation/tools/record.sh --camera   # + изображение камеры (большой объём!)
#
# Запись сохраняется в ~/simulation/bags/flight_<дата_время>/
SIM_DIR="$(cd "$(dirname "$0")/.." && pwd)"
source "$SIM_DIR/tools/ros_env.sh" || exit 1

# служебный процесс ros2 daemon может помнить старый список тем — перезапускаем
ros2 daemon stop >/dev/null 2>&1
FMU=$(ros2 topic list | grep '^/fmu/out/' | tr '\n' ' ')
if [ -z "$FMU" ]; then
  echo "Нет тем /fmu/out. Запущены ли PX4 и ros_run.sh?"
  exit 1
fi
EXTRA=""
if [ "$1" = "--camera" ]; then
  EXTRA="/camera/down/image /camera/down/camera_info"
  echo "С камерой: около 40 МБ на каждую секунду записи."
fi
mkdir -p "$SIM_DIR/bags"
OUT="$SIM_DIR/bags/flight_$(date +%Y%m%d_%H%M%S)"
echo "Запись в $OUT — остановить Ctrl+C"
ros2 bag record -s mcap -o "$OUT" /clock /lidar/scan /lidar/points $FMU $EXTRA
echo
ros2 bag info "$OUT"
