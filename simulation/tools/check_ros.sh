#!/usr/bin/env bash
# Проверка, что телеметрия PX4 и данные лидара приходят в ROS 2.
# Запускать, когда работают Gazebo, PX4 и ros_run.sh.
#
#     bash ~/simulation/tools/check_ros.sh
SIM_DIR="$(cd "$(dirname "$0")/.." && pwd)"
source "$SIM_DIR/tools/ros_env.sh" || exit 1

# служебный процесс ros2 daemon может помнить старый список тем — перезапускаем
ros2 daemon stop >/dev/null 2>&1
TOPICS=$(ros2 topic list)
echo "Темы PX4 (/fmu/out), $(echo "$TOPICS" | grep -c '^/fmu/out/') шт.:"
echo "$TOPICS" | grep '^/fmu/out/' | sed 's/^/  /'
echo

rate() {   # $1 — регулярное выражение для имени темы
  local t
  t=$(echo "$TOPICS" | grep -E "$1" | head -1)
  if [ -z "$t" ]; then
    printf "  %-46s НЕТ ТЕМЫ\n" "$1"
    return
  fi
  local hz
  hz=$(timeout 8 ros2 topic hz -w 20 "$t" 2>/dev/null | grep -m1 "average rate" | awk '{print $3}')
  if [ -n "$hz" ]; then
    printf "  %-46s %6.1f Гц\n" "$t" "$hz"
  else
    printf "  %-46s НЕТ ДАННЫХ\n" "$t"
  fi
}

echo "Частота основных тем:"
rate '^/fmu/out/vehicle_status(_v[0-9]+)?$'
rate '^/fmu/out/vehicle_local_position(_v[0-9]+)?$'
rate '^/fmu/out/vehicle_global_position(_v[0-9]+)?$'
rate '^/fmu/out/vehicle_attitude(_v[0-9]+)?$'
rate '^/fmu/out/battery_status(_v[0-9]+)?$'
rate '^/fmu/out/failsafe_flags(_v[0-9]+)?$'
rate '^/lidar/scan$'
rate '^/lidar/points$'
rate '^/camera/down/image$'
rate '^/clock$'
echo
echo "Подробно по лидару: python3 ~/simulation/tools/lidar_check.py"
