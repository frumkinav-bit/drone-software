#!/usr/bin/env bash
# Мосты симуляция -> ROS 2. Запускать в отдельном окне, когда работают Gazebo и PX4.
#   - Micro XRCE-DDS Agent: телеметрия PX4 в темы /fmu/out/...
#   - ros_gz_bridge: лидар (/lidar/scan, /lidar/points), камера (/camera/down/...), /clock
# Остановить — Ctrl+C.
#
#     bash ~/simulation/tools/ros_run.sh
SIM_DIR="$(cd "$(dirname "$0")/.." && pwd)"
source "$SIM_DIR/tools/ros_env.sh" || exit 1
command -v MicroXRCEAgent >/dev/null || { echo "Нет MicroXRCEAgent. Сначала: bash $SIM_DIR/tools/install_ros.sh"; exit 1; }

LOG=/tmp/microxrce_agent.log
MicroXRCEAgent udp4 -p 8888 > "$LOG" 2>&1 &
AGENT=$!
trap 'kill $AGENT 2>/dev/null; echo; echo "Мосты остановлены."' EXIT
echo "Micro XRCE-DDS Agent запущен (порт 8888, журнал $LOG)."
echo "Мост Gazebo -> ROS 2: $SIM_DIR/ros/ros_gz_bridge.yaml"
echo "Проверка — в другом окне: bash ~/simulation/tools/check_ros.sh"
echo
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:="$SIM_DIR/ros/ros_gz_bridge.yaml"
