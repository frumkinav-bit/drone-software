#!/usr/bin/env bash
# Установка ROS 2 Jazzy, моста Gazebo -> ROS 2, Micro XRCE-DDS Agent и px4_msgs.
# Один раз, 30–60 минут (спросит пароль). Можно запускать повторно: готовые шаги пропускаются.
#
#     bash ~/simulation/tools/install_ros.sh
set -e
WS=$HOME/ros2_ws
AGENT_DIR=$HOME/Micro-XRCE-DDS-Agent

echo "=== 1/3 ROS 2 Jazzy и мост ros_gz ==="
if [ ! -f /etc/apt/sources.list.d/ros2.sources ] && [ ! -f /etc/apt/sources.list.d/ros2.list ]; then
  sudo apt-get update
  sudo apt-get install -y software-properties-common curl python3
  sudo add-apt-repository -y universe
  # пакет ros2-apt-source настраивает ключ и адрес репозитория ROS 2
  URL=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    | python3 -c "import sys, json; a = json.load(sys.stdin)['assets']; \
print(next(x['browser_download_url'] for x in a if x['name'].startswith('ros2-apt-source') and 'noble' in x['name']))")
  echo "Скачиваю $URL"
  curl -L -o /tmp/ros2-apt-source.deb "$URL"
  sudo dpkg -i /tmp/ros2-apt-source.deb
fi
sudo apt-get update
sudo apt-get install -y ros-jazzy-ros-base ros-jazzy-ros-gz ros-jazzy-rosbag2-storage-mcap \
  ros-dev-tools python3-colcon-common-extensions
echo "ROS 2 Jazzy установлен."

echo "=== 2/3 Micro XRCE-DDS Agent v2.4.3 (телеметрия PX4 -> ROS 2) ==="
if command -v MicroXRCEAgent >/dev/null; then
  echo "уже установлен, пропускаю"
else
  [ -d "$AGENT_DIR" ] || git clone -b v2.4.3 --depth 1 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "$AGENT_DIR"
  mkdir -p "$AGENT_DIR/build" && cd "$AGENT_DIR/build"
  cmake ..
  make -j"$(nproc)"
  sudo make install
  sudo ldconfig /usr/local/lib/
  echo "Micro XRCE-DDS Agent установлен."
fi

echo "=== 3/3 Пакет px4_msgs (ветка release/1.17, как прошивка PX4 v1.17.0) ==="
mkdir -p "$WS/src"
[ -d "$WS/src/px4_msgs" ] || git clone -b release/1.17 --depth 1 https://github.com/PX4/px4_msgs.git "$WS/src/px4_msgs"
source /opt/ros/jazzy/setup.bash
cd "$WS"
colcon build --packages-select px4_msgs
echo
echo "Готово. Запуск моста — см. README, раздел «Телеметрия и лидар в ROS 2»."
