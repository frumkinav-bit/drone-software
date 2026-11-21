# Окружение ROS 2 для работы с симуляцией. Подключать в окне Ubuntu:
#     source ~/simulation/tools/ros_env.sh
# В ~/.bashrc не прописывается специально: окна с Gazebo и PX4 работают без ROS 2.
if [ ! -f /opt/ros/jazzy/setup.bash ]; then
  echo "ROS 2 Jazzy не установлен. Сначала: bash ~/simulation/tools/install_ros.sh"
  return 1 2>/dev/null || exit 1
fi
source /opt/ros/jazzy/setup.bash
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"
export GZ_IP=${GZ_IP:-127.0.0.1}
