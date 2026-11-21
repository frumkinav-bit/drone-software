"""Демо лидара без симулятора: искусственный скан -> lidar_processor -> телеметрия, журнал и диагностика.

Запускает всё из telemetry_demo.launch.py (тестовые данные, telemetry_node, журнал CSV, diagnostics_node)
и добавляет lidar_scan_player (искусственный скан) и lidar_processor.
Запускать из корня репозитория:  ros2 launch uav_bringup lidar_demo.launch.py
По умолчанию летит «маршрутный» файл sample_mission.csv (32 с). Скан: стена спереди приближается до 0,8 м
и уходит, на 25-й секунде скан пропадает (должно появиться LIDAR_LOST).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Берём готовый launch-файл telemetry_demo и подставляем в него файл с маршрутом
    demo = os.path.join(get_package_share_directory('uav_bringup'), 'launch', 'telemetry_demo.launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('csv_path', default_value='data/test/sample_mission.csv'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(demo),
            launch_arguments={'csv_path': LaunchConfiguration('csv_path')}.items(),
        ),
        Node(
            package='uav_lidar',
            executable='lidar_scan_player',
            name='lidar_scan_player',
            output='screen',
        ),
        Node(
            package='uav_lidar',
            executable='lidar_processor',
            name='lidar_processor',
            output='screen',
        ),
    ])
