"""Демо: тестовые данные -> telemetry_node -> журнал CSV. Запускать из корня репозитория."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        # Аргументы запуска: их можно менять в командной строке (csv_path:=... rate_hz:=...)
        DeclareLaunchArgument('csv_path', default_value='data/test/sample_telemetry.csv'),
        DeclareLaunchArgument('rate_hz', default_value='1.0'),
        DeclareLaunchArgument('uav_id', default_value='uav_01'),

        Node(
            package='uav_autopilot',
            executable='test_data_player',
            name='test_data_player',
            output='screen',  # выводить сообщения узла в терминал
            parameters=[{
                'csv_path': ParameterValue(LaunchConfiguration('csv_path'), value_type=str),
                'rate_hz': ParameterValue(LaunchConfiguration('rate_hz'), value_type=float),
            }],
        ),
        Node(
            package='uav_telemetry',
            executable='telemetry_node',
            name='telemetry_node',
            output='screen',
            parameters=[{
                'uav_id': ParameterValue(LaunchConfiguration('uav_id'), value_type=str),
            }],
        ),
        Node(
            package='uav_telemetry',
            executable='telemetry_logger',
            name='telemetry_logger',
            output='screen',
        ),
        Node(
            package='uav_telemetry',
            executable='diagnostics_node',
            name='diagnostics_node',
            output='screen',
        ),
    ])
