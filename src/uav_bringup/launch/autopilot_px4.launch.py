"""Запуск с настоящим PX4 (SITL или реальный борт): autopilot_bridge в режиме px4,
телеметрия, журнал и диагностика.

Нужны: собранный px4_msgs (ветка release/1.17), запущенный Micro XRCE-DDS Agent и PX4.
Запускать на ноутбуке группы моделирования, рядом с симуляцией, из корня репозитория:

  ros2 launch uav_bringup autopilot_px4.launch.py use_sim_time:=true consider_rc_loss:=false

Значения true и false пишутся с маленькой буквы.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Время симуляции: true, если мост Gazebo публикует /clock (тогда узлы идут по времени Gazebo)
    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        # В симуляции без пульта PX4 считает пульт потерянным; тогда пишем consider_rc_loss:=false
        DeclareLaunchArgument('consider_rc_loss', default_value='true'),
        DeclareLaunchArgument('uav_id', default_value='uav_01'),

        Node(
            package='uav_autopilot',
            executable='autopilot_bridge',
            name='autopilot_bridge',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'source': 'px4',
                'consider_rc_loss': ParameterValue(LaunchConfiguration('consider_rc_loss'), value_type=bool),
            }],
        ),
        Node(
            package='uav_telemetry',
            executable='telemetry_node',
            name='telemetry_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'uav_id': ParameterValue(LaunchConfiguration('uav_id'), value_type=str),
            }],
        ),
        Node(
            package='uav_telemetry',
            executable='telemetry_logger',
            name='telemetry_logger',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='uav_telemetry',
            executable='diagnostics_node',
            name='diagnostics_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
