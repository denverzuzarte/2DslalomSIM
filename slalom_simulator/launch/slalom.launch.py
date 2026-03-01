#!/usr/bin/env python3
"""
Launch file for AUV 3D Simulator.

Removed topics:
  /control/command  — internal bridge (eliminated)
  /control/force    — redundant force echo (eliminated)

Changed topics:
  /imu1/accel, /imu2/accel → ImuAccel (combined linear + angular)
  /controller/force         → ImuAccel (6D output from controller)
  /controller/setpoint      → AuvState (full state setpoint)

Kalman node: always launched but publishes only if kalman_enabled=true in params.yaml.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    localization_arg = DeclareLaunchArgument(
        'localization',
        default_value='imu1',
        description='Localization source for controller: imu1 or imu2'
    )

    pkg_share = FindPackageShare('slalom_simulator')
    params_file = PathJoinSubstitution([pkg_share, 'config', 'params.yaml'])

    simulator_node = Node(
        package='slalom_simulator',
        executable='simulator_node',
        name='simulator_node',
        output='screen',
        parameters=[params_file],
    )

    imu1_localization_node = Node(
        package='slalom_simulator',
        executable='imu1_localization_node',
        name='imu1_localization_node',
        output='screen',
        parameters=[params_file],
    )

    imu2_localization_node = Node(
        package='slalom_simulator',
        executable='imu2_localization_node',
        name='imu2_localization_node',
        output='screen',
        parameters=[params_file],
    )

    # Kalman node: always running, publishing gated by kalman_enabled param
    kalman_localization_node = Node(
        package='slalom_simulator',
        executable='kalman_localization_node',
        name='kalman_localization_node',
        output='screen',
        parameters=[params_file],
    )

    pressure_sensor_node = Node(
        package='slalom_simulator',
        executable='pressure_sensor_node',
        name='pressure_sensor_node',
        output='screen',
        parameters=[params_file],
    )

    controller_node = Node(
        package='slalom_simulator',
        executable='controller_node',
        name='controller_node',
        output='screen',
        parameters=[
            params_file,
            {'localization_source': LaunchConfiguration('localization')}
        ],
    )

    return LaunchDescription([
        localization_arg,
        simulator_node,
        imu1_localization_node,
        imu2_localization_node,
        kalman_localization_node,
        pressure_sensor_node,
        controller_node,
    ])
