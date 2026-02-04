#!/usr/bin/env python3
"""
Launch file for 2D Slalom Simulator
Launches all nodes with configurable localization source
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    localization_arg = DeclareLaunchArgument(
        'localization',
        default_value='imu1',
        description='Localization source: imu1, imu2, or kalman'
    )

    # Get package share directory
    pkg_share = FindPackageShare('slalom_simulator')

    # Path to params file
    params_file = PathJoinSubstitution([pkg_share, 'config', 'params.yaml'])

    # Simulator node
    simulator_node = Node(
        package='slalom_simulator',
        executable='simulator_node',
        name='simulator_node',
        output='screen',
        parameters=[params_file],
    )

    # IMU1 localization node
    imu1_localization_node = Node(
        package='slalom_simulator',
        executable='imu1_localization_node',
        name='imu1_localization_node',
        output='screen',
        parameters=[params_file],
    )

    # IMU2 localization node
    imu2_localization_node = Node(
        package='slalom_simulator',
        executable='imu2_localization_node',
        name='imu2_localization_node',
        output='screen',
        parameters=[params_file],
    )

    # Kalman localization node (stub)
    kalman_localization_node = Node(
        package='slalom_simulator',
        executable='kalman_localization_node',
        name='kalman_localization_node',
        output='screen',
        parameters=[params_file],
    )

    # Controller node
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
        controller_node,
    ])
