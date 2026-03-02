#!/usr/bin/env python3
"""
Launch file for AUV 3D Simulator.

Topics:
  /imu1/data, /imu2/data     — sensor_msgs/Imu (full orientation + accel + angular velocity)
  /controller/global_forces  — GlobalForces (force command to simulator)
  /controller/setpoint       — AuvState (full state setpoint to controller)
  /ground_truth              — AuvState (ground truth from simulator)
  /localization/pose         — AuvState (localization output, source selectable)
  /vision/detections         — ObjectDetections (pole detections)

Localization options (set via 'localization' launch arg):
  ground_truth — perfect localization from simulator (default)
  imu1         — IMU1 dead-reckoning + pressure sensor
  imu2         — IMU2 dead-reckoning + pressure sensor
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    localization_arg = DeclareLaunchArgument(
        'localization',
        default_value='ground_truth',
        description='Localization source: ground_truth, imu1, or imu2'
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

    ground_truth_localization_node = Node(
        package='slalom_simulator',
        executable='ground_truth_localization_node',
        name='ground_truth_localization_node',
        output='screen',
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
        ground_truth_localization_node,
        imu1_localization_node,
        imu2_localization_node,
        pressure_sensor_node,
        controller_node,
    ])

