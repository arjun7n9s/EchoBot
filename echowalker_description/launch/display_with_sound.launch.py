#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    # Get the package directories
    echowalker_description_pkg = get_package_share_directory('echowalker_description')
    echowalker_control_pkg = get_package_share_directory('echowalker_control')

    # Include the display launch file
    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(echowalker_description_pkg, 'launch', 'display.launch.py')
        ]),
        launch_arguments={
            'use_gui': 'false',  # Disable GUI when using gait controller
            'use_rviz': 'true'
        }.items()
    )

    # Launch sound simulator node
    sound_simulator_node = Node(
        package='echowalker_control',
        executable='sound_simulator.py',
        name='sound_simulator',
        output='screen'
    )

    # Launch gait controller node
    gait_controller_node = Node(
        package='echowalker_control',
        executable='gait_controller.py',
        name='gait_controller',
        output='screen'
    )

    # Create and return the launch description
    return LaunchDescription([
        display_launch,
        sound_simulator_node,
        gait_controller_node
    ])
