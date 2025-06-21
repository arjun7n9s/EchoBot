#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    declare_speed = DeclareLaunchArgument(
        'speed',
        default_value='0.2',
        description='Walking speed of the robot'
    )

    declare_direction_topic = DeclareLaunchArgument(
        'direction_topic',
        default_value='/sound_direction',
        description='Topic name for sound direction'
    )

    # Get launch configurations
    speed = LaunchConfiguration('speed')
    direction_topic = LaunchConfiguration('direction_topic')

    # Launch sound simulator node
    sound_simulator_node = Node(
        package='echowalker_control',
        executable='sound_simulator.py',
        name='sound_simulator',
        output='screen',
        parameters=[
            {'publish_rate': 10.0},  # Hz
            {'topic_name': direction_topic}
        ]
    )

    # Launch gait controller node
    gait_controller_node = Node(
        package='echowalker_control',
        executable='gait_controller.py',
        name='gait_controller',
        output='screen',
        parameters=[
            {'walking_speed': speed},
            {'direction_topic': direction_topic}
        ]
    )

    # Create and return the launch description
    return LaunchDescription([
        declare_speed,
        declare_direction_topic,
        sound_simulator_node,
        gait_controller_node
    ])
