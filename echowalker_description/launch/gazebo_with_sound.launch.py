#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Get the package directories
    echowalker_description_pkg = get_package_share_directory('echowalker_description')
    echowalker_control_pkg = get_package_share_directory('echowalker_control')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    walking_speed = LaunchConfiguration('walking_speed', default='0.3')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_walking_speed = DeclareLaunchArgument(
        'walking_speed',
        default_value='0.3',
        description='Walking speed of the robot (0.0 to 1.0)'
    )
    
    # Include the basic Gazebo launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(echowalker_description_pkg, 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # Launch sound simulator node (delayed to allow Gazebo to start)
    sound_simulator_node = TimerAction(
        period=5.0,  # Wait 5 seconds for Gazebo to fully start
        actions=[
            Node(
                package='echowalker_control',
                executable='sound_simulator.py',
                name='sound_simulator',
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'publish_rate': 10.0},
                    {'circle_radius': 4.0},
                    {'circle_period': 45.0}
                ]
            )
        ]
    )
    
    # Launch Gazebo gait controller node (delayed to allow controllers to start)
    gait_controller_node = TimerAction(
        period=7.0,  # Wait 7 seconds for controllers to be ready
        actions=[
            Node(
                package='echowalker_control',
                executable='gazebo_gait_controller.py',
                name='gazebo_gait_controller',
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'walking_speed': walking_speed},
                    {'direction_topic': '/sound_direction'},
                    {'effort_scaling': 20.0}
                ]
            )
        ]
    )
    
    # RViz for visualization (optional)
    rviz_config_path = os.path.join(echowalker_description_pkg, 'rviz', 'gazebo_echowalker.rviz')
    rviz_node = TimerAction(
        period=3.0,  # Wait 3 seconds for robot_state_publisher
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config_path],
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )
    
    # Create and return the launch description
    return LaunchDescription([
        declare_use_sim_time,
        declare_walking_speed,
        gazebo_launch,
        sound_simulator_node,
        gait_controller_node,
        rviz_node
    ])