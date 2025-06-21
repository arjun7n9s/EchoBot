#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Get the package directory
    pkg_path = get_package_share_directory('echowalker_description')
    
    # Set paths to the URDF and world files
    urdf_path = os.path.join(pkg_path, 'urdf', 'echowalker_gazebo.urdf.xacro')
    world_path = os.path.join(pkg_path, 'worlds', 'echowalker.sdf')
    
    # Set the path to the controller configuration file
    controller_config_path = os.path.join(pkg_path, 'config', 'echowalker_controllers.yaml')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.5')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_x_pose = DeclareLaunchArgument(
        'x_pose',
        default_value='0.0',
        description='X position to spawn the robot'
    )
    
    declare_y_pose = DeclareLaunchArgument(
        'y_pose',
        default_value='0.0',
        description='Y position to spawn the robot'
    )
    
    declare_z_pose = DeclareLaunchArgument(
        'z_pose',
        default_value='0.5',
        description='Z position to spawn the robot'
    )
    
    # Convert xacro to URDF
    robot_description = Command(['xacro ', urdf_path])
    
    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': world_path + ' -v 4'
        }.items()
    )
    
    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_echowalker',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'echowalker',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose
        ],
        output='screen'
    )
    
    # Bridge between Gazebo and ROS topics
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/model/echowalker/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model'
        ],
        output='screen'
    )
    
    # Controller manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            controller_config_path,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # Joint state broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )
    
    # Effort controllers spawner
    effort_controllers_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['effort_controllers'],
        output='screen'
    )
    
    # Create and return the launch description
    return LaunchDescription([
        declare_use_sim_time,
        declare_x_pose,
        declare_y_pose,
        declare_z_pose,
        robot_state_publisher_node,
        gazebo_launch,
        spawn_robot,
        bridge,
        controller_manager,
        joint_state_broadcaster_spawner,
        effort_controllers_spawner
    ])