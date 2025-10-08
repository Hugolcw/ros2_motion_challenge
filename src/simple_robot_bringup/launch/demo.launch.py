#!/usr/bin/env python3
"""
Launch file for Simple Robot Motion Challenge Demo
This launch file starts:
- Robot state publisher with the robot URDF
- Joint state publisher 
- MoveIt move_group node for motion planning
- RViz with MoveIt plugin for visualization

Usage:
    ros2 launch simple_robot_bringup demo.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    
    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Path to MoveIt config package
    moveit_config_pkg = FindPackageShare('simple_robot_moveit_config')
    
    # Include the robot state publisher launch file
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                moveit_config_pkg,
                'launch',
                'rsp.launch.py'
            ])
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # Include move_group launch file
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                moveit_config_pkg,
                'launch',
                'move_group.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'publish_monitored_planning_scene': 'true',
        }.items()
    )
    
    # Include MoveIt RViz launch file
    moveit_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                moveit_config_pkg,
                'launch',
                'moveit_rviz.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )
    
    # Joint State Publisher node (for manual control in RViz)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'publish_rate': 50.0,
        }],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        # Launch all components
        rsp_launch,
        joint_state_publisher,
        move_group_launch,
        moveit_rviz_launch,
    ])
