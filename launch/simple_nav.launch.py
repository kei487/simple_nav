#!/usr/bin/env python3
# SPDX-License-Identifier: MIT License

"""
Simple Nav Launch File

This launch file starts both the A* path planner and Pure Pursuit controller nodes
for autonomous navigation with a differential drive robot.

Usage:
    ros2 launch simple_nav simple_nav.launch.py
    
    # With custom parameters:
    ros2 launch simple_nav simple_nav.launch.py use_sim_time:=true lookahead_distance:=0.8
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for simple_nav package"""
    
    # Get package directory
    pkg_simple_nav = get_package_share_directory('simple_nav')
    
    # Path to config file
    default_config_path = os.path.join(pkg_simple_nav, 'config', 'simple_nav.yaml')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )
    
    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_path,
        description='Full path to the ROS 2 parameters file to use'
    )
    
    declare_lookahead_distance = DeclareLaunchArgument(
        'lookahead_distance',
        default_value='0.5',
        description='Lookahead distance for Pure Pursuit [m]'
    )
    
    declare_target_velocity = DeclareLaunchArgument(
        'target_linear_velocity',
        default_value='0.3',
        description='Target linear velocity [m/s]'
    )
    
    declare_control_frequency = DeclareLaunchArgument(
        'control_frequency',
        default_value='20.0',
        description='Control loop frequency [Hz]'
    )
    
    declare_goal_tolerance = DeclareLaunchArgument(
        'goal_tolerance_dist',
        default_value='0.1',
        description='Goal tolerance distance [m]'
    )
    
    declare_map_frame = DeclareLaunchArgument(
        'map_frame',
        default_value='map',
        description='Map frame ID'
    )
    
    declare_robot_base_frame = DeclareLaunchArgument(
        'robot_base_frame',
        default_value='base_link',
        description='Robot base frame ID'
    )
    
    declare_use_dijkstra = DeclareLaunchArgument(
        'use_dijkstra',
        default_value='false',
        description='Use Dijkstra algorithm instead of A* for path planning'
    )
    
    declare_publish_searched_map = DeclareLaunchArgument(
        'publish_searched_map',
        default_value='false',
        description='Publish searched map for visualization'
    )
    
    # A* Path Planner Node
    a_star_planner_node = Node(
        package='simple_nav',
        executable='a_star_planner',
        name='a_star_planner',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'use_dijkstra': LaunchConfiguration('use_dijkstra'),
                'publish_searched_map': LaunchConfiguration('publish_searched_map'),
            }
        ],
        remappings=[
            ('/costmap_2d','/map')
            # ('/scan_filtered','/scan')
            # Add remappings if needed
        ]
    )
    
    # Pure Pursuit Controller Node
    pure_pursuit_controller_node = Node(
        package='simple_nav',
        executable='pure_pursuit_controller',
        name='pure_pursuit_controller',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'lookahead_distance': LaunchConfiguration('lookahead_distance'),
                'target_linear_velocity': LaunchConfiguration('target_linear_velocity'),
                'control_frequency': LaunchConfiguration('control_frequency'),
                'goal_tolerance_dist': LaunchConfiguration('goal_tolerance_dist'),
                'map_frame': LaunchConfiguration('map_frame'),
                'robot_base_frame': LaunchConfiguration('robot_base_frame'),
            }
        ],
        remappings=[
            ('/cmd_vel','/sim_cmd_vel')
            # Add remappings if needed
        ]
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_config_file)
    ld.add_action(declare_lookahead_distance)
    ld.add_action(declare_target_velocity)
    ld.add_action(declare_control_frequency)
    ld.add_action(declare_goal_tolerance)
    ld.add_action(declare_map_frame)
    ld.add_action(declare_robot_base_frame)
    ld.add_action(declare_use_dijkstra)
    ld.add_action(declare_publish_searched_map)
    
    # Add nodes
    ld.add_action(a_star_planner_node)
    ld.add_action(pure_pursuit_controller_node)
    
    return ld

