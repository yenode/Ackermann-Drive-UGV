#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    custom_ackermann_pkg = FindPackageShare('custom_ackermann_controller')
    
    adaptive_scan_config = PathJoinSubstitution([
        custom_ackermann_pkg, 'config', 'adaptive_scan_matcher.yaml'
    ])
    
    enhanced_ekf_config = PathJoinSubstitution([
        custom_ackermann_pkg, 'config', 'enhanced_ekf.yaml'
    ])
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time (false for real hardware)'
    )
    
    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan',
        description='Laser scan topic name'
    )
    
    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='base_footprint',
        description='Robot base frame'
    )
    
    adaptive_scan_matcher_node = Node(
        package='custom_ackermann_controller',
        executable='adaptive_scan_matcher',
        name='adaptive_scan_matcher',
        output='screen',
        parameters=[
            adaptive_scan_config,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'scan_topic': LaunchConfiguration('scan_topic'),
                'base_frame': LaunchConfiguration('base_frame'),
            }
        ],
        remappings=[
            ('/scan', LaunchConfiguration('scan_topic')),
        ],
        respawn=True,
        respawn_delay=2.0,
    )
    
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            enhanced_ekf_config,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
        remappings=[
            ('/odometry/filtered', '/odometry/filtered'),
        ],
        respawn=True,
        respawn_delay=2.0,
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        scan_topic_arg,
        base_frame_arg,
        
        adaptive_scan_matcher_node,
        ekf_node,
    ])
