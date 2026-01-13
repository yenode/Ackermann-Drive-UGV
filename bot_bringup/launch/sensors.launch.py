from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    
    urg_node2_dir = get_package_share_directory('urg_node2')
    
    urg_persistent_launch = os.path.join(
        urg_node2_dir,
        'launch',
        'urg_node2_persistent.launch.py'
    )
    
    mpu9250_node = Node(
        package='mpu9250_node',
        executable='mpu9250_node',
        name='mpu9250_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }],
        respawn=True,
        respawn_delay=2.0,
    )
    
    encoder_node = Node(
        package='encoder_node',
        executable='esp32_bridge_node',
        name='esp32_bridge_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }],
        respawn=True,
        respawn_delay=2.0,
    )
    
    # Velocity corrector - combines wheel speed with scan matching direction
    velocity_corrector = Node(
        package='encoder_node',
        executable='velocity_corrector_node',
        name='velocity_corrector',
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }],
        respawn=True,
        respawn_delay=2.0,
    )
    
    urg_lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(urg_persistent_launch),
        launch_arguments={
            'serial_port': '/dev/ttyHOKUYO',
            'serial_baud': '115200',
            'auto_start': 'true',
            'node_name': 'urg_node2',
            'scan_topic_name': 'scan',
        }.items()
    )
    
    return LaunchDescription([
        mpu9250_node,
        encoder_node,
        velocity_corrector,
        urg_lidar_launch,
    ])
