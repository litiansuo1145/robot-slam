from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions


import os
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare(package='pavo2s_ros').find('pavo2s_ros')
   
    return LaunchDescription([
        launch_ros.actions.Node(
            package='pavo2s_ros', 
            executable='pavo2s_pcd_node', 
            name='pavo2s_pcd_node',
            parameters=[{   'lidar_ip': '10.10.10.101',
                            'lidar_port': int(2368),
                            'dest_port': int(2368),
                            'frame_id': 'pavo2s_frame',
                            'topic':'pavo2s_pcd',
                            'angle_start': int(4000),
                            'angle_end': int(32000),
                            'inverted': bool(False),
                            'echo_mode': bool(True),
                            'resolution': int(16),
                            'motor_speed': int(25),
                            'tail_filter': int(0),
                            'front_filter': int(0),
                            'anti_disturb': int(0),
                            'timeout': int(180),
                            }],
            output='screen'),
    ])

