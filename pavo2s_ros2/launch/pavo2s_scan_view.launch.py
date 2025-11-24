from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions


import os
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare(package='pavo2s_ros').find('pavo2s_ros')
  # Set the path to the RViz configuration settings
    rviz_config_dir = os.path.join(pkg_share,'rviz/pavo2s_scan.rviz') ## rviz的配置文件
   
    return LaunchDescription([
        launch_ros.actions.Node(
            package='pavo2s_ros', 
            executable='pavo2s_scan_node',
            name='pavo2s_scan_node',            
            parameters=[{   'lidar_ip': '10.10.10.101',
                            'lidar_port': int(2368),
                            'dest_port': int(2368),
                            'frame_id': 'pavo2s_frame',
                            'topic': 'pavo2s_scan',
                            'range_min': float(0.1),
                            'range_max': float(35.0),
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
        launch_ros.actions.Node(
            package = 'tf2_ros',
            executable = 'static_transform_publisher',
            #name='rviz2',
            arguments = ['0', '0', '0', '0', '0', '0', 'map', 'pavo2s_frame']),
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2', ## rviz2节点
            name='rviz2',
            arguments=['-d',rviz_config_dir],
            output='screen'),
    ])
