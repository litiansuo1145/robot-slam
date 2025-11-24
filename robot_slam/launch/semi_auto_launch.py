import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
    
        Node(
            package='robot_slam',
            executable='base_controller',
            name='base_controller_node',
            output='screen',
            parameters=[{'serial_port': '/dev/ttyUSB0', 'baud_rate': 115200}]
        ),


        TimerAction(
            period=2.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(get_package_share_directory('ydlidar_ros2_driver'), 'launch', 'x3_ydlidar_launch.py')
                    )
                )
            ]
        ),
        
       
        TimerAction(
            period=4.0,
            actions=[
           
                Node(
                    package='robot_slam',
                    executable='obstacle_avoider',
                    name='obstacle_avoider_node',
                    output='screen',
                    parameters=[{'avoidance_distance': 0.5}]
                ),
           
                Node(
                    package='robot_slam',
                    executable='teleop_keyboard',
                    name='teleop_node',
                    output='screen',
                    prefix='xterm -e'
                ),
           
                Node(
                    package='robot_slam',
                    executable='arbiter',
                    name='arbiter_node',
                    output='screen'
                ),
            ]
        )
    ])
