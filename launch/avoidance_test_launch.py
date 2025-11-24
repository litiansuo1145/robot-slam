import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        # === 第1步：首先启动底层控制器 (翻译官) ===
        # 它会立刻占用单片机的串口，防止被雷达驱动误用
        Node(
            package='robot_slam',
            executable='base_controller',
            name='base_controller_node',
            output='screen',
            parameters=[{'serial_port': '/dev/ttyUSB1', 'baud_rate': 115200}]
        ),

        # === 第2步：延迟2秒后，再启动雷达驱动 ===
        # 这给了udev足够的时间来创建/dev/ydlidar别名，
        # 也避免了和base_controller同时抢占系统资源。
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

        # === 第3步：再延迟1秒后（总共延迟3秒），启动其他应用节点 ===
        # 这确保了雷达驱动已经稳定运行并开始发布/scan话题
        TimerAction(
            period=3.0,
            actions=[
                # 启动键盘遥控器 (在一个新窗口)
                Node(
                    package='robot_slam',
                    executable='teleop_keyboard',
                    name='teleop_keyboard_node',
                    output='screen',
                    prefix='xterm -e'
                ),
               
               
                    ]
                )
            ]
        )
    ])
