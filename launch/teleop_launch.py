import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    这个启动文件会同时启动两个节点：
    1. 键盘遥控节点：在一个新的终端窗口中运行，作为遥控器。
    2. 底层控制器节点：在当前终端运行，作为翻译官，连接单片机。
    """
    return LaunchDescription([
        
        # --- 启动遥控器节点 ---
        Node(
            package='robot_slam',
            executable='teleop_keyboard',
            name='teleop_keyboard_node',
            output='screen',
            # 这个prefix参数会在一个新的xterm终端窗口中启动此节点
            # 确保您已经安装了 xterm (sudo apt-get install xterm)
            prefix='xterm -e'
        ),
        
        # --- 启动翻译官（底层控制器）节点 ---
        Node(
            package='robot_slam',
            executable='base_controller',
            name='base_controller_node',
            output='screen',
            parameters=[
                # 在这里指定您的单片机串口信息
                # !! 确保这是您单片机的正确串口和波特率 !!
                {'serial_port': '/dev/ttyUSB1'},
                {'baud_rate': 115200} # 已根据您的信息更新
            ]
        )
    ])