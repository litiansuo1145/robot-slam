from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. 启动底层控制器 (翻译官), 监听 /cmd_vel
        Node(
            package='robot_slam',
            executable='base_controller',
            name='base_controller_node',
            output='screen',
            parameters=[
                {'serial_port': '/dev/ttyACM0'}, # 确保是您正确的单片机端口
                {'baud_rate': 115200}
            ]
        ),

        # 2. 在一个新的xterm窗口中启动键盘遥控器
        Node(
            package='robot_slam',
            executable='teleop_keyboard',
            name='teleop_keyboard_node',
            # output='screen' # 在xterm中，输出会显示在新窗口，这里可以省略
            prefix='xterm -e', # 弹出一个新的xterm窗口来运行此节点
        ),
    ])
