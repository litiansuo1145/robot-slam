import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    这个启动文件将按顺序启动一个完整的自主清洁系统，并给予更长的启动延迟。
    """
    
    # --- 启动底层控制器 (翻译官) ---
    base_controller_node = Node(
        package='robot_slam',
        executable='base_controller',
        name='base_controller_node',
        output='screen',
        parameters=[
            {'serial_port': '/dev/ttyUSB0'},
            {'baud_rate': 115200}
        ]
    )

    # --- 延迟4秒后，再启动传感器 ---
    # <-- 关键修改：将延迟从2.0秒增加到4.0秒 -->
    ydlidar_node = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('ydlidar_ros2_driver'), 'launch', 'x3_ydlidar_launch.py')
                )
            )
        ]
    )

    # --- 再延迟2秒后 (总共6秒)，启动感知和规划节点 ---
    # <-- 关键修改：将延迟从4.0秒增加到6.0秒 -->
    application_nodes = TimerAction(
        period=6.0,
        actions=[
            # 启动避障节点 (反射神经)
            Node(
                package='robot_slam',
                executable='obstacle_avoider',
                name='obstacle_avoider_node',
                output='screen',
                parameters=[
                    {'avoidance_distance': 0.4},
                ]
            ),

            # 启动覆盖路径规划器 (大脑)
            Node(
                package='robot_slam',
                executable='coverage_planner',
                name='coverage_planner_node',
                output='screen',
                parameters=[
                    {'pool_length': 5.0},
                    {'pool_width': 3.0},
                    {'robot_linear_speed': 0.2},
                    {'robot_angular_speed': 0.2},
                    {'robot_coverage_width': 0.5}
                ]
            )
        ]
    )

    return LaunchDescription([
        base_controller_node,
        ydlidar_node,
        application_nodes
    ])
