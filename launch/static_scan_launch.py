import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取ydlidar驱动包的路径
    ydlidar_pkg_share = get_package_share_directory('ydlidar_ros2_driver')
    
    # 1. 启动雷达驱动
    ydlidar_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ydlidar_pkg_share, 'launch', 'x3_ydlidar_launch.py')
        )
    )

    # 2. 启动静态TF: base_link -> laser
    # 这个TF描述了雷达在机器人本体上的安装位置。
    tf_laser_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        # arguments=['x', 'y', 'z', 'yaw', 'pitch', 'roll', 'parent_frame', 'child_frame']
        arguments=['0.0', '0.0', '0.3', '0', '0', '0', 'base_link', 'laser'],
    )
    
    # 3. 启动静态TF: map -> base_link
    # 这是实现“原地不动”的关键！
    # 我们“锁死”了机器人本体(base_link)和世界(map)的相对位置。
    # 两个坐标系的原点和姿态完全重合 (x=0, y=0, z=0, yaw=0, pitch=0, roll=0)。
    tf_map_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_map',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
    )

    # 4. 启动RViz2用于可视化
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        ydlidar_node,
        tf_laser_node,
        tf_map_node,
        rviz_node
    ])
