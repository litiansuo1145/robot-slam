import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取我们自己功能包'robot_slam'的路径
    pkg_share = get_package_share_directory('robot_slam')

    # 获取ydlidar驱动包的路径
    ydlidar_pkg_share = get_package_share_directory('ydlidar_ros2_driver')
    
    # 1. 启动雷达驱动
    ydlidar_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ydlidar_pkg_share, 'launch', 'x3_ydlidar_launch.py')
        )
    )

    # 2. 启动静态TF发布 (base_link -> laser)
    # !! 您必须根据实际情况精确测量这些值 !! (x, y, z)
    tf_laser_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=['0.0', '0.0', '0.0', '0', '0', '0', 'base_link', 'laser'], # <-- 修改这里的XYZ偏移量
    )
    
    # 3. 启动SLAM Toolbox节点
    slam_config_path = os.path.join(pkg_share, 'config', 'slam.yaml')
    
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config_path],
    )

    # 4. 启动RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        ydlidar_node,
        tf_laser_node,
        slam_toolbox_node,
        rviz_node
    ])
