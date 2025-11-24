import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_slam')
    ydlidar_pkg_share = get_package_share_directory('ydlidar_ros2_driver')
    cartographer_ros_pkg_share = get_package_share_directory('cartographer_ros')

    # 1. 启动雷达驱动
    ydlidar_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ydlidar_pkg_share, 'launch', 'x3_ydlidar_launch.py')
        )
    )

    # 2. 启动静态TF发布 (base_link -> laser)
    tf_laser_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=['0.0', '0.0', '0.3', '0', '0', '0', 'base_link', 'laser'],
    )
    
    # 3. 启动 Cartographer 节点
    cartographer_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cartographer_ros_pkg_share, 'launch', 'cartographer.launch.py')
        ),
        launch_arguments={
            'configuration_directory': os.path.join(pkg_share, 'config'),
            'configuration_basename': 'cartographer.lua',
        }.items()
    )
    
    # 4. 启动 Cartographer Occupancy Grid Node
    occupancy_grid_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cartographer_ros_pkg_share, 'launch', 'occupancy_grid.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'False',
            'resolution': '0.05'
        }.items()
    )

    # 5. 启动RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        ydlidar_node,
        tf_laser_node,
        cartographer_node,
        occupancy_grid_node,
        rviz_node
    ])
