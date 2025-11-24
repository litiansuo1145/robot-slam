import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # ================= 配置 =================
    package_name = 'robot_slam'
    slam_config = 'slam_toolbox_params.yaml'
    ekf_config = 'ekf.yaml' # 你的 EKF 配置文件名
    
    lidar_frame = 'pavo2s_frame'
    # =======================================

    pkg_share = FindPackageShare(package_name)
    
    # 1. 雷达驱动
    pavo_pkg = FindPackageShare('pavo2s_ros')
    start_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pavo_pkg, 'launch', 'pavo2s_scan.launch.py'])
        )
    )

    # 2. IMU 驱动 (Witmotion)
    imu_pkg = FindPackageShare('witmotion_ros2')
    start_imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([imu_pkg, 'launch', 'witmotion.launch.py'])
        )
    )

    # 3. 静态 TF: base_link -> pavo2s_frame (连接底盘和雷达)
    # 【必须保留】
    tf_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=['0.1', '0', '0', '0', '0', '0', 'base_link', lidar_frame]
    )

    # 4. EKF 节点 (代替之前的 odom_to_base 静态 TF)
    # 这个节点会订阅 /imu，并发布 odom -> base_link 的动态 TF
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[PathJoinSubstitution([pkg_share, 'config', ekf_config])]
    )

    # 5. SLAM Toolbox
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[PathJoinSubstitution([pkg_share, 'config', slam_config])],
    )

    # 6. Rviz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        start_lidar,
        start_imu,
        tf_base_to_laser,
        # tf_odom_to_base, # <--- 这一行必须删掉或注释掉，否则 TF 会打架！
        ekf_node,
        slam_toolbox,
        rviz
    ])