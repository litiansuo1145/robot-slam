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
    ekf_config = 'ekf.yaml'
    rviz_config = 'slam.rviz'  
    
    lidar_frame = 'pavo2s_frame'
    imu_frame = 'imu_link' # EKF 配置文件里通常叫 imu_link
    # =======================================

    pkg_share = FindPackageShare(package_name)
    
    # ====================================================
    # 1. 核心硬件驱动
    # ====================================================
    
    # 1.1 图传节点 (SSH 必须加 DISPLAY=:0)
    video_node = Node(
        package=package_name,
        executable='video_node',
        name='usv_video_zmq',
        output='screen',
        additional_env={'DISPLAY': ':0'} 
    )

    # 1.2 电机驱动 (ttyACM0)
    motor_node = Node(
        package=package_name,
        executable='motor_node',
        name='usv_motor_driver',
        output='screen'
    )

    # ====================================================
    # 2. 传感器驱动
    # ====================================================

    # 2.1 雷达驱动 (Pavo2s - 网口)
    pavo_pkg = FindPackageShare('pavo2s_ros')
    start_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pavo_pkg, 'launch', 'pavo2s_scan.launch.py'])
        )
    )

    # 2.2 IMU 驱动 (【已修改】替换为你自己的 YZF143)
    # 对应命令: ros2 run imu_yzf143_ros2 imu_node
    start_imu = Node(
        package='imu_yzf143_ros2',
        executable='imu_node',
        name='imu_node',
        output='screen',
        parameters=[{
            'port': '/dev/ttyUSB0',   
            'baud': 115200,          
            'frame_id': imu_frame,
            # 如果驱动不支持 topic_name 参数，下面这行可能无效，但没关系，重点是 remappings
            'topic_name': '/imu_data' 
        }],
        remappings=[('imu_data', '/imu_data')]
    )

    # ====================================================
    # 3. 算法与变换
    # ====================================================

    # 3.1 静态 TF (Base -> Laser)
    tf_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=['0.1', '0', '0', '0', '0', '0', 'base_link', lidar_frame]
    )

    # 3.2 【新增】静态 TF (Base -> IMU)
    # 必须告诉 EKF IMU 在哪里，不然会丢包。假设 IMU 就在车中心
    tf_base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_imu_tf',
        arguments=['0.0', '0', '0', '0', '0', '0', 'base_link', imu_frame]
    )

    # 3.3 EKF 节点
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[PathJoinSubstitution([pkg_share, 'config', ekf_config])]
    )

    # 3.4 SLAM Toolbox
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[PathJoinSubstitution([pkg_share, 'config', slam_config])],
    )

    # 3.5 Rviz
    rviz_config_path = PathJoinSubstitution([pkg_share, 'rviz', rviz_config])
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        video_node,
        motor_node,
        start_lidar,
        start_imu,
        tf_base_to_laser,
        tf_base_to_imu,  # 记得加入这个
        ekf_node,
        slam_toolbox,
        rviz
    ])