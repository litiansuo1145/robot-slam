import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 定义所有需要用到的文件和目录的路径
    robot_slam_pkg = get_package_share_directory('robot_slam')
    cartographer_config_dir = os.path.join(robot_slam_pkg, 'config')
    configuration_basename = 'cartographer.lua'

    lidar_pkg = get_package_share_directory('rplidar_ros')
    lidar_launch_file = os.path.join(lidar_pkg, 'launch', 'rplidar_c1_launch.py')

    imu_pkg = get_package_share_directory('witmotion_ros2')
    imu_launch_file = os.path.join(imu_pkg, 'launch', 'witmotion.launch.py')
    
    rviz_config_file = os.path.join(robot_slam_pkg, 'rviz', 'slam.rviz')

    return LaunchDescription([
        # --- 静态坐标变换 (TF) ---
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_laser',
            arguments=['0.1', '0', '0.05', '0', '0', '0', 'base_link', 'laser_frame'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_imu',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link'],
        ),

        # --- 包含您的驱动启动文件 ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch_file),
            launch_arguments={'frame_id': 'laser_frame'}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(imu_launch_file)
        ),

        # --- Cartographer SLAM 节点 ---
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', configuration_basename],
        ),

        # --- Occupancy Grid 节点 ---
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node', # <-- 已修正为正确的名字
            name='occupancy_grid_node',
            output='screen',
            arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
        ),

        # --- RViz2 可视化 ---
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen',
        ),
    ])