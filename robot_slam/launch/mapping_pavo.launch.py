import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # ================= 1. 核心配置区 =================
    
    package_name = 'robot_slam'
    lua_file_name = 'cartographer_new.lua'

    # --- 雷达配置 (Pavo2s) ---
    lidar_topic = '/pavo2s_scan'
    lidar_frame = 'pavo2s_frame'
    # 雷达安装位置 [x, y, z, yaw, pitch, roll]
    lidar_tf_args = ['0.1', '0.0', '0.0', '0.0', '0.0', '0.0']

    # --- IMU 配置 ---
    imu_topic = '/imu' 
    # 重点修改：因为你的 yaml 里写的是 base_link，这里不需要额外的 TF 了
    # Cartographer 会直接认为 IMU 数据就是底盘的数据
    
    # ===============================================

    pkg_share = get_package_share_directory(package_name)
    config_dir = os.path.join(pkg_share, 'config')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'slam.rviz')

    # 获取驱动 launch 文件路径
    try:
        pavo_pkg = get_package_share_directory('pavo2s_ros')
        pavo_launch = os.path.join(pavo_pkg, 'launch', 'pavo2s_scan.launch.py')
        #imu_pkg = get_package_share_directory('witmotion_ros2')
        #imu_launch = os.path.join(imu_pkg, 'launch', 'witmotion.launch.py')
    except Exception:
        print("Error: 找不到驱动包")
        pavo_launch = ''
        #imu_launch = ''

    return LaunchDescription([
        # 1. 启动硬件
        IncludeLaunchDescription(PythonLaunchDescriptionSource(pavo_launch)),
        #IncludeLaunchDescription(PythonLaunchDescriptionSource(imu_launch)),

        # 2. 发布 TF (只发布雷达的，IMU的不需要了)
       # Node(
       #     package='tf2_ros',
       #     executable='static_transform_publisher',
       #     name='base_to_pavo_tf',
       #     arguments=[*lidar_tf_args, 'base_link', lidar_frame]
       # ),
        
        # 注意：这里删除了 base_link -> imu_link 的 TF 节点
        # 因为 IMU 数据已经是 base_link 了，如果再发 TF 会造成循环引用报错

        # 3. Cartographer 节点
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=[
                '-configuration_directory', config_dir,
                '-configuration_basename', lua_file_name,
            ],
            remappings=[
                ('/scan', lidar_topic),
                #('/imu', imu_topic),
            ]
        ),

        # 4. 栅格地图节点
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': False, 'resolution': 0.05}]
        ),

        # 5. Rviz2
       # Node(
       #     package='rviz2',
       #     executable='rviz2',
       #     name='rviz2',
       #     arguments=['-d', rviz_config_file],
       # output='screen',
       # ),
    ])