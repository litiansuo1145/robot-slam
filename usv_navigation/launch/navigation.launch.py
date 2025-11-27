import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # ========================== 路径和参数定义 ==============================
    usv_nav_pkg_dir = get_package_share_directory('usv_navigation')
    nav2_bringup_pkg_dir = get_package_share_directory('nav2_bringup')
    
    lidar_pkg_name = 'rplidar_ros'
    imu_pkg_name = 'witmotion_ros2'

    lidar_pkg_dir = get_package_share_directory(lidar_pkg_name)
    imu_pkg_dir = get_package_share_directory(imu_pkg_name)

    urdf_file_path = os.path.join(usv_nav_pkg_dir, 'urdf', 'my_robot.urdf')
    map_yaml_path = os.path.join(usv_nav_pkg_dir, 'maps', 'my_map.yaml')
    nav2_params_yaml_path = os.path.join(usv_nav_pkg_dir, 'config', 'nav2_params.yaml')
    ekf_config_path = os.path.join(usv_nav_pkg_dir, 'config', 'ekf.yaml')
    rviz_config_path = os.path.join(usv_nav_pkg_dir, 'rviz', 'navigation.rviz')
    
    robot_description_raw = xacro.process_file(urdf_file_path).toxml()

    # ========================== 定义要启动的节点 ==============================

    # --- 硬件驱动 ---
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lidar_pkg_dir, 'launch', 'rplidar_c1_launch.py')
        ),
        launch_arguments={'frame_id': 'laser_frame'}.items()
    )
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(imu_pkg_dir, 'launch', 'witmotion.launch.py')
        )
    )
    
    # --- 机器人核心 ---
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw, 'use_sim_time': False}]
    )
    usv_controller_node = Node(
        package='usv_navigation',
        executable='usv_controller',
        name='usv_controller',
        output='screen'
    )
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path],
    )

    # --- 启动我们自己的、唯一的 Map Server ---
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': False}, 
                    {'yaml_filename': map_yaml_path}]
    )

    # --- 启动我们自己的、唯一的 AMCL 定位节点 ---
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params_yaml_path]
    )

    # --- 手动启动 Nav2 的核心组件，不包含定位和地图 ---
    # 这个 navigation_launch.py 只启动 controller, planner, behavior server 等
    navigation2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_pkg_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'False',
            'params_file': nav2_params_yaml_path,
            'autostart': 'False',  # 【关键】设置为 False，让我们的统一管理器来激活它们
        }.items()
    )
    
    # --- 启动一个【统一的】生命周期管理器来管理【所有】组件 ---
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'autostart': True},
                    {'node_names': [ # 明确列出所有需要管理的节点
                        'map_server',
                        'amcl',
                        'controller_server',
                        'smoother_server',
                        'planner_server',
                        'behavior_server',
                        'bt_navigator',
                        'waypoint_follower',
                        'velocity_smoother'
                    ]}]
    )

    # --- 可视化 ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path]
    )

    # ========================== 返回启动描述 ==============================
    return LaunchDescription([
        lidar_launch,
        imu_launch,
        robot_state_publisher_node,
        usv_controller_node,
        ekf_node,
        
        # 手动启动地图和定位
        map_server_node,
        amcl_node,

        # 启动Nav2核心
        navigation2_launch,

        # 启动统一的管理器
        lifecycle_manager_node,
        
        rviz_node
    ])