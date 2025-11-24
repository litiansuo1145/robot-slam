include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  -- 直接追踪雷达坐标系，绕过 TF 延迟问题
  tracking_frame = "pavo2s_frame",
  published_frame = "pavo2s_frame",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = false,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  
  -- 宽容的时间同步设置
  lookup_transform_timeout_sec = 1.0,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  
  -- 降采样：只处理 50% 的数据，应对 25Hz 高频
  rangefinder_sampling_ratio = 0.5,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

-- 【这就是报错缺失的那一行，必须加在这里！】
MAP_BUILDER.use_trajectory_builder_2d = true

-- 减少后台线程，防止 Jetson 卡死
MAP_BUILDER.num_background_threads = 2

-- 纯雷达模式参数
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.min_range = 0.6
TRAJECTORY_BUILDER_2D.max_range = 20.0
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.1

-- 开启 CSM 暴力匹配
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1.0

TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)

return options