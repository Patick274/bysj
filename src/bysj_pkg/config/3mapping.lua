include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  
  -- 坐标系名称设定（必须和我们之前 Launch 里的一致）
  map_frame = "map",
  tracking_frame = "imu_link",   -- 【实验二核心】追踪底盘中心
  published_frame = "base_link",
  odom_frame = "odom",
  
  -- 是否由 Cartographer 自己提供里程计（也就是它来发布 map -> odom 的 TF）
  provide_odom_frame = false,      
  publish_frame_projected_to_2d = true,
  use_pose_extrapolator = true,
  
  -- 传感器开关控制 (消融实验的变量)
  use_odometry  = true,            --【实验一、二核心】彻底关闭外部轮式里程计
  use_nav_sat   = false,            -- 关闭 GPS
  use_landmarks = false,            -- 关闭路标
  
  -- 激光雷达配置
  num_laser_scans = 1,            -- 我们只有一个单线雷达
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- 2D 轨迹构建器具体参数
TRAJECTORY_BUILDER_2D.use_imu_data = true   --【实验二核心】开启 IMU 融合
TRAJECTORY_BUILDER_2D.min_range = 0.12      -- 雷达最近探测距离 (防遮挡)
TRAJECTORY_BUILDER_2D.max_range = 8.0       -- 雷达最远探测距离 (YDLIDAR X2 最大约 8 米)
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0

-- 开启在线相关性扫描匹配 (纯激光 SLAM 极其依赖这个来推算位置)
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1
-- 重力常数 需要修改
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 10.
return options