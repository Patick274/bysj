include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  
  -- 1. 坐标系名称设定
  map_frame = "map",
  tracking_frame = "base_link",   -- 【实验一】纯激光没有IMU，必须追踪底盘
  published_frame = "base_link",
  odom_frame = "odom",
  
  -- 2. 里程计与传感器开关
  provide_odom_frame = true,      -- 【实验一】由算法自己推算位置
  publish_frame_projected_to_2d = true,
  use_pose_extrapolator = true,
  
  use_odometry = false,           -- 【实验一】关闭轮式里程计
  use_nav_sat = false,            
  use_landmarks = false,          
  
  -- 3. 雷达配置
  num_laser_scans = 1,            
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  
  -- 4. 频率与超时设置
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  
  -- 5. 【修复报错的核心】强制要求的采样率参数
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- 2D 轨迹构建器具体参数
TRAJECTORY_BUILDER_2D.use_imu_data = false  -- 【实验一】关闭 IMU 融合
TRAJECTORY_BUILDER_2D.min_range = 0.12      
TRAJECTORY_BUILDER_2D.max_range = 8.0       
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0

-- 开启在线相关性扫描匹配 (纯激光 SLAM 必须依赖这个)
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(5.0)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1.0

-- 【最强防漂移】运动过滤：静止时完全不建图
TRAJECTORY_BUILDER_2D.motion_filter = {
  max_time_seconds = 0.5,
  max_distance_meters = 0.02,    -- 移动小于2厘米不建图
  max_angle_radians = math.rad(0.5) -- 旋转小于0.5度不建图
}

-- 禁止累积点云造成残影
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
return options