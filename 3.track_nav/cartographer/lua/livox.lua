

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,
  use_odometry = false,          -- 不使用轮式里程计 (无人机无轮式里程计)
  use_nav_sat = false,           -- 室内无 GPS
  use_landmarks = false,
  num_laser_scans = 1,           -- 单激光扫描输入
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

-- ====== 扫描匹配参数 ======
TRAJECTORY_BUILDER_2D.min_range = 0.1           -- 最小距离 (提升，避免近距离噪声)
TRAJECTORY_BUILDER_2D.max_range = 25.0          -- 最大距离 (Mid-360 室内 25m 足够，减少远距噪声)
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 8.5

-- 不使用 IMU (FAST-LIO2 已处理 IMU 融合，2D SLAM 不需要重复)
TRAJECTORY_BUILDER_2D.use_imu_data = false

-- Ceres 扫描匹配权重 (提高 translation_weight 减少平移漂移)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 0.5   -- 原 0.2 -> 0.5
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 5.0

-- 实时相关扫描匹配 (提高鲁棒性，尤其适用于快速移动的无人机)
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15    -- 原 0.1 -> 0.15 (稍大搜索窗口)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(5.)  -- 新增: 允许 ±5° 旋转搜索
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 10.

-- 运动过滤器 (降低灵敏度，确保移动中也能持续建图)
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.3)  -- 原 0.2 -> 0.3 度

-- 累积帧数 (1 = 每帧都处理)
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

-- Z 轴过滤范围 (2D 激光切片已在 pointcloud_to_laserscan 阶段完成，
-- 这里设宽一些作为二次过滤，避免极端噪声)
TRAJECTORY_BUILDER_2D.min_z = -0.5
TRAJECTORY_BUILDER_2D.max_z = 1.5

-- ====== 位姿图优化 ======
-- 提高优化频率以更快收敛闭环
POSE_GRAPH.constraint_builder.min_score = 0.62                  -- 原 0.65 -> 0.62 (放宽回环检测)
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.62  -- 同步放宽
POSE_GRAPH.optimization_problem.huber_scale = 1e2
POSE_GRAPH.optimize_every_n_nodes = 20  -- 原 30 -> 20 (更频繁优化)

return options
