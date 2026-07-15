#pragma once

namespace navdog
{

// =============================================================================
// 5.1 RuntimeConfig
// =============================================================================

struct RuntimeConfig
{
  double control_rate_hz{50.0};
  double status_rate_hz{10.0};
};

// =============================================================================
// 5.2 TaskConfig
// =============================================================================

// =============================================================================
// 5.3 StartAlignConfig
// =============================================================================

struct StartAlignConfig
{
  double enter_deg{15.0};
  double exit_deg{5.0};

  double max_hold_sec{2.0};

  double kp_yaw{1.2};
  double max_yaw_rate{0.3};

  double target_min_dist_m{0.20};
};

// =============================================================================
// 5.4 RouteProgressConfig
// =============================================================================

struct RouteProgressConfig
{
  // 小于此长度的路线段视为重复点或退化段。
  double min_segment_length_m{0.01};

  // 每次更新最多允许从当前进度向前搜索的路线距离。
  // 防止闭环、交叉路线中直接跳到很远的未来路线段。
  double max_forward_search_m{3.0};

  // 仅用于输出 on_route 诊断，不参与路线进度有效性判断。
  double on_route_lateral_tolerance_m{0.30};
};

// =============================================================================
// 5.4b RouteCorridorConfig
// =============================================================================

struct RouteCorridorConfig
{
  // 从当前路线进度向前检查的最大路线距离。
  double lookahead_distance_m{3.0};
};

// =============================================================================
// 5.4c RouteCorridorObservationConfig
// =============================================================================

struct RouteCorridorObservationConfig
{
  // SCAN 三维地图数据允许的最大时间年龄。
  double map_timeout_sec{0.30};

  // 评估起点相对当前 RouteProgress 最多允许落后多少。
  // 用于跨线程或跨节点传递时过滤旧结果。
  double max_progress_lag_m{0.50};
};

// =============================================================================
// 5.5 GoalConfig
// =============================================================================

struct GoalConfig
{
  double switch_dist{0.8};
  double finish_dist{0.15};
  double yaw_tolerance_deg{5.0};
};

// =============================================================================
// 5.5 PlannerConfig
// =============================================================================

struct PlannerConfig
{
  double planning_timeout_sec{5.0};
};

// =============================================================================
// 5.6 SafetyConfig
// =============================================================================

struct SafetyConfig
{
  double slow_down_front{1.5};
  double emergency_stop{0.45};

  double odom_timeout_sec{0.5};
  double obstacle_timeout_sec{0.5};
  double planner_cmd_timeout_sec{0.3};
  double future_tolerance_sec{0.05};
};

// =============================================================================
// 5.7 LimitConfig
// =============================================================================

struct LimitConfig
{
  double max_vx{1.0};
  double max_vy{0.35};
  double max_yaw_rate{0.65};

  double max_accel_x{0.5};
  double max_accel_y{0.4};
  double max_accel_yaw{0.8};
};

// =============================================================================
// 5.7b NavigationModeConfig
// =============================================================================

struct NavigationModeConfig
{
  // BLOCKED 进入正式绕障判断范围。
  double avoid_enter_distance_m{1.50};

  // 小于等于该距离时，不等待确认，立即进入 LOCAL_AVOID。
  double avoid_immediate_distance_m{0.80};

  // 普通近距离 BLOCKED 必须连续保持的时间。
  double avoid_block_confirm_sec{0.20};

  // LOCAL_AVOID 进入后最短保持时间。
  double local_avoid_min_hold_sec{0.50};

  // 原始路线连续 CLEAR 后才能尝试接回。
  double route_clear_confirm_sec{0.40};

  // 接回路线的横向误差要求。
  double rejoin_lateral_tolerance_m{0.20};

  // 12 degrees。
  double rejoin_heading_tolerance_rad{
      0.20943951023931953};

  // 横向和航向条件连续满足时间。
  double rejoin_confirm_sec{0.30};
};

// =============================================================================
// 5.7c RouteFollowerConfig
// =============================================================================

struct RouteFollowerConfig
{
  double lookahead_distance_m{1.0};
  double kp_x{0.8};
  double kp_y{1.0};
  double kp_yaw{1.2};

  double heading_turn_only_threshold_rad{
      0.8};

  double max_vx{0.8};
};

// =============================================================================
// 5.7d TrajectoryFollowerConfig
// =============================================================================

struct TrajectoryFollowerConfig
{
  double time_forward_sec{0.8};
  double kp_pos{0.8};
  double kp_yaw{1.5};

  double heading_turn_only_threshold_rad{
      0.8};
};

// =============================================================================
// 5.7e RejoinTargetSelectorConfig
// =============================================================================

struct RejoinTargetSelectorConfig
{
  double default_forward_distance_m{2.5};
  double min_forward_distance_m{1.0};
  double max_forward_distance_m{3.0};

  double route_yaw_tolerance_rad{
      0.3490658503988659};  // 20 degrees
};

// =============================================================================
// 5.7f GoalControllerConfig
// =============================================================================

struct GoalControllerConfig
{
  double near_goal_switch_dist{0.8};
  double near_goal_kp_v{0.3};
  double near_goal_min_v{0.10};
  double near_goal_max_v{0.25};

  double near_goal_turn_only_deg{88.0};
  double near_goal_turn_only_rad{
      1.53588974175801};  // 88 degrees

  double near_goal_kp_w{1.2};
  double near_goal_max_w{0.22};
  double obstacle_finish_timeout_sec{5.0};

  double finish_dist{0.15};
  double finish_yaw_tolerance_deg{5.0};
  double finish_yaw_tolerance_rad{
      0.08726646259971647};  // 5 degrees
};

// =============================================================================
// 5.7g PlannerTriggerConfig
// =============================================================================

struct PlannerTriggerConfig
{
  // 规划失败后再次请求的最小间隔。
  double replan_retry_interval_sec{0.30};

  // 轨迹过期裕量：exec_time > duration + margin 才视为失效。
  double trajectory_expiry_margin_sec{0.2};

  // 轨迹剩余时间小于此值时触发重规划。
  double min_remaining_duration_sec{0.3};

  // 局部轨迹 source_stamp 允许的最大年龄（仅在接受新轨迹时检查一次）。
  double trajectory_source_max_age_sec{0.30};

  // 局部轨迹 source_stamp 允许的未来容差（仅在接受新轨迹时检查一次）。
  double trajectory_future_tolerance_sec{0.05};

  // 目标点变化超过此阈值时触发重规划。
  double target_change_threshold_m{0.30};
};

// =============================================================================
// 5.8 总配置
// =============================================================================

struct NavdogConfig
{
  RuntimeConfig runtime{};
  PlannerConfig planner{};
  StartAlignConfig start_align{};
  RouteProgressConfig route_progress{};
  RouteCorridorConfig route_corridor{};
  RouteCorridorObservationConfig route_corridor_observation{};
  GoalConfig goal{};
  SafetyConfig safety{};
  LimitConfig limits{};
  NavigationModeConfig navigation_mode{};
  RouteFollowerConfig route_follower{};
  TrajectoryFollowerConfig trajectory_follower{};
  RejoinTargetSelectorConfig rejoin_target{};
  GoalControllerConfig goal_controller{};
  PlannerTriggerConfig planner_trigger{};
};

}  // namespace navdog
