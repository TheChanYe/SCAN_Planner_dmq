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

struct TaskConfig
{
  double default_max_vx{0.4};
  double min_max_vx{0.15};
  double max_max_vx{1.0};
};

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
// 5.8 总配置
// =============================================================================

struct NavdogConfig
{
  RuntimeConfig runtime{};
  TaskConfig task{};
  PlannerConfig planner{};
  StartAlignConfig start_align{};
  RouteProgressConfig route_progress{};
  RouteCorridorConfig route_corridor{};
  RouteCorridorObservationConfig route_corridor_observation{};
  GoalConfig goal{};
  SafetyConfig safety{};
  LimitConfig limits{};
};

}  // namespace navdog
