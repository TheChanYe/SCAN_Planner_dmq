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
  double max_yaw_rate{0.3};
};

// =============================================================================
// 5.4 GoalConfig
// =============================================================================

struct GoalConfig
{
  double switch_dist{0.8};
  double finish_dist{0.15};
  double yaw_tolerance_deg{5.0};
};

// =============================================================================
// 5.5 SafetyConfig
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
// 5.6 LimitConfig
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
// 5.7 总配置
// =============================================================================

struct NavdogConfig
{
  RuntimeConfig runtime{};
  TaskConfig task{};
  StartAlignConfig start_align{};
  GoalConfig goal{};
  SafetyConfig safety{};
  LimitConfig limits{};
};

}  // namespace navdog
