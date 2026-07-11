#pragma once

#include <cstdint>
#include <limits>
#include <vector>

namespace navdog
{

// =============================================================================
// 4.1 任务模式
// =============================================================================

enum class TaskMode : std::uint8_t
{
  NORMAL_AVOID = 1,
  ROUTE_ONLY = 2,
  CHARGING = 3
};

// =============================================================================
// 4.2 导航状态
// =============================================================================

enum class NavState : std::uint8_t
{
  IDLE = 0,
  PLANNING,
  START_ALIGN,
  TRACKING,
  PAUSED,
  RECOVERY,
  GOAL_ALIGN,
  SUCCEEDED,
  EMERGENCY_STOP,
  FAILED
};

// =============================================================================
// 4.3 规划器状态
// =============================================================================

enum class PlannerState : std::uint8_t
{
  UNAVAILABLE = 0,
  IDLE,
  PLANNING,
  READY,
  EXECUTING,
  FAILED
};

// =============================================================================
// 4.4 速度来源
// =============================================================================

enum class CommandSource : std::uint8_t
{
  NONE = 0,
  IDLE_STOP,
  PLANNING_STOP,
  FAILED_STOP,
  PLANNER,
  START_ALIGN,
  GOAL_ALIGN,
  PAUSE_STOP,
  CANCEL_STOP,
  SAFETY_SLOW,
  SAFETY_STOP,
  RECOVERY
};

// =============================================================================
// 4.5 规划动作类型
// =============================================================================

enum class PlannerActionType : std::uint8_t
{
  NONE = 0,
  SET_ROUTE,
  CANCEL,
  PAUSE,
  RESUME,
  UPDATE_SPEED_LIMIT,
  REQUEST_REPLAN
};

// =============================================================================
// 4.6 导航事件类型
// =============================================================================

enum class NavigationEventType : std::uint8_t
{
  NONE = 0,
  START_TASK,
  CANCEL_TASK,
  PAUSE,
  RESUME,
  UPDATE_MAX_VX,
  DYNAMIC_OBSTACLE_UPDATE
};

// =============================================================================
// 4.7 路线点
// =============================================================================

struct RoutePoint
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw{0.0};
  bool has_yaw{false};
};

// =============================================================================
// 4.8 导航任务
// =============================================================================

struct NavigationTask
{
  std::uint64_t sequence{0};
  TaskMode mode{TaskMode::NORMAL_AVOID};
  double max_vx{0.4};
  std::vector<RoutePoint> points;
};

// =============================================================================
// 4.9 机器人状态
// =============================================================================

struct RobotState
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw{0.0};

  double vx{0.0};
  double vy{0.0};
  double yaw_rate{0.0};

  double stamp_sec{0.0};
  bool valid{false};
};

// =============================================================================
// 4.10 障碍物摘要
// =============================================================================

struct ObstacleSummary
{
  double front_min{std::numeric_limits<double>::infinity()};
  double left_min{std::numeric_limits<double>::infinity()};
  double right_min{std::numeric_limits<double>::infinity()};
  double rear_min{std::numeric_limits<double>::infinity()};

  double stamp_sec{0.0};
  bool valid{false};
};

// =============================================================================
// 4.11 速度命令
// =============================================================================

struct VelocityCommand
{
  double vx{0.0};
  double vy{0.0};
  double yaw_rate{0.0};

  double stamp_sec{0.0};
  bool valid{false};
  CommandSource source{CommandSource::NONE};
};

// =============================================================================
// 4.12 规划器反馈
// =============================================================================

struct PlannerFeedback
{
  PlannerState state{PlannerState::UNAVAILABLE};

  // Core correlation id. The ROS adapter must set this to the
  // NavigationTask::sequence associated with the planner request.
  std::uint64_t trajectory_id{0};

  // Must use the same time domain as NavigationCoordinator::update(now_sec).
  double stamp_sec{0.0};

  bool valid{false};
};

// =============================================================================
// 4.13 导航事件
// =============================================================================

struct NavigationEvent
{
  NavigationEventType type{NavigationEventType::NONE};
  NavigationTask task{};
  double max_vx{0.0};
};

// =============================================================================
// 4.14 规划动作
// =============================================================================

struct PlannerAction
{
  PlannerActionType type{PlannerActionType::NONE};
  NavigationTask task{};
  double max_vx{0.0};
};

// =============================================================================
// 4.15 核心输入
// =============================================================================

struct CoreInput
{
  RobotState robot{};
  ObstacleSummary obstacles{};
  PlannerFeedback planner{};
  VelocityCommand planner_cmd{};
};

// =============================================================================
// 4.16 核心输出
// =============================================================================

struct CoreOutput
{
  NavState state{NavState::IDLE};
  VelocityCommand final_cmd{};
  PlannerAction planner_action{};
  std::uint64_t task_sequence{0};
};

}  // namespace navdog
