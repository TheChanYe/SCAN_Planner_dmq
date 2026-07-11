#pragma once

#include <cstddef>
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
  TRACKING_STOP,
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
// 4.8b 路线进度
// =============================================================================

struct RouteProgress
{
  // 对应当前 NavigationTask::sequence。
  std::uint64_t task_sequence{0};

  // 原始 NavigationTask::points 中的路线段起点索引。
  // 表示当前投影位于 points[segment_index]
  // 到 points[segment_index + 1] 之间。
  std::size_t segment_index{0};

  // 当前投影在线段上的比例，范围 [0, 1]。
  double segment_ratio{0.0};

  // 从路线起点到当前投影位置的累计长度。
  double arc_length_m{0.0};

  // 原始有效路线总长度。
  double total_length_m{0.0};

  // total_length_m - arc_length_m。
  double remaining_distance_m{0.0};

  // 当前机器人在路线上的投影点。
  double projected_x{0.0};
  double projected_y{0.0};

  // 当前有效路线段方向，单位 rad。
  double route_yaw{0.0};

  // 机器人当前位置到投影点的二维距离。
  double lateral_error_m{0.0};

  // lateral_error_m 是否在配置阈值内。
  bool on_route{false};

  // 与 NavigationCoordinator::update(now_sec)
  // 使用同一时间基准。
  double stamp_sec{0.0};

  bool valid{false};
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
// 4.9b 二维障碍物
// =============================================================================

struct ObstacleCircle
{
  // 必须与 NavigationTask 路点、RobotState
  // 使用相同的世界坐标系。
  double x{0.0};
  double y{0.0};

  // 障碍物当前已经具备的有效半径。
  //
  // 原始点障碍可使用 0。
  // 聚类障碍可以使用聚类本体半径。
  // 已膨胀障碍可以使用膨胀后的有效半径。
  //
  // 适配层和核心层不得对同一膨胀量重复计算。
  double effective_radius_m{0.0};
};

struct ObstacleField
{
  // 此集合应由适配层完成必要的降采样或聚类。
  // 不建议直接把完整高密度点云逐点传入核心。
  std::vector<ObstacleCircle> obstacles;

  // 与 NavigationCoordinator::update(now_sec)
  // 使用同一时间基准。
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
// 4.10b 路线走廊评估结果
// =============================================================================

struct RouteCorridorAssessment
{
  std::uint64_t task_sequence{0};

  // true 表示前方被检查的路线走廊中存在碰撞。
  bool blocked{false};

  // 本周期实际检查的前方路线长度。
  // 路线剩余距离不足 lookahead 时可以小于配置值。
  double checked_distance_m{0.0};

  // 最早碰撞点距离当前路线进度多远。
  // CLEAR 时保持 infinity。
  double first_blocked_distance_ahead_m{
      std::numeric_limits<double>::infinity()};

  // 最早碰撞点在原始路线上的累计长度。
  // CLEAR 时保持 infinity。
  double first_blocked_arc_length_m{
      std::numeric_limits<double>::infinity()};

  // 所有已检查路线段与所有障碍物之间的最小有符号间隙。
  //
  // clearance =
  // 障碍物中心到路线段距离
  // - corridor_radius
  // - obstacle.effective_radius
  //
  // > 0：仍有安全间隙
  // = 0：刚好接触边界
  // < 0：发生重叠
  //
  // 没有障碍物时保持 infinity。
  double minimum_clearance_m{
      std::numeric_limits<double>::infinity()};

  // 造成最早阻塞的 ObstacleField::obstacles 索引。
  // CLEAR 时使用 size_t 最大值。
  std::size_t obstacle_index{
      std::numeric_limits<std::size_t>::max()};

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
  ObstacleField obstacle_field{};
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
  RouteProgress route_progress{};
  RouteCorridorAssessment route_corridor{};
};

}  // namespace navdog
