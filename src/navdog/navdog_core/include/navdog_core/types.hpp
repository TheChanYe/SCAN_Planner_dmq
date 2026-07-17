#pragma once

#include <navdog_task/task_types.hpp>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace navdog
{

using navdog_task::NavigationEvent;
using navdog_task::NavigationEventType;
using navdog_task::NavigationTask;
using navdog_task::RoutePoint;
using navdog_task::TaskHandleResult;
using navdog_task::TaskMode;
using navdog_task::TaskSession;

// =============================================================================
// 4.1 任务模式
// =============================================================================

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
// 4.10b 路线走廊评估来源
// =============================================================================

enum class RouteCorridorSource : std::uint8_t
{
  NONE = 0,
  SCAN_INFLATED_GRID_3D
};

// =============================================================================
// 4.10c 路线走廊评估结果
// =============================================================================

struct RouteCorridorAssessment
{
  // 评估数据来源。
  RouteCorridorSource source{
      RouteCorridorSource::NONE};

  std::uint64_t task_sequence{0};

  // true 表示前方被检查的路线走廊中存在碰撞。
  bool blocked{false};

  // 本次评估是从原始路线的哪个累计进度开始。
  double evaluated_from_arc_length_m{0.0};

  // 本次实际检查的前方路线长度。
  double checked_distance_m{0.0};

  // 最早膨胀占据采样点距离评估起点的距离。
  // CLEAR 时保持 infinity。
  double first_blocked_distance_ahead_m{
      std::numeric_limits<double>::infinity()};

  // 最早膨胀占据采样点在原始路线上的累计距离。
  // CLEAR 时保持 infinity。
  double first_blocked_arc_length_m{
      std::numeric_limits<double>::infinity()};

  // SCAN 三维膨胀栅格地图的分辨率。
  double map_resolution_m{0.0};

  // 实际路线采样间隔，预期约为 resolution / 2。
  double sample_step_m{0.0};

  // 本次使用的身体中心查询高度。
  double query_z_m{0.0};

  std::size_t samples_checked{0};

  // 只要任何必查采样点位于地图外，就置 true。
  bool out_of_map{false};

  // SCAN 三维地图实际最后更新时间。
  double map_stamp_sec{0.0};

  // 本评估结果生成时间。
  double evaluation_stamp_sec{0.0};

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

  // 外部 SCAN 适配层提供的最新路线走廊评估。
  RouteCorridorAssessment route_corridor_observation{};
};

// =============================================================================
// 4.16a 导航子模式
// =============================================================================

enum class NavigationMode : std::uint8_t
{
  NONE = 0,

  ROUTE_FOLLOW,
  LOCAL_AVOID
};

// =============================================================================
// 4.16b 参考意图
// =============================================================================

enum class ReferenceIntent : std::uint8_t
{
  NONE = 0,

  GLOBAL_ROUTE,
  LOCAL_AVOIDANCE
};

// =============================================================================
// 4.16c 模式原因
// =============================================================================

enum class NavigationModeReason : std::uint8_t
{
  NONE = 0,

  INITIALIZED,

  ROUTE_CLEAR,
  BLOCK_IMMEDIATE,

  ROUTE_ONLY_BLOCKED,

  LOCAL_AVOID_ACTIVE,

  WAITING_FOR_CORRIDOR,
  WAITING_FOR_ROBOT,

  TASK_CHANGED
};

// =============================================================================
// 4.16d 模式状态
// =============================================================================

struct NavigationModeStatus
{
  NavigationMode mode{NavigationMode::NONE};
  NavigationMode previous_mode{NavigationMode::NONE};

  ReferenceIntent reference_intent{
      ReferenceIntent::NONE};

  NavigationModeReason reason{
      NavigationModeReason::NONE};

  std::uint64_t task_sequence{0};

  // 模式是否已经初始化。
  bool initialized{false};

  // 本次 update 是否发生模式变化。
  bool transitioned{false};

  // 当前 Corridor Gate 是否提供了可用的 CLEAR/BLOCKED。
  bool corridor_available{false};

  // 当前有效评估是否为 BLOCKED。
  bool route_blocked{false};

  // BLOCKED 且 first_blocked_distance <= 2.0 m。
  bool route_blocked_near{false};

  // 当前任务是否允许进入 LOCAL_AVOID。
  bool avoidance_allowed{false};

  // 当前模式进入时间。
  double mode_enter_stamp_sec{0.0};

  // 最近一次模式切换时间。
  double transition_stamp_sec{0.0};

  // 连续近距离 BLOCKED 持续时间。
  double blocked_confirm_elapsed_sec{0.0};

  // 连续 CLEAR 持续时间。
  double clear_confirm_elapsed_sec{0.0};

  // 本任务累计进入 LOCAL_AVOID 的次数。
  std::uint32_t avoidance_cycle_count{0};

  double stamp_sec{0.0};
  bool valid{false};
};

// =============================================================================
// 4.16e 三维占据查询抽象接口
//
// navdog_core 不直接依赖 GridMap / SCAN，通过此接口由外部实现注入。
// =============================================================================

class OccupancyQuery3D
{
public:
  virtual ~OccupancyQuery3D() = default;

  virtual bool ready() const noexcept = 0;

  // true: 点位于地图内且不在三维膨胀占据中。
  virtual bool isFree(
      double x,
      double y,
      double z,
      double yaw) const noexcept = 0;
};

// =============================================================================
// 4.16f 带时间的轨迹点
// =============================================================================

struct TimedTrajectoryPoint
{
  double time_from_start_sec{0.0};

  double x{0.0};
  double y{0.0};
  double z{0.0};

  double vx{0.0};
  double vy{0.0};

  double yaw{0.0};
  bool has_yaw{false};
};

// =============================================================================
// 4.16f 局部重规划原因与请求
// =============================================================================

enum class LocalReplanReason : std::uint8_t
{
  NONE = 0,
  ENTER_AVOID,
  TRAJECTORY_ENDING,
  FUTURE_COLLISION,
  PREVIOUS_FAILED,
  TASK_CHANGED
};

struct LocalPlanRequest
{
  RoutePoint start{};
  RoutePoint start_vel{};

  RoutePoint target{};
  RoutePoint target_vel{};

  NavigationMode purpose{NavigationMode::NONE};
  LocalReplanReason reason{LocalReplanReason::NONE};

  std::uint64_t task_sequence{0};
  std::uint64_t plan_sequence{0};

  double max_vx{0.4};
  double robot_z{0.0};
  // Timestamp supplied by Core using the runtime's monotonic control clock.
  double request_stamp_sec{0.0};

  bool valid{false};
};

// =============================================================================
// 4.16g 局部规划状态
// =============================================================================

enum class LocalPlanState
{
  IDLE = 0,
  QUEUED,
  PLANNING,
  READY,
  FAILED
};

// =============================================================================
// 4.16h 局部轨迹
//
// SCAN LocalTrajData 经 adapter 采样后得到的通用轨迹描述。
// 必须定义在 LocalPlannerAdapter 之前，因为接口使用 LocalTrajectory。
// =============================================================================

struct LocalTrajectory
{
  std::uint64_t task_sequence{0};
  std::uint64_t plan_sequence{0};

  NavigationMode purpose{
      NavigationMode::NONE};

  std::vector<TimedTrajectoryPoint> points{};

  double source_stamp_sec{0.0};
  double duration_sec{0.0};

  bool valid{false};
};

// =============================================================================
// 4.16i 局部规划适配器抽象接口
//
// navdog_core 不直接依赖任何具体局部规划器、地图或线性代数实现。
// 由 navdog_scan_adapter 等包提供具体实现并注入到 Coordinator。
// =============================================================================

class LocalPlannerAdapter
{
public:
  virtual ~LocalPlannerAdapter() = default;

  // 异步请求一次局部规划。返回 true 表示请求已成功提交到适配器队列。
  virtual bool requestLocalPlan(
      const LocalPlanRequest& request) = 0;

  // 获取当前最新轨迹。调用方负责匹配 task_sequence / purpose / plan_sequence。
  virtual LocalTrajectory getLocalTrajectory(
      NavigationMode purpose,
      std::uint64_t task_sequence) const = 0;

  virtual bool hasValidTrajectory(
      NavigationMode purpose,
      std::uint64_t task_sequence) const = 0;

  // 查询指定请求的当前规划状态。
  virtual LocalPlanState localPlanState(
      NavigationMode purpose,
      std::uint64_t task_sequence,
      std::uint64_t plan_sequence) const = 0;

  // 检查当前轨迹从 from_time_sec 开始是否与三维膨胀占据发生碰撞。
  // 没有轨迹、查询不可用或地图外时返回 true（视为不可执行）。
  virtual bool isTrajectoryColliding(
      const LocalTrajectory& trajectory,
      double from_time_sec) const = 0;
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
  NavigationModeStatus navigation_mode{};
};

}  // namespace navdog
