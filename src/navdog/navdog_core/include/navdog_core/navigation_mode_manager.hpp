#pragma once

#include "navdog_core/config.hpp"
#include "navdog_core/route_corridor_observation_gate.hpp"
#include "navdog_core/types.hpp"

#include <cstdint>

namespace navdog
{

// =============================================================================
// NavigationModeUpdateResult
//
// NavigationModeManager::update() 的返回状态码，表示本次更新是否成功以及失败原因。
// =============================================================================

enum class NavigationModeUpdateResult : std::uint8_t
{
  IDLE = 0,               // 尚未调用过 update 的初始状态

  UPDATED,                // 本次更新成功完成
  WAITING_FOR_CORRIDOR,   // 路线走廊Corridor评估结果尚不可用/未就绪
  WAITING_FOR_ROBOT,      // 等待机器人状态（当前未使用，预留）

  INVALID_TIME,           // 时间戳非法（非有限数或时间倒流）
  INVALID_CONFIG,         // 配置参数非法
  INVALID_TASK,           // 任务非法（sequence为0或mode不支持）
  INVALID_PROGRESS,       // 路线进度无效或与当前任务不匹配
  INVALID_ROBOT,          // 机器人数值非法（NaN/Inf）
  INVALID_CORRIDOR_RESULT // 走廊Corridor评估结果非法或与当前任务不匹配
};

// =============================================================================
// NavigationModeOutput
//
// update() 的完整输出：结果码 + 当前完整的模式状态快照。
// =============================================================================

struct NavigationModeOutput
{
  NavigationModeUpdateResult result{
      NavigationModeUpdateResult::IDLE};

  NavigationModeStatus status{};
};

// =============================================================================
// NavigationModeManager
//
// 负责在 ROUTE_FOLLOW（沿全局路线跟踪）与 LOCAL_AVOID（交给SCAN局部避障）
// 两种导航模式之间切换的核心状态机。依据路线走廊（corridor）评估结果和
// 障碍物方向概况，结合"进入/退出确认时长"防抖机制，决定何时从全局跟踪
// 切换到局部避障，以及何时确认避障完成并返回全局跟踪。
// =============================================================================

class NavigationModeManager
{
public:
  // 构造函数：传入模式切换相关配置（进入/退出确认时长、距离阈值、
  // 最小停留时间等），默认使用 NavigationModeConfig 默认值。
  explicit NavigationModeManager(
      const NavigationModeConfig& config =
          NavigationModeConfig{},
      const StairUpConfig& stair_config = StairUpConfig{});

  // reset：将状态机完全重置到初始状态（清空当前模式、任务序号、所有确认计时器），
  // 在任务取消/重新开始导航时调用。
  void reset() noexcept;

  // update：状态机主入口，每个控制周期调用一次。
  // 输入：
  //   task      - 当前导航任务
  //   robot     - 机器人当前状态
  //   progress  - 路线跟踪进度
  //   corridor  - 路线走廊观测评估结果（前方是否被阻塞）
  //   obstacles - 障碍物方向概况（前/左/右最近距离，用于退出判定）
  //   now_sec   - 当前时间戳（秒）
  // 输出：NavigationModeOutput，包含结果码与完整模式状态。
  NavigationModeOutput update(
      const NavigationTask& task,
      const RobotState& robot,
      const RouteProgress& progress,
      const RouteCorridorObservationOutput& corridor,
      const ObstacleSummary& obstacles,
      double now_sec);

  NavigationModeOutput update(
      const NavigationTask& task,
      const RobotState& robot,
      const RouteProgress& progress,
      const RouteElevationAssessment& route_elevation,
      const RouteCorridorObservationOutput& corridor,
      const ObstacleSummary& obstacles,
      double now_sec);

  // status：获取当前完整的模式状态快照（不触发状态转换）。
  const NavigationModeStatus& status() const noexcept;

private:
  // isConfigValid：校验配置参数自身是否合法（时间非负、距离阈值关系合理等）。
  bool isConfigValid() const noexcept;

  // isTaskValid：校验任务 sequence 非0 且 mode 属于支持的三种模式之一。
  bool isTaskValid(
      const NavigationTask& task) const noexcept;

  // isProgressValid：校验路线进度是否有效且属于当前任务。
  bool isProgressValid(
      const NavigationTask& task,
      const RouteProgress& progress) const noexcept;

  // isRobotNumericValid：校验机器人 x/y/yaw 是否均为有限数。
  bool isRobotNumericValid(
      const RobotState& robot) const noexcept;

  // taskAllowsAvoidance：判断当前任务模式是否允许进入局部避障
  // （ROUTE_ONLY 模式不允许避障，只能严格沿路线走）。
  bool taskAllowsAvoidance(
      TaskMode task_mode) const noexcept;

  // initializeForTask：当检测到新任务（或首次初始化）时重置所有状态，
  // 并将初始模式固定为 ROUTE_FOLLOW（新任务总是从全局跟踪开始）。
  void initializeForTask(
      const NavigationTask& task,
      double now_sec);

  // transitionTo：执行一次模式转换，更新 previous_mode/mode/reason 等字段，
  // 并清空所有"进入/退出确认"计时器（避免上一个模式的确认进度影响新模式）。
  void transitionTo(
      NavigationMode new_mode,
      NavigationModeReason reason,
      const RouteProgress& progress,
      double now_sec);

  NavigationModeConfig config_{};        // 模式切换配置参数
  StairUpConfig stair_config_{};
  NavigationModeStatus status_{};        // 当前完整模式状态

  std::uint64_t active_task_sequence_{0}; // 当前正在跟踪的任务序号

  double blocked_candidate_start_sec_{0.0}; // "发现阻塞"候选确认开始时刻
  bool blocked_candidate_active_{false};    // 阻塞确认计时器是否已启动

  double clear_candidate_start_sec_{0.0};   // "可以退出避障"候选确认开始时刻
  bool clear_candidate_active_{false};      // 退出确认计时器是否已启动

  double last_update_stamp_sec_{0.0};       // 上一次 update 调用的时间戳，用于检测时间倒流
  bool has_last_update_stamp_{false};       // 是否已有上一次时间戳记录

};

}  // namespace navdog
