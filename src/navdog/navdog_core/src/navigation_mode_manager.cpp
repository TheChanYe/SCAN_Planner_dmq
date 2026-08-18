#include "navdog_core/navigation_mode_manager.hpp"

#include <algorithm>
#include <cmath>

namespace navdog
{

namespace
{
constexpr double kTimeEpsilonSec = 1e-9;
}

NavigationModeManager::NavigationModeManager(
    const NavigationModeConfig& config,
    const StairUpConfig& stair_config)
    : config_(config),
      stair_config_(stair_config)
{
}

// reset：将状态机完全重置到初始状态，在任务取消/重新开始导航时调用。
void NavigationModeManager::reset() noexcept
{
  status_ = NavigationModeStatus{};
  active_task_sequence_ = 0;
  last_update_stamp_sec_ = 0.0;
  has_last_update_stamp_ = false;

  blocked_candidate_active_ = false;
  blocked_candidate_start_sec_ = 0.0;
  clear_candidate_active_ = false;
  clear_candidate_start_sec_ = 0.0;
  stair_clear_candidate_active_ = false;
  stair_clear_candidate_start_sec_ = 0.0;
}

// isConfigValid：校验配置自身的合法性：确认时长不能为负、立即进入距离不能大于
// 普通进入距离（否则逻辑矛盾）、退出各方向净空距离不能为负。
bool NavigationModeManager::isConfigValid() const noexcept
{
  if (config_.enter_confirm_sec < 0.0)
    return false;
  if (config_.exit_clear_confirm_sec < 0.0)
    return false;
  if (config_.immediate_enter_distance_m <= 0.0)
    return false;
  if (config_.immediate_enter_distance_m >
      config_.enter_blocked_distance_m)
    return false;
  if (config_.exit_front_clearance_m < 0.0)
    return false;
  if (config_.exit_left_clearance_m < 0.0)
    return false;
  if (config_.exit_right_clearance_m < 0.0)
    return false;
  if (!std::isfinite(stair_config_.lookahead_distance_m) ||
      stair_config_.lookahead_distance_m <= 0.0 ||
      !std::isfinite(stair_config_.sample_step_m) ||
      stair_config_.sample_step_m <= 0.0 ||
      !std::isfinite(stair_config_.trigger_rise_m) ||
      stair_config_.trigger_rise_m <= 0.0 ||
      !std::isfinite(stair_config_.min_local_slope_m_per_m) ||
      stair_config_.min_local_slope_m_per_m <= 0.0 ||
      !std::isfinite(stair_config_.flat_tolerance_m) ||
      stair_config_.flat_tolerance_m < 0.0 ||
      stair_config_.flat_tolerance_m >= stair_config_.trigger_rise_m ||
      !std::isfinite(stair_config_.exit_progress_margin_m) ||
      stair_config_.exit_progress_margin_m < 0.0 ||
      !std::isfinite(stair_config_.exit_confirm_sec) ||
      stair_config_.exit_confirm_sec < 0.0)
  {
    return false;
  }
  return true;
}

// isTaskValid：任务 sequence 为0视为无效任务，且 mode 必须属于支持的三种模式之一。
bool NavigationModeManager::isTaskValid(
    const NavigationTask& task) const noexcept
{
  if (task.sequence == 0)
    return false;
  return task.mode == TaskMode::NORMAL_AVOID ||
         task.mode == TaskMode::ROUTE_ONLY ||
         task.mode == TaskMode::CHARGING;
}

// isProgressValid：路线进度必须有效、属于当前任务、弧长为非负有限数、时间戳为有限数。
bool NavigationModeManager::isProgressValid(
    const NavigationTask& task,
    const RouteProgress& progress) const noexcept
{
  return progress.valid &&
      progress.task_sequence == task.sequence &&
      std::isfinite(progress.arc_length_m) &&
      progress.arc_length_m >= 0.0 &&
      std::isfinite(progress.stamp_sec);
}

// isRobotNumericValid：机器人 x/y/yaw 必须均为有限数。
bool NavigationModeManager::isRobotNumericValid(
    const RobotState& robot) const noexcept
{
  return std::isfinite(robot.x) && std::isfinite(robot.y) &&
         std::isfinite(robot.yaw);
}

// taskAllowsAvoidance：只有 NORMAL_AVOID 和 CHARGING 任务模式允许进入局部避障，
// ROUTE_ONLY 模式必须严格沿预定义路线行走，即使前方被阻塞也不能自行迁移。
bool NavigationModeManager::taskAllowsAvoidance(
    TaskMode task_mode) const noexcept
{
  return task_mode == TaskMode::NORMAL_AVOID ||
         task_mode == TaskMode::CHARGING;
}

// initializeForTask：当检测到新任务（或首次初始化）时重置所有状态字段：
// 把模式固定为 ROUTE_FOLLOW（新任务总是从全局跟踪开始，不继承上个任务的LOCAL_AVOID状态），
// 根据是否之前已初始化过区分 reason 为 INITIALIZED 或 TASK_CHANGED，
// 并清空阻塞/退出确认计时器。
void NavigationModeManager::initializeForTask(
    const NavigationTask& task,
    double now_sec)
{
  const bool was_initialized = status_.initialized;
  status_ = NavigationModeStatus{};
  status_.mode = NavigationMode::ROUTE_FOLLOW;
  status_.previous_mode = NavigationMode::NONE;
  status_.reference_intent = ReferenceIntent::GLOBAL_ROUTE;
  status_.reason = was_initialized
      ? NavigationModeReason::TASK_CHANGED
      : NavigationModeReason::INITIALIZED;
  status_.task_sequence = task.sequence;
  status_.initialized = true;
  status_.transitioned = true;
  status_.mode_enter_stamp_sec = now_sec;
  status_.transition_stamp_sec = now_sec;
  active_task_sequence_ = task.sequence;

  blocked_candidate_active_ = false;
  blocked_candidate_start_sec_ = 0.0;
  clear_candidate_active_ = false;
  clear_candidate_start_sec_ = 0.0;
  stair_clear_candidate_active_ = false;
  stair_clear_candidate_start_sec_ = 0.0;
}

// transitionTo：执行一次模式转换。
// 步骤：记录 previous_mode，更新为新模式并标记 reason/时间戳；
// 根据新模式同步更新 reference_intent（LOCAL_AVOID对应局部避障意图，
// 否则为全局路线意图）；若进入 LOCAL_AVOID 则累加避障次数计数器；
// 最后清空所有进入/退出确认计时器，避免上一个模式的确认进度污染新模式。
void NavigationModeManager::transitionTo(
    NavigationMode new_mode,
    NavigationModeReason reason,
    const RouteProgress& progress,
    double now_sec)
{
  (void)progress;
  status_.previous_mode = status_.mode;
  status_.mode = new_mode;
  status_.reason = reason;
  status_.transitioned = true;
  status_.mode_enter_stamp_sec = now_sec;
  status_.transition_stamp_sec = now_sec;
  status_.reference_intent = new_mode == NavigationMode::LOCAL_AVOID
      ? ReferenceIntent::LOCAL_AVOIDANCE
      : ReferenceIntent::GLOBAL_ROUTE;
  if (new_mode == NavigationMode::LOCAL_AVOID)
    ++status_.avoidance_cycle_count;

  // Clear candidate timers on any transition.
  blocked_candidate_active_ = false;
  blocked_candidate_start_sec_ = 0.0;
  clear_candidate_active_ = false;
  clear_candidate_start_sec_ = 0.0;
  stair_clear_candidate_active_ = false;
  stair_clear_candidate_start_sec_ = 0.0;
}

// update：导航模式状态机主入口，每个控制周期调用一次。整体流程：
//   1. 时间戳校验：非有限数或相比上次调用时间倒流 -> INVALID_TIME；
//   2. 配置/任务合法性校验（isConfigValid/isTaskValid），失败则直接返回对应错误码；
//   3. 若检测到任务序号变化或尚未初始化，调用 initializeForTask 重置为新任务的
//      ROUTE_FOLLOW 初始状态，并记住初始化原因供末尾回填；
//   4. 同步更新 task_sequence 与 avoidance_allowed（是否允许避障）；
//   5. 路线进度/机器人数值校验失败则直接返回对应错误码；
//   6. 判断走廊评估结果是否为 CLEAR/BLOCKED：若都不是（如还在计算中），
//      根据具体错误类型返回 INVALID_CORRIDOR_RESULT 或 WAITING_FOR_CORRIDOR，
//      并将 corridor_available/route_blocked 相关字段清零；
//   7. 走廊评估结果必须属于当前任务，否则 INVALID_CORRIDOR_RESULT；
//   8. 计算 blocked_in_avoid_range（阻塞且首个阻塞点距离在"进入避障距离"以内）
//      作为 route_blocked_near，并更新当前阻塞/退出确认计时器的已经过时长；
//   9. 【ROUTE_FOLLOW -> LOCAL_AVOID】：当前处于全局跟踪且前方在避障距离内被阻塞时：
//      - 若任务不允许避障，只记录 reason=ROUTE_ONLY_BLOCKED，不切换；
//      - 若距离<=立即进入距离，不等待确认直接切换（BLOCK_IMMEDIATE，应对紧急障碍）；
//      - 否则启动/继续阻塞确认计时，持续足够时间(enter_confirm_sec)后才切换
//        （BLOCK_CONFIRMED，防止瞬时噪声误触发）；
//  10. 【LOCAL_AVOID -> ROUTE_FOLLOW】：当前处于局部避障时，退出需同时满足：
//      ① 已达到最小停留时长(min_local_avoid_hold_sec)；
//      ② 走廊重新变为 CLEAR 且属于当前任务（确认原路线重新可通行，
//        否则会在 corridor 仍为 BLOCKED 时过早退出导致立即重新进入）；
//      ③ 前/左/右方向的障碍物净空都达标（clearance_satisfied）；
//      以上三项均满足后启动/继续退出确认计时，持续足够时间(exit_clear_confirm_sec)
//      后才真正切换回 ROUTE_FOLLOW（ROUTE_CLEAR）；任一条不满足则重置退出确认计时器；
//  11. 否则（处于 ROUTE_FOLLOW 且未被阻塞）：重置阻塞确认计时器，标记 reason=ROUTE_CLEAR；
//  12. 若本次刚刚初始化且未发生转换，回填初始化时记录的 reason；
//  13. 更新时间戳与有效标志，返回 UPDATED 结果。
// 输入：task/robot/progress/corridor/obstacles/now_sec；输出：NavigationModeOutput。
NavigationModeOutput NavigationModeManager::update(
    const NavigationTask& task,
    const RobotState& robot,
    const RouteProgress& progress,
    const RouteCorridorObservationOutput& corridor,
    const ObstacleSummary& obstacles,
    double now_sec)
{
  return update(task, robot, progress, RouteElevationAssessment{},
      corridor, obstacles, now_sec);
}

NavigationModeOutput NavigationModeManager::update(
    const NavigationTask& task,
    const RobotState& robot,
    const RouteProgress& progress,
    const RouteElevationAssessment& route_elevation,
    const RouteCorridorObservationOutput& corridor,
    const ObstacleSummary& obstacles,
    double now_sec)
{
  NavigationModeOutput output{};
  status_.transitioned = false;

  if (!std::isfinite(now_sec) ||
      (has_last_update_stamp_ &&
       now_sec + kTimeEpsilonSec < last_update_stamp_sec_))
  {
    output.result = NavigationModeUpdateResult::INVALID_TIME;
    output.status = status_;
    return output;
  }
  last_update_stamp_sec_ = now_sec;
  has_last_update_stamp_ = true;

  if (!isConfigValid())
  {
    output.result = NavigationModeUpdateResult::INVALID_CONFIG;
    output.status = status_;
    return output;
  }
  if (!isTaskValid(task))
  {
    output.result = NavigationModeUpdateResult::INVALID_TASK;
    output.status = status_;
    return output;
  }

  const bool initialized_this_update = !status_.initialized ||
      task.sequence != active_task_sequence_;
  if (initialized_this_update)
    initializeForTask(task, now_sec);
  const NavigationModeReason initialization_reason = status_.reason;

  status_.task_sequence = task.sequence;
  status_.avoidance_allowed = taskAllowsAvoidance(task.mode);
  if (!isProgressValid(task, progress))
  {
    output.result = NavigationModeUpdateResult::INVALID_PROGRESS;
    output.status = status_;
    return output;
  }
  if (robot.valid && !isRobotNumericValid(robot))
  {
    output.result = NavigationModeUpdateResult::INVALID_ROBOT;
    output.status = status_;
    return output;
  }

  const bool route_ascending = stair_config_.enabled &&
      route_elevation.valid && route_elevation.ascending;
  const bool stair_activated_this_update = route_ascending &&
      status_.avoidance_allowed && !status_.stair_up_active &&
      status_.mode == NavigationMode::ROUTE_FOLLOW;
  if (route_ascending && status_.avoidance_allowed &&
      (status_.stair_up_active ||
       status_.mode == NavigationMode::ROUTE_FOLLOW))
  {
    status_.stair_hold_until_arc_m = std::max(
        status_.stair_hold_until_arc_m,
        route_elevation.checked_until_arc_m);
    status_.stair_up_active = true;
    if (status_.mode == NavigationMode::ROUTE_FOLLOW)
    {
      transitionTo(NavigationMode::LOCAL_AVOID,
          NavigationModeReason::ROUTE_ASCENDING, progress, now_sec);
    }
  }

  const bool is_clear =
      corridor.result == RouteCorridorObservationResult::CLEAR;
  const bool is_blocked =
      corridor.result == RouteCorridorObservationResult::BLOCKED;
  if (!is_clear && !is_blocked)
  {
    if (corridor.result == RouteCorridorObservationResult::INVALID_TIME ||
        corridor.result == RouteCorridorObservationResult::INVALID_CONFIG ||
        corridor.result == RouteCorridorObservationResult::INVALID_PROGRESS)
    {
      output.result = NavigationModeUpdateResult::INVALID_CORRIDOR_RESULT;
    }
    else
    {
      status_.corridor_available = false;
      status_.route_blocked = false;
      status_.route_blocked_near = false;
      if (!status_.transitioned)
        status_.reason = NavigationModeReason::WAITING_FOR_CORRIDOR;
      output.result = NavigationModeUpdateResult::WAITING_FOR_CORRIDOR;
    }
    status_.stamp_sec = now_sec;
    status_.valid = status_.initialized;
    output.status = status_;
    return output;
  }

  if (!corridor.assessment.valid ||
      corridor.assessment.task_sequence != task.sequence)
  {
    output.result = NavigationModeUpdateResult::INVALID_CORRIDOR_RESULT;
    output.status = status_;
    return output;
  }

  status_.corridor_available = true;
  status_.route_blocked = is_blocked;
  status_.route_blocked_near = is_blocked;

  const bool blocked_in_avoid_range = is_blocked &&
      std::isfinite(corridor.assessment.first_blocked_distance_ahead_m) &&
      corridor.assessment.first_blocked_distance_ahead_m <=
          config_.enter_blocked_distance_m + kTimeEpsilonSec;
  status_.route_blocked_near = blocked_in_avoid_range;

  status_.blocked_confirm_elapsed_sec =
      blocked_candidate_active_ ? (now_sec - blocked_candidate_start_sec_) : 0.0;
  status_.clear_confirm_elapsed_sec =
      clear_candidate_active_ ? (now_sec - clear_candidate_start_sec_) : 0.0;

  // --- ROUTE_FOLLOW → LOCAL_AVOID ---
  if (status_.mode == NavigationMode::ROUTE_FOLLOW &&
      blocked_in_avoid_range)
  {
    if (!status_.avoidance_allowed)
    {
      status_.reason = NavigationModeReason::ROUTE_ONLY_BLOCKED;
    }
    else
    {
      const double blocked_dist =
          corridor.assessment.first_blocked_distance_ahead_m;

      // Immediate enter for very close obstacles.
      if (blocked_dist <= config_.immediate_enter_distance_m)
      {
        blocked_candidate_active_ = false;
        blocked_candidate_start_sec_ = 0.0;

        // 障碍物已经非常接近，不等待连续确认。
        transitionTo(NavigationMode::LOCAL_AVOID,
            NavigationModeReason::BLOCK_IMMEDIATE, progress, now_sec);
      }
      else
      {
        // 第一次发现普通距离内阻塞时启动确认计时。
        if (!blocked_candidate_active_)
        {
          blocked_candidate_active_ = true;
          blocked_candidate_start_sec_ = now_sec;
        }

        const double held = now_sec - blocked_candidate_start_sec_;

        if (held >= config_.enter_confirm_sec)
        {
          blocked_candidate_active_ = false;
          blocked_candidate_start_sec_ = 0.0;

          // 障碍物不是紧急距离，但已经连续稳定存在。
          transitionTo(NavigationMode::LOCAL_AVOID,
              NavigationModeReason::BLOCK_CONFIRMED, progress, now_sec);
        }
      }
    }
  }
  // --- LOCAL_AVOID → ROUTE_FOLLOW ---
  else if (status_.mode == NavigationMode::LOCAL_AVOID)
  {
    const double mode_hold_sec =
        now_sec - status_.mode_enter_stamp_sec;
    const bool minimum_hold_satisfied =
        mode_hold_sec >= config_.min_local_avoid_hold_sec;

    // Check directional clearance from obstacles.
    const double front_min = obstacles.valid
        ? obstacles.front_min
        : std::numeric_limits<double>::infinity();
    const double left_min = obstacles.valid
        ? obstacles.left_min
        : std::numeric_limits<double>::infinity();
    const double right_min = obstacles.valid
        ? obstacles.right_min
        : std::numeric_limits<double>::infinity();

    const bool clearance_satisfied =
        obstacles.valid &&
        front_min >= config_.exit_front_clearance_m &&
        left_min >= config_.exit_left_clearance_m &&
        right_min >= config_.exit_right_clearance_m;

    // Corridor must be CLEAR for exit: confirms the original route is
    // passable again.  Without this check the robot would exit SCAN while
    // the corridor still reports BLOCKED, causing an immediate re-entry.
    const bool corridor_clear =
        is_clear &&
        corridor.assessment.valid &&
        corridor.assessment.task_sequence == task.sequence;

    // Exit LOCAL_AVOID requires ALL of:
    //   1. Minimum hold time satisfied
    //   2. Route corridor CLEAR
    //   3. Directional obstacle clearance satisfied (from ObstacleSummary)
    //   4. All of the above held continuously for exit_clear_confirm_sec
    const bool stair_route_flat = !status_.stair_up_active ||
        (route_elevation.valid && !route_elevation.ascending &&
         route_elevation.rise_m <= stair_config_.flat_tolerance_m +
             kTimeEpsilonSec);
    const bool stair_progress_satisfied = !status_.stair_up_active ||
        progress.arc_length_m + stair_config_.exit_progress_margin_m +
            kTimeEpsilonSec >= status_.stair_hold_until_arc_m;
    const bool stair_release_conditions = stair_route_flat &&
        stair_progress_satisfied;

    // Stair preference has its own lifetime. Once the route is flat and the
    // held stair arc has been traversed, release it even if an ordinary
    // obstacle still keeps LOCAL_AVOID active. Otherwise a box encountered
    // after the stairs would continue suppressing SCAN's lateral fallback.
    if (status_.stair_up_active)
    {
      if (stair_release_conditions)
      {
        if (!stair_clear_candidate_active_)
        {
          stair_clear_candidate_active_ = true;
          stair_clear_candidate_start_sec_ = now_sec;
        }
        if (now_sec - stair_clear_candidate_start_sec_ >=
            stair_config_.exit_confirm_sec)
        {
          status_.stair_up_active = false;
          status_.stair_hold_until_arc_m = 0.0;
          stair_clear_candidate_active_ = false;
          stair_clear_candidate_start_sec_ = 0.0;
        }
      }
      else
      {
        stair_clear_candidate_active_ = false;
        stair_clear_candidate_start_sec_ = 0.0;
      }
    }
    else
    {
      stair_clear_candidate_active_ = false;
      stair_clear_candidate_start_sec_ = 0.0;
    }

    const bool all_exit_conditions =
        minimum_hold_satisfied &&
        corridor_clear &&
        clearance_satisfied &&
        stair_release_conditions;

    if (all_exit_conditions)
    {
      if (!clear_candidate_active_)
      {
        clear_candidate_active_ = true;
        clear_candidate_start_sec_ = now_sec;
      }

      const double clear_held = now_sec - clear_candidate_start_sec_;
      const double exit_confirm_sec = status_.stair_up_active
          ? stair_config_.exit_confirm_sec
          : config_.exit_clear_confirm_sec;
      if (clear_held >= exit_confirm_sec)
      {
        clear_candidate_active_ = false;
        clear_candidate_start_sec_ = 0.0;

        transitionTo(NavigationMode::ROUTE_FOLLOW,
            NavigationModeReason::ROUTE_CLEAR, progress, now_sec);
        status_.stair_up_active = false;
        status_.stair_hold_until_arc_m = 0.0;
      }
    }
    else
    {
      // Any exit condition not met — reset clear candidate timer.
      clear_candidate_active_ = false;
      clear_candidate_start_sec_ = 0.0;
    }

    if (!status_.transitioned && !stair_activated_this_update)
      status_.reason = NavigationModeReason::LOCAL_AVOID_ACTIVE;
  }
  else
  {
    // ROUTE_FOLLOW and not blocked.
    blocked_candidate_active_ = false;
    blocked_candidate_start_sec_ = 0.0;

    status_.reason = NavigationModeReason::ROUTE_CLEAR;
  }

  if (initialized_this_update && !status_.transitioned)
    status_.reason = initialization_reason;
  status_.stamp_sec = now_sec;
  status_.valid = true;
  output.result = NavigationModeUpdateResult::UPDATED;
  output.status = status_;
  return output;
}

// status：获取当前完整模式状态快照（只读，不触发任何状态转换）。
const NavigationModeStatus&
NavigationModeManager::status() const noexcept
{
  return status_;
}

}  // namespace navdog
