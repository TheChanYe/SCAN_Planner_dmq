#include "navdog_core/navigation_coordinator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace navdog
{

namespace
{

constexpr double kEpsilon = 1e-9;
constexpr double kPi = 3.14159265358979323846;

}  // namespace

// =============================================================================
// Constructor
// 构造函数：保存配置，并用对应子配置逐个初始化所有子控制器（任务管理器、路线管理器、
// 起点对齐、路径观测门控、模式管理器、路线跟随器、终点对齐器、安全监督层）。
// =============================================================================

NavigationCoordinator::NavigationCoordinator(
    const NavdogConfig& config,
    const navdog_task::TaskConfig& task_config)
    : config_(config),
      state_(NavState::IDLE),
      task_manager_(task_config),
      route_manager_(config.route_progress),
      start_align_controller_(config.start_align),
      route_corridor_observation_gate_(
          config.route_corridor_observation),
      navigation_mode_manager_(config.navigation_mode, config.stair_up),
      route_follower_(config.route_follower),
      goal_controller_(config.goal_controller),
      safety_supervisor_(config.safety, config.limits)
{
}

// =============================================================================
// reset
// 完全重置整个协调器：回到IDLE状态，清空任务、待处理规划动作队列与规划握手上下文，
// 并重置所有子控制器的内部状态。适用于进程启动初始化或彻底恢复现场。
// =============================================================================

void NavigationCoordinator::reset()
{
  state_ = NavState::IDLE;
  task_manager_.reset();
  pending_planner_actions_.clear();
  clearPlanningContext();
  resetNearGoalBlockedTimer();
  start_align_controller_.reset();
  route_manager_.reset();
  navigation_mode_manager_.reset();
  goal_controller_.reset();
  safety_supervisor_.reset();

  last_mode_ = NavigationMode::NONE;
  obstacle_finished_ = false;
  state_before_pause_ = NavState::IDLE;
}

// =============================================================================
// resetNearGoalBlockedTimer
// 重置“接近终点但被阻挡”计时器。当机器人靠近终点且路线被阻时会启动计时，
// 超过 obstacle_finish_timeout_sec 则判定为完成任务；本函数用于清除该计时状态。
// =============================================================================

void NavigationCoordinator::resetNearGoalBlockedTimer() noexcept
{
  near_goal_blocked_since_sec_ = 0.0;
  near_goal_blocked_timer_active_ = false;
}

// enqueuePlannerAction：将一个规划器动作加入待发送队列。
// 规则：NONE类型不入队；若为CANCEL类型，先清空队列中已有的待发动作（取消优先），
// 再入队，避免旧的SET_ROUTE等动作在取消后仍被发送给规划器。
void NavigationCoordinator::enqueuePlannerAction(
    const PlannerAction& action)
{
  if (action.type == PlannerActionType::NONE)
  {
    return;
  }

  if (action.type == PlannerActionType::CANCEL)
  {
    pending_planner_actions_.clear();
  }

  pending_planner_actions_.push_back(action);
}

// =============================================================================
// takeNextPlannerAction
// 从待发队列弹出并返回一个规划器动作（先入先出），队列为空时返回默认构造的 PlannerAction
// （type=NONE）。每个 control cycle 最多取一个动作下发给规划器。
// =============================================================================

PlannerAction NavigationCoordinator::takeNextPlannerAction()
{
  if (pending_planner_actions_.empty())
  {
    return PlannerAction{};
  }

  const PlannerAction action =
      pending_planner_actions_.front();

  pending_planner_actions_.pop_front();

  return action;
}

// =============================================================================
// clearPlanningContext
// 清除本次规划握手上下文（是否已发送请求、发送时刻、期望的轨迹ID），在规划完成/失败/
// 进入新任务时调用，避免旧的反馈被误认为当前规划的反馈。
// =============================================================================

void NavigationCoordinator::clearPlanningContext() noexcept
{
  planning_request_sent_ = false;
  planning_started_sec_ = 0.0;
  expected_trajectory_id_ = 0;
}

// =============================================================================
// startPlanningContext
// 开启一次新的规划握手：仅当动作类型为SET_ROUTE、时间有限且任务sequence非零时才生效，
// 记录发送时刻与期望的轨迹ID（=任务sequence），供后续 isPlannerFeedbackUsable 校验反馈合法性。
// 输出：是否成功开启（失败时调用方应进入失败状态）。
// =============================================================================

bool NavigationCoordinator::startPlanningContext(
    const PlannerAction& set_route_action,
    double now_sec) noexcept
{
  if (set_route_action.type !=
      PlannerActionType::SET_ROUTE)
  {
    return false;
  }

  if (!std::isfinite(now_sec))
  {
    return false;
  }

  if (set_route_action.task.sequence == 0)
  {
    return false;
  }

  planning_request_sent_ = true;
  planning_started_sec_ = now_sec;
  expected_trajectory_id_ =
      set_route_action.task.sequence;

  return true;
}

// =============================================================================
// isPlannerFeedbackUsable
// 校验规划器反馈是否可信：必须已发送过请求、feedback.valid为真、时间戳与now_sec均有限、
// 轨迹ID与本次期望的一致且非零、反馈时间不早于发送时刻且不晚于当前时刻（防时间递归异常）。
// =============================================================================

bool NavigationCoordinator::isPlannerFeedbackUsable(
    const PlannerFeedback& feedback,
    double now_sec) const noexcept
{
  if (!planning_request_sent_)
  {
    return false;
  }

  if (!feedback.valid)
  {
    return false;
  }

  if (!std::isfinite(feedback.stamp_sec) ||
      !std::isfinite(now_sec))
  {
    return false;
  }

  if (feedback.trajectory_id == 0 ||
      feedback.trajectory_id != expected_trajectory_id_)
  {
    return false;
  }

  if (feedback.stamp_sec < planning_started_sec_)
  {
    return false;
  }

  if (feedback.stamp_sec > now_sec)
  {
    return false;
  }

  return true;
}

// =============================================================================
// updatePlanningState
// 在 PLANNING 状态下，根据规划器反馈推进状态机。
// 步骤：
//   1. 若反馈可用(isPlannerFeedbackUsable)，根据 feedback.state分支：
//      READY/EXECUTING → 进入 START_ALIGN 并清除规划上下文；FAILED → 进入失败状态；
//      其余（UNAVAILABLE/IDLE/PLANNING）继续等待；
//   2. 若反馈不可用或仍在等待，检查规划超时时长(planning_timeout_sec)，超时则进入失败状态。
// =============================================================================

void NavigationCoordinator::updatePlanningState(
    const PlannerFeedback& feedback,
    double now_sec)
{
  if (state_ != NavState::PLANNING ||
      !planning_request_sent_)
  {
    return;
  }

  if (isPlannerFeedbackUsable(feedback, now_sec))
  {
    switch (feedback.state)
    {
      case PlannerState::READY:
      case PlannerState::EXECUTING:
        state_ = NavState::START_ALIGN;
        clearPlanningContext();
        start_align_controller_.reset();
        return;

      case PlannerState::FAILED:
        enterFailedState();
        return;

      case PlannerState::UNAVAILABLE:
      case PlannerState::IDLE:
      case PlannerState::PLANNING:
        break;
    }
  }

  const double timeout_sec =
      config_.planner.planning_timeout_sec;

  if (!std::isfinite(timeout_sec) ||
      timeout_sec <= 0.0)
  {
    return;
  }

  if (!std::isfinite(now_sec) ||
      now_sec < planning_started_sec_)
  {
    return;
  }

  if ((now_sec - planning_started_sec_) >
      timeout_sec)
  {
    enterFailedState();
  }
}

// =============================================================================
// enterFailedState
// 进入 FAILED 状态并清理所有与当前任务相关的中间状态（待发动作、规划握手、起点对齐、
// 路线、模式管理器、终点对齐器、安全监督层），避免失败后遗留残留状态影响下一个任务。
// =============================================================================

void NavigationCoordinator::enterFailedState() noexcept
{
  const std::uint64_t sequence =
      task_manager_.session().sequence;
  if (task_manager_.hasActiveTask() &&
      sequence != 0)
  {
    task_manager_.complete(sequence);
  }

  state_ = NavState::FAILED;

  pending_planner_actions_.clear();

  clearPlanningContext();
  resetNearGoalBlockedTimer();
  start_align_controller_.reset();
  route_manager_.reset();
  navigation_mode_manager_.reset();
  goal_controller_.reset();
  safety_supervisor_.reset();
  obstacle_finished_ = false;
}

bool NavigationCoordinator::failActiveTask() noexcept
{
  if (!task_manager_.hasActiveTask())
    return false;

  enterFailedState();
  return true;
}

// =============================================================================
// makeZeroCommand
// 构造一个全零速度指令（vx=vy=yaw_rate=0，valid=true），并标记来源source与时间戳。
// now_sec 非有限时回退为0.0。各状态分支在无法产生真实控制量时都会回退到该函数。
// =============================================================================

VelocityCommand NavigationCoordinator::makeZeroCommand(
    CommandSource source,
    double now_sec) const noexcept
{
  VelocityCommand command{};

  command.vx = 0.0;
  command.vy = 0.0;
  command.yaw_rate = 0.0;

  command.valid = true;
  command.source = source;
  command.stamp_sec =
      std::isfinite(now_sec) ? now_sec : 0.0;

  return command;
}

// =============================================================================
// executeRouteFollow
// ROUTE_FOLLOW 模式下的执行逻辑：处理路线阻挡、近终点限速并交给 RouteFollower。
// 步骤：
//   1. ROUTE_ONLY阻塞必须停车；普通避障确认期继续跟随但限速到SCAN交接速度；
//   2. 计算是否near_goal（距终点小于near_goal_switch_dist）；
//   3. near_goal时根据剩余路程占switch_dist的比例线性插值限速（near_goal_max_v→near_goal_min_v）；
//   4. 交由 RouteFollower 按pure pursuit策略跟随路线。
// =============================================================================

VelocityCommand NavigationCoordinator::executeRouteFollow(
    const NavigationTask& task,
    const RobotState& robot,
    const RouteProgress& progress,
    const NavigationModeStatus& mode_status,
    double max_vx,
    double now_sec)
{
  (void)now_sec;

  if (mode_status.reason == NavigationModeReason::ROUTE_ONLY_BLOCKED)
  {
    return makeZeroCommand(
        CommandSource::TRACKING_STOP, now_sec);
  }

  const bool near_goal =
      !task.points.empty() &&
      std::isfinite(robot.x) && std::isfinite(robot.y) &&
      std::hypot(task.points.back().x - robot.x,
                 task.points.back().y - robot.y) <=
          config_.goal_controller.near_goal_switch_dist;

  double effective_max_vx = max_vx;
  if (mode_status.mode == NavigationMode::ROUTE_FOLLOW &&
      mode_status.route_blocked_near &&
      mode_status.avoidance_allowed)
  {
    effective_max_vx = std::min(
        effective_max_vx,
        config_.navigation_mode.handoff_linear_speed_mps);
  }

  if (near_goal)
  {
    const double remaining = std::max(
        0.0,
        progress.remaining_distance_m);

    const double switch_dist =
        config_.goal_controller.near_goal_switch_dist;

    if (switch_dist > kEpsilon)
    {
      const double scale =
          std::min(1.0, remaining / switch_dist);
      effective_max_vx = std::min(
          effective_max_vx,
          config_.goal_controller.near_goal_max_v * scale +
              config_.goal_controller.near_goal_min_v *
                  (1.0 - scale));
    }

    effective_max_vx = std::max(
        config_.goal_controller.near_goal_min_v,
        std::min(
            config_.goal_controller.near_goal_max_v,
            effective_max_vx));
  }

  return route_follower_.update(
      task, robot, progress, effective_max_vx, now_sec);
}

// =============================================================================
// executeLocalAvoid
// LOCAL_AVOID 模式下本协调器不产生实际控制量，只返回零速度：实际的局部避障速度由 SCAN
// 原生闭环控制器产生，并在 Mux 层选择（本类不插手该链路）。
// =============================================================================

VelocityCommand NavigationCoordinator::executeLocalAvoid(
    const NavigationTask& task,
    const RobotState& robot,
    const RouteProgress& progress,
    const NavigationModeStatus& mode_status,
    double max_vx,
    double now_sec)
{
  (void)task;
  (void)robot;
  (void)progress;
  (void)mode_status;
  (void)max_vx;
  // LOCAL_AVOID velocity is produced by the native SCAN closed-loop
  // controller and selected by the Mux.  Coordinator outputs zero.
  return makeZeroCommand(CommandSource::TRACKING_STOP, now_sec);
}

// =============================================================================
// executeMode
// 根据最终目标边界与当前导航模式（ROUTE_FOLLOW/LOCAL_AVOID）分发执行函数，并处理
// “靠近终点但被阻”的超时判完成逻辑。
// 步骤：
//   1. 计算到终点距离，若已进入finish_dist，则交给GoalController判定完成或最终对齐；
//   2. 若路径观测不可用(corridor_available=false)，重置计时器并返回零速度；
//   3. 判断是否near_goal_blocked（近终点且路线被阻）；
//   4. 若near_goal_blocked，启动/继续计时，超过obstacle_finish_timeout_sec则判定任务完成
//      并完整清理，未超时则返回零速度；
//   5. 未被阻时重置计时器，并在模式发生切换时也重置（避免跨模式遗留计时）；
//   6. 根据 mode_status.mode 调用 executeRouteFollow 或 executeLocalAvoid，并记录 last_mode_。
// =============================================================================

VelocityCommand NavigationCoordinator::executeMode(
    const NavigationTask& task,
    const RobotState& robot,
    const RouteProgress& progress,
    const NavigationModeStatus& mode_status,
    const ObstacleSummary& obstacles,
    const RouteCorridorAssessment& corridor,
    bool corridor_available,
    double max_vx,
    double now_sec)
{
  (void)obstacles;
  (void)corridor;
  const double goal_distance = task.points.empty() || !robot.valid ||
      !std::isfinite(robot.x) || !std::isfinite(robot.y)
      ? std::numeric_limits<double>::infinity()
      : std::hypot(task.points.back().x - robot.x,
                   task.points.back().y - robot.y);
  if (std::isfinite(goal_distance) &&
      goal_distance <= config_.goal_controller.finish_dist)
  {
    const auto result = goal_controller_.update(
        task,
        robot,
        progress,
        max_vx,
        std::min(config_.limits.max_yaw_rate,
                 config_.goal_controller.near_goal_max_w),
        now_sec);

    if (result.finished)
    {
      state_ = NavState::SUCCEEDED;
      obstacle_finished_ = false;
      task_manager_.complete(task_manager_.session().sequence);
      resetNearGoalBlockedTimer();
      navigation_mode_manager_.reset();
      safety_supervisor_.reset();
    }
    else
    {
      state_ = NavState::GOAL_ALIGN;
    }

    last_mode_ = mode_status.mode;
    return result.command;
  }

  // Corridor / robot not ready: do not execute any real controller.
  if (!corridor_available)
  {
    resetNearGoalBlockedTimer();
    return makeZeroCommand(
        CommandSource::TRACKING_STOP, now_sec);
  }

  const bool near_goal_blocked =
      std::isfinite(goal_distance) &&
      goal_distance <= config_.goal_controller.near_goal_switch_dist &&
      mode_status.route_blocked_near;
  if (near_goal_blocked)
  {
    if (!near_goal_blocked_timer_active_)
    {
      near_goal_blocked_since_sec_ = now_sec;
      near_goal_blocked_timer_active_ = true;
    }
    const double elapsed = now_sec - near_goal_blocked_since_sec_;
    if (std::isfinite(elapsed) && elapsed >=
        config_.goal_controller.obstacle_finish_timeout_sec)
    {
      state_ = NavState::SUCCEEDED;
      obstacle_finished_ = true;
      task_manager_.complete(task_manager_.session().sequence);
      resetNearGoalBlockedTimer();
      navigation_mode_manager_.reset();
      safety_supervisor_.reset();
    }

    return makeZeroCommand(CommandSource::TRACKING_STOP, now_sec);
  }
  resetNearGoalBlockedTimer();

  if (last_mode_ != mode_status.mode)
  {
    resetNearGoalBlockedTimer();
  }

  VelocityCommand raw_cmd =
      makeZeroCommand(CommandSource::TRACKING_STOP, now_sec);

  switch (mode_status.mode)
  {
    case NavigationMode::ROUTE_FOLLOW:
      raw_cmd = executeRouteFollow(
          task,
          robot,
          progress,
          mode_status,
          max_vx,
          now_sec);
      break;

    case NavigationMode::LOCAL_AVOID:
      raw_cmd = executeLocalAvoid(
          task,
          robot,
          progress,
          mode_status,
          max_vx,
          now_sec);
      break;

    case NavigationMode::NONE:
      break;
  }

  last_mode_ = mode_status.mode;

  return raw_cmd;
}

// =============================================================================
// handleEvent
// 负责处理任务类事件。
// =============================================================================
TaskHandleResult NavigationCoordinator::handleEvent(
    NavigationEvent event)
{
  const std::uint64_t prior_sequence = task_manager_.session().sequence;
  const bool terminal_cancel_ack =
      event.type == NavigationEventType::CANCEL_TASK &&
      !task_manager_.hasActiveTask() &&
      (state_ == NavState::SUCCEEDED || state_ == NavState::FAILED);
  navdog_task::TaskTransition task_output =
      task_manager_.handleEvent(std::move(event));
  if (terminal_cancel_ack &&
      task_output.result == TaskHandleResult::CANCEL_IGNORED)
  {
    task_output.result = TaskHandleResult::CANCELLED;
  }
  /* 任务处理结果，输入任务 */
  switch (task_output.result)
  {
    case TaskHandleResult::STARTED: // 任务开始
      if (!route_manager_.acceptRoute(
              task_output.session.sequence,
              std::move(task_output.accepted_route))) // 如果路线不合法，进入失败状态
      {
        state_ = NavState::FAILED;
        return TaskHandleResult::REJECTED_INVALID_TASK; // 任务拒绝
      }
      clearPlanningContext(); // 清除规划上下文
      resetNearGoalBlockedTimer(); // 重置接近目标阻塞计时器
      obstacle_finished_ = false;
      start_align_controller_.reset();
      navigation_mode_manager_.reset();
      goal_controller_.reset();
      safety_supervisor_.reset();
      state_ = NavState::PLANNING; // 进入规划状态
      {
        PlannerAction action{};
        action.type = PlannerActionType::SET_ROUTE;
        action.task.sequence = task_output.session.sequence;
        action.task.mode = task_output.session.mode;
        action.task.max_vx = task_output.session.max_vx;
        action.max_vx = task_output.session.max_vx;
        enqueuePlannerAction(action);
      }
      break;

    case TaskHandleResult::CANCELLED: // 任务取消
      clearPlanningContext();
      resetNearGoalBlockedTimer();
      obstacle_finished_ = false;
      start_align_controller_.reset();
      route_manager_.reset();
      navigation_mode_manager_.reset();
      goal_controller_.reset();
      safety_supervisor_.reset();
      state_ = NavState::IDLE;
      { PlannerAction action{}; action.type = PlannerActionType::CANCEL;
        action.task.sequence = prior_sequence;
        enqueuePlannerAction(action); }
      break;

    case TaskHandleResult::MAX_VX_UPDATED: // 速度更新
      if (state_ == NavState::PLANNING ||
          state_ == NavState::START_ALIGN ||
          state_ == NavState::TRACKING ||
          state_ == NavState::GOAL_ALIGN ||
          state_ == NavState::RECOVERY ||
          state_ == NavState::PAUSED)
      {
        PlannerAction action{};
        action.type = PlannerActionType::UPDATE_SPEED_LIMIT;
        action.task.sequence = task_output.session.sequence;
        action.max_vx = task_output.session.max_vx;
        enqueuePlannerAction(action);
      }
      break;

    case TaskHandleResult::PAUSED: // 任务暂停
      if (state_ != NavState::PAUSED)
      {
        state_before_pause_ = state_;
        state_ = NavState::PAUSED;
        { PlannerAction action{}; action.type = PlannerActionType::PAUSE;
          enqueuePlannerAction(action); }
      }
      break;

    case TaskHandleResult::RESUMED: // 任务恢复
      if (state_ == NavState::PAUSED)
      {
        state_ = state_before_pause_ == NavState::IDLE
            ? NavState::TRACKING : state_before_pause_;
        { PlannerAction action{}; action.type = PlannerActionType::RESUME;
          enqueuePlannerAction(action); }
      }
      break;

    case TaskHandleResult::NONE: // 忽略
    case TaskHandleResult::REJECTED_BUSY: // 忙碌拒绝
    case TaskHandleResult::REJECTED_INVALID_TASK: // 无效任务拒绝
    case TaskHandleResult::CANCEL_IGNORED: // 忽略取消
    case TaskHandleResult::PAUSE_RESUME_IGNORED: // 忽略暂停恢复
    case TaskHandleResult::MAX_VX_UNCHANGED: // 忽略速度更新
    case TaskHandleResult::MAX_VX_UPDATE_IGNORED: // 忽略速度更新
    case TaskHandleResult::REJECTED_INVALID_MAX_VX: // 忽略速度更新
    case TaskHandleResult::UNSUPPORTED_EVENT: // 忽略
      break;
  }

  return task_output.result;
}

// =============================================================================
// hasActiveTask
// 返回当前任务管理器中是否存在一个处于活动状态的任务（已START且未完成/取消）。
// =============================================================================

bool NavigationCoordinator::hasActiveTask() const noexcept
{
  return task_manager_.hasActiveTask();
}

// =============================================================================
// route/session views
// routeManager/taskSession：分别返回内部路线管理器与任务会话的只读引用，供上层/测试读取内部状态使用。
// =============================================================================

const RouteManager& NavigationCoordinator::routeManager() const noexcept
{ return route_manager_; }

const navdog_task::TaskSession& NavigationCoordinator::taskSession() const noexcept
{ return task_manager_.session(); }

// =============================================================================
// update
// =============================================================================
/**
 * @brief update
 * 更新导航协调器状态。
 * @param input 核心输入数据
 * @param now_sec 当前时间（秒）
 * @return 核心输出数据
 */
CoreOutput NavigationCoordinator::update(
    const CoreInput& input,
    double now_sec)
{
  CoreOutput output{};

  // --- Planning feedback and action emission ---
  if (state_ == NavState::PLANNING &&
      !planning_request_sent_) // 如果当前状态为规划状态且没有发送规划请求，则发送规划请求
  {
    output.planner_action =
        takeNextPlannerAction();

    if (output.planner_action.type ==
        PlannerActionType::SET_ROUTE)
    {
      if (!startPlanningContext(
              output.planner_action,
              now_sec))
      {
        output.planner_action = PlannerAction{};
        enterFailedState();
      }
    }
  }
  else // 否则，更新规划状态
  {
    if (state_ == NavState::PLANNING)
    {
      updatePlanningState(
          input.planner,
          now_sec);
    }

    output.planner_action =
        takeNextPlannerAction();
  }

  // 设置默认速度命令为零
  VelocityCommand final_cmd =
      makeZeroCommand(
          CommandSource::IDLE_STOP,
          now_sec);

  // 开始处理状态机逻辑，如果当前状态为 START_ALIGN，则进行对齐控制器更新
  if (state_ == NavState::START_ALIGN)
  {
    /* 任务没有激活或没有路线，则进入失败状态*/
    if (!task_manager_.hasActiveTask() || !route_manager_.hasRoute())
    {
      enterFailedState();

      final_cmd =
          makeZeroCommand(
              CommandSource::FAILED_STOP,
              now_sec);
    }
    else /* 任务激活且路线存在，则进行对齐控制器更新*/
    {
      // Progress and corridor observation must be available before deciding
      // whether initial yaw alignment is appropriate.  Otherwise a blocked
      // route can spend seconds rotating toward an obstacle.
      const RouteProgressOutput progress_output =
          route_manager_.updateProgress(input.robot, now_sec);
      bool corridor_clear_for_align = false;
      if (progress_output.result == RouteProgressResult::VALID)
      {
        output.route_progress = progress_output.progress;
        const RouteCorridorObservationOutput obs_output =
            route_corridor_observation_gate_.evaluate(
                progress_output.progress, input.route_corridor_observation,
                now_sec);
        if (obs_output.result == RouteCorridorObservationResult::CLEAR ||
            obs_output.result == RouteCorridorObservationResult::BLOCKED)
          output.route_corridor = obs_output.assessment;

        // A confirmed blocked inflated corridor skips the alignment phase.
        // NavigationModeManager remains the sole owner of the mode decision;
        // it will enter LOCAL_AVOID on the following TRACKING update.
        if (obs_output.result == RouteCorridorObservationResult::BLOCKED)
        {
          state_ = NavState::TRACKING;
          start_align_controller_.reset();
          final_cmd = makeZeroCommand(CommandSource::TRACKING_STOP, now_sec);
        }

        // On the first progress sample, wait exactly one control cycle for a
        // map-backed corridor assessment rather than rotating blindly.
        if (obs_output.result != RouteCorridorObservationResult::CLEAR)
        {
          final_cmd = makeZeroCommand(CommandSource::START_ALIGN, now_sec);
        }
        else
          corridor_clear_for_align = true;
      }
      else
      {
        final_cmd = makeZeroCommand(CommandSource::START_ALIGN, now_sec);
      }

      if (state_ != NavState::START_ALIGN || !corridor_clear_for_align)
      {
        // Either the route is blocked (TRACKING will evaluate the mode next
        // cycle) or no map-backed corridor observation is ready yet.
      }
      else
      {
      const StartAlignOutput align_output =
          start_align_controller_.update(
              route_manager_.taskView(),
              input.robot,
              now_sec); // 更新对齐控制器，获取对齐输出

      switch (align_output.result) // 根据对齐结果进行处理
      {
        case StartAlignResult::WAITING_FOR_ROBOT: // 等待机器人对齐
        case StartAlignResult::ALIGNING: // 正在对齐
          final_cmd = align_output.command;
          break;

        case StartAlignResult::ALIGNED: // 对齐完成
          state_ = NavState::TRACKING;
          start_align_controller_.reset();
          goal_controller_.reset();

          final_cmd =
              makeZeroCommand(
                  CommandSource::TRACKING_STOP,
                  now_sec);
          break;

        case StartAlignResult::TIMED_OUT: // 对齐超时
        case StartAlignResult::INVALID_TASK: // 无效任务
        case StartAlignResult::INVALID_TIME: // 无效时间
        case StartAlignResult::INVALID_CONFIG: // 无效配置
          enterFailedState();

          final_cmd =
              makeZeroCommand(
                  CommandSource::FAILED_STOP,
                  now_sec);
          break;

        case StartAlignResult::IDLE: // 空闲状态
          final_cmd =
              makeZeroCommand(
                  CommandSource::START_ALIGN,
                  now_sec);
          break;
      }
      }
    }
  }
  else if (output.planner_action.type ==
           PlannerActionType::CANCEL) // 如果规划动作为取消，则发送零速度命令
  {
    final_cmd =
        makeZeroCommand(
            CommandSource::CANCEL_STOP,
            now_sec);
  }
  else // 否则，根据当前状态进行处理
  {
    switch (state_) // 根据状态进行处理
    {
      case NavState::IDLE: // 空闲状态，发送零速度命令
        final_cmd =
            makeZeroCommand(
                CommandSource::IDLE_STOP,
                now_sec);
        break;

      case NavState::PLANNING: // 规划状态，发送零速度命令
        final_cmd =
            makeZeroCommand(
                CommandSource::PLANNING_STOP,
                now_sec);
        break;

      case NavState::PAUSED: // 暂停状态，发送零速度命令
        final_cmd =
            makeZeroCommand(CommandSource::PAUSE_STOP, now_sec);
        break;

      case NavState::TRACKING: // 跟踪状态，处理跟踪逻辑
      {
        /* 任务没有激活或没有路线，则进入失败状态*/
        if (!task_manager_.hasActiveTask() || !route_manager_.hasRoute())
        {
          enterFailedState();

          final_cmd =
              makeZeroCommand(
                  CommandSource::FAILED_STOP,
                  now_sec);
        }
        else /* 任务激活且路线存在，则处理跟踪逻辑*/
        {
          const RouteProgressOutput progress_output =
              route_manager_.updateProgress(input.robot, now_sec);

          switch (progress_output.result) // 根据路线进度结果进行处理
          {
            case RouteProgressResult::VALID: // 路线进度有效
            {
              output.route_progress =
                  progress_output.progress;
              output.route_elevation = route_manager_.assessElevation(
                  progress_output.progress, config_.stair_up);

              RouteCorridorObservationOutput obs_output =
                  route_corridor_observation_gate_.evaluate(
                      progress_output.progress,
                      input.route_corridor_observation,
                      now_sec);

              // A clear corridor alone is not enough to leave avoidance when
              // the companion obstacle summary from the current map snapshot
              // is unavailable. Keep LOCAL_AVOID latched until both views are
              // valid again.
              if (navigation_mode_manager_.status().mode ==
                      NavigationMode::LOCAL_AVOID &&
                  obs_output.result ==
                      RouteCorridorObservationResult::CLEAR &&
                  !input.obstacles.valid)
              {
                obs_output.result =
                    RouteCorridorObservationResult::WAITING_FOR_OBSERVATION;
              }

              // Output route_corridor for CLEAR/BLOCKED.
              if (obs_output.result ==
                      RouteCorridorObservationResult::CLEAR ||
                  obs_output.result ==
                      RouteCorridorObservationResult::BLOCKED)
              {
                output.route_corridor =
                    obs_output.assessment;
              }

              // Call NavigationModeManager.
              NavigationTask task_metadata{};
              task_metadata.sequence = task_manager_.session().sequence;
              task_metadata.mode = task_manager_.session().mode;
              task_metadata.max_vx = task_manager_.session().max_vx;
              const NavigationModeOutput mode_output =
                  navigation_mode_manager_.update(
                      task_metadata,
                      input.robot,
                      progress_output.progress,
                      output.route_elevation,
                      obs_output,
                      input.obstacles,
                      now_sec);

              const bool corridor_available =
                  (obs_output.result ==
                       RouteCorridorObservationResult::CLEAR ||
                   obs_output.result ==
                       RouteCorridorObservationResult::BLOCKED);

              switch (mode_output.result)
              {
                case NavigationModeUpdateResult::UPDATED:
                {
                  output.navigation_mode =
                      mode_output.status;
                  final_cmd = executeMode(
                      route_manager_.taskView(),
                      input.robot,
                      progress_output.progress,
                      mode_output.status,
                      input.obstacles,
                      obs_output.assessment,
                      corridor_available,
                      task_manager_.session().max_vx,
                      now_sec);
                  break;
                }

                case NavigationModeUpdateResult::WAITING_FOR_CORRIDOR:
                case NavigationModeUpdateResult::WAITING_FOR_ROBOT:
                case NavigationModeUpdateResult::IDLE:
                  output.navigation_mode =
                      mode_output.status;
                  final_cmd = makeZeroCommand(
                      CommandSource::TRACKING_STOP,
                      now_sec);
                  break;

                default:
                  enterFailedState();

                  final_cmd =
                      makeZeroCommand(
                          CommandSource::FAILED_STOP,
                          now_sec);
                  break;
              }

              break;
            }

            case RouteProgressResult::WAITING_FOR_ROBOT:
            case RouteProgressResult::IDLE:
              final_cmd =
                  makeZeroCommand(
                      CommandSource::TRACKING_STOP,
                      now_sec);
              break;

            case RouteProgressResult::INVALID_TIME:
            case RouteProgressResult::INVALID_CONFIG:
            case RouteProgressResult::INVALID_TASK:
              enterFailedState();

              final_cmd =
                  makeZeroCommand(
                      CommandSource::FAILED_STOP,
                      now_sec);
              break;
          }
        }
      }
      break;

      case NavState::GOAL_ALIGN:
      {
        if (!task_manager_.hasActiveTask() || !route_manager_.hasRoute())
        {
          enterFailedState();
          final_cmd = makeZeroCommand(CommandSource::FAILED_STOP, now_sec);
          break;
        }

        const RouteProgressOutput progress_output =
            route_manager_.updateProgress(input.robot, now_sec);
        if (progress_output.result != RouteProgressResult::VALID)
        {
          final_cmd = makeZeroCommand(CommandSource::GOAL_ALIGN, now_sec);
          break;
        }

        output.route_progress = progress_output.progress;
        const auto result = goal_controller_.update(
            route_manager_.taskView(),
            input.robot,
            progress_output.progress,
            task_manager_.session().max_vx,
            std::min(config_.limits.max_yaw_rate,
                     config_.goal_controller.near_goal_max_w),
            now_sec);
        final_cmd = result.command;
        if (result.position_lost)
        {
          state_ = NavState::TRACKING;
        }
        else if (result.finished)
        {
          state_ = NavState::SUCCEEDED;
          obstacle_finished_ = false;
          task_manager_.complete(task_manager_.session().sequence);
          resetNearGoalBlockedTimer();
          navigation_mode_manager_.reset();
          safety_supervisor_.reset();
        }
        break;
      }

      case NavState::FAILED:
        final_cmd =
            makeZeroCommand(
                CommandSource::FAILED_STOP,
                now_sec);
        break;

      case NavState::SUCCEEDED:
        final_cmd =
            makeZeroCommand(
                CommandSource::TRACKING_STOP,
                now_sec);
        break;

      default:
        final_cmd =
            makeZeroCommand(
                CommandSource::SAFETY_STOP,
                now_sec);
        break;
    }
  }

  // SafetySupervisor is the single final gate after every active controller.
  // Runtime receives this result verbatim and performs no navigation decision.
  if (task_manager_.hasActiveTask() &&
      (state_ == NavState::START_ALIGN || state_ == NavState::TRACKING ||
       state_ == NavState::GOAL_ALIGN || state_ == NavState::RECOVERY))
  {
    SafetySupervisor::Context safety_context{};
    safety_context.robot = input.robot;
    safety_context.obstacles = input.obstacles;
    safety_context.corridor = output.route_corridor.valid
        ? output.route_corridor : input.route_corridor_observation;
    safety_context.prefer_route_corridor_front =
        output.route_elevation.ascending ||
        output.navigation_mode.stair_up_active;
    safety_context.map_valid = safety_context.corridor.valid;
    safety_context.map_stamp_sec = safety_context.corridor.map_stamp_sec;
    if (state_ == NavState::START_ALIGN && !safety_context.map_valid)
    {
      // Before progress exists there is no route-corridor result yet; the
      // obstacle summary is still derived from the same current map snapshot.
      safety_context.map_valid = input.obstacles.valid;
      safety_context.map_stamp_sec = input.obstacles.stamp_sec;
    }
    final_cmd = safety_supervisor_.apply(final_cmd, safety_context,
        task_manager_.session().max_vx, now_sec);
  }

  output.state = state_;
  output.task_sequence =
      task_manager_.session().sequence;
  output.final_cmd = final_cmd;
  output.obstacle_finished = obstacle_finished_;

  return output;
}

// =============================================================================
// state
// 返回当前导航状态机状态（NavState）。
// =============================================================================

NavState NavigationCoordinator::state() const noexcept
{
  return state_;
}

// =============================================================================
// config
// 返回当前使用的全部配置。
// =============================================================================

const NavdogConfig& NavigationCoordinator::config() const noexcept
{
  return config_;
}

}  // namespace navdog
