#include "navdog_core/navigation_coordinator.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
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
      navigation_mode_manager_(config.navigation_mode),
      route_follower_(config.route_follower),
      goal_controller_(config.goal_controller),
      safety_supervisor_(config.safety, config.limits)
{
}

// =============================================================================
// reset
// =============================================================================

void NavigationCoordinator::reset()
{
  state_ = NavState::IDLE;
  last_logged_state_ = NavState::IDLE;
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
  state_before_pause_ = NavState::IDLE;
}

// =============================================================================
// resetNearGoalBlockedTimer
// =============================================================================

void NavigationCoordinator::resetNearGoalBlockedTimer() noexcept
{
  near_goal_blocked_since_sec_ = 0.0;
  near_goal_blocked_timer_active_ = false;
}

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
// =============================================================================

void NavigationCoordinator::clearPlanningContext() noexcept
{
  planning_request_sent_ = false;
  planning_started_sec_ = 0.0;
  expected_trajectory_id_ = 0;
}

// =============================================================================
// startPlanningContext
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
// =============================================================================

void NavigationCoordinator::enterFailedState() noexcept
{
  state_ = NavState::FAILED;

  pending_planner_actions_.clear();

  clearPlanningContext();
  resetNearGoalBlockedTimer();
  start_align_controller_.reset();
  route_manager_.reset();
  navigation_mode_manager_.reset();
  goal_controller_.reset();
  safety_supervisor_.reset();
}

// =============================================================================
// makeZeroCommand
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

  // Do not bypass corridor blocking state away from the finish region.
  if (mode_status.route_blocked_near ||
      mode_status.reason == NavigationModeReason::ROUTE_ONLY_BLOCKED)
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
  const double goal_distance = task.points.empty()
      ? std::numeric_limits<double>::infinity()
      : std::hypot(task.points.back().x - robot.x,
                   task.points.back().y - robot.y);

  double effective_max_vx = max_vx;

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

  // Only hand over to GoalController for final yaw alignment / finish.
  if (std::isfinite(goal_distance) &&
      goal_distance <= config_.goal_controller.finish_dist)
  {
    const auto result = goal_controller_.update(
        task,
        robot,
        progress,
        effective_max_vx,
        std::min(config_.limits.max_yaw_rate,
                 config_.goal_controller.near_goal_max_w),
        now_sec);

    if (result.finished)
    {
      state_ = NavState::SUCCEEDED;

      // Full cleanup: do not leave a stale local trajectory / safety state
      // after the task is successfully completed.
      resetNearGoalBlockedTimer();
      safety_supervisor_.reset();
    }

    return result.command;
  }

  return route_follower_.update(
      task, robot, progress, effective_max_vx, now_sec);
}

// =============================================================================
// executeLocalAvoid
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
  // Corridor / robot not ready: do not execute any real controller.
  if (!corridor_available)
  {
    resetNearGoalBlockedTimer();
    return makeZeroCommand(
        CommandSource::TRACKING_STOP, now_sec);
  }

  const double goal_distance = task.points.empty()
      ? std::numeric_limits<double>::infinity()
      : std::hypot(task.points.back().x - robot.x,
                   task.points.back().y - robot.y);
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
      resetNearGoalBlockedTimer();
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
// =============================================================================

TaskHandleResult NavigationCoordinator::handleEvent(
    NavigationEvent event)
{
  const std::uint64_t prior_sequence = task_manager_.session().sequence;
  navdog_task::TaskTransition task_output =
      task_manager_.handleEvent(std::move(event));

  switch (task_output.result)
  {
    case TaskHandleResult::STARTED:
      if (!route_manager_.acceptRoute(
              task_output.session.sequence,
              std::move(task_output.accepted_route)))
      {
        state_ = NavState::FAILED;
        return TaskHandleResult::REJECTED_INVALID_TASK;
      }
      clearPlanningContext();
      resetNearGoalBlockedTimer();
      start_align_controller_.reset();
      navigation_mode_manager_.reset();
      goal_controller_.reset();
      safety_supervisor_.reset();
      state_ = NavState::PLANNING;
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

    case TaskHandleResult::CANCELLED:
      clearPlanningContext();
      resetNearGoalBlockedTimer();
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

    case TaskHandleResult::MAX_VX_UPDATED:
      if (state_ == NavState::PLANNING ||
          state_ == NavState::START_ALIGN ||
          state_ == NavState::TRACKING)
      {
        { PlannerAction action{};
          action.type = PlannerActionType::UPDATE_SPEED_LIMIT;
          action.task.sequence = task_output.session.sequence;
          action.max_vx = task_output.session.max_vx;
          enqueuePlannerAction(action); }
      }
      break;

    case TaskHandleResult::PAUSED:
      if (state_ != NavState::PAUSED)
      {
        state_before_pause_ = state_;
        state_ = NavState::PAUSED;
        { PlannerAction action{}; action.type = PlannerActionType::PAUSE;
          enqueuePlannerAction(action); }
      }
      break;

    case TaskHandleResult::RESUMED:
      if (state_ == NavState::PAUSED)
      {
        state_ = state_before_pause_ == NavState::IDLE
            ? NavState::TRACKING : state_before_pause_;
        { PlannerAction action{}; action.type = PlannerActionType::RESUME;
          enqueuePlannerAction(action); }
      }
      break;

    case TaskHandleResult::NONE:
    case TaskHandleResult::REJECTED_BUSY:
    case TaskHandleResult::REJECTED_INVALID_TASK:
    case TaskHandleResult::CANCEL_IGNORED:
    case TaskHandleResult::PAUSE_RESUME_IGNORED:
    case TaskHandleResult::MAX_VX_UNCHANGED:
    case TaskHandleResult::MAX_VX_UPDATE_IGNORED:
    case TaskHandleResult::REJECTED_INVALID_MAX_VX:
    case TaskHandleResult::UNSUPPORTED_EVENT:
      break;
  }

  return task_output.result;
}

// =============================================================================
// hasActiveTask
// =============================================================================

bool NavigationCoordinator::hasActiveTask() const noexcept
{
  return task_manager_.hasActiveTask();
}

// =============================================================================
// route/session views
// =============================================================================

const RouteManager& NavigationCoordinator::routeManager() const noexcept
{ return route_manager_; }

const navdog_task::TaskSession& NavigationCoordinator::taskSession() const noexcept
{ return task_manager_.session(); }

// =============================================================================
// update
// =============================================================================

CoreOutput NavigationCoordinator::update(
    const CoreInput& input,
    double now_sec)
{
  CoreOutput output{};

  if (state_ != last_logged_state_)
  {
    const auto prev = last_logged_state_;
    last_logged_state_ = state_;
    std::printf("[NavigationCoordinator] NAV_STATE %u -> %u\n",
        static_cast<unsigned>(prev),
        static_cast<unsigned>(state_));
    std::fflush(stdout);
  }

  // --- Planning feedback and action emission ---
  if (state_ == NavState::PLANNING &&
      !planning_request_sent_)
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
  else
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

  // --- Build final_cmd ---
  VelocityCommand final_cmd =
      makeZeroCommand(
          CommandSource::IDLE_STOP,
          now_sec);

  // --- START_ALIGN processing ---
  if (state_ == NavState::START_ALIGN)
  {
    if (!task_manager_.hasActiveTask() || !route_manager_.hasRoute())
    {
      enterFailedState();

      final_cmd =
          makeZeroCommand(
              CommandSource::FAILED_STOP,
              now_sec);
    }
    else
    {
      const StartAlignOutput align_output =
          start_align_controller_.update(
              route_manager_.taskView(),
              input.robot,
              now_sec);

      switch (align_output.result)
      {
        case StartAlignResult::WAITING_FOR_ROBOT:
        case StartAlignResult::ALIGNING:
          final_cmd = align_output.command;
          break;

        case StartAlignResult::ALIGNED:
          state_ = NavState::TRACKING;
          start_align_controller_.reset();
          goal_controller_.reset();

          final_cmd =
              makeZeroCommand(
                  CommandSource::TRACKING_STOP,
                  now_sec);
          break;

        case StartAlignResult::TIMED_OUT:
        case StartAlignResult::INVALID_TASK:
        case StartAlignResult::INVALID_TIME:
        case StartAlignResult::INVALID_CONFIG:
          enterFailedState();

          final_cmd =
              makeZeroCommand(
                  CommandSource::FAILED_STOP,
                  now_sec);
          break;

        case StartAlignResult::IDLE:
          final_cmd =
              makeZeroCommand(
                  CommandSource::START_ALIGN,
                  now_sec);
          break;
      }
    }
  }
  else if (output.planner_action.type ==
           PlannerActionType::CANCEL)
  {
    final_cmd =
        makeZeroCommand(
            CommandSource::CANCEL_STOP,
            now_sec);
  }
  else
  {
    switch (state_)
    {
      case NavState::IDLE:
        final_cmd =
            makeZeroCommand(
                CommandSource::IDLE_STOP,
                now_sec);
        break;

      case NavState::PLANNING:
        final_cmd =
            makeZeroCommand(
                CommandSource::PLANNING_STOP,
                now_sec);
        break;

      case NavState::PAUSED:
        final_cmd =
            makeZeroCommand(CommandSource::PAUSE_STOP, now_sec);
        break;

      case NavState::TRACKING:
      {
        if (!task_manager_.hasActiveTask() || !route_manager_.hasRoute())
        {
          enterFailedState();

          final_cmd =
              makeZeroCommand(
                  CommandSource::FAILED_STOP,
                  now_sec);
        }
        else
        {
          const RouteProgressOutput progress_output =
              route_manager_.updateProgress(input.robot, now_sec);

          switch (progress_output.result)
          {
            case RouteProgressResult::VALID:
            {
              output.route_progress =
                  progress_output.progress;

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
                      obs_output,
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

  return output;
}

// =============================================================================
// state
// =============================================================================

NavState NavigationCoordinator::state() const noexcept
{
  return state_;
}

// =============================================================================
// config
// =============================================================================

const NavdogConfig& NavigationCoordinator::config() const noexcept
{
  return config_;
}

}  // namespace navdog
