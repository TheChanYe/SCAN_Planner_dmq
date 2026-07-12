#include "navdog_core/navigation_coordinator.hpp"

#include <cmath>
#include <limits>

namespace navdog
{

// =============================================================================
// Constructor
// =============================================================================

NavigationCoordinator::NavigationCoordinator(
    const NavdogConfig& config)
    : config_(config),
      state_(NavState::IDLE),
      task_manager_(config.task),
      start_align_controller_(config.start_align),
      route_progress_tracker_(config.route_progress),
      route_corridor_observation_gate_(
          config.route_corridor_observation),
      navigation_mode_manager_(config.navigation_mode),
      route_follower_(config.route_follower),
      trajectory_follower_(config.trajectory_follower),
      rejoin_target_selector_(config.rejoin_target),
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
  task_manager_.reset();
  pending_planner_actions_.clear();
  clearPlanningContext();
  start_align_controller_.reset();
  route_progress_tracker_.reset();
  navigation_mode_manager_.reset();
  trajectory_follower_.reset();
  goal_controller_.reset();

  last_mode_ = NavigationMode::NONE;
  last_local_plan_task_sequence_ = 0;
  last_local_plan_plan_sequence_ = 0;
  last_local_plan_request_stamp_sec_ = 0.0;
  local_plan_request_pending_ = false;
}

// =============================================================================
// setLocalPlannerAdapter
// =============================================================================

void NavigationCoordinator::setLocalPlannerAdapter(
    LocalPlannerAdapter* adapter) noexcept
{
  local_planner_adapter_ = adapter;
}

// =============================================================================
// setOccupancyQuery
// =============================================================================

void NavigationCoordinator::setOccupancyQuery(
    OccupancyQuery3D* query) noexcept
{
  occupancy_query_ = query;
}

// =============================================================================
// enqueuePlannerAction
// =============================================================================

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
  start_align_controller_.reset();
  route_progress_tracker_.reset();
  navigation_mode_manager_.reset();
  trajectory_follower_.reset();
  goal_controller_.reset();
}

// =============================================================================
// resetStartAlign
// =============================================================================

void NavigationCoordinator::resetStartAlign() noexcept
{
  start_align_controller_.reset();
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
// trajectoryEndingSoon
// =============================================================================

bool NavigationCoordinator::trajectoryEndingSoon(
    const LocalTrajectory& trajectory,
    double now_sec) const noexcept
{
  if (!trajectory.valid ||
      !std::isfinite(trajectory.source_stamp_sec) ||
      !std::isfinite(trajectory.duration_sec))
  {
    return true;
  }

  const double elapsed = now_sec - trajectory.source_stamp_sec;
  const double remaining =
      trajectory.duration_sec - elapsed;

  return remaining <=
         config_.planner_trigger.min_remaining_duration_sec;
}

// =============================================================================
// needsNewLocalPlan
// =============================================================================

bool NavigationCoordinator::needsNewLocalPlan(
    NavigationMode mode,
    const RouteProgress& progress,
    double now_sec) const
{
  if (mode != NavigationMode::LOCAL_AVOID &&
      mode != NavigationMode::ROUTE_REJOIN)
  {
    return false;
  }

  if (last_mode_ != mode)
  {
    return true;
  }

  if (last_local_plan_task_sequence_ != progress.task_sequence)
  {
    return true;
  }

  if (!local_planner_adapter_)
  {
    return true;
  }

  if (!local_planner_adapter_->hasValidTrajectory(
          mode, progress.task_sequence))
  {
    return true;
  }

  const LocalTrajectory trajectory =
      local_planner_adapter_->getLocalTrajectory(
          mode, progress.task_sequence);

  if (trajectoryEndingSoon(trajectory, now_sec))
    return true;

  if (!trajectory.valid)
    return true;

  if (local_planner_adapter_->isTrajectoryColliding(
          mode, progress.task_sequence))
  {
    return true;
  }

  const double elapsed_since_request =
      now_sec - last_local_plan_request_stamp_sec_;
  if (elapsed_since_request >=
      config_.planner_trigger.replan_retry_interval_sec)
  {
    return true;
  }

  return false;
}

// =============================================================================
// requestLocalPlanIfNeeded
// =============================================================================

void NavigationCoordinator::requestLocalPlanIfNeeded(
    const NavigationTask& task,
    const RobotState& robot,
    const RouteProgress& progress,
    NavigationMode mode,
    double now_sec)
{
  if (mode != NavigationMode::LOCAL_AVOID &&
      mode != NavigationMode::ROUTE_REJOIN)
  {
    return;
  }

  if (!needsNewLocalPlan(mode, progress, now_sec))
  {
    return;
  }

  if (!local_planner_adapter_)
  {
    local_plan_request_pending_ = true;
    return;
  }

  LocalPlanRequest request{};
  request.purpose = mode;
  request.task_sequence = task.sequence;
  request.plan_sequence =
      last_local_plan_plan_sequence_ + 1;
  request.max_vx = task.max_vx;
  request.robot_z = robot.z;
  request.valid = true;

  request.start.x = robot.x;
  request.start.y = robot.y;
  request.start.z = robot.z;
  request.start.has_yaw = true;
  request.start.yaw = robot.yaw;

  request.start_vel.x = robot.vx;
  request.start_vel.y = robot.vy;
  request.start_vel.has_yaw = true;
  request.start_vel.yaw = robot.yaw_rate;

  if (mode == NavigationMode::LOCAL_AVOID)
  {
    const double target_arc =
        progress.arc_length_m +
        config_.rejoin_target.default_forward_distance_m;

    RoutePoint target{};
    bool found = false;

    for (double delta = config_.rejoin_target.min_forward_distance_m;
         delta <= config_.rejoin_target.max_forward_distance_m;
         delta += 0.2)
    {
      const double arc = progress.arc_length_m + delta;
      // Interpolate along route.
      double accumulated = 0.0;
      for (std::size_t i = 1; i < task.points.size(); ++i)
      {
        const double dx = task.points[i].x - task.points[i - 1].x;
        const double dy = task.points[i].y - task.points[i - 1].y;
        const double seg_len = std::hypot(dx, dy);
        if (seg_len < 1e-9)
          continue;

        if (accumulated + seg_len >= arc)
        {
          const double ratio = (arc - accumulated) / seg_len;
          target.x = task.points[i - 1].x + ratio * dx;
          target.y = task.points[i - 1].y + ratio * dy;
          target.z = task.points[i - 1].z + ratio *
              (task.points[i].z - task.points[i - 1].z);
          target.yaw = std::atan2(dy, dx);
          target.has_yaw = true;
          found = true;
          break;
        }
        accumulated += seg_len;
      }
      if (found)
        break;
    }

    if (!found)
    {
      local_plan_request_pending_ = true;
      return;
    }

    if (occupancy_query_ && occupancy_query_->ready())
    {
      if (!occupancy_query_->isFree(
              target.x, target.y, target.z,
              target.has_yaw ? target.yaw : robot.yaw))
      {
        local_plan_request_pending_ = true;
        return;
      }
    }

    request.target = target;
    request.target_vel = RoutePoint{};
  }
  else if (mode == NavigationMode::ROUTE_REJOIN)
  {
    const auto& mode_status =
        navigation_mode_manager_.status();

    const auto rejoin_result = rejoin_target_selector_.select(
        task,
        progress,
        mode_status,
        robot,
        occupancy_query_);

    if (!rejoin_result.valid)
    {
      local_plan_request_pending_ = true;
      return;
    }

    request.target = rejoin_result.target;
    request.target_vel = RoutePoint{};
  }

  if (local_planner_adapter_->requestLocalPlan(request))
  {
    last_local_plan_task_sequence_ = task.sequence;
    last_local_plan_plan_sequence_ = request.plan_sequence;
    last_local_plan_request_stamp_sec_ = now_sec;
    local_plan_request_pending_ = false;
  }
  else
  {
    local_plan_request_pending_ = true;
  }
}

// =============================================================================
// executeRouteFollow
// =============================================================================

VelocityCommand NavigationCoordinator::executeRouteFollow(
    const NavigationTask& task,
    const RobotState& robot,
    const RouteProgress& progress,
    double max_vx,
    double now_sec)
{
  trajectory_follower_.reset();

  if (goal_controller_.isNearGoal(progress))
  {
    const auto result = goal_controller_.update(
        task,
        robot,
        progress,
        max_vx,
        config_.limits.max_yaw_rate,
        now_sec);

    if (result.finished)
    {
      state_ = NavState::SUCCEEDED;
    }

    return result.command;
  }

  return route_follower_.update(
      task, robot, progress, max_vx, now_sec);
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
  (void)mode_status;

  if (!local_planner_adapter_)
  {
    return makeZeroCommand(
        CommandSource::TRACKING_STOP, now_sec);
  }

  const LocalTrajectory trajectory =
      local_planner_adapter_->getLocalTrajectory(
          NavigationMode::LOCAL_AVOID,
          progress.task_sequence);

  if (!trajectory.valid)
  {
    return makeZeroCommand(
        CommandSource::TRACKING_STOP, now_sec);
  }

  if (trajectoryEndingSoon(trajectory, now_sec))
  {
    return makeZeroCommand(
        CommandSource::TRACKING_STOP, now_sec);
  }

  return trajectory_follower_.update(
      trajectory,
      robot,
      max_vx,
      config_.limits.max_vy,
      config_.limits.max_yaw_rate,
      NavigationMode::LOCAL_AVOID,
      progress.task_sequence,
      now_sec);
}

// =============================================================================
// executeRouteRejoin
// =============================================================================

VelocityCommand NavigationCoordinator::executeRouteRejoin(
    const NavigationTask& task,
    const RobotState& robot,
    const RouteProgress& progress,
    const NavigationModeStatus& mode_status,
    double max_vx,
    double now_sec)
{
  if (!local_planner_adapter_)
  {
    return makeZeroCommand(
        CommandSource::TRACKING_STOP, now_sec);
  }

  const LocalTrajectory trajectory =
      local_planner_adapter_->getLocalTrajectory(
          NavigationMode::ROUTE_REJOIN,
          progress.task_sequence);

  if (!trajectory.valid)
  {
    return makeZeroCommand(
        CommandSource::TRACKING_STOP, now_sec);
  }

  if (trajectoryEndingSoon(trajectory, now_sec))
  {
    return makeZeroCommand(
        CommandSource::TRACKING_STOP, now_sec);
  }

  return trajectory_follower_.update(
      trajectory,
      robot,
      max_vx,
      config_.limits.max_vy,
      config_.limits.max_yaw_rate,
      NavigationMode::ROUTE_REJOIN,
      progress.task_sequence,
      now_sec);
}

// =============================================================================
// executeMode
// =============================================================================

VelocityCommand NavigationCoordinator::executeMode(
    const NavigationTask& task,
    const RobotState& robot,
    const RouteProgress& progress,
    const NavigationModeStatus& mode_status,
    double max_vx,
    double now_sec)
{
  requestLocalPlanIfNeeded(
      task, robot, progress, mode_status.mode, now_sec);

  VelocityCommand raw_cmd =
      makeZeroCommand(CommandSource::TRACKING_STOP, now_sec);

  switch (mode_status.mode)
  {
    case NavigationMode::ROUTE_FOLLOW:
      raw_cmd = executeRouteFollow(
          task, robot, progress, max_vx, now_sec);
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

    case NavigationMode::ROUTE_REJOIN:
      raw_cmd = executeRouteRejoin(
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

  SafetySupervisor::Context safety_context{};
  safety_context.robot = robot;
  safety_context.obstacles = {};
  safety_context.corridor = {};

  if (mode_status.mode == NavigationMode::LOCAL_AVOID ||
      mode_status.mode == NavigationMode::ROUTE_REJOIN)
  {
    if (local_planner_adapter_)
    {
      safety_context.trajectory =
          local_planner_adapter_->getLocalTrajectory(
              mode_status.mode,
              progress.task_sequence);
    }
  }

  safety_context.map_valid = false;
  safety_context.map_stamp_sec = 0.0;

  return safety_supervisor_.apply(
      raw_cmd, safety_context, max_vx, now_sec);
}

// =============================================================================
// handleEvent
// =============================================================================

TaskHandleResult NavigationCoordinator::handleEvent(
    const NavigationEvent& event)
{
  const TaskManagerOutput task_output =
      task_manager_.handleEvent(event);

  switch (task_output.result)
  {
    case TaskHandleResult::STARTED:
      clearPlanningContext();
      start_align_controller_.reset();
      route_progress_tracker_.reset();
      navigation_mode_manager_.reset();
      trajectory_follower_.reset();
      goal_controller_.reset();
      state_ = NavState::PLANNING;
      enqueuePlannerAction(task_output.planner_action);
      break;

    case TaskHandleResult::CANCELLED:
      clearPlanningContext();
      start_align_controller_.reset();
      route_progress_tracker_.reset();
      navigation_mode_manager_.reset();
      trajectory_follower_.reset();
      goal_controller_.reset();
      state_ = NavState::IDLE;
      enqueuePlannerAction(task_output.planner_action);
      break;

    case TaskHandleResult::MAX_VX_UPDATED:
      if (state_ == NavState::PLANNING ||
          state_ == NavState::START_ALIGN ||
          state_ == NavState::TRACKING)
      {
        enqueuePlannerAction(task_output.planner_action);
      }
      break;

    case TaskHandleResult::NONE:
    case TaskHandleResult::REJECTED_BUSY:
    case TaskHandleResult::REJECTED_INVALID_TASK:
    case TaskHandleResult::CANCEL_IGNORED:
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
// copyActiveTask
// =============================================================================

bool NavigationCoordinator::copyActiveTask(
    NavigationTask& task) const
{
  return task_manager_.copyActiveTask(task);
}

// =============================================================================
// update
// =============================================================================

CoreOutput NavigationCoordinator::update(
    const CoreInput& input,
    double now_sec)
{
  CoreOutput output{};

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
    NavigationTask active_task{};

    if (!task_manager_.copyActiveTask(active_task))
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
              active_task,
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
          navigation_mode_manager_.reset();
          trajectory_follower_.reset();

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

      case NavState::TRACKING:
      {
        NavigationTask active_task{};

        if (!task_manager_.copyActiveTask(active_task))
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
              route_progress_tracker_.update(
                  active_task,
                  input.robot,
                  now_sec);

          switch (progress_output.result)
          {
            case RouteProgressResult::VALID:
            {
              output.route_progress =
                  progress_output.progress;

              const RouteCorridorObservationOutput obs_output =
                  route_corridor_observation_gate_.evaluate(
                      progress_output.progress,
                      input.route_corridor_observation,
                      now_sec);

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
              const NavigationModeOutput mode_output =
                  navigation_mode_manager_.update(
                      active_task,
                      input.robot,
                      progress_output.progress,
                      obs_output,
                      now_sec);

              switch (mode_output.result)
              {
                case NavigationModeUpdateResult::UPDATED:
                case NavigationModeUpdateResult::WAITING_FOR_CORRIDOR:
                case NavigationModeUpdateResult::WAITING_FOR_ROBOT:
                case NavigationModeUpdateResult::IDLE:
                  output.navigation_mode =
                      mode_output.status;
                  final_cmd = executeMode(
                      active_task,
                      input.robot,
                      progress_output.progress,
                      mode_output.status,
                      active_task.max_vx,
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

  output.state = state_;
  output.task_sequence =
      task_manager_.activeSequence();
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
