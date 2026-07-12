#include "navdog_core/navigation_coordinator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace navdog
{

namespace
{

constexpr double kEpsilon = 1e-9;
constexpr double kPi = 3.14159265358979323846;
constexpr double kLocalAvoidSearchStepM = 0.20;
constexpr double kRejoinEndSpeedMps = 0.20;

double normalizeAngle(double angle) noexcept
{
  while (angle > kPi)
    angle -= 2.0 * kPi;
  while (angle < -kPi)
    angle += 2.0 * kPi;
  return angle;
}

}  // namespace

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
  clearLocalPlanningContext();
  start_align_controller_.reset();
  route_progress_tracker_.reset();
  navigation_mode_manager_.reset();
  trajectory_follower_.reset();
  goal_controller_.reset();
  safety_supervisor_.reset();

  last_mode_ = NavigationMode::NONE;
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
// clearLocalPlanningContext
// =============================================================================

void NavigationCoordinator::clearLocalPlanningContext() noexcept
{
  last_local_plan_task_sequence_ = 0;
  last_local_plan_plan_sequence_ = 0;
  last_local_plan_request_stamp_sec_ = 0.0;
  last_local_plan_failed_ = false;

  expected_local_plan_sequence_ = 0;
  expected_local_plan_task_sequence_ = 0;
  expected_local_plan_purpose_ = NavigationMode::NONE;

  accepted_task_sequence_ = 0;
  accepted_plan_sequence_ = 0;
  accepted_purpose_ = NavigationMode::NONE;

  last_request_target_x_ = 0.0;
  last_request_target_y_ = 0.0;
  force_replan_ = false;
  resetNearGoalBlockedTimer();

  trajectory_follower_.reset();
}

void NavigationCoordinator::resetNearGoalBlockedTimer() noexcept
{
  near_goal_blocked_since_sec_ = 0.0;
  near_goal_blocked_timer_active_ = false;
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
  clearLocalPlanningContext();
  start_align_controller_.reset();
  route_progress_tracker_.reset();
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
// interpolateRoutePoint
// =============================================================================

bool NavigationCoordinator::interpolateRoutePoint(
    const NavigationTask& task,
    double target_arc_length_m,
    RoutePoint& out) const noexcept
{
  out = RoutePoint{};

  const auto& points = task.points;
  if (points.size() < 2)
    return false;

  if (target_arc_length_m <= 0.0)
  {
    out = points.front();
    return true;
  }

  double accumulated = 0.0;
  for (std::size_t i = 1; i < points.size(); ++i)
  {
    const double dx = points[i].x - points[i - 1].x;
    const double dy = points[i].y - points[i - 1].y;
    const double seg_len = std::hypot(dx, dy);

    if (seg_len < kEpsilon)
      continue;

    if (accumulated + seg_len >= target_arc_length_m)
    {
      const double ratio =
          (target_arc_length_m - accumulated) / seg_len;
      out.x = points[i - 1].x + ratio * dx;
      out.y = points[i - 1].y + ratio * dy;
      out.z = points[i - 1].z + ratio *
          (points[i].z - points[i - 1].z);

      if (points[i - 1].has_yaw && points[i].has_yaw)
      {
        const double yaw_diff = normalizeAngle(
            points[i].yaw - points[i - 1].yaw);
        out.yaw = normalizeAngle(
            points[i - 1].yaw + ratio * yaw_diff);
        out.has_yaw = true;
      }
      else if (points[i].has_yaw)
      {
        out.yaw = points[i].yaw;
        out.has_yaw = true;
      }
      else if (points[i - 1].has_yaw)
      {
        out.yaw = points[i - 1].yaw;
        out.has_yaw = true;
      }
      else
      {
        out.yaw = std::atan2(dy, dx);
        out.has_yaw = false;
      }

      return true;
    }

    accumulated += seg_len;
  }

  out = points.back();
  return true;
}

// =============================================================================
// selectLocalAvoidTarget
// =============================================================================

bool NavigationCoordinator::selectLocalAvoidTarget(
    const NavigationTask& task,
    const RobotState& robot,
    const RouteProgress& progress,
    RoutePoint& out_target) const
{
  out_target = RoutePoint{};

  if (task.points.size() < 2 ||
      !progress.valid ||
      !std::isfinite(progress.arc_length_m) ||
      !std::isfinite(progress.total_length_m))
  {
    return false;
  }

  if (!occupancy_query_ || !occupancy_query_->ready())
    return false;

  const double remaining =
      progress.total_length_m - progress.arc_length_m;
  const bool terminal_only =
      remaining + kEpsilon <
          config_.rejoin_target.min_forward_distance_m;
  const double min_arc = terminal_only
      ? progress.total_length_m
      : progress.arc_length_m +
          config_.rejoin_target.min_forward_distance_m;
  const double max_arc = std::min(
      progress.total_length_m,
      progress.arc_length_m +
          config_.rejoin_target.max_forward_distance_m);

  if (min_arc > max_arc)
    return false;

  const double preferred_arc = std::max(
      min_arc,
      std::min(
          progress.arc_length_m +
              config_.rejoin_target.default_forward_distance_m,
          progress.total_length_m));

  std::vector<double> candidate_arcs;
  candidate_arcs.reserve(64);

  // Forward sweep from preferred.
  double arc = preferred_arc;
  while (arc <= max_arc + kEpsilon)
  {
    candidate_arcs.push_back(arc);
    arc += kLocalAvoidSearchStepM;
  }

  // Backward sweep from preferred - step down to min_arc.
  arc = preferred_arc - kLocalAvoidSearchStepM;
  while (arc >= min_arc - kEpsilon)
  {
    candidate_arcs.push_back(arc);
    arc -= kLocalAvoidSearchStepM;
  }

  candidate_arcs.push_back(min_arc);
  if (max_arc > min_arc + kEpsilon)
    candidate_arcs.push_back(max_arc);

  for (double candidate_arc : candidate_arcs)
  {
    if (candidate_arc <= progress.arc_length_m)
      continue;
    if (candidate_arc > progress.total_length_m + kEpsilon)
      continue;

    RoutePoint candidate{};
    if (!interpolateRoutePoint(task, candidate_arc, candidate))
      continue;

    if (!std::isfinite(candidate.x) ||
        !std::isfinite(candidate.y) ||
        !std::isfinite(candidate.z) ||
        !std::isfinite(candidate.yaw))
    {
      continue;
    }

    const double yaw = candidate.has_yaw
                           ? candidate.yaw
                           : robot.yaw;

    if (!occupancy_query_->isFree(
            candidate.x,
            candidate.y,
            candidate.z,
            yaw))
    {
      continue;
    }

    out_target = candidate;
    return true;
  }

  return false;
}

// =============================================================================
// trajectoryEndingSoon
// =============================================================================

bool NavigationCoordinator::trajectoryEndingSoon(
    const LocalTrajectory& trajectory,
    double exec_time_sec) const noexcept
{
  if (!trajectory.valid ||
      !std::isfinite(trajectory.duration_sec))
  {
    return true;
  }

  const double remaining =
      trajectory.duration_sec - exec_time_sec;

  return remaining <=
         config_.planner_trigger.min_remaining_duration_sec;
}

// =============================================================================
// isTrajectoryExecutable
// =============================================================================

bool NavigationCoordinator::isTrajectoryExecutable(
    const LocalTrajectory& trajectory,
    NavigationMode expected_mode,
    std::uint64_t expected_task_sequence,
    std::uint64_t expected_plan_sequence) const noexcept
{
  if (!trajectory.valid)
    return false;

  if (trajectory.purpose != expected_mode)
    return false;

  if (trajectory.task_sequence != expected_task_sequence)
    return false;

  if (trajectory.plan_sequence != expected_plan_sequence)
    return false;

  if (!std::isfinite(trajectory.duration_sec) ||
      trajectory.duration_sec <= 0.0)
  {
    return false;
  }

  return true;
}

// =============================================================================
// isTrajectoryHealthy
// =============================================================================

bool NavigationCoordinator::isTrajectoryHealthy(
    NavigationMode mode,
    const RouteProgress& progress,
    double now_sec)
{
  if (!local_planner_adapter_)
    return false;

  const LocalTrajectory trajectory =
      local_planner_adapter_->getLocalTrajectory(
          mode, progress.task_sequence);

  if (!isTrajectoryExecutable(
          trajectory,
          mode,
          progress.task_sequence,
          expected_local_plan_sequence_))
  {
    return false;
  }

  const double exec_time_sec =
      trajectory_follower_.trajectoryTimeSec();

  if (exec_time_sec >
      trajectory.duration_sec +
          config_.planner_trigger.trajectory_expiry_margin_sec)
  {
    return false;
  }

  if (trajectoryEndingSoon(trajectory, exec_time_sec))
    return false;

  if (local_planner_adapter_->isTrajectoryColliding(
          mode,
          progress.task_sequence,
          expected_local_plan_sequence_,
          exec_time_sec))
  {
    return false;
  }

  const bool newly_accepted =
      trajectory.task_sequence != accepted_task_sequence_ ||
      trajectory.plan_sequence != accepted_plan_sequence_ ||
      trajectory.purpose != accepted_purpose_;
  if (newly_accepted)
  {
    if (!std::isfinite(trajectory.source_stamp_sec))
      return false;

    const double age = now_sec - trajectory.source_stamp_sec;
    if (!std::isfinite(age) ||
        age < -config_.planner_trigger.trajectory_future_tolerance_sec ||
        age > config_.planner_trigger.trajectory_source_max_age_sec)
    {
      return false;
    }

    accepted_task_sequence_ = trajectory.task_sequence;
    accepted_plan_sequence_ = trajectory.plan_sequence;
    accepted_purpose_ = trajectory.purpose;
  }

  return true;
}

// =============================================================================
// needsNewLocalPlan
// =============================================================================

bool NavigationCoordinator::needsNewLocalPlan(
    NavigationMode mode,
    const RouteProgress& progress,
    double now_sec)
{
  if (mode != NavigationMode::LOCAL_AVOID &&
      mode != NavigationMode::ROUTE_REJOIN)
  {
    return false;
  }

  // Mode just entered.
  if (last_mode_ != mode)
  {
    return true;
  }

  // Task changed.
  if (last_local_plan_task_sequence_ != progress.task_sequence)
  {
    return true;
  }

  if (!local_planner_adapter_)
  {
    return true;
  }

  // No matching valid trajectory.
  if (!isTrajectoryHealthy(mode, progress, now_sec))
  {
    return true;
  }

  // Target changed significantly compared to last request.
  if (force_replan_)
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

  if (!std::isfinite(robot.x) || !std::isfinite(robot.y) ||
      !std::isfinite(robot.z) || !std::isfinite(robot.yaw) ||
      !std::isfinite(robot.vx) || !std::isfinite(robot.vy) ||
      !std::isfinite(robot.yaw_rate))
  {
    return;
  }

  if (!local_planner_adapter_)
  {
    return;
  }

  if (!occupancy_query_ || !occupancy_query_->ready())
  {
    expected_local_plan_sequence_ = 0;
    expected_local_plan_task_sequence_ = 0;
    expected_local_plan_purpose_ = NavigationMode::NONE;
    trajectory_follower_.reset();
    return;
  }

  LocalPlanRequest request{};
  request.purpose = mode;
  request.task_sequence = task.sequence;
  request.plan_sequence = last_local_plan_plan_sequence_ + 1;
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

  RoutePoint target{};
  bool target_ok = false;

  if (mode == NavigationMode::LOCAL_AVOID)
  {
    target_ok = selectLocalAvoidTarget(
        task, robot, progress, target);

    if (!target_ok)
    {
      // Do not reuse the previous trajectory while waiting for a valid target.
      expected_local_plan_sequence_ = 0;
      expected_local_plan_task_sequence_ = 0;
      expected_local_plan_purpose_ = NavigationMode::NONE;
      trajectory_follower_.reset();
      return;
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
      // Do not reuse the previous trajectory while waiting for a valid rejoin target.
      expected_local_plan_sequence_ = 0;
      expected_local_plan_task_sequence_ = 0;
      expected_local_plan_purpose_ = NavigationMode::NONE;
      trajectory_follower_.reset();
      return;
    }

    target = rejoin_result.target;
    target_ok = true;

    request.target = target;

    const double rejoin_speed =
        std::min(task.max_vx, kRejoinEndSpeedMps);
    request.target_vel.x =
        rejoin_speed * std::cos(target.yaw);
    request.target_vel.y =
        rejoin_speed * std::sin(target.yaw);
    request.target_vel.has_yaw = true;
    request.target_vel.yaw = target.yaw;
  }

  if (!target_ok)
  {
    return;
  }

  // Detect target change to force replan.
  const double target_dx = target.x - last_request_target_x_;
  const double target_dy = target.y - last_request_target_y_;
  if (std::hypot(target_dx, target_dy) >
      config_.planner_trigger.target_change_threshold_m)
  {
    force_replan_ = true;
  }

  if (!needsNewLocalPlan(mode, progress, now_sec))
  {
    // Target did not actually change enough to require replan.
    force_replan_ = false;
    return;
  }

  // If we are still waiting for a previous request to complete, throttle.
  const LocalPlanState plan_state =
      local_planner_adapter_->localPlanState(
          expected_local_plan_purpose_,
          expected_local_plan_task_sequence_,
          expected_local_plan_sequence_);

  if (plan_state == LocalPlanState::QUEUED ||
      plan_state == LocalPlanState::PLANNING)
  {
    return;
  }

  if (plan_state == LocalPlanState::FAILED)
  {
    last_local_plan_failed_ = true;
  }

  // If previous request failed, wait for retry interval.
  if (last_local_plan_failed_)
  {
    const double elapsed =
        now_sec - last_local_plan_request_stamp_sec_;
    if (elapsed < config_.planner_trigger.replan_retry_interval_sec)
    {
      return;
    }
  }

  if (local_planner_adapter_->requestLocalPlan(request))
  {
    last_local_plan_task_sequence_ = task.sequence;
    last_local_plan_plan_sequence_ = request.plan_sequence;
    last_local_plan_request_stamp_sec_ = now_sec;
    last_local_plan_failed_ = false;

    expected_local_plan_sequence_ = request.plan_sequence;
    expected_local_plan_task_sequence_ = request.task_sequence;
    expected_local_plan_purpose_ = request.purpose;

    last_request_target_x_ = target.x;
    last_request_target_y_ = target.y;
    force_replan_ = false;
  }
  else
  {
    last_local_plan_failed_ = true;
    last_local_plan_request_stamp_sec_ = now_sec;

    // Do not reuse the previous trajectory while replanning fails.
    expected_local_plan_sequence_ = 0;
    expected_local_plan_task_sequence_ = 0;
    expected_local_plan_purpose_ = NavigationMode::NONE;
    trajectory_follower_.reset();
  }
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
  trajectory_follower_.reset();

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
      clearLocalPlanningContext();
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
  (void)mode_status;
  (void)max_vx;

  if (!local_planner_adapter_)
  {
    trajectory_follower_.reset();
    return makeZeroCommand(
        CommandSource::TRACKING_STOP, now_sec);
  }

  const LocalPlanState plan_state =
      local_planner_adapter_->localPlanState(
          NavigationMode::LOCAL_AVOID,
          progress.task_sequence,
          expected_local_plan_sequence_);

  if (plan_state == LocalPlanState::QUEUED ||
      plan_state == LocalPlanState::PLANNING)
  {
    trajectory_follower_.reset();
    return makeZeroCommand(
        CommandSource::TRACKING_STOP, now_sec);
  }

  if (plan_state == LocalPlanState::FAILED)
  {
    trajectory_follower_.reset();
    last_local_plan_failed_ = true;
    return makeZeroCommand(
        CommandSource::TRACKING_STOP, now_sec);
  }

  const LocalTrajectory trajectory =
      local_planner_adapter_->getLocalTrajectory(
          NavigationMode::LOCAL_AVOID,
          progress.task_sequence);

  if (!isTrajectoryExecutable(
          trajectory,
          NavigationMode::LOCAL_AVOID,
          progress.task_sequence,
          expected_local_plan_sequence_))
  {
    trajectory_follower_.reset();
    return makeZeroCommand(
        CommandSource::TRACKING_STOP, now_sec);
  }

  const double exec_time_sec =
      trajectory_follower_.trajectoryTimeSec();

  if (exec_time_sec >
      trajectory.duration_sec +
          config_.planner_trigger.trajectory_expiry_margin_sec)
  {
    trajectory_follower_.reset();
    return makeZeroCommand(
        CommandSource::TRACKING_STOP, now_sec);
  }

  if (trajectoryEndingSoon(trajectory, exec_time_sec))
  {
    trajectory_follower_.reset();
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
  (void)task;
  (void)mode_status;
  (void)max_vx;

  if (!local_planner_adapter_)
  {
    trajectory_follower_.reset();
    return makeZeroCommand(
        CommandSource::TRACKING_STOP, now_sec);
  }

  const LocalPlanState plan_state =
      local_planner_adapter_->localPlanState(
          NavigationMode::ROUTE_REJOIN,
          progress.task_sequence,
          expected_local_plan_sequence_);

  if (plan_state == LocalPlanState::QUEUED ||
      plan_state == LocalPlanState::PLANNING)
  {
    trajectory_follower_.reset();
    return makeZeroCommand(
        CommandSource::TRACKING_STOP, now_sec);
  }

  if (plan_state == LocalPlanState::FAILED)
  {
    trajectory_follower_.reset();
    last_local_plan_failed_ = true;
    return makeZeroCommand(
        CommandSource::TRACKING_STOP, now_sec);
  }

  const LocalTrajectory trajectory =
      local_planner_adapter_->getLocalTrajectory(
          NavigationMode::ROUTE_REJOIN,
          progress.task_sequence);

  if (!isTrajectoryExecutable(
          trajectory,
          NavigationMode::ROUTE_REJOIN,
          progress.task_sequence,
          expected_local_plan_sequence_))
  {
    trajectory_follower_.reset();
    return makeZeroCommand(
        CommandSource::TRACKING_STOP, now_sec);
  }

  const double exec_time_sec =
      trajectory_follower_.trajectoryTimeSec();

  if (exec_time_sec >
      trajectory.duration_sec +
          config_.planner_trigger.trajectory_expiry_margin_sec)
  {
    trajectory_follower_.reset();
    return makeZeroCommand(
        CommandSource::TRACKING_STOP, now_sec);
  }

  if (trajectoryEndingSoon(trajectory, exec_time_sec))
  {
    trajectory_follower_.reset();
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
    const ObstacleSummary& obstacles,
    const RouteCorridorAssessment& corridor,
    bool corridor_available,
    double max_vx,
    double now_sec)
{
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
      clearLocalPlanningContext();
      safety_supervisor_.reset();
    }

    SafetySupervisor::Context safety_context{};
    safety_context.robot = robot;
    safety_context.obstacles = obstacles;
    safety_context.corridor = corridor;
    safety_context.map_valid = true;
    safety_context.map_stamp_sec = corridor.map_stamp_sec;
    return safety_supervisor_.apply(
        makeZeroCommand(CommandSource::TRACKING_STOP, now_sec),
        safety_context, max_vx, now_sec);
  }
  resetNearGoalBlockedTimer();

  // A mode transition invalidates the old mode's trajectory identity. Clear
  // it before submitting the first request for the new mode.
  if (last_mode_ != mode_status.mode)
  {
    trajectory_follower_.reset();
    expected_local_plan_sequence_ = 0;
    expected_local_plan_task_sequence_ = 0;
    expected_local_plan_purpose_ = NavigationMode::NONE;
    accepted_task_sequence_ = 0;
    accepted_plan_sequence_ = 0;
    accepted_purpose_ = NavigationMode::NONE;
  }

  requestLocalPlanIfNeeded(
      task, robot, progress, mode_status.mode, now_sec);

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
  safety_context.obstacles = obstacles;
  safety_context.corridor = corridor;
  safety_context.map_valid = corridor_available;
  safety_context.map_stamp_sec = corridor.map_stamp_sec;

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
      clearLocalPlanningContext();
      start_align_controller_.reset();
      route_progress_tracker_.reset();
      navigation_mode_manager_.reset();
      goal_controller_.reset();
      safety_supervisor_.reset();
      state_ = NavState::PLANNING;
      enqueuePlannerAction(task_output.planner_action);
      break;

    case TaskHandleResult::CANCELLED:
      clearPlanningContext();
      clearLocalPlanningContext();
      start_align_controller_.reset();
      route_progress_tracker_.reset();
      navigation_mode_manager_.reset();
      goal_controller_.reset();
      safety_supervisor_.reset();
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
          trajectory_follower_.reset();
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
                      active_task,
                      input.robot,
                      progress_output.progress,
                      mode_output.status,
                      input.obstacles,
                      obs_output.assessment,
                      corridor_available,
                      active_task.max_vx,
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
