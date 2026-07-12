#include "navdog_scan_adapter/scan_local_planner_adapter.hpp"

#include <bspline_opt/uniform_bspline.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace navdog_scan_adapter
{

namespace
{

constexpr double kEpsilon = 1e-9;
constexpr double kSampleDtSec = 0.05;

double rosTimeToSec(const ros::Time& t)
{
  return static_cast<double>(t.sec) +
         static_cast<double>(t.nsec) * 1e-9;
}

}  // namespace

// =============================================================================
// Constructor
// =============================================================================

ScanLocalPlannerAdapter::ScanLocalPlannerAdapter(
    const navdog::PlannerTriggerConfig& config,
    const std::shared_ptr<InflatedGridQuery3D>& grid_query,
    const std::shared_ptr<scan_planner::SCANPlannerManager>&
        planner_manager)
    : config_(config),
      grid_query_(grid_query),
      planner_manager_(planner_manager)
{
}

// =============================================================================
// requestLocalPlan
// =============================================================================

bool ScanLocalPlannerAdapter::requestLocalPlan(
    const navdog::LocalPlanRequest& request)
{
  if (!request.valid ||
      request.purpose == navdog::NavigationMode::NONE ||
      request.purpose == navdog::NavigationMode::ROUTE_FOLLOW ||
      !planner_manager_)
  {
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  if (!shouldReplan(request))
  {
    return true;
  }

  Eigen::Vector3d start_pt(request.start.x, request.start.y, request.robot_z);
  Eigen::Vector3d start_vel(
      request.start_vel.x, request.start_vel.y, 0.0);
  Eigen::Vector3d start_acc(0.0, 0.0, 0.0);

  Eigen::Vector3d end_pt(request.target.x, request.target.y, request.robot_z);
  Eigen::Vector3d end_vel(
      request.target_vel.x, request.target_vel.y, 0.0);

  const bool flag_poly_init = true;
  const bool flag_random_poly_traj = false;

  const bool ok = planner_manager_->reboundReplan(
      start_pt,
      start_vel,
      start_acc,
      end_pt,
      end_vel,
      flag_poly_init,
      flag_random_poly_traj);

  if (ok)
  {
    last_request_ = request;
    last_plan_stamp_sec_ = rosTimeToSec(ros::Time::now());
    planning_in_progress_ = false;
    cached_trajectory_ = sampleLocalTrajData(
        request.task_sequence,
        request.plan_sequence,
        request.purpose);
  }
  else
  {
    planning_in_progress_ = false;
  }

  return ok;
}

// =============================================================================
// sampleLocalTrajData
// =============================================================================

navdog::LocalTrajectory ScanLocalPlannerAdapter::sampleLocalTrajData(
    std::uint64_t task_sequence,
    std::uint64_t plan_sequence,
    navdog::NavigationMode purpose)
{
  navdog::LocalTrajectory trajectory{};

  if (!planner_manager_)
    return trajectory;

  const scan_planner::LocalTrajData& local_data =
      planner_manager_->local_data_;

  if (local_data.duration_ <= kEpsilon)
    return trajectory;

  const scan_planner::UniformBspline& pos_traj =
      local_data.position_traj_;
  const scan_planner::UniformBspline& vel_traj =
      local_data.velocity_traj_;

  trajectory.task_sequence = task_sequence;
  trajectory.plan_sequence = plan_sequence;
  trajectory.purpose = purpose;
  trajectory.duration_sec = local_data.duration_;
  trajectory.source_stamp_sec = rosTimeToSec(local_data.start_time_);

  const int sample_count = static_cast<int>(
      std::ceil(local_data.duration_ / kSampleDtSec)) + 1;

  for (int i = 0; i < sample_count; ++i)
  {
    const double t = std::min(
        local_data.duration_,
        static_cast<double>(i) * kSampleDtSec);

    Eigen::Vector3d pos = pos_traj.evaluateDeBoorT(t);
    Eigen::Vector3d vel = vel_traj.evaluateDeBoorT(t);

    navdog::TimedTrajectoryPoint point{};
    point.time_from_start_sec = t;
    point.x = pos(0);
    point.y = pos(1);
    point.z = pos(2);
    point.vx = vel(0);
    point.vy = vel(1);

    if (i + 1 < sample_count)
    {
      const double t_next = std::min(
          local_data.duration_,
          static_cast<double>(i + 1) * kSampleDtSec);
      Eigen::Vector3d pos_next = pos_traj.evaluateDeBoorT(t_next);
      const double dx = pos_next(0) - pos(0);
      const double dy = pos_next(1) - pos(1);
      if (dx * dx + dy * dy > 1e-6)
      {
        point.yaw = std::atan2(dy, dx);
        point.has_yaw = true;
      }
    }

    trajectory.points.push_back(point);
  }

  trajectory.valid = !trajectory.points.empty();
  return trajectory;
}

// =============================================================================
// getLocalTrajectory
// =============================================================================

navdog::LocalTrajectory ScanLocalPlannerAdapter::getLocalTrajectory(
    navdog::NavigationMode purpose,
    std::uint64_t task_sequence) const
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (last_request_.task_sequence != task_sequence ||
      last_request_.purpose != purpose)
  {
    return navdog::LocalTrajectory{};
  }

  return cached_trajectory_;
}

// =============================================================================
// hasValidTrajectory
// =============================================================================

bool ScanLocalPlannerAdapter::hasValidTrajectory(
    navdog::NavigationMode purpose,
    std::uint64_t task_sequence) const
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (last_request_.task_sequence != task_sequence ||
      last_request_.purpose != purpose)
  {
    return false;
  }

  return cached_trajectory_.valid &&
         cached_trajectory_.duration_sec > kEpsilon;
}

// =============================================================================
// isTrajectoryColliding
// =============================================================================

bool ScanLocalPlannerAdapter::isTrajectoryColliding(
    navdog::NavigationMode purpose,
    std::uint64_t task_sequence) const
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (last_request_.task_sequence != task_sequence ||
      last_request_.purpose != purpose)
  {
    return false;
  }

  return checkTrajectoryCollision(cached_trajectory_);
}

// =============================================================================
// checkTrajectoryCollision
// =============================================================================

bool ScanLocalPlannerAdapter::checkTrajectoryCollision(
    const navdog::LocalTrajectory& trajectory) const
{
  if (!grid_query_ || !grid_query_->ready())
    return false;

  for (const auto& point : trajectory.points)
  {
    const InflatedGridQueryResult result = grid_query_->query(
        point.x,
        point.y,
        point.z,
        point.has_yaw ? point.yaw : 0.0);

    if (result == InflatedGridQueryResult::OCCUPIED ||
        result == InflatedGridQueryResult::OUT_OF_MAP ||
        result == InflatedGridQueryResult::INVALID)
    {
      return true;
    }
  }

  return false;
}

// =============================================================================
// shouldReplan
// =============================================================================

bool ScanLocalPlannerAdapter::shouldReplan(
    const navdog::LocalPlanRequest& request) const
{
  if (last_request_.purpose != request.purpose ||
      last_request_.task_sequence != request.task_sequence ||
      last_request_.plan_sequence != request.plan_sequence)
  {
    return true;
  }

  if (last_request_.task_sequence != request.task_sequence ||
      last_request_.purpose != request.purpose ||
      !cached_trajectory_.valid ||
      cached_trajectory_.duration_sec <= kEpsilon)
  {
    return true;
  }

  const double now = rosTimeToSec(ros::Time::now());

  if (now - last_plan_stamp_sec_ > config_.replan_retry_interval_sec)
    return true;

  const navdog::LocalTrajectory& traj = cached_trajectory_;

  if (traj.valid)
  {
    if (traj.duration_sec - request.start.time_from_start_sec <=
        config_.min_remaining_duration_sec)
    {
      return true;
    }

    if (checkTrajectoryCollision(traj))
      return true;

    const double dx = request.target.x - last_request_.target.x;
    const double dy = request.target.y - last_request_.target.y;
    if (std::hypot(dx, dy) > 0.3)
      return true;
  }

  return false;
}

}  // namespace navdog_scan_adapter
