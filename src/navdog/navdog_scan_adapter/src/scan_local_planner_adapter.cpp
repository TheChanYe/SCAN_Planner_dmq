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
constexpr double kCollisionLookBehindSec = 0.05;

double rosTimeToSec(const ros::Time& t)
{
  return static_cast<double>(t.sec) +
         static_cast<double>(t.nsec) * 1e-9;
}

bool finitePoint(const navdog::RoutePoint& point, bool require_z)
{
  if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
      (require_z && !std::isfinite(point.z)))
  {
    return false;
  }
  return !point.has_yaw || std::isfinite(point.yaw);
}

bool validRequest(const navdog::LocalPlanRequest& request)
{
  const bool valid_purpose =
      request.purpose == navdog::NavigationMode::LOCAL_AVOID ||
      request.purpose == navdog::NavigationMode::ROUTE_REJOIN;
  return request.valid && valid_purpose &&
      request.task_sequence != 0 && request.plan_sequence != 0 &&
      finitePoint(request.start, true) &&
      finitePoint(request.start_vel, false) &&
      finitePoint(request.target, true) &&
      finitePoint(request.target_vel, false) &&
      std::isfinite(request.robot_z) &&
      std::isfinite(request.max_vx) && request.max_vx > 0.0;
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
  worker_thread_ = std::thread(
      &ScanLocalPlannerAdapter::planningLoop, this);
}

// =============================================================================
// Destructor
// =============================================================================

ScanLocalPlannerAdapter::~ScanLocalPlannerAdapter()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    shutdown_ = true;
    has_pending_request_ = false;
  }
  cv_.notify_all();

  if (worker_thread_.joinable())
  {
    worker_thread_.join();
  }
}

// =============================================================================
// requestLocalPlan
// =============================================================================

bool ScanLocalPlannerAdapter::requestLocalPlan(
    const navdog::LocalPlanRequest& request)
{
  if (!validRequest(request) || !planner_manager_)
  {
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);

    // Newest-wins: overwrite any pending request.
    pending_request_ = request;
    has_pending_request_ = true;
  }

  cv_.notify_one();
  return true;
}

// =============================================================================
// planningLoop
// =============================================================================

void ScanLocalPlannerAdapter::planningLoop()
{
  while (true)
  {
    navdog::LocalPlanRequest request{};
    bool has_request = false;

    {
      std::unique_lock<std::mutex> lock(mutex_);
      cv_.wait(lock, [this]() {
        return shutdown_ || has_pending_request_;
      });

      if (shutdown_)
        return;

      if (has_pending_request_)
      {
        request = pending_request_;
        has_request = true;
        has_pending_request_ = false;
        active_request_ = request;
        has_active_request_ = true;
      }
    }

    if (!has_request)
      continue;

    const bool ok = doReboundReplan(request);

    {
      std::lock_guard<std::mutex> lock(mutex_);

      if (shutdown_)
        return;

      has_active_request_ = false;
      completed_request_ = request;

      if (ok)
      {
        cached_trajectory_ = sampleLocalTrajData(
            request.task_sequence,
            request.plan_sequence,
            request.purpose);
        completed_state_ = cached_trajectory_.valid
                     ? navdog::LocalPlanState::READY
                     : navdog::LocalPlanState::FAILED;
      }
      else
      {
        cached_trajectory_ = navdog::LocalTrajectory{};
        completed_state_ = navdog::LocalPlanState::FAILED;
      }
    }
  }
}

// =============================================================================
// doReboundReplan
// =============================================================================

bool ScanLocalPlannerAdapter::doReboundReplan(
    const navdog::LocalPlanRequest& request)
{
  if (!planner_manager_)
    return false;

  Eigen::Vector3d start_pt(
      request.start.x, request.start.y, request.robot_z);
  Eigen::Vector3d start_vel(
      request.start_vel.x, request.start_vel.y, 0.0);
  Eigen::Vector3d start_acc(0.0, 0.0, 0.0);

  Eigen::Vector3d end_pt(
      request.target.x, request.target.y, request.robot_z);
  Eigen::Vector3d end_vel(
      request.target_vel.x, request.target_vel.y, 0.0);

  const bool flag_poly_init = true;
  const bool flag_random_poly_traj = false;

  return planner_manager_->reboundReplan(
      start_pt,
      start_vel,
      start_acc,
      end_pt,
      end_vel,
      flag_poly_init,
      flag_random_poly_traj);
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

  if (!std::isfinite(local_data.duration_) ||
      local_data.duration_ <= kEpsilon)
    return trajectory;

  const scan_planner::UniformBspline& pos_traj =
      local_data.position_traj_;
  const scan_planner::UniformBspline& vel_traj =
      local_data.velocity_traj_;

  trajectory.task_sequence = task_sequence;
  trajectory.plan_sequence = plan_sequence;
  trajectory.purpose = purpose;
  trajectory.duration_sec = local_data.duration_;
  trajectory.source_stamp_sec = rosTimeToSec(ros::Time::now());

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

  if (!isSampledTrajectoryValid(trajectory))
    return navdog::LocalTrajectory{};

  trajectory.valid = true;
  return trajectory;
}

bool ScanLocalPlannerAdapter::isSampledTrajectoryValid(
    const navdog::LocalTrajectory& trajectory) noexcept
{
  if (!std::isfinite(trajectory.duration_sec) ||
      trajectory.duration_sec <= 0.0 ||
      trajectory.points.size() < 2)
  {
    return false;
  }

  double previous_time = -1.0;
  for (const auto& point : trajectory.points)
  {
    if (!std::isfinite(point.time_from_start_sec) ||
        point.time_from_start_sec < 0.0 ||
        point.time_from_start_sec + kEpsilon < previous_time ||
        !std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z) || !std::isfinite(point.vx) ||
        !std::isfinite(point.vy) ||
        (point.has_yaw && !std::isfinite(point.yaw)))
    {
      return false;
    }
    previous_time = point.time_from_start_sec;
  }

  return std::abs(trajectory.points.back().time_from_start_sec -
                  trajectory.duration_sec) <= kEpsilon;
}

// =============================================================================
// getLocalTrajectory
// =============================================================================

navdog::LocalTrajectory ScanLocalPlannerAdapter::getLocalTrajectory(
    navdog::NavigationMode purpose,
    std::uint64_t task_sequence) const
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (completed_request_.task_sequence != task_sequence ||
      completed_request_.purpose != purpose)
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

  if (completed_request_.task_sequence != task_sequence ||
      completed_request_.purpose != purpose)
  {
    return false;
  }

  return cached_trajectory_.valid &&
         cached_trajectory_.duration_sec > kEpsilon;
}

// =============================================================================
// localPlanState
// =============================================================================

navdog::LocalPlanState ScanLocalPlannerAdapter::localPlanState(
    navdog::NavigationMode purpose,
    std::uint64_t task_sequence,
    std::uint64_t plan_sequence) const
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (has_pending_request_ &&
      pending_request_.purpose == purpose &&
      pending_request_.task_sequence == task_sequence &&
      pending_request_.plan_sequence == plan_sequence)
  {
    return navdog::LocalPlanState::QUEUED;
  }

  if (has_active_request_ &&
      active_request_.purpose == purpose &&
      active_request_.task_sequence == task_sequence &&
      active_request_.plan_sequence == plan_sequence)
  {
    return navdog::LocalPlanState::PLANNING;
  }

  if (completed_request_.purpose != purpose ||
      completed_request_.task_sequence != task_sequence ||
      completed_request_.plan_sequence != plan_sequence)
  {
    return navdog::LocalPlanState::IDLE;
  }

  return completed_state_;
}

// =============================================================================
// isTrajectoryColliding
// =============================================================================

bool ScanLocalPlannerAdapter::isTrajectoryColliding(
    navdog::NavigationMode purpose,
    std::uint64_t task_sequence,
    std::uint64_t plan_sequence,
    double from_time_sec) const
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (completed_request_.task_sequence != task_sequence ||
      completed_request_.purpose != purpose ||
      completed_request_.plan_sequence != plan_sequence)
  {
    return true;
  }

  return checkTrajectoryCollision(
      cached_trajectory_, from_time_sec);
}

// =============================================================================
// checkTrajectoryCollision
// =============================================================================

bool ScanLocalPlannerAdapter::checkTrajectoryCollision(
    const navdog::LocalTrajectory& trajectory,
    double from_time_sec) const
{
  if (!grid_query_ || !grid_query_->ready())
    return true;

  const double t_start =
      std::max(0.0, from_time_sec - kCollisionLookBehindSec);

  for (const auto& point : trajectory.points)
  {
    if (point.time_from_start_sec < t_start)
      continue;

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

}  // namespace navdog_scan_adapter
