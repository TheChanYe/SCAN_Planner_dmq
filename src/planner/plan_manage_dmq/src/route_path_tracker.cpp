#include "plan_manage_dmq/route_path_tracker.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace scan_planner_dmq
{
namespace
{

constexpr double kEpsilon = 1e-9;
constexpr double kDuplicatePointDistanceM = 0.01;
constexpr double kPi = 3.14159265358979323846;

double normalizeAngle(double angle) noexcept
{
  while (angle > kPi) angle -= 2.0 * kPi;
  while (angle < -kPi) angle += 2.0 * kPi;
  return angle;
}

double clamp(double value, double low, double high) noexcept
{
  return std::max(low, std::min(high, value));
}

bool finitePoint(const Eigen::Vector3d& point) noexcept
{
  return std::isfinite(point.x()) && std::isfinite(point.y()) &&
         std::isfinite(point.z());
}

}  // namespace

RoutePathTracker::RoutePathTracker(const RoutePathTrackerConfig& config)
    : config_(config)
{
}

bool RoutePathTracker::setPath(
    const std::vector<Eigen::Vector3d>& points)
{
  reset();
  points_.reserve(points.size());
  for (const auto& point : points)
  {
    if (!finitePoint(point)) continue;
    if (!points_.empty() &&
        (point - points_.back()).head<2>().norm() <
            kDuplicatePointDistanceM)
      continue;
    points_.push_back(point);
  }

  if (points_.size() < 2)
  {
    reset();
    return false;
  }

  cumulative_length_.resize(points_.size(), 0.0);
  for (std::size_t i = 1; i < points_.size(); ++i)
  {
    const double segment_length =
        (points_[i] - points_[i - 1]).head<2>().norm();
    cumulative_length_[i] = cumulative_length_[i - 1] + segment_length;
  }

  if (cumulative_length_.back() <= kEpsilon)
  {
    reset();
    return false;
  }

  reacquire_requested_ = true;
  return true;
}

void RoutePathTracker::reset() noexcept
{
  points_.clear();
  cumulative_length_.clear();
  segment_index_ = 0;
  progress_m_ = 0.0;
  reacquire_requested_ = true;
}

void RoutePathTracker::requestReacquire() noexcept
{
  reacquire_requested_ = true;
}

bool RoutePathTracker::projectProgress(
    const Eigen::Vector3d& robot_position,
    double& projected_arc,
    std::size_t& projected_segment) const noexcept
{
  if (points_.size() < 2 || !finitePoint(robot_position)) return false;

  double best_distance_sq = std::numeric_limits<double>::infinity();
  double best_arc = progress_m_;
  std::size_t best_segment = segment_index_;
  const std::size_t first_segment = reacquire_requested_ ? 0 : segment_index_;
  const double search_end = progress_m_ +
      std::max(0.0, config_.max_forward_search_m);

  for (std::size_t i = first_segment; i + 1 < points_.size(); ++i)
  {
    if (!reacquire_requested_ && cumulative_length_[i] > search_end)
      break;

    const Eigen::Vector2d start = points_[i].head<2>();
    const Eigen::Vector2d delta =
        points_[i + 1].head<2>() - start;
    const double length_sq = delta.squaredNorm();
    if (length_sq <= kEpsilon) continue;

    const double ratio = clamp(
        (robot_position.head<2>() - start).dot(delta) / length_sq,
        0.0, 1.0);
    const Eigen::Vector2d projection = start + ratio * delta;
    const double distance_sq =
        (robot_position.head<2>() - projection).squaredNorm();
    const double arc = cumulative_length_[i] +
        ratio * std::sqrt(length_sq);
    if (arc + 0.05 < progress_m_)
      continue;

    if (distance_sq < best_distance_sq)
    {
      best_distance_sq = distance_sq;
      best_arc = arc;
      best_segment = i;
    }
  }

  if (!std::isfinite(best_distance_sq)) return false;
  projected_arc = std::max(progress_m_, best_arc);
  projected_segment = best_segment;
  return true;
}

bool RoutePathTracker::sampleAtArc(
    double arc,
    Eigen::Vector3d& point,
    std::size_t* segment) const noexcept
{
  if (points_.size() < 2 || cumulative_length_.empty()) return false;

  const double bounded_arc = clamp(arc, 0.0, cumulative_length_.back());
  auto upper = std::upper_bound(
      cumulative_length_.begin(), cumulative_length_.end(), bounded_arc);
  std::size_t i = upper == cumulative_length_.begin()
      ? 0
      : static_cast<std::size_t>(upper - cumulative_length_.begin() - 1);
  i = std::min(i, points_.size() - 2);

  const double segment_length =
      cumulative_length_[i + 1] - cumulative_length_[i];
  const double ratio = segment_length > kEpsilon
      ? (bounded_arc - cumulative_length_[i]) / segment_length
      : 0.0;
  point = points_[i] + ratio * (points_[i + 1] - points_[i]);
  if (segment) *segment = i;
  return finitePoint(point);
}

RoutePathTrackerOutput RoutePathTracker::update(
    const Eigen::Vector3d& robot_position,
    double robot_yaw) noexcept
{
  RoutePathTrackerOutput output{};
  if (!std::isfinite(robot_yaw) || points_.size() < 2) return output;

  double projected_arc = 0.0;
  std::size_t projected_segment = 0;
  if (!projectProgress(robot_position, projected_arc, projected_segment))
    return output;

  progress_m_ = projected_arc;
  segment_index_ = projected_segment;
  reacquire_requested_ = false;

  const double total_length = cumulative_length_.back();
  const double remaining = std::max(0.0, total_length - progress_m_);
  const double target_arc = std::min(
      total_length,
      progress_m_ + std::max(0.0, config_.lookahead_distance_m));

  Eigen::Vector3d target;
  Eigen::Vector3d heading_start;
  Eigen::Vector3d heading_end;
  if (!sampleAtArc(target_arc, target) ||
      !sampleAtArc(std::max(progress_m_, target_arc -
          0.5 * std::max(0.0, config_.heading_lookahead_m)),
          heading_start) ||
      !sampleAtArc(std::min(total_length, target_arc +
          0.5 * std::max(0.0, config_.heading_lookahead_m)),
          heading_end))
    return output;

  Eigen::Vector2d tangent =
      heading_end.head<2>() - heading_start.head<2>();
  if (tangent.norm() <= kEpsilon)
  {
    tangent = points_[segment_index_ + 1].head<2>() -
        points_[segment_index_].head<2>();
  }
  if (tangent.norm() <= kEpsilon) return output;
  tangent.normalize();

  const double max_vx = std::max(0.0, config_.max_vx);
  Eigen::Vector2d velocity_world = max_vx * tangent +
      std::max(0.0, config_.kp_position) *
          (target.head<2>() - robot_position.head<2>());
  const double velocity_norm = velocity_world.norm();
  if (velocity_norm > max_vx && velocity_norm > kEpsilon)
    velocity_world *= max_vx / velocity_norm;

  const double desired_yaw = std::atan2(
      velocity_world.y(), velocity_world.x());
  const double yaw_error = normalizeAngle(desired_yaw - robot_yaw);
  const double heading_scale = std::max(0.0, std::cos(yaw_error));
  double commanded_vx = velocity_world.norm() * heading_scale;

  const double slowdown_distance =
      std::max(0.0, config_.near_goal_slowdown_m);
  if (slowdown_distance > kEpsilon && remaining < slowdown_distance)
  {
    commanded_vx *= clamp(remaining / slowdown_distance, 0.0, 1.0);
  }

  output.vx = clamp(commanded_vx, 0.0, max_vx);
  output.vy = 0.0;
  output.yaw_rate = clamp(
      std::max(0.0, config_.kp_yaw) * yaw_error,
      -std::max(0.0, config_.max_yaw_rate),
      std::max(0.0, config_.max_yaw_rate));
  output.progress_m = progress_m_;
  output.remaining_m = remaining;
  output.target_x = target.x();
  output.target_y = target.y();
  output.valid = std::isfinite(output.vx) &&
      std::isfinite(output.yaw_rate);
  return output;
}

}  // namespace scan_planner_dmq
