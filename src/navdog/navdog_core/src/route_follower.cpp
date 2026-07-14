#include "navdog_core/route_follower.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace navdog
{

namespace
{

constexpr double kEpsilon = 1e-9;
constexpr double kPi = 3.14159265358979323846;

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

RouteFollower::RouteFollower(
    const RouteFollowerConfig& config)
    : config_(config)
{
}

// =============================================================================
// interpolateRoutePoint
// =============================================================================

bool RouteFollower::interpolateRoutePoint(
    const NavigationTask& task,
    double target_arc_length_m,
    double& out_x,
    double& out_y,
    double& out_yaw) const noexcept
{
  out_x = 0.0;
  out_y = 0.0;
  out_yaw = 0.0;

  const auto& points = task.points;
  if (points.size() < 2)
    return false;

  if (target_arc_length_m <= 0.0)
  {
    out_x = points.front().x;
    out_y = points.front().y;
    out_yaw = points.front().has_yaw
        ? points.front().yaw
        : 0.0;
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
      out_x = points[i - 1].x + ratio * dx;
      out_y = points[i - 1].y + ratio * dy;

      out_yaw = std::atan2(dy, dx);
      if (points[i - 1].has_yaw && points[i].has_yaw)
      {
        double yaw_diff = normalizeAngle(
            points[i].yaw - points[i - 1].yaw);
        out_yaw = normalizeAngle(
            points[i - 1].yaw + ratio * yaw_diff);
      }

      return true;
    }

    accumulated += seg_len;
  }

  // target_arc_length_m beyond route end.
  out_x = points.back().x;
  out_y = points.back().y;
  out_yaw = points.back().has_yaw
      ? points.back().yaw
      : std::atan2(
            points.back().y - points[points.size() - 2].y,
            points.back().x - points[points.size() - 2].x);
  return true;
}

// =============================================================================
// isYawAligned
// =============================================================================

bool RouteFollower::isYawAligned(
    double heading_error) const noexcept
{
  return std::abs(heading_error) <=
         config_.heading_turn_only_threshold_rad;
}

VelocityCommand RouteFollower::updatePointGoal(
    const NavigationTask& task,
    const RobotState& robot,
    const RouteProgress& progress,
    double max_vx,
    double now_sec) const
{
  VelocityCommand cmd{};
  cmd.stamp_sec = now_sec;
  if (task.points.empty() || !robot.valid || !progress.valid)
  {
    cmd.source = CommandSource::TRACKING_STOP;
    return cmd;
  }

  const RoutePoint& target = task.points.back();
  const double dx = target.x - robot.x;
  const double dy = target.y - robot.y;
  const double distance = std::hypot(dx, dy);
  const double desired_yaw = distance > kEpsilon
      ? std::atan2(dy, dx)
      : robot.yaw;
  const double heading_error = normalizeAngle(desired_yaw - robot.yaw);
  const double effective_max_vx =
      std::max(0.0, std::min(max_vx, config_.max_vx));

  if (!isYawAligned(heading_error))
  {
    cmd.yaw_rate = std::max(
        -config_.kp_yaw,
        std::min(config_.kp_yaw, config_.kp_yaw * heading_error));
  }
  else
  {
    double target_speed = std::min(
        effective_max_vx,
        std::max(0.10, config_.kp_x * distance));
    if (distance < 0.30)
      target_speed = std::min(effective_max_vx, config_.kp_x * distance);

    const double c = std::cos(robot.yaw);
    const double s = std::sin(robot.yaw);
    const double lateral_error = -s * dx + c * dy;
    cmd.vx = target_speed;
    cmd.vy = config_.kp_y * lateral_error;
    cmd.yaw_rate = config_.kp_yaw * heading_error;
  }

  if (!std::isfinite(cmd.vx)) cmd.vx = 0.0;
  if (!std::isfinite(cmd.vy)) cmd.vy = 0.0;
  if (!std::isfinite(cmd.yaw_rate)) cmd.yaw_rate = 0.0;
  cmd.vx = std::max(0.0, std::min(cmd.vx, effective_max_vx));
  cmd.valid = true;
  cmd.source = CommandSource::PLANNER;
  return cmd;
}

// =============================================================================
// update
// =============================================================================

VelocityCommand RouteFollower::update(
    const NavigationTask& task,
    const RobotState& robot,
    const RouteProgress& progress,
    double max_vx,
    double now_sec)
{
  VelocityCommand cmd{};

  if (task.points.empty() ||
      !progress.valid ||
      !std::isfinite(progress.arc_length_m) ||
      !std::isfinite(progress.total_length_m) ||
      progress.arc_length_m < 0.0 ||
      !robot.valid)
  {
    cmd.valid = false;
    cmd.source = CommandSource::TRACKING_STOP;
    cmd.stamp_sec = now_sec;
    return cmd;
  }

  if (task.points.size() == 1 || progress.total_length_m <= 1e-6)
    return updatePointGoal(task, robot, progress, max_vx, now_sec);

  const double effective_max_vx =
      std::max(0.0, std::min(max_vx, config_.max_vx));

  const double target_arc =
      progress.arc_length_m + config_.lookahead_distance_m;

  double look_x = 0.0;
  double look_y = 0.0;
  double look_yaw = 0.0;

  if (!interpolateRoutePoint(
          task, target_arc, look_x, look_y, look_yaw))
  {
    cmd.valid = false;
    cmd.source = CommandSource::TRACKING_STOP;
    cmd.stamp_sec = now_sec;
    return cmd;
  }

  const double ex_world = look_x - robot.x;
  const double ey_world = look_y - robot.y;

  const double heading_error =
      normalizeAngle(look_yaw - robot.yaw);

  const bool aligned = isYawAligned(heading_error);

  const double c = std::cos(robot.yaw);
  const double s = std::sin(robot.yaw);

  // World error rotated into robot frame.
  const double ex_robot = c * ex_world + s * ey_world;
  const double ey_robot = -s * ex_world + c * ey_world;

  if (!aligned)
  {
    cmd.vx = 0.0;
    cmd.vy = 0.0;
    cmd.yaw_rate =
        std::max(-config_.kp_yaw,
            std::min(config_.kp_yaw,
                config_.kp_yaw * heading_error));
  }
  else
  {
    const double route_vx = effective_max_vx;
    cmd.vx = route_vx + config_.kp_x * ex_robot;
    cmd.vy = config_.kp_y * ey_robot;
    cmd.yaw_rate = config_.kp_yaw * heading_error;
  }

  // Clamp and sanitize.
  if (!std::isfinite(cmd.vx) || cmd.vx < 0.0)
    cmd.vx = 0.0;
  cmd.vx = std::min(cmd.vx, effective_max_vx);

  if (!std::isfinite(cmd.vy))
    cmd.vy = 0.0;

  if (!std::isfinite(cmd.yaw_rate))
    cmd.yaw_rate = 0.0;

  cmd.stamp_sec = now_sec;
  cmd.valid = true;
  cmd.source = CommandSource::PLANNER;

  return cmd;
}

}  // namespace navdog
