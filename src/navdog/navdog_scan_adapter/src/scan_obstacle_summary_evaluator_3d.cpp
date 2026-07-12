#include "navdog_scan_adapter/scan_obstacle_summary_evaluator_3d.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace navdog_scan_adapter
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
double radians(double degrees) { return degrees * kPi / 180.0; }
}

ScanObstacleSummaryEvaluator3D::ScanObstacleSummaryEvaluator3D(
    const Config& config,
    const std::shared_ptr<InflatedGridQuery3D>& grid)
    : config_(config), grid_(grid)
{
}

double ScanObstacleSummaryEvaluator3D::evaluateSector(
    const navdog::RobotState& robot,
    double center_angle,
    double half_angle,
    double range) const
{
  const int ray_count = std::max(1, config_.rays_per_sector);
  const double step = std::max(grid_->resolutionM() * 0.5, 0.025);
  double nearest = std::numeric_limits<double>::infinity();
  for (int ray = 0; ray < ray_count; ++ray)
  {
    const double ratio = ray_count == 1
        ? 0.5 : static_cast<double>(ray) / (ray_count - 1);
    const double angle = robot.yaw + center_angle - half_angle +
        2.0 * half_angle * ratio;
    for (double distance = step; distance <= range + 1e-9;
         distance += step)
    {
      const auto result = grid_->query(
          robot.x + distance * std::cos(angle),
          robot.y + distance * std::sin(angle),
          robot.z, angle);
      if (result != InflatedGridQueryResult::FREE)
      {
        nearest = std::min(nearest, distance);
        break;
      }
    }
  }
  return nearest;
}

navdog::ObstacleSummary ScanObstacleSummaryEvaluator3D::evaluate(
    const navdog::RobotState& robot,
    double now_sec) const
{
  navdog::ObstacleSummary result{};
  if (!grid_ || !grid_->ready() || !robot.valid ||
      !std::isfinite(robot.x) || !std::isfinite(robot.y) ||
      !std::isfinite(robot.z) || !std::isfinite(robot.yaw) ||
      !std::isfinite(now_sec))
  {
    return result;
  }

  result.front_min = evaluateSector(robot, 0.0,
      radians(config_.front_half_angle_deg), config_.front_range_m);
  result.left_min = evaluateSector(robot, kPi * 0.5,
      radians(config_.side_half_angle_deg), config_.side_range_m);
  result.right_min = evaluateSector(robot, -kPi * 0.5,
      radians(config_.side_half_angle_deg), config_.side_range_m);
  result.rear_min = evaluateSector(robot, kPi,
      radians(config_.rear_half_angle_deg), config_.rear_range_m);
  result.stamp_sec = now_sec;
  result.valid = true;
  return result;
}

}  // namespace navdog_scan_adapter
