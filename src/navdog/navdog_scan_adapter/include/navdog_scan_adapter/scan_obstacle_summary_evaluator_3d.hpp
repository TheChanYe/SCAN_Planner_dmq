#pragma once

#include "navdog_scan_adapter/inflated_grid_query_3d.hpp"

#include <navdog_core/types.hpp>

#include <memory>

namespace navdog_scan_adapter
{

class ScanObstacleSummaryEvaluator3D
{
public:
  struct Config
  {
    double front_range_m{2.0};
    double side_range_m{1.0};
    double rear_range_m{1.0};
    double front_half_angle_deg{30.0};
    double side_half_angle_deg{30.0};
    double rear_half_angle_deg{30.0};
    int rays_per_sector{7};
  };

  ScanObstacleSummaryEvaluator3D(
      const Config& config,
      const std::shared_ptr<InflatedGridQuery3D>& grid);

  navdog::ObstacleSummary evaluate(
      const navdog::RobotState& robot,
      double now_sec) const;

private:
  double evaluateSector(
      const navdog::RobotState& robot,
      double center_angle,
      double half_angle,
      double range) const;

  Config config_{};
  std::shared_ptr<InflatedGridQuery3D> grid_{};
};

}  // namespace navdog_scan_adapter
