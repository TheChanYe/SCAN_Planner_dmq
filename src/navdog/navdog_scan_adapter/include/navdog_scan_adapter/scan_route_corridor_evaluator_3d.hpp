#pragma once

#include "navdog_scan_adapter/inflated_grid_query_3d.hpp"

#include <navdog_core/config.hpp>
#include <navdog_core/types.hpp>

#include <memory>

namespace navdog_scan_adapter
{

class ScanRouteCorridorEvaluator3D
{
public:
  ScanRouteCorridorEvaluator3D(
      const navdog::RouteCorridorConfig& config,
      const std::shared_ptr<InflatedGridQuery3D>& grid);

  navdog::RouteCorridorAssessment evaluate(
      const navdog::NavigationTask& task,
      const navdog::RouteProgress& progress,
      const navdog::RobotState& robot,
      double now_sec) const;

private:
  navdog::RouteCorridorConfig config_;
  std::shared_ptr<InflatedGridQuery3D> grid_;
};

}  // namespace navdog_scan_adapter
