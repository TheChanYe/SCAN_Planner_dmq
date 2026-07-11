#include "navdog_scan_adapter/scan_grid_map_query.hpp"

#include <plan_env/grid_map.h>

#include <cmath>

namespace navdog_scan_adapter
{

ScanGridMapQuery::ScanGridMapQuery(
    const std::shared_ptr<GridMap>& grid_map)
    : grid_map_(grid_map)
{
}

bool ScanGridMapQuery::ready() const noexcept
{
  if (!grid_map_)
    return false;
  return grid_map_->hasInflatedObservation();
}

double ScanGridMapQuery::resolutionM() const noexcept
{
  if (!grid_map_)
    return 0.0;
  return grid_map_->getResolution();
}

double ScanGridMapQuery::mapStampSec() const noexcept
{
  if (!grid_map_)
    return 0.0;
  return grid_map_->getLastOccupancyUpdateStampSec();
}

InflatedGridQueryResult ScanGridMapQuery::query(
    double x,
    double y,
    double z,
    double yaw) const noexcept
{
  if (!grid_map_)
    return InflatedGridQueryResult::INVALID;

  if (!std::isfinite(x) || !std::isfinite(y) ||
      !std::isfinite(z) || !std::isfinite(yaw))
  {
    return InflatedGridQueryResult::INVALID;
  }

  Eigen::Vector3d pos(x, y, z);

  if (!grid_map_->isInMap(pos))
    return InflatedGridQueryResult::OUT_OF_MAP;

  if (grid_map_->getInflateOccupancy(pos, yaw) != 0)
    return InflatedGridQueryResult::OCCUPIED;

  return InflatedGridQueryResult::FREE;
}

}  // namespace navdog_scan_adapter
