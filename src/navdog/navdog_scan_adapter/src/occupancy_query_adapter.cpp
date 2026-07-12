#include "navdog_scan_adapter/occupancy_query_adapter.hpp"

namespace navdog_scan_adapter
{

OccupancyQueryAdapter::OccupancyQueryAdapter(
    const std::shared_ptr<InflatedGridQuery3D>& grid)
    : grid_(grid)
{
}

bool OccupancyQueryAdapter::ready() const noexcept
{
  return grid_ && grid_->ready();
}

bool OccupancyQueryAdapter::isFree(
    double x,
    double y,
    double z,
    double yaw) const noexcept
{
  if (!grid_)
    return false;

  const InflatedGridQueryResult result =
      grid_->query(x, y, z, yaw);

  return result == InflatedGridQueryResult::FREE;
}

}  // namespace navdog_scan_adapter
