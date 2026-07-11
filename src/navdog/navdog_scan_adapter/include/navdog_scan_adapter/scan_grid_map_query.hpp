#pragma once

#include "navdog_scan_adapter/inflated_grid_query_3d.hpp"

#include <plan_env/grid_map.h>

#include <memory>

namespace navdog_scan_adapter
{

class ScanGridMapQuery : public InflatedGridQuery3D
{
public:
  explicit ScanGridMapQuery(const GridMap::Ptr& grid_map);

  bool ready() const noexcept override;
  double resolutionM() const noexcept override;
  double mapStampSec() const noexcept override;

  InflatedGridQueryResult query(
      double x,
      double y,
      double z,
      double yaw) const noexcept override;

private:
  GridMap::Ptr grid_map_;
};

}  // namespace navdog_scan_adapter
