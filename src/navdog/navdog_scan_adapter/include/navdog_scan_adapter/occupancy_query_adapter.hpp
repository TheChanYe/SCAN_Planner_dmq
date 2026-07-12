#pragma once

#include "navdog_scan_adapter/inflated_grid_query_3d.hpp"

#include <navdog_core/types.hpp>

#include <memory>

namespace navdog_scan_adapter
{

// =============================================================================
// OccupancyQueryAdapter
//
// 将 InflatedGridQuery3D 适配为 navdog_core::OccupancyQuery3D。
// =============================================================================

class OccupancyQueryAdapter : public navdog::OccupancyQuery3D
{
public:
  explicit OccupancyQueryAdapter(
      const std::shared_ptr<InflatedGridQuery3D>& grid);

  bool ready() const noexcept override;

  bool isFree(
      double x,
      double y,
      double z,
      double yaw) const noexcept override;

private:
  std::shared_ptr<InflatedGridQuery3D> grid_;
};

}  // namespace navdog_scan_adapter
