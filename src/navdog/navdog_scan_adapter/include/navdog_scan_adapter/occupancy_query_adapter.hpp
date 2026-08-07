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
  // 构造函数：持有底层膨胀地图查询接口的共享指针。
  explicit OccupancyQueryAdapter(
      const std::shared_ptr<InflatedGridQuery3D>& grid);

  // 地图是否已就绪可用。
  bool ready() const noexcept override;

  // 查询世界坐标(x,y,z,yaw)处是否为空闲（仅FREE结果视为空闲，
  // 其余均视为不可用）。
  bool isFree(
      double x,
      double y,
      double z,
      double yaw) const noexcept override;

private:
  std::shared_ptr<InflatedGridQuery3D> grid_;  // 底层膨胀地图查询实例
};

}  // namespace navdog_scan_adapter
