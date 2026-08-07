#include "navdog_scan_adapter/occupancy_query_adapter.hpp"

namespace navdog_scan_adapter
{

// 构造函数：保存底层膨胀地图查询接口。
OccupancyQueryAdapter::OccupancyQueryAdapter(
    const std::shared_ptr<InflatedGridQuery3D>& grid)
    : grid_(grid)
{
}

// ready：底层指针非空且地图已就绪才返回true。
bool OccupancyQueryAdapter::ready() const noexcept
{
  return grid_ && grid_->ready();
}

// isFree：底层指针为空直接返回false；否则调用底层query得到结果枚举，
// 仅当结果为FREE时才认为该位置空闲（OCCUPIED/OUT_OF_MAP/INVALID均视为非空闲）。
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
