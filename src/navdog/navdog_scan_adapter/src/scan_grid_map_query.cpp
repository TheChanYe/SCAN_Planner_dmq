#include "navdog_scan_adapter/scan_grid_map_query.hpp"

#include <plan_env/grid_map.h>

#include <cmath>

namespace navdog_scan_adapter
{

// 构造函数：保存GridMap共享指针。
ScanGridMapQuery::ScanGridMapQuery(
    const std::shared_ptr<GridMap>& grid_map)
    : grid_map_(grid_map)
{
}

// ready：地图为空时返回false；否则转发GridMap::hasInflatedObservation()（是否已
// 有足够膨胀观测）。
bool ScanGridMapQuery::ready() const noexcept
{
  if (!grid_map_)
    return false;
  return grid_map_->hasInflatedObservation();
}

// resolutionM：地图为空返回0；否则返回地图分辨率。
double ScanGridMapQuery::resolutionM() const noexcept
{
  if (!grid_map_)
    return 0.0;
  return grid_map_->getResolution();
}

// mapStampSec：地图为空返回0；否则返回最后一次占据更新时间戳。
double ScanGridMapQuery::mapStampSec() const noexcept
{
  if (!grid_map_)
    return 0.0;
  return grid_map_->getLastOccupancyUpdateStampSec();
}

// query：查询世界坐标(x,y,z,yaw)处是否膨胀占据。
// 步骤：1.地图为空返回INVALID；2.任一坐标非有限数返回INVALID；
// 3.坐标不在地图范围内返回OUT_OF_MAP；4.调用GridMap::getInflateOccupancy
// （带朝向yaw的膨胀查询）非0则为OCCUPIED，否则为FREE。
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
