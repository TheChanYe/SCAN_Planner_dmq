#pragma once

#include "navdog_scan_adapter/inflated_grid_query_3d.hpp"

#include <memory>

class GridMap;

namespace navdog_scan_adapter
{

// ScanGridMapQuery
// 将 SCAN 规划模块自己的 GridMap 适配为 InflatedGridQuery3D 接口，供 navdog_scan_adapter
// 层使用。
class ScanGridMapQuery : public InflatedGridQuery3D
{
public:
  // 构造函数：保存 GridMap 共享指针。
  explicit ScanGridMapQuery(
      const std::shared_ptr<GridMap>& grid_map);

  // 地图是否已有膨胀观测。
  bool ready() const noexcept override;
  // 返回地图分辨率。
  double resolutionM() const noexcept override;
  // 返回地图最后一次占据更新时间戳。
  double mapStampSec() const noexcept override;

  // 查询世界坐标处是否膨胀占据。
  InflatedGridQueryResult query(
      double x,
      double y,
      double z,
      double yaw) const noexcept override;

private:
  std::shared_ptr<GridMap> grid_map_;  // 底层SCAN规划地图实例
};

}  // namespace navdog_scan_adapter
