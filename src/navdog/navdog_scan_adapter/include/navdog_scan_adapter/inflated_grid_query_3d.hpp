#pragma once

#include <cstdint>

namespace navdog_scan_adapter
{

// InflatedGridQueryResult
// 膨胀占据查询结果枚举：FREE为空闲；OCCUPIED为已被占据；OUT_OF_MAP为超出地图
// 范围；INVALID为输入非法或地图不可用。
enum class InflatedGridQueryResult : std::uint8_t
{
  FREE = 0,
  OCCUPIED,
  OUT_OF_MAP,
  INVALID
};

// InflatedGridQuery3D
// 膨胀占据地图的纯虚拟查询接口，不依赖具体地图实现（GridMap等），
// 供 navdog_scan_adapter 层的具体实现（如ScanGridMapQuery）继承。
class InflatedGridQuery3D
{
public:
  virtual ~InflatedGridQuery3D() = default;

  // 地图是否已就绪（已有膨胀观测），未就绪时不应依赖查询结果。
  virtual bool ready() const noexcept = 0;

  // 返回地图分辨率（米/格）。
  virtual double resolutionM() const noexcept = 0;

  // 返回地图最后一次占据更新的时间戳（秒），用于新鲜度/超时判断。
  virtual double mapStampSec() const noexcept = 0;

  // 查询世界坐标(x,y,z,yaw)处是否膨胀占据，返回对应结果枚举。
  virtual InflatedGridQueryResult query(
      double x,
      double y,
      double z,
      double yaw) const noexcept = 0;
};

}  // namespace navdog_scan_adapter
