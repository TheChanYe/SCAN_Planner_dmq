#pragma once

#include <cstdint>

namespace navdog_scan_adapter
{

enum class InflatedGridQueryResult : std::uint8_t
{
  FREE = 0,
  OCCUPIED,
  OUT_OF_MAP,
  INVALID
};

class InflatedGridQuery3D
{
public:
  virtual ~InflatedGridQuery3D() = default;

  virtual bool ready() const noexcept = 0;

  virtual double resolutionM() const noexcept = 0;

  virtual double mapStampSec() const noexcept = 0;

  virtual InflatedGridQueryResult query(
      double x,
      double y,
      double z,
      double yaw) const noexcept = 0;
};

}  // namespace navdog_scan_adapter
