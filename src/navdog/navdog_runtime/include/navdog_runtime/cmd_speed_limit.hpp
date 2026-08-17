#pragma once

#include <algorithm>
#include <cmath>

namespace navdog_runtime
{

inline double effectiveLinearSpeedLimit(
    double mode_limit_mps,
    double task_max_vx_mps,
    bool task_max_vx_valid) noexcept
{
  if (!std::isfinite(mode_limit_mps) || mode_limit_mps <= 0.0)
    return 0.0;
  if (!task_max_vx_valid || !std::isfinite(task_max_vx_mps) ||
      task_max_vx_mps <= 0.0)
    return mode_limit_mps;
  return std::min(mode_limit_mps, task_max_vx_mps);
}

}  // namespace navdog_runtime
