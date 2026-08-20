#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

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

inline double proportionalMotionScale(
    double vx,
    double vy,
    double max_linear_speed_mps) noexcept
{
  constexpr double kEpsilon = 1e-9;
  const double speed = std::hypot(vx, vy);
  if (!std::isfinite(speed) ||
      !std::isfinite(max_linear_speed_mps) ||
      max_linear_speed_mps <= 0.0)
  {
    return 0.0;
  }
  if (speed <= max_linear_speed_mps || speed <= kEpsilon)
    return 1.0;
  return max_linear_speed_mps / speed;
}

inline bool takeoverGenerationReady(
    std::uint32_t expected_generation,
    std::uint32_t ready_generation) noexcept
{
  return expected_generation != 0 &&
      ready_generation == expected_generation;
}

}  // namespace navdog_runtime
