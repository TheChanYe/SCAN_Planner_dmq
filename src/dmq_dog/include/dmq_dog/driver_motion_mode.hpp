#pragma once

#include <cstdint>

namespace dmq_dog
{

constexpr std::uint8_t kNavigationModeLocalAvoid = 2;

inline bool forwardMotionAdapterEnabled(
    bool prefer_forward_motion, std::uint8_t navigation_mode) noexcept
{
  return prefer_forward_motion &&
      navigation_mode != kNavigationModeLocalAvoid;
}

}  // namespace dmq_dog
