#pragma once

#include <cmath>

namespace navdog_runtime
{

enum class TurnVoiceEdge
{
  NONE,
  ENTER,
  EXIT
};

struct TurnVoiceLatchResult
{
  bool active{false};
  TurnVoiceEdge edge{TurnVoiceEdge::NONE};
};

inline TurnVoiceLatchResult updateTurnVoiceLatch(
    bool active, bool tracking, double yaw_rate,
    double enter_yaw_rate, double exit_yaw_rate) noexcept
{
  if (!tracking)
    return {false, active ? TurnVoiceEdge::EXIT : TurnVoiceEdge::NONE};

  const double abs_yaw_rate = std::fabs(yaw_rate);
  if (active)
  {
    if (abs_yaw_rate <= exit_yaw_rate)
      return {false, TurnVoiceEdge::EXIT};
    return {true, TurnVoiceEdge::NONE};
  }

  if (abs_yaw_rate >= enter_yaw_rate)
    return {true, TurnVoiceEdge::ENTER};
  return {false, TurnVoiceEdge::NONE};
}

}  // namespace navdog_runtime
