#pragma once

#include <cmath>

namespace navdog_runtime
{

enum class ScanRecoveryAction
{
  RESEND_SYNC,
  RESET_REFERENCE,
  ENTER_BACKOFF
};

inline ScanRecoveryAction scanRecoveryActionAfterTimeout(
    int timeout_count, int fast_attempts) noexcept
{
  if (timeout_count <= 1)
    return ScanRecoveryAction::RESEND_SYNC;
  if (timeout_count == fast_attempts)
    return ScanRecoveryAction::RESET_REFERENCE;
  return ScanRecoveryAction::ENTER_BACKOFF;
}

inline bool scanReferenceBuildTimedOut(double sent_sec, double now_sec,
    double timeout_sec) noexcept
{
  return std::isfinite(sent_sec) && sent_sec > 0.0 &&
      std::isfinite(now_sec) && now_sec >= sent_sec &&
      std::isfinite(timeout_sec) && timeout_sec > 0.0 &&
      now_sec - sent_sec >= timeout_sec;
}

}  // namespace navdog_runtime
