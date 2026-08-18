#pragma once

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

}  // namespace navdog_runtime
