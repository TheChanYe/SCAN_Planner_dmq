#include "navdog_runtime/scan_recovery_policy.hpp"

#include <gtest/gtest.h>

namespace navdog_runtime
{

TEST(ScanRecoveryPolicyTest, FirstTimeoutOnlyResendsSync)
{
  EXPECT_EQ(scanRecoveryActionAfterTimeout(1, 2),
      ScanRecoveryAction::RESEND_SYNC);
}

TEST(ScanRecoveryPolicyTest, SecondTimeoutRebuildsReference)
{
  EXPECT_EQ(scanRecoveryActionAfterTimeout(2, 2),
      ScanRecoveryAction::RESET_REFERENCE);
}

TEST(ScanRecoveryPolicyTest, LaterTimeoutsRemainRetryableInBackoff)
{
  EXPECT_EQ(scanRecoveryActionAfterTimeout(3, 2),
      ScanRecoveryAction::ENTER_BACKOFF);
  EXPECT_EQ(scanRecoveryActionAfterTimeout(10, 2),
      ScanRecoveryAction::ENTER_BACKOFF);
}

}  // namespace navdog_runtime
