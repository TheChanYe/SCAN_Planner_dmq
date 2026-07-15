#include <gtest/gtest.h>
#include "navdog_core/recovery_controller.hpp"

TEST(RecoveryController, DefaultsToNoInventedRecoveryMotion)
{
  navdog::RecoveryController controller(navdog::NavdogConfig{});
  const auto output = controller.update({}, {}, nullptr, 1.0);
  EXPECT_FALSE(output.active);
  EXPECT_EQ(navdog::RecoveryMode::NONE, output.mode);
}
