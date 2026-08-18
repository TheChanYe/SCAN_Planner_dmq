#include "navdog_runtime/turn_voice_latch.hpp"

#include <gtest/gtest.h>

namespace navdog_runtime
{

TEST(TurnVoiceLatchTest, EntersImmediatelyWithoutLinearSpeedGate)
{
  const auto result = updateTurnVoiceLatch(false, true, 0.30, 0.25, 0.15);
  EXPECT_TRUE(result.active);
  EXPECT_EQ(result.edge, TurnVoiceEdge::ENTER);
}

TEST(TurnVoiceLatchTest, SustainedTurnDoesNotRepeatAndExitRearms)
{
  auto result = updateTurnVoiceLatch(true, true, 0.20, 0.25, 0.15);
  EXPECT_TRUE(result.active);
  EXPECT_EQ(result.edge, TurnVoiceEdge::NONE);

  result = updateTurnVoiceLatch(true, true, 0.15, 0.25, 0.15);
  EXPECT_FALSE(result.active);
  EXPECT_EQ(result.edge, TurnVoiceEdge::EXIT);

  result = updateTurnVoiceLatch(false, true, -0.30, 0.25, 0.15);
  EXPECT_TRUE(result.active);
  EXPECT_EQ(result.edge, TurnVoiceEdge::ENTER);
}

TEST(TurnVoiceLatchTest, NonTrackingStateCannotEnter)
{
  const auto result = updateTurnVoiceLatch(false, false, 0.50, 0.25, 0.15);
  EXPECT_FALSE(result.active);
  EXPECT_EQ(result.edge, TurnVoiceEdge::NONE);
}

}  // namespace navdog_runtime
