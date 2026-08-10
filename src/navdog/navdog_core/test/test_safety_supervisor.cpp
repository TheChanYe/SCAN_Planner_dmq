#include <gtest/gtest.h>

#include "navdog_core/safety_supervisor.hpp"

namespace navdog
{
namespace
{

SafetySupervisor::Context context(double planar_front_m)
{
  SafetySupervisor::Context value{};
  value.robot.valid = true;
  value.robot.stamp_sec = 1.0;
  value.obstacles.valid = true;
  value.obstacles.stamp_sec = 1.0;
  value.obstacles.front_min = planar_front_m;
  value.corridor.valid = true;
  value.corridor.map_stamp_sec = 1.0;
  value.prefer_route_corridor_front = true;
  value.map_valid = true;
  value.map_stamp_sec = 1.0;
  return value;
}

VelocityCommand forwardCommand()
{
  VelocityCommand command{};
  command.vx = 0.4;
  command.valid = true;
  command.source = CommandSource::PLANNER;
  return command;
}

TEST(SafetySupervisorTest, StairUsesClear3dCorridorInsteadOfPlanarFront)
{
  NavdogConfig config{};
  SafetySupervisor supervisor(config.safety, config.limits);
  auto safety_context = context(0.3);
  safety_context.corridor.blocked = false;

  supervisor.apply(forwardCommand(), safety_context, 0.4, 0.95);
  const auto command = supervisor.apply(
      forwardCommand(), safety_context, 0.4, 1.0);
  EXPECT_GT(command.vx, 0.0);
}

TEST(SafetySupervisorTest, StairStillStopsFor3dCorridorObstacle)
{
  NavdogConfig config{};
  SafetySupervisor supervisor(config.safety, config.limits);
  auto safety_context = context(2.0);
  safety_context.corridor.blocked = true;
  safety_context.corridor.first_blocked_distance_ahead_m = 0.3;

  supervisor.apply(forwardCommand(), safety_context, 0.4, 0.95);
  const auto command = supervisor.apply(
      forwardCommand(), safety_context, 0.4, 1.0);
  EXPECT_EQ(command.vx, 0.0);
  EXPECT_EQ(command.source, CommandSource::SAFETY_STOP);
}

}  // namespace
}  // namespace navdog

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
