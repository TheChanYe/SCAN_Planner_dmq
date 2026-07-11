#include <gtest/gtest.h>

#include "navdog_core/navigation_coordinator.hpp"

namespace navdog
{
namespace
{

// =============================================================================
// 8.1 默认状态测试
// =============================================================================

TEST(NavigationCoordinatorTest, DefaultStateIsIdle)
{
  NavigationCoordinator coordinator;
  EXPECT_EQ(coordinator.state(), NavState::IDLE);
}

// =============================================================================
// 8.2 默认输出安全性测试
// =============================================================================

TEST(NavigationCoordinatorTest, DefaultOutputIsSafeZero)
{
  NavigationCoordinator coordinator;

  CoreOutput output = coordinator.update(CoreInput{}, 12.5);

  EXPECT_EQ(output.state, NavState::IDLE);
  EXPECT_EQ(output.task_sequence, 0u);
  EXPECT_EQ(output.planner_action.type, PlannerActionType::NONE);

  EXPECT_TRUE(output.final_cmd.valid);
  EXPECT_DOUBLE_EQ(output.final_cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(output.final_cmd.vy, 0.0);
  EXPECT_DOUBLE_EQ(output.final_cmd.yaw_rate, 0.0);
  EXPECT_DOUBLE_EQ(output.final_cmd.stamp_sec, 12.5);
  EXPECT_EQ(output.final_cmd.source, CommandSource::IDLE_STOP);
}

// =============================================================================
// 8.3 不透传规划速度测试
// =============================================================================

TEST(NavigationCoordinatorTest, DoesNotPassThroughPlannerCmd)
{
  NavigationCoordinator coordinator;

  CoreInput input;
  input.planner_cmd.vx = 0.5;
  input.planner_cmd.vy = 0.2;
  input.planner_cmd.yaw_rate = 0.3;
  input.planner_cmd.valid = true;
  input.planner_cmd.source = CommandSource::PLANNER;

  CoreOutput output = coordinator.update(input, 20.0);

  EXPECT_EQ(output.state, NavState::IDLE);
  EXPECT_DOUBLE_EQ(output.final_cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(output.final_cmd.vy, 0.0);
  EXPECT_DOUBLE_EQ(output.final_cmd.yaw_rate, 0.0);
  EXPECT_TRUE(output.final_cmd.valid);
  EXPECT_EQ(output.final_cmd.source, CommandSource::IDLE_STOP);
}

// =============================================================================
// 8.4 reset 测试
// =============================================================================

TEST(NavigationCoordinatorTest, ResetKeepsIdle)
{
  NavigationCoordinator coordinator;

  coordinator.reset();

  EXPECT_EQ(coordinator.state(), NavState::IDLE);
}

}  // namespace
}  // namespace navdog
