#include "navdog_runtime/navdog_runtime_node.hpp"

#include <gtest/gtest.h>

namespace navdog_runtime
{
namespace
{
navdog::NavigationEvent makeStartEvent()
{
  navdog::NavigationEvent event{};
  event.type = navdog::NavigationEventType::START_TASK;
  event.task.max_vx = 0.4;
  navdog::RoutePoint a{};
  navdog::RoutePoint b{}; b.x = 2.0;
  event.task.points = {a, b};
  return event;
}
TEST(RuntimeStatusTest, IdleMapsToStatus0)
{
  navdog::CoreOutput output{}; int status, error;
  NavdogRuntimeNode::statusForOutput(output, false, status, error);
  EXPECT_EQ(status, 0); EXPECT_EQ(error, 0);
}

TEST(RuntimeStatusTest, TrackingMapsToStatus1)
{
  navdog::CoreOutput output{}; output.state = navdog::NavState::TRACKING;
  int status, error; NavdogRuntimeNode::statusForOutput(output, false, status, error);
  EXPECT_EQ(status, 1); EXPECT_EQ(error, 0);
}

TEST(RuntimeStatusTest, PausedMapsToStatus5)
{
  navdog::CoreOutput output{}; output.state = navdog::NavState::PAUSED;
  int status, error; NavdogRuntimeNode::statusForOutput(output, false, status, error);
  EXPECT_EQ(status, 5);
}

TEST(RuntimeStatusTest, FailedMapsToError2)
{
  navdog::CoreOutput output{}; output.state = navdog::NavState::FAILED;
  int status, error; NavdogRuntimeNode::statusForOutput(output, false, status, error);
  EXPECT_EQ(status, 0); EXPECT_EQ(error, 2);
}

TEST(RuntimeStatusTest, FeedbackUsesFinalCommand)
{
  navdog::VelocityCommand command{};
  command.valid = true; command.vx = 0.2; command.vy = -0.1; command.yaw_rate = 0.15;
  const auto twist = NavdogRuntimeNode::toTwist(command);
  EXPECT_DOUBLE_EQ(twist.linear.x, 0.2);
  EXPECT_DOUBLE_EQ(twist.linear.y, -0.1);
  EXPECT_DOUBLE_EQ(twist.angular.z, 0.15);
}

TEST(RuntimeNodeTest, SetRouteActionProducesReadyFeedback)
{
  navdog::PlannerAction action{};
  action.type = navdog::PlannerActionType::SET_ROUTE;
  action.task.sequence = 9;
  const auto feedback = NavdogRuntimeNode::feedbackForAction(action, 2.0);
  EXPECT_TRUE(feedback.valid);
  EXPECT_EQ(feedback.state, navdog::PlannerState::READY);
  EXPECT_EQ(feedback.trajectory_id, 9u);
}

TEST(RuntimeNodeTest, ReadyFeedbackMovesPlanningToStartAlign)
{
  navdog::NavigationCoordinator coordinator;
  coordinator.handleEvent(makeStartEvent());
  const auto first = coordinator.update(navdog::CoreInput{}, 1.0);
  navdog::CoreInput input{};
  input.planner = NavdogRuntimeNode::feedbackForAction(first.planner_action, 1.0);
  EXPECT_EQ(coordinator.update(input, 1.1).state, navdog::NavState::START_ALIGN);
}

TEST(RuntimeNodeTest, PausedStatePublishesImmediateZero)
{
  navdog::NavigationCoordinator coordinator;
  coordinator.handleEvent(makeStartEvent());
  navdog::NavigationEvent pause{}; pause.type = navdog::NavigationEventType::PAUSE;
  coordinator.handleEvent(pause);
  const auto output = coordinator.update(navdog::CoreInput{}, 1.0);
  EXPECT_EQ(output.state, navdog::NavState::PAUSED);
  EXPECT_DOUBLE_EQ(NavdogRuntimeNode::toTwist(output.final_cmd).linear.x, 0.0);
}

TEST(RuntimeNodeTest, CancelPublishesImmediateZero)
{
  navdog::NavigationCoordinator coordinator;
  coordinator.handleEvent(makeStartEvent());
  navdog::NavigationEvent cancel{}; cancel.type = navdog::NavigationEventType::CANCEL_TASK;
  coordinator.handleEvent(cancel);
  const auto output = coordinator.update(navdog::CoreInput{}, 1.0);
  EXPECT_EQ(output.state, navdog::NavState::IDLE);
  EXPECT_DOUBLE_EQ(NavdogRuntimeNode::toTwist(output.final_cmd).linear.x, 0.0);
}

TEST(RuntimeNodeTest, FinalCommandPublishedToCmdVel)
{
  navdog::VelocityCommand command{};
  command.valid = true;
  command.vx = 0.31;
  EXPECT_DOUBLE_EQ(NavdogRuntimeNode::toTwist(command).linear.x, 0.31);
}

TEST(RuntimeNodeTest, OnlyFinalCmdIsPublished)
{
  navdog::CoreOutput output{};
  output.final_cmd.valid = true;
  output.final_cmd.vx = 0.2;
  navdog::VelocityCommand unrelated{};
  unrelated.valid = true;
  unrelated.vx = 0.9;
  EXPECT_DOUBLE_EQ(NavdogRuntimeNode::toTwist(output.final_cmd).linear.x, 0.2);
}
}
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
