#include <gtest/gtest.h>

#include "navdog_core/navigation_coordinator.hpp"

namespace navdog
{
namespace
{

// =============================================================================
// Helper
// =============================================================================

NavigationEvent makeValidStartEvent()
{
  NavigationEvent event{};
  event.type = NavigationEventType::START_TASK;
  event.task.mode = TaskMode::NORMAL_AVOID;
  event.task.max_vx = 0.4;

  RoutePoint point{};
  point.x = 1.0;
  point.y = 0.0;
  point.z = 0.0;

  event.task.points.push_back(point);

  return event;
}

// =============================================================================
// DefaultStateIsIdle
// =============================================================================

TEST(NavigationCoordinatorTest, DefaultStateIsIdle)
{
  NavigationCoordinator coordinator;
  EXPECT_EQ(coordinator.state(), NavState::IDLE);
}

// =============================================================================
// DefaultOutputIsSafeZero
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
// DoesNotPassThroughPlannerCmd
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
// ResetKeepsIdle
// =============================================================================

TEST(NavigationCoordinatorTest, ResetKeepsIdle)
{
  NavigationCoordinator coordinator;

  coordinator.reset();

  EXPECT_EQ(coordinator.state(), NavState::IDLE);
}

// =============================================================================
// 13.2 StartTaskTransitionsToPlanning
// =============================================================================

TEST(NavigationCoordinatorTest, StartTaskTransitionsToPlanning)
{
  NavigationCoordinator coordinator;

  NavigationEvent event = makeValidStartEvent();
  TaskHandleResult result = coordinator.handleEvent(event);

  EXPECT_EQ(result, TaskHandleResult::STARTED);
  EXPECT_EQ(coordinator.state(), NavState::PLANNING);
  EXPECT_TRUE(coordinator.hasActiveTask());
}

// =============================================================================
// 13.3 StartTaskEmitsSetRouteOnce
// =============================================================================

TEST(NavigationCoordinatorTest, StartTaskEmitsSetRouteOnce)
{
  NavigationCoordinator coordinator;

  coordinator.handleEvent(makeValidStartEvent());

  // First update: should emit SET_ROUTE
  CoreOutput out1 = coordinator.update(CoreInput{}, 1.0);
  EXPECT_EQ(out1.state, NavState::PLANNING);
  EXPECT_EQ(out1.task_sequence, 1u);
  EXPECT_EQ(out1.planner_action.type, PlannerActionType::SET_ROUTE);
  EXPECT_EQ(out1.planner_action.task.sequence, 1u);
  EXPECT_DOUBLE_EQ(out1.final_cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(out1.final_cmd.vy, 0.0);
  EXPECT_DOUBLE_EQ(out1.final_cmd.yaw_rate, 0.0);
  EXPECT_EQ(out1.final_cmd.source, CommandSource::PLANNING_STOP);

  // Second update: no more action
  CoreOutput out2 = coordinator.update(CoreInput{}, 2.0);
  EXPECT_EQ(out2.planner_action.type, PlannerActionType::NONE);
  EXPECT_EQ(out2.task_sequence, 1u);
  EXPECT_EQ(out2.final_cmd.source, CommandSource::PLANNING_STOP);
}

// =============================================================================
// 13.4 InvalidTaskDoesNotLeaveIdle
// =============================================================================

TEST(NavigationCoordinatorTest, InvalidTaskDoesNotLeaveIdle)
{
  NavigationCoordinator coordinator;

  NavigationEvent event{};
  event.type = NavigationEventType::START_TASK;
  event.task.mode = TaskMode::NORMAL_AVOID;
  event.task.max_vx = 0.4;
  // points empty

  TaskHandleResult result = coordinator.handleEvent(event);

  EXPECT_EQ(result, TaskHandleResult::REJECTED_INVALID_TASK);
  EXPECT_EQ(coordinator.state(), NavState::IDLE);
  EXPECT_FALSE(coordinator.hasActiveTask());

  CoreOutput output = coordinator.update(CoreInput{}, 1.0);
  EXPECT_EQ(output.planner_action.type, PlannerActionType::NONE);
  EXPECT_EQ(output.final_cmd.source, CommandSource::IDLE_STOP);
}

// =============================================================================
// 13.5 BusyTaskDoesNotReplaceActiveTask
// =============================================================================

TEST(NavigationCoordinatorTest, BusyTaskDoesNotReplaceActiveTask)
{
  NavigationCoordinator coordinator;

  // Task A
  NavigationEvent eventA = makeValidStartEvent();
  eventA.task.points[0].x = 10.0;
  coordinator.handleEvent(eventA);

  // Consume SET_ROUTE
  coordinator.update(CoreInput{}, 1.0);

  // Task B
  NavigationEvent eventB = makeValidStartEvent();
  eventB.task.points[0].x = 20.0;
  TaskHandleResult result = coordinator.handleEvent(eventB);

  EXPECT_EQ(result, TaskHandleResult::REJECTED_BUSY);
  EXPECT_EQ(coordinator.state(), NavState::PLANNING);

  NavigationTask stored{};
  coordinator.copyActiveTask(stored);
  EXPECT_EQ(stored.sequence, 1u);
  EXPECT_DOUBLE_EQ(stored.points[0].x, 10.0);

  CoreOutput output = coordinator.update(CoreInput{}, 2.0);
  EXPECT_EQ(output.planner_action.type, PlannerActionType::NONE);
}

// =============================================================================
// 13.6 CancelTransitionsToIdleAndEmitsOnce
// =============================================================================

TEST(NavigationCoordinatorTest, CancelTransitionsToIdleAndEmitsOnce)
{
  NavigationCoordinator coordinator;

  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 1.0);  // consume SET_ROUTE

  NavigationEvent cancel{};
  cancel.type = NavigationEventType::CANCEL_TASK;
  TaskHandleResult result = coordinator.handleEvent(cancel);

  EXPECT_EQ(result, TaskHandleResult::CANCELLED);

  // First update after cancel
  CoreOutput out1 = coordinator.update(CoreInput{}, 2.0);
  EXPECT_EQ(out1.state, NavState::IDLE);
  EXPECT_FALSE(coordinator.hasActiveTask());
  EXPECT_EQ(out1.planner_action.type, PlannerActionType::CANCEL);
  EXPECT_EQ(out1.planner_action.task.sequence, 1u);
  EXPECT_EQ(out1.task_sequence, 0u);
  EXPECT_DOUBLE_EQ(out1.final_cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(out1.final_cmd.vy, 0.0);
  EXPECT_DOUBLE_EQ(out1.final_cmd.yaw_rate, 0.0);
  EXPECT_EQ(out1.final_cmd.source, CommandSource::CANCEL_STOP);

  // Second update: no more action
  CoreOutput out2 = coordinator.update(CoreInput{}, 3.0);
  EXPECT_EQ(out2.planner_action.type, PlannerActionType::NONE);
  EXPECT_EQ(out2.final_cmd.source, CommandSource::IDLE_STOP);
}

// =============================================================================
// 13.7 CancelDropsPendingSetRoute
// =============================================================================

TEST(NavigationCoordinatorTest, CancelDropsPendingSetRoute)
{
  NavigationCoordinator coordinator;

  coordinator.handleEvent(makeValidStartEvent());
  // Do NOT call update — SET_ROUTE still pending

  NavigationEvent cancel{};
  cancel.type = NavigationEventType::CANCEL_TASK;
  coordinator.handleEvent(cancel);

  // First update: must be CANCEL, not SET_ROUTE
  CoreOutput out1 = coordinator.update(CoreInput{}, 1.0);
  EXPECT_EQ(out1.state, NavState::IDLE);
  EXPECT_FALSE(coordinator.hasActiveTask());
  EXPECT_EQ(out1.planner_action.type, PlannerActionType::CANCEL);

  // Second update: nothing
  CoreOutput out2 = coordinator.update(CoreInput{}, 2.0);
  EXPECT_EQ(out2.planner_action.type, PlannerActionType::NONE);
}

// =============================================================================
// 13.8 MaxVxUpdateEmitsActionOnce
// =============================================================================

TEST(NavigationCoordinatorTest, MaxVxUpdateEmitsActionOnce)
{
  NavigationCoordinator coordinator;

  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 1.0);  // consume SET_ROUTE

  NavigationEvent speedUp{};
  speedUp.type = NavigationEventType::UPDATE_MAX_VX;
  speedUp.max_vx = 0.6;

  TaskHandleResult result = coordinator.handleEvent(speedUp);
  EXPECT_EQ(result, TaskHandleResult::MAX_VX_UPDATED);

  // First update
  CoreOutput out1 = coordinator.update(CoreInput{}, 2.0);
  EXPECT_EQ(out1.state, NavState::PLANNING);
  EXPECT_EQ(out1.planner_action.type, PlannerActionType::UPDATE_SPEED_LIMIT);
  EXPECT_EQ(out1.planner_action.task.sequence, 1u);
  EXPECT_DOUBLE_EQ(out1.planner_action.max_vx, 0.6);
  EXPECT_EQ(out1.final_cmd.source, CommandSource::PLANNING_STOP);

  // Second update: no more action
  CoreOutput out2 = coordinator.update(CoreInput{}, 3.0);
  EXPECT_EQ(out2.planner_action.type, PlannerActionType::NONE);
}

// =============================================================================
// 13.9 UnchangedMaxVxEmitsNoAction
// =============================================================================

TEST(NavigationCoordinatorTest, UnchangedMaxVxEmitsNoAction)
{
  NavigationCoordinator coordinator;

  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 1.0);  // consume SET_ROUTE

  NavigationEvent sameSpeed{};
  sameSpeed.type = NavigationEventType::UPDATE_MAX_VX;
  sameSpeed.max_vx = 0.4;

  TaskHandleResult result = coordinator.handleEvent(sameSpeed);
  EXPECT_EQ(result, TaskHandleResult::MAX_VX_UNCHANGED);

  CoreOutput output = coordinator.update(CoreInput{}, 2.0);
  EXPECT_EQ(output.planner_action.type, PlannerActionType::NONE);
  EXPECT_EQ(coordinator.state(), NavState::PLANNING);
}

// =============================================================================
// 13.10 UnsupportedEventDoesNotChangeCoordinator
// =============================================================================

TEST(NavigationCoordinatorTest, UnsupportedEventDoesNotChangeCoordinator)
{
  NavigationCoordinator coordinator;

  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 1.0);  // consume SET_ROUTE

  NavigationEvent pause{};
  pause.type = NavigationEventType::PAUSE;
  TaskHandleResult result = coordinator.handleEvent(pause);

  EXPECT_EQ(result, TaskHandleResult::UNSUPPORTED_EVENT);
  EXPECT_EQ(coordinator.state(), NavState::PLANNING);
  EXPECT_TRUE(coordinator.hasActiveTask());

  CoreOutput output = coordinator.update(CoreInput{}, 2.0);
  EXPECT_EQ(output.planner_action.type, PlannerActionType::NONE);
}

// =============================================================================
// 13.11 ResetClearsTaskAndPendingAction
// =============================================================================

TEST(NavigationCoordinatorTest, ResetClearsTaskAndPendingAction)
{
  NavigationCoordinator coordinator;

  coordinator.handleEvent(makeValidStartEvent());
  // Do NOT call update — SET_ROUTE still pending

  coordinator.reset();

  CoreOutput output = coordinator.update(CoreInput{}, 1.0);
  EXPECT_EQ(output.state, NavState::IDLE);
  EXPECT_FALSE(coordinator.hasActiveTask());
  EXPECT_EQ(output.task_sequence, 0u);
  EXPECT_EQ(output.planner_action.type, PlannerActionType::NONE);
  EXPECT_EQ(output.final_cmd.source, CommandSource::IDLE_STOP);
}

// =============================================================================
// 13.12 PlanningStateStillRejectsPlannerCmd
// =============================================================================

TEST(NavigationCoordinatorTest, PlanningStateStillRejectsPlannerCmd)
{
  NavigationCoordinator coordinator;

  coordinator.handleEvent(makeValidStartEvent());

  CoreInput input;
  input.planner_cmd.vx = 0.5;
  input.planner_cmd.vy = 0.2;
  input.planner_cmd.yaw_rate = 0.3;
  input.planner_cmd.valid = true;

  CoreOutput output = coordinator.update(input, 1.0);

  EXPECT_DOUBLE_EQ(output.final_cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(output.final_cmd.vy, 0.0);
  EXPECT_DOUBLE_EQ(output.final_cmd.yaw_rate, 0.0);
  EXPECT_EQ(output.final_cmd.source, CommandSource::PLANNING_STOP);
}

}  // namespace
}  // namespace navdog

// =============================================================================
// main
// =============================================================================

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
