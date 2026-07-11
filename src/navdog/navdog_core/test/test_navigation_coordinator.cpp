#include <gtest/gtest.h>
#include <cmath>
#include <limits>

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

CoreInput makePlannerInput(
    PlannerState state,
    std::uint64_t trajectory_id,
    double stamp_sec)
{
  CoreInput input{};
  input.planner.state = state;
  input.planner.trajectory_id = trajectory_id;
  input.planner.stamp_sec = stamp_sec;
  input.planner.valid = true;
  return input;
}

// =============================================================================
// DefaultStateIsIdle
// =============================================================================

CoreInput makeRobotInput(
    double x,
    double y,
    double yaw);

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

  CoreOutput out1 = coordinator.update(CoreInput{}, 1.0);
  EXPECT_EQ(out1.state, NavState::PLANNING);
  EXPECT_EQ(out1.task_sequence, 1u);
  EXPECT_EQ(out1.planner_action.type, PlannerActionType::SET_ROUTE);
  EXPECT_EQ(out1.planner_action.task.sequence, 1u);
  EXPECT_DOUBLE_EQ(out1.final_cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(out1.final_cmd.vy, 0.0);
  EXPECT_DOUBLE_EQ(out1.final_cmd.yaw_rate, 0.0);
  EXPECT_EQ(out1.final_cmd.source, CommandSource::PLANNING_STOP);

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

  NavigationEvent eventA = makeValidStartEvent();
  eventA.task.points[0].x = 10.0;
  coordinator.handleEvent(eventA);

  coordinator.update(CoreInput{}, 1.0);

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
  coordinator.update(CoreInput{}, 1.0);

  NavigationEvent cancel{};
  cancel.type = NavigationEventType::CANCEL_TASK;
  TaskHandleResult result = coordinator.handleEvent(cancel);

  EXPECT_EQ(result, TaskHandleResult::CANCELLED);

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

  NavigationEvent cancel{};
  cancel.type = NavigationEventType::CANCEL_TASK;
  coordinator.handleEvent(cancel);

  CoreOutput out1 = coordinator.update(CoreInput{}, 1.0);
  EXPECT_EQ(out1.state, NavState::IDLE);
  EXPECT_FALSE(coordinator.hasActiveTask());
  EXPECT_EQ(out1.planner_action.type, PlannerActionType::CANCEL);

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
  coordinator.update(CoreInput{}, 1.0);

  NavigationEvent speedUp{};
  speedUp.type = NavigationEventType::UPDATE_MAX_VX;
  speedUp.max_vx = 0.6;

  TaskHandleResult result = coordinator.handleEvent(speedUp);
  EXPECT_EQ(result, TaskHandleResult::MAX_VX_UPDATED);

  CoreOutput out1 = coordinator.update(CoreInput{}, 2.0);
  EXPECT_EQ(out1.state, NavState::PLANNING);
  EXPECT_EQ(out1.planner_action.type, PlannerActionType::UPDATE_SPEED_LIMIT);
  EXPECT_EQ(out1.planner_action.task.sequence, 1u);
  EXPECT_DOUBLE_EQ(out1.planner_action.max_vx, 0.6);
  EXPECT_EQ(out1.final_cmd.source, CommandSource::PLANNING_STOP);

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
  coordinator.update(CoreInput{}, 1.0);

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
  coordinator.update(CoreInput{}, 1.0);

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

// =============================================================================
// 17.1 SetRouteStartsPlanningAndIgnoresSameCycleFeedback
// =============================================================================

TEST(NavigationCoordinatorTest, SetRouteStartsPlanningAndIgnoresSameCycleFeedback)
{
  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeValidStartEvent());

  CoreInput input = makePlannerInput(PlannerState::READY, 1u, 1.0);
  CoreOutput out1 = coordinator.update(input, 1.0);

  EXPECT_EQ(out1.planner_action.type, PlannerActionType::SET_ROUTE);
  EXPECT_EQ(out1.state, NavState::PLANNING);
  EXPECT_EQ(out1.final_cmd.source, CommandSource::PLANNING_STOP);
}

// =============================================================================
// 17.2 ReadyFeedbackTransitionsToStartAlign
// =============================================================================

TEST(NavigationCoordinatorTest, ReadyFeedbackTransitionsToStartAlign)
{
  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 1.0);  // emit SET_ROUTE

  CoreInput input = makePlannerInput(PlannerState::READY, 1u, 1.1);
  CoreOutput output = coordinator.update(input, 1.1);

  EXPECT_EQ(output.state, NavState::START_ALIGN);
  EXPECT_EQ(output.task_sequence, 1u);
  EXPECT_TRUE(coordinator.hasActiveTask());
  EXPECT_DOUBLE_EQ(output.final_cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(output.final_cmd.vy, 0.0);
  EXPECT_DOUBLE_EQ(output.final_cmd.yaw_rate, 0.0);
  EXPECT_EQ(output.final_cmd.source, CommandSource::START_ALIGN);
}

// =============================================================================
// 17.3 ExecutingFeedbackTransitionsToStartAlign
// =============================================================================

TEST(NavigationCoordinatorTest, ExecutingFeedbackTransitionsToStartAlign)
{
  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 1.0);

  CoreInput input = makePlannerInput(PlannerState::EXECUTING, 1u, 1.1);
  CoreOutput output = coordinator.update(input, 1.1);

  EXPECT_EQ(output.state, NavState::START_ALIGN);
  EXPECT_EQ(output.final_cmd.source, CommandSource::START_ALIGN);
}

// =============================================================================
// 17.4 FailedFeedbackTransitionsToFailed
// =============================================================================

TEST(NavigationCoordinatorTest, FailedFeedbackTransitionsToFailed)
{
  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 1.0);

  CoreInput input = makePlannerInput(PlannerState::FAILED, 1u, 1.1);
  CoreOutput output = coordinator.update(input, 1.1);

  EXPECT_EQ(output.state, NavState::FAILED);
  EXPECT_EQ(output.task_sequence, 1u);
  EXPECT_TRUE(coordinator.hasActiveTask());
  EXPECT_DOUBLE_EQ(output.final_cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(output.final_cmd.vy, 0.0);
  EXPECT_DOUBLE_EQ(output.final_cmd.yaw_rate, 0.0);
  EXPECT_EQ(output.final_cmd.source, CommandSource::FAILED_STOP);
}

// =============================================================================
// 17.5 WaitingFeedbackKeepsPlanning
// =============================================================================

TEST(NavigationCoordinatorTest, WaitingFeedbackKeepsPlanning)
{
  {
    NavigationCoordinator coordinator;
    coordinator.handleEvent(makeValidStartEvent());
    coordinator.update(CoreInput{}, 1.0);

    CoreInput input = makePlannerInput(PlannerState::UNAVAILABLE, 1u, 1.1);
    CoreOutput output = coordinator.update(input, 1.1);

    EXPECT_EQ(output.state, NavState::PLANNING);
    EXPECT_EQ(output.planner_action.type, PlannerActionType::NONE);
  }
  {
    NavigationCoordinator coordinator;
    coordinator.handleEvent(makeValidStartEvent());
    coordinator.update(CoreInput{}, 1.0);

    CoreInput input = makePlannerInput(PlannerState::IDLE, 1u, 1.1);
    CoreOutput output = coordinator.update(input, 1.1);

    EXPECT_EQ(output.state, NavState::PLANNING);
    EXPECT_EQ(output.planner_action.type, PlannerActionType::NONE);
  }
  {
    NavigationCoordinator coordinator;
    coordinator.handleEvent(makeValidStartEvent());
    coordinator.update(CoreInput{}, 1.0);

    CoreInput input = makePlannerInput(PlannerState::PLANNING, 1u, 1.1);
    CoreOutput output = coordinator.update(input, 1.1);

    EXPECT_EQ(output.state, NavState::PLANNING);
    EXPECT_EQ(output.planner_action.type, PlannerActionType::NONE);
  }
}

// =============================================================================
// 17.6 InvalidFeedbackIsIgnored
// =============================================================================

TEST(NavigationCoordinatorTest, InvalidFeedbackIsIgnored)
{
  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 1.0);

  CoreInput input{};
  input.planner.state = PlannerState::READY;
  input.planner.trajectory_id = 1u;
  input.planner.stamp_sec = 1.1;
  input.planner.valid = false;

  CoreOutput output = coordinator.update(input, 1.1);
  EXPECT_EQ(output.state, NavState::PLANNING);
}

// =============================================================================
// 17.7 MismatchedTrajectoryIdIsIgnored
// =============================================================================

TEST(NavigationCoordinatorTest, MismatchedTrajectoryIdIsIgnored)
{
  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 1.0);

  CoreInput input = makePlannerInput(PlannerState::READY, 999u, 1.1);
  CoreOutput output = coordinator.update(input, 1.1);

  EXPECT_EQ(output.state, NavState::PLANNING);
}

// =============================================================================
// 17.8 ZeroTrajectoryIdIsIgnored
// =============================================================================

TEST(NavigationCoordinatorTest, ZeroTrajectoryIdIsIgnored)
{
  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 1.0);

  CoreInput input = makePlannerInput(PlannerState::READY, 0u, 1.1);
  CoreOutput output = coordinator.update(input, 1.1);

  EXPECT_EQ(output.state, NavState::PLANNING);
}

// =============================================================================
// 17.9 StaleFeedbackIsIgnored
// =============================================================================

TEST(NavigationCoordinatorTest, StaleFeedbackIsIgnored)
{
  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 10.0);  // SET_ROUTE at 10.0

  CoreInput input = makePlannerInput(PlannerState::READY, 1u, 9.9);
  CoreOutput output = coordinator.update(input, 10.1);

  EXPECT_EQ(output.state, NavState::PLANNING);
}

// =============================================================================
// 17.10 FutureFeedbackIsIgnored
// =============================================================================

TEST(NavigationCoordinatorTest, FutureFeedbackIsIgnored)
{
  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 10.0);

  CoreInput input = makePlannerInput(PlannerState::READY, 1u, 10.2);
  CoreOutput output = coordinator.update(input, 10.1);

  EXPECT_EQ(output.state, NavState::PLANNING);
}

// =============================================================================
// 17.11 NonFiniteFeedbackStampIsIgnored
// =============================================================================

TEST(NavigationCoordinatorTest, NonFiniteFeedbackStampIsIgnored)
{
  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 1.0);

  CoreInput input = makePlannerInput(
      PlannerState::READY, 1u,
      std::numeric_limits<double>::quiet_NaN());
  CoreOutput output = coordinator.update(input, 1.1);

  EXPECT_EQ(output.state, NavState::PLANNING);
}

// =============================================================================
// 17.12 DoesNotTimeoutBeforeSetRouteEmission
// =============================================================================

TEST(NavigationCoordinatorTest, DoesNotTimeoutBeforeSetRouteEmission)
{
  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeValidStartEvent());

  CoreOutput output = coordinator.update(CoreInput{}, 100.0);

  EXPECT_EQ(output.planner_action.type, PlannerActionType::SET_ROUTE);
  EXPECT_EQ(output.state, NavState::PLANNING);
}

// =============================================================================
// 17.13 DoesNotTimeoutAtExactBoundary
// =============================================================================

TEST(NavigationCoordinatorTest, DoesNotTimeoutAtExactBoundary)
{
  NavdogConfig config;
  config.planner.planning_timeout_sec = 2.0;
  NavigationCoordinator coordinator(config);

  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 1.0);  // SET_ROUTE at 1.0

  CoreOutput output = coordinator.update(CoreInput{}, 3.0);  // exactly 2.0
  EXPECT_EQ(output.state, NavState::PLANNING);
}

// =============================================================================
// 17.14 PlanningTimeoutTransitionsToFailed
// =============================================================================

TEST(NavigationCoordinatorTest, PlanningTimeoutTransitionsToFailed)
{
  NavdogConfig config;
  config.planner.planning_timeout_sec = 2.0;
  NavigationCoordinator coordinator(config);

  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 1.0);  // SET_ROUTE at 1.0

  CoreOutput output = coordinator.update(CoreInput{}, 3.001);

  EXPECT_EQ(output.state, NavState::FAILED);
  EXPECT_EQ(output.final_cmd.source, CommandSource::FAILED_STOP);
  EXPECT_TRUE(coordinator.hasActiveTask());
  EXPECT_EQ(output.task_sequence, 1u);
}

// =============================================================================
// 17.15 CancelFromStartAlignReturnsIdle
// =============================================================================

TEST(NavigationCoordinatorTest, CancelFromStartAlignReturnsIdle)
{
  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 1.0);  // SET_ROUTE

  CoreInput ready = makePlannerInput(PlannerState::READY, 1u, 1.1);
  coordinator.update(ready, 1.1);  // → START_ALIGN

  NavigationEvent cancel{};
  cancel.type = NavigationEventType::CANCEL_TASK;
  coordinator.handleEvent(cancel);

  CoreOutput output = coordinator.update(CoreInput{}, 1.2);

  EXPECT_EQ(output.state, NavState::IDLE);
  EXPECT_EQ(output.planner_action.type, PlannerActionType::CANCEL);
  EXPECT_EQ(output.final_cmd.source, CommandSource::CANCEL_STOP);
  EXPECT_FALSE(coordinator.hasActiveTask());
}

// =============================================================================
// 17.16 CancelFromFailedReturnsIdle
// =============================================================================

TEST(NavigationCoordinatorTest, CancelFromFailedReturnsIdle)
{
  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 1.0);  // SET_ROUTE

  CoreInput failed = makePlannerInput(PlannerState::FAILED, 1u, 1.1);
  coordinator.update(failed, 1.1);  // → FAILED

  NavigationEvent cancel{};
  cancel.type = NavigationEventType::CANCEL_TASK;
  coordinator.handleEvent(cancel);

  CoreOutput output = coordinator.update(CoreInput{}, 1.2);

  EXPECT_EQ(output.state, NavState::IDLE);
  EXPECT_EQ(output.planner_action.type, PlannerActionType::CANCEL);
  EXPECT_FALSE(coordinator.hasActiveTask());
}

// =============================================================================
// 17.17 FailedStateIgnoresLaterReadyFeedback
// =============================================================================

TEST(NavigationCoordinatorTest, FailedStateIgnoresLaterReadyFeedback)
{
  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 1.0);

  CoreInput failed = makePlannerInput(PlannerState::FAILED, 1u, 1.1);
  coordinator.update(failed, 1.1);

  CoreInput ready = makePlannerInput(PlannerState::READY, 1u, 1.2);
  CoreOutput output = coordinator.update(ready, 1.2);

  EXPECT_EQ(output.state, NavState::FAILED);
}

// =============================================================================
// 17.18 StartAlignIgnoresLaterFailedFeedback
// =============================================================================

TEST(NavigationCoordinatorTest, StartAlignIgnoresLaterFailedFeedback)
{
  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 1.0);

  CoreInput ready = makePlannerInput(PlannerState::READY, 1u, 1.1);
  coordinator.update(ready, 1.1);

  CoreInput failed = makePlannerInput(PlannerState::FAILED, 1u, 1.2);
  CoreOutput output = coordinator.update(failed, 1.2);

  EXPECT_EQ(output.state, NavState::START_ALIGN);
}

// =============================================================================
// 17.19 FailureDropsPendingSpeedUpdate
// =============================================================================

TEST(NavigationCoordinatorTest, FailureDropsPendingSpeedUpdate)
{
  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 1.0);  // SET_ROUTE

  NavigationEvent speedUp{};
  speedUp.type = NavigationEventType::UPDATE_MAX_VX;
  speedUp.max_vx = 0.6;
  coordinator.handleEvent(speedUp);  // enqueue UPDATE_SPEED_LIMIT

  CoreInput failed = makePlannerInput(PlannerState::FAILED, 1u, 1.1);
  CoreOutput out1 = coordinator.update(failed, 1.1);

  EXPECT_EQ(out1.state, NavState::FAILED);
  EXPECT_EQ(out1.planner_action.type, PlannerActionType::NONE);

  CoreOutput out2 = coordinator.update(CoreInput{}, 1.2);
  EXPECT_EQ(out2.planner_action.type, PlannerActionType::NONE);
}

// =============================================================================
// 17.20 ReadyKeepsPendingSpeedUpdate
// =============================================================================

TEST(NavigationCoordinatorTest, ReadyKeepsPendingSpeedUpdate)
{
  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 1.0);  // SET_ROUTE

  NavigationEvent speedUp{};
  speedUp.type = NavigationEventType::UPDATE_MAX_VX;
  speedUp.max_vx = 0.6;
  coordinator.handleEvent(speedUp);  // enqueue UPDATE_SPEED_LIMIT

  CoreInput ready = makePlannerInput(PlannerState::READY, 1u, 1.1);
  CoreOutput output = coordinator.update(ready, 1.1);

  EXPECT_EQ(output.state, NavState::START_ALIGN);
  EXPECT_EQ(output.planner_action.type, PlannerActionType::UPDATE_SPEED_LIMIT);
}

// =============================================================================
// 17.21 ResetClearsPlanningContext
// =============================================================================

TEST(NavigationCoordinatorTest, ResetClearsPlanningContext)
{
  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 1.0);  // SET_ROUTE

  coordinator.reset();

  CoreInput ready = makePlannerInput(PlannerState::READY, 1u, 1.1);
  CoreOutput output = coordinator.update(ready, 1.1);

  EXPECT_EQ(output.state, NavState::IDLE);
  EXPECT_EQ(output.planner_action.type, PlannerActionType::NONE);
  EXPECT_EQ(output.task_sequence, 0u);
}

// =============================================================================
// 26.1 ReadyWithLargeYawProducesAlignCommand
// =============================================================================

TEST(NavigationCoordinatorTest, ReadyWithLargeYawProducesAlignCommand)
{
  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 1.0);

  CoreInput ready = makePlannerInput(PlannerState::READY, 1u, 1.1);
  coordinator.update(ready, 1.1);

  CoreInput robot_in = makeRobotInput(0, 0, 90.0 * 3.14159265358979323846 / 180.0);
  CoreOutput output = coordinator.update(robot_in, 1.2);

  EXPECT_EQ(output.state, NavState::START_ALIGN);
  EXPECT_DOUBLE_EQ(output.final_cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(output.final_cmd.vy, 0.0);
  EXPECT_NE(output.final_cmd.yaw_rate, 0.0);
  EXPECT_EQ(output.final_cmd.source, CommandSource::START_ALIGN);
}

// =============================================================================
// 26.2 StartAlignYawRateUsesCorrectDirection
// =============================================================================

TEST(NavigationCoordinatorTest, StartAlignYawRateUsesCorrectDirection)
{
  constexpr double kDeg = 3.14159265358979323846 / 180.0;

  // Positive error (target > robot) → negative yaw_rate
  {
    NavigationCoordinator coordinator;
    coordinator.handleEvent(makeValidStartEvent());
    coordinator.update(CoreInput{}, 1.0);

    CoreInput ready = makePlannerInput(PlannerState::READY, 1u, 1.1);
    coordinator.update(ready, 1.1);

    // route east (0°), robot at +30° → error = -30° → negative rate
    CoreInput robot_in = makeRobotInput(0, 0, 30.0 * kDeg);
    CoreOutput output = coordinator.update(robot_in, 1.2);

    EXPECT_EQ(output.state, NavState::START_ALIGN);
    EXPECT_LT(output.final_cmd.yaw_rate, 0.0);
  }
  // Negative error (target < robot) → positive yaw_rate
  {
    NavigationCoordinator coordinator;
    coordinator.handleEvent(makeValidStartEvent());
    coordinator.update(CoreInput{}, 1.0);

    CoreInput ready = makePlannerInput(PlannerState::READY, 1u, 1.1);
    coordinator.update(ready, 1.1);

    // route east (0°), robot at -30° → error = +30° → positive rate
    CoreInput robot_in = makeRobotInput(0, 0, -30.0 * kDeg);
    CoreOutput output = coordinator.update(robot_in, 1.2);

    EXPECT_EQ(output.state, NavState::START_ALIGN);
    EXPECT_GT(output.final_cmd.yaw_rate, 0.0);
  }
}

// =============================================================================
// 26.3 StartAlignYawRateIsLimited
// =============================================================================

TEST(NavigationCoordinatorTest, StartAlignYawRateIsLimited)
{
  constexpr double kDeg = 3.14159265358979323846 / 180.0;

  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 1.0);

  CoreInput ready = makePlannerInput(PlannerState::READY, 1u, 1.1);
  coordinator.update(ready, 1.1);

  CoreInput robot_in = makeRobotInput(0, 0, 170.0 * kDeg);
  CoreOutput output = coordinator.update(robot_in, 1.2);

  EXPECT_EQ(output.state, NavState::START_ALIGN);
  EXPECT_LE(std::fabs(output.final_cmd.yaw_rate), 0.3 + 1e-9);
}

// =============================================================================
// 26.4 SmallInitialErrorTransitionsDirectlyToTracking
// =============================================================================

TEST(NavigationCoordinatorTest, SmallInitialErrorTransitionsDirectlyToTracking)
{
  constexpr double kDeg = 3.14159265358979323846 / 180.0;

  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 1.0);

  CoreInput ready = makePlannerInput(PlannerState::READY, 1u, 1.1);
  coordinator.update(ready, 1.1);

  // 10° error < enter=15° → directly ALIGNED → TRACKING
  CoreInput robot_in = makeRobotInput(0, 0, 10.0 * kDeg);
  CoreOutput output = coordinator.update(robot_in, 1.2);

  EXPECT_EQ(output.state, NavState::TRACKING);
  EXPECT_DOUBLE_EQ(output.final_cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(output.final_cmd.vy, 0.0);
  EXPECT_DOUBLE_EQ(output.final_cmd.yaw_rate, 0.0);
  EXPECT_EQ(output.final_cmd.source, CommandSource::TRACKING_STOP);
}

// =============================================================================
// 26.5 AlignmentTransitionsToTrackingAtExitThreshold
// =============================================================================

TEST(NavigationCoordinatorTest, AlignmentTransitionsToTrackingAtExitThreshold)
{
  constexpr double kDeg = 3.14159265358979323846 / 180.0;

  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 1.0);

  CoreInput ready = makePlannerInput(PlannerState::READY, 1u, 1.1);
  coordinator.update(ready, 1.1);

  // 30° → ALIGNING
  CoreInput robot1 = makeRobotInput(0, 0, 30.0 * kDeg);
  CoreOutput out1 = coordinator.update(robot1, 1.2);
  EXPECT_EQ(out1.state, NavState::START_ALIGN);

  // 10° → still ALIGNING (above exit=5°)
  CoreInput robot2 = makeRobotInput(0, 0, 10.0 * kDeg);
  CoreOutput out2 = coordinator.update(robot2, 1.3);
  EXPECT_EQ(out2.state, NavState::START_ALIGN);

  // 4° → TRACKING (below exit=5°)
  CoreInput robot3 = makeRobotInput(0, 0, 4.0 * kDeg);
  CoreOutput out3 = coordinator.update(robot3, 1.4);
  EXPECT_EQ(out3.state, NavState::TRACKING);
  EXPECT_EQ(out3.final_cmd.source, CommandSource::TRACKING_STOP);
}

// =============================================================================
// 26.6 StartAlignTimeoutTransitionsToFailed
// =============================================================================

TEST(NavigationCoordinatorTest, StartAlignTimeoutTransitionsToFailed)
{
  constexpr double kDeg = 3.14159265358979323846 / 180.0;

  NavdogConfig config;
  config.start_align.max_hold_sec = 2.0;
  NavigationCoordinator coordinator(config);

  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 1.0);

  CoreInput ready = makePlannerInput(PlannerState::READY, 1u, 1.1);
  coordinator.update(ready, 1.1);

  // Large error, past timeout
  CoreInput robot = makeRobotInput(0, 0, 30.0 * kDeg);
  CoreOutput output = coordinator.update(robot, 3.2);

  EXPECT_EQ(output.state, NavState::FAILED);
  EXPECT_EQ(output.final_cmd.source, CommandSource::FAILED_STOP);
  EXPECT_TRUE(coordinator.hasActiveTask());
  EXPECT_EQ(output.task_sequence, 1u);
}

// =============================================================================
// 26.7 MissingRobotWaitsThenTimesOut
// =============================================================================

TEST(NavigationCoordinatorTest, MissingRobotWaitsThenTimesOut)
{
  NavdogConfig config;
  config.start_align.max_hold_sec = 2.0;
  NavigationCoordinator coordinator(config);

  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 1.0);

  CoreInput ready = makePlannerInput(PlannerState::READY, 1u, 1.1);
  coordinator.update(ready, 1.1);

  // Invalid robot → WAITING_FOR_ROBOT, still START_ALIGN
  CoreInput bad_robot{};
  CoreOutput out1 = coordinator.update(bad_robot, 2.0);
  EXPECT_EQ(out1.state, NavState::START_ALIGN);
  EXPECT_DOUBLE_EQ(out1.final_cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(out1.final_cmd.vy, 0.0);
  EXPECT_DOUBLE_EQ(out1.final_cmd.yaw_rate, 0.0);

  // Past timeout → FAILED
  CoreOutput out2 = coordinator.update(bad_robot, 3.2);
  EXPECT_EQ(out2.state, NavState::FAILED);
}

// =============================================================================
// 26.8 InvalidStartDirectionTransitionsToFailed
// =============================================================================

TEST(NavigationCoordinatorTest, InvalidStartDirectionTransitionsToFailed)
{
  NavigationCoordinator coordinator;

  // Task with single point at robot location, no yaw
  NavigationEvent event{};
  event.type = NavigationEventType::START_TASK;
  event.task.mode = TaskMode::NORMAL_AVOID;
  event.task.max_vx = 0.4;
  RoutePoint p{};
  p.x = 0.0;
  p.y = 0.0;
  event.task.points.push_back(p);

  coordinator.handleEvent(event);
  coordinator.update(CoreInput{}, 1.0);  // SET_ROUTE

  CoreInput ready = makePlannerInput(PlannerState::READY, 1u, 1.1);
  coordinator.update(ready, 1.1);

  // Valid robot at (0,0), task point at (0,0) → INVALID_TASK
  CoreInput robot_in = makeRobotInput(0, 0, 0);
  CoreOutput output = coordinator.update(robot_in, 1.2);

  EXPECT_EQ(output.state, NavState::FAILED);
  EXPECT_TRUE(coordinator.hasActiveTask());
}

// =============================================================================
// 26.9 CancelDuringStartAlignReturnsIdle
// =============================================================================

TEST(NavigationCoordinatorTest, CancelDuringStartAlignReturnsIdle)
{
  constexpr double kDeg = 3.14159265358979323846 / 180.0;

  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 1.0);

  CoreInput ready = makePlannerInput(PlannerState::READY, 1u, 1.1);
  coordinator.update(ready, 1.1);

  // Start rotating
  CoreInput robot = makeRobotInput(0, 0, 30.0 * kDeg);
  coordinator.update(robot, 1.2);

  NavigationEvent cancel{};
  cancel.type = NavigationEventType::CANCEL_TASK;
  coordinator.handleEvent(cancel);

  CoreOutput output = coordinator.update(CoreInput{}, 1.3);

  EXPECT_EQ(output.state, NavState::IDLE);
  EXPECT_EQ(output.planner_action.type, PlannerActionType::CANCEL);
  EXPECT_DOUBLE_EQ(output.final_cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(output.final_cmd.vy, 0.0);
  EXPECT_DOUBLE_EQ(output.final_cmd.yaw_rate, 0.0);
  EXPECT_EQ(output.final_cmd.source, CommandSource::CANCEL_STOP);
}

// =============================================================================
// 26.10 ResetDuringStartAlignClearsController
// =============================================================================

TEST(NavigationCoordinatorTest, ResetDuringStartAlignClearsController)
{
  constexpr double kDeg = 3.14159265358979323846 / 180.0;

  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 1.0);

  CoreInput ready = makePlannerInput(PlannerState::READY, 1u, 1.1);
  coordinator.update(ready, 1.1);

  CoreInput robot = makeRobotInput(0, 0, 30.0 * kDeg);
  coordinator.update(robot, 1.2);

  coordinator.reset();

  CoreOutput output = coordinator.update(CoreInput{}, 1.3);

  EXPECT_EQ(output.state, NavState::IDLE);
  EXPECT_FALSE(coordinator.hasActiveTask());
  EXPECT_DOUBLE_EQ(output.final_cmd.yaw_rate, 0.0);
}

// =============================================================================
// 26.11 TrackingStillRejectsPlannerCmd
// =============================================================================

TEST(NavigationCoordinatorTest, TrackingStillRejectsPlannerCmd)
{
  constexpr double kDeg = 3.14159265358979323846 / 180.0;

  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 1.0);

  CoreInput ready = makePlannerInput(PlannerState::READY, 1u, 1.1);
  coordinator.update(ready, 1.1);

  // Small error → directly to TRACKING
  CoreInput robot1 = makeRobotInput(0, 0, 10.0 * kDeg);
  coordinator.update(robot1, 1.2);

  // Now in TRACKING, send planner_cmd
  CoreInput input{};
  input.planner_cmd.vx = 0.5;
  input.planner_cmd.vy = 0.2;
  input.planner_cmd.yaw_rate = 0.2;
  input.planner_cmd.valid = true;

  CoreOutput output = coordinator.update(input, 1.3);

  EXPECT_EQ(output.state, NavState::TRACKING);
  EXPECT_DOUBLE_EQ(output.final_cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(output.final_cmd.vy, 0.0);
  EXPECT_DOUBLE_EQ(output.final_cmd.yaw_rate, 0.0);
  EXPECT_EQ(output.final_cmd.source, CommandSource::TRACKING_STOP);
}

// =============================================================================
// 26.12 NonFiniteSetRouteTimeTransitionsToFailed
// =============================================================================

TEST(NavigationCoordinatorTest, NonFiniteSetRouteTimeTransitionsToFailed)
{
  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeValidStartEvent());

  CoreOutput output = coordinator.update(
      CoreInput{},
      std::numeric_limits<double>::quiet_NaN());

  EXPECT_EQ(output.planner_action.type, PlannerActionType::NONE);
  EXPECT_EQ(output.state, NavState::FAILED);
  EXPECT_EQ(output.final_cmd.source, CommandSource::FAILED_STOP);
  EXPECT_TRUE(coordinator.hasActiveTask());
}

// =============================================================================
// 26.13 FailedAlignmentRequiresCancelBeforeNewTask
// =============================================================================

TEST(NavigationCoordinatorTest, FailedAlignmentRequiresCancelBeforeNewTask)
{
  constexpr double kDeg = 3.14159265358979323846 / 180.0;

  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 1.0);

  CoreInput ready = makePlannerInput(PlannerState::READY, 1u, 1.1);
  coordinator.update(ready, 1.1);

  // Invalid direction → FAILED (robot at same point as task)
  // Use task with point at origin, robot at origin
  // Actually our makeValidStartEvent has point at (1,0), so let's timeout instead
  NavdogConfig cfg2;
  cfg2.start_align.max_hold_sec = 0.5;
  NavigationCoordinator coord2(cfg2);

  coord2.handleEvent(makeValidStartEvent());
  coord2.update(CoreInput{}, 1.0);

  CoreInput ready2 = makePlannerInput(PlannerState::READY, 1u, 1.1);
  coord2.update(ready2, 1.1);

  // Large error + timeout
  CoreInput robot = makeRobotInput(0, 0, 30.0 * kDeg);
  coord2.update(robot, 1.7);

  // Now try new task → should be rejected
  NavigationEvent newTask = makeValidStartEvent();
  TaskHandleResult result1 = coord2.handleEvent(newTask);
  EXPECT_EQ(result1, TaskHandleResult::REJECTED_BUSY);

  // Cancel first
  NavigationEvent cancel{};
  cancel.type = NavigationEventType::CANCEL_TASK;
  coord2.handleEvent(cancel);
  coord2.update(CoreInput{}, 1.8);

  // Now new task should work
  TaskHandleResult result2 = coord2.handleEvent(newTask);
  EXPECT_EQ(result2, TaskHandleResult::STARTED);
}

// =============================================================================
// Helper: make robot state
// =============================================================================

RobotState makeRobotState(
    double x, double y, double yaw)
{
  RobotState robot{};
  robot.x = x;
  robot.y = y;
  robot.yaw = yaw;
  robot.valid = true;
  return robot;
}

CoreInput makeRobotInput(
    double x, double y, double yaw)
{
  CoreInput input{};
  input.robot = makeRobotState(x, y, yaw);
  return input;
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
