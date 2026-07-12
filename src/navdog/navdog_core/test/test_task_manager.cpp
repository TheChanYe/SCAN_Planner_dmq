#include "navdog_core/task_manager.hpp"

#include <cmath>
#include <gtest/gtest.h>
#include <limits>

using namespace navdog;

// =============================================================================
// Helper: build a valid event with one route point
// =============================================================================

static NavigationEvent makeValidStartEvent()
{
  NavigationEvent event{};
  event.type = NavigationEventType::START_TASK;
  event.task.mode = TaskMode::NORMAL_AVOID;
  event.task.max_vx = 0.4;
  event.task.sequence = 999;  // must be overridden

  RoutePoint pt{};
  pt.x = 1.0;
  pt.y = 2.0;
  pt.z = 0.0;
  pt.has_yaw = false;
  event.task.points.push_back(pt);

  return event;
}

// =============================================================================
// 8.1 DefaultStateHasNoActiveTask
// =============================================================================

TEST(TaskManagerTest, DefaultStateHasNoActiveTask)
{
  TaskManager mgr;

  EXPECT_FALSE(mgr.hasActiveTask());
  EXPECT_EQ(mgr.activeSequence(), 0u);

  NavigationTask out{};
  EXPECT_FALSE(mgr.copyActiveTask(out));

  const NavigationTask expected_default{};

  EXPECT_EQ(out.sequence, expected_default.sequence);
  EXPECT_EQ(out.mode, expected_default.mode);
  EXPECT_DOUBLE_EQ(out.max_vx, expected_default.max_vx);
  EXPECT_EQ(out.points.size(), expected_default.points.size());
}

// =============================================================================
// 8.2 StartsValidTaskAndAssignsInternalSequence
// =============================================================================

TEST(TaskManagerTest, StartsValidTaskAndAssignsInternalSequence)
{
  TaskManager mgr;

  NavigationEvent event = makeValidStartEvent();
  event.task.sequence = 999;

  TaskManagerOutput output = mgr.handleEvent(event);

  EXPECT_EQ(output.result, TaskHandleResult::STARTED);
  EXPECT_TRUE(mgr.hasActiveTask());
  EXPECT_EQ(mgr.activeSequence(), 1u);
  EXPECT_EQ(output.planner_action.type, PlannerActionType::SET_ROUTE);
  EXPECT_EQ(output.planner_action.task.sequence, 1u);

  // Verify internal sequence is 1, not 999
  NavigationTask stored{};
  EXPECT_TRUE(mgr.copyActiveTask(stored));
  EXPECT_EQ(stored.sequence, 1u);
}

// =============================================================================
// 8.3 StartTaskClampsHighMaxVx
// =============================================================================

TEST(TaskManagerTest, StartTaskClampsHighMaxVx)
{
  TaskConfig config;
  config.min_max_vx = 0.15;
  config.max_max_vx = 1.0;

  TaskManager mgr(config);

  NavigationEvent event = makeValidStartEvent();
  event.task.max_vx = 2.0;

  TaskManagerOutput output = mgr.handleEvent(event);

  EXPECT_EQ(output.result, TaskHandleResult::STARTED);
  EXPECT_DOUBLE_EQ(output.planner_action.max_vx, 1.0);

  NavigationTask stored{};
  mgr.copyActiveTask(stored);
  EXPECT_DOUBLE_EQ(stored.max_vx, 1.0);
}

// =============================================================================
// 8.3 StartTaskClampsLowMaxVx
// =============================================================================

TEST(TaskManagerTest, StartTaskClampsLowMaxVx)
{
  TaskConfig config;
  config.min_max_vx = 0.15;
  config.max_max_vx = 1.0;

  TaskManager mgr(config);

  NavigationEvent event = makeValidStartEvent();
  event.task.max_vx = 0.05;  // valid positive, but below min

  TaskManagerOutput output = mgr.handleEvent(event);

  EXPECT_EQ(output.result, TaskHandleResult::STARTED);
  EXPECT_DOUBLE_EQ(output.planner_action.max_vx, 0.15);

  NavigationTask stored{};
  mgr.copyActiveTask(stored);
  EXPECT_DOUBLE_EQ(stored.max_vx, 0.15);
}

// =============================================================================
// 8.4 RejectsSecondTaskWhileActive
// =============================================================================

TEST(TaskManagerTest, RejectsSecondTaskWhileActive)
{
  TaskManager mgr;

  // Start task A
  NavigationEvent eventA = makeValidStartEvent();
  eventA.task.points[0].x = 10.0;
  TaskManagerOutput outA = mgr.handleEvent(eventA);
  EXPECT_EQ(outA.result, TaskHandleResult::STARTED);
  EXPECT_EQ(mgr.activeSequence(), 1u);

  // Try task B
  NavigationEvent eventB = makeValidStartEvent();
  eventB.task.points[0].x = 20.0;
  TaskManagerOutput outB = mgr.handleEvent(eventB);

  EXPECT_EQ(outB.result, TaskHandleResult::REJECTED_BUSY);
  EXPECT_EQ(outB.planner_action.type, PlannerActionType::NONE);

  // Task A unchanged
  NavigationTask stored{};
  mgr.copyActiveTask(stored);
  EXPECT_EQ(stored.sequence, 1u);
  EXPECT_DOUBLE_EQ(stored.points[0].x, 10.0);
}

// =============================================================================
// 8.5 RejectsTaskWithEmptyRoute
// =============================================================================

TEST(TaskManagerTest, RejectsTaskWithEmptyRoute)
{
  TaskManager mgr;

  NavigationEvent event{};
  event.type = NavigationEventType::START_TASK;
  event.task.mode = TaskMode::NORMAL_AVOID;
  event.task.max_vx = 0.4;
  // points is empty

  TaskManagerOutput output = mgr.handleEvent(event);

  EXPECT_EQ(output.result, TaskHandleResult::REJECTED_INVALID_TASK);
  EXPECT_FALSE(mgr.hasActiveTask());

  // Sequence not consumed: next valid task should get 1
  NavigationEvent valid = makeValidStartEvent();
  TaskManagerOutput out2 = mgr.handleEvent(valid);
  EXPECT_EQ(out2.result, TaskHandleResult::STARTED);
  EXPECT_EQ(mgr.activeSequence(), 1u);
}

// =============================================================================
// 8.6 RejectsTaskWithNonFinitePoint
// =============================================================================

TEST(TaskManagerTest, RejectsTaskWithNonFinitePoint)
{
  TaskManager mgr;

  NavigationEvent event = makeValidStartEvent();
  event.task.points[0].x = std::numeric_limits<double>::quiet_NaN();

  TaskManagerOutput output = mgr.handleEvent(event);

  EXPECT_EQ(output.result, TaskHandleResult::REJECTED_INVALID_TASK);
  EXPECT_FALSE(mgr.hasActiveTask());
}

// =============================================================================
// 8.7 RejectsNonFiniteYawWhenYawIsEnabled
// =============================================================================

TEST(TaskManagerTest, RejectsNonFiniteYawWhenYawIsEnabled)
{
  TaskManager mgr;

  NavigationEvent event = makeValidStartEvent();
  event.task.points[0].has_yaw = true;
  event.task.points[0].yaw = std::numeric_limits<double>::quiet_NaN();

  TaskManagerOutput output = mgr.handleEvent(event);

  EXPECT_EQ(output.result, TaskHandleResult::REJECTED_INVALID_TASK);
  EXPECT_FALSE(mgr.hasActiveTask());
}

TEST(TaskManagerTest, AcceptsTaskWithYawDisabledAndNonFiniteYaw)
{
  TaskManager mgr;

  NavigationEvent event = makeValidStartEvent();
  event.task.points[0].has_yaw = false;
  event.task.points[0].yaw = std::numeric_limits<double>::quiet_NaN();

  TaskManagerOutput output = mgr.handleEvent(event);

  EXPECT_EQ(output.result, TaskHandleResult::STARTED);
  EXPECT_TRUE(mgr.hasActiveTask());
}

// =============================================================================
// 8.8 RejectsUnknownTaskMode
// =============================================================================

TEST(TaskManagerTest, RejectsUnknownTaskMode)
{
  TaskManager mgr;

  NavigationEvent event = makeValidStartEvent();
  event.task.mode = static_cast<TaskMode>(99);

  TaskManagerOutput output = mgr.handleEvent(event);

  EXPECT_EQ(output.result, TaskHandleResult::REJECTED_INVALID_TASK);
  EXPECT_FALSE(mgr.hasActiveTask());
}

// =============================================================================
// 8.9 RejectsInvalidStartMaxVx
// =============================================================================

TEST(TaskManagerTest, RejectsZeroMaxVx)
{
  TaskManager mgr;

  NavigationEvent event = makeValidStartEvent();
  event.task.max_vx = 0.0;

  TaskManagerOutput output = mgr.handleEvent(event);

  EXPECT_EQ(output.result, TaskHandleResult::REJECTED_INVALID_TASK);
  EXPECT_FALSE(mgr.hasActiveTask());
}

TEST(TaskManagerTest, RejectsNaNMaxVx)
{
  TaskManager mgr;

  NavigationEvent event = makeValidStartEvent();
  event.task.max_vx = std::numeric_limits<double>::quiet_NaN();

  TaskManagerOutput output = mgr.handleEvent(event);

  EXPECT_EQ(output.result, TaskHandleResult::REJECTED_INVALID_TASK);
  EXPECT_FALSE(mgr.hasActiveTask());
}

// =============================================================================
// 8.10 CancelUnlocksActiveTask
// =============================================================================

TEST(TaskManagerTest, CancelUnlocksActiveTask)
{
  TaskManager mgr;

  // Start a task first
  NavigationEvent start = makeValidStartEvent();
  mgr.handleEvent(start);
  ASSERT_TRUE(mgr.hasActiveTask());
  ASSERT_EQ(mgr.activeSequence(), 1u);

  // Cancel
  NavigationEvent cancel{};
  cancel.type = NavigationEventType::CANCEL_TASK;

  TaskManagerOutput output = mgr.handleEvent(cancel);

  EXPECT_EQ(output.result, TaskHandleResult::CANCELLED);
  EXPECT_EQ(output.planner_action.type, PlannerActionType::CANCEL);
  EXPECT_EQ(output.planner_action.task.sequence, 1u);
  EXPECT_FALSE(mgr.hasActiveTask());
  EXPECT_EQ(mgr.activeSequence(), 0u);
}

// =============================================================================
// 8.11 CancelWithoutTaskIsIgnored
// =============================================================================

TEST(TaskManagerTest, CancelWithoutTaskIsIgnored)
{
  TaskManager mgr;

  NavigationEvent cancel{};
  cancel.type = NavigationEventType::CANCEL_TASK;

  TaskManagerOutput output = mgr.handleEvent(cancel);

  EXPECT_EQ(output.result, TaskHandleResult::CANCEL_IGNORED);
  EXPECT_EQ(output.planner_action.type, PlannerActionType::NONE);
}

// =============================================================================
// 8.12 SequenceIncrementsAfterCancel
// =============================================================================

TEST(TaskManagerTest, SequenceIncrementsAfterCancel)
{
  TaskManager mgr;

  // Start A → seq=1
  NavigationEvent startA = makeValidStartEvent();
  TaskManagerOutput outA = mgr.handleEvent(startA);
  EXPECT_EQ(outA.result, TaskHandleResult::STARTED);
  EXPECT_EQ(mgr.activeSequence(), 1u);

  // Cancel A
  NavigationEvent cancel{};
  cancel.type = NavigationEventType::CANCEL_TASK;
  mgr.handleEvent(cancel);
  EXPECT_FALSE(mgr.hasActiveTask());

  // Start B → seq=2
  NavigationEvent startB = makeValidStartEvent();
  startB.task.points[0].x = 99.0;
  TaskManagerOutput outB = mgr.handleEvent(startB);
  EXPECT_EQ(outB.result, TaskHandleResult::STARTED);
  EXPECT_EQ(mgr.activeSequence(), 2u);
}

// =============================================================================
// 8.13 ResetClearsTaskAndRestartsSequence
// =============================================================================

TEST(TaskManagerTest, ResetClearsTaskAndRestartsSequence)
{
  TaskManager mgr;

  // Start A → seq=1
  NavigationEvent startA = makeValidStartEvent();
  mgr.handleEvent(startA);
  EXPECT_EQ(mgr.activeSequence(), 1u);

  // Cancel
  NavigationEvent cancel{};
  cancel.type = NavigationEventType::CANCEL_TASK;
  mgr.handleEvent(cancel);

  // Start B → seq=2
  NavigationEvent startB = makeValidStartEvent();
  mgr.handleEvent(startB);
  EXPECT_EQ(mgr.activeSequence(), 2u);

  // Reset
  mgr.reset();
  EXPECT_FALSE(mgr.hasActiveTask());
  EXPECT_EQ(mgr.activeSequence(), 0u);

  // Start C → seq=1 again
  NavigationEvent startC = makeValidStartEvent();
  mgr.handleEvent(startC);
  EXPECT_EQ(mgr.activeSequence(), 1u);
}

// =============================================================================
// 8.14 UpdatesActiveTaskMaxVxWithoutReplacingRoute
// =============================================================================

TEST(TaskManagerTest, UpdatesActiveTaskMaxVxWithoutReplacingRoute)
{
  TaskManager mgr;

  // Start task with 3 points
  NavigationEvent start = makeValidStartEvent();
  start.task.max_vx = 0.4;
  RoutePoint pt2{};
  pt2.x = 3.0;
  pt2.y = 4.0;
  pt2.z = 0.0;
  start.task.points.push_back(pt2);
  RoutePoint pt3{};
  pt3.x = 5.0;
  pt3.y = 6.0;
  pt3.z = 0.0;
  start.task.points.push_back(pt3);

  mgr.handleEvent(start);
  ASSERT_TRUE(mgr.hasActiveTask());
  ASSERT_EQ(mgr.activeSequence(), 1u);

  // Update max_vx
  NavigationEvent update{};
  update.type = NavigationEventType::UPDATE_MAX_VX;
  update.max_vx = 0.8;

  TaskManagerOutput output = mgr.handleEvent(update);

  EXPECT_EQ(output.result, TaskHandleResult::MAX_VX_UPDATED);
  EXPECT_EQ(output.planner_action.type, PlannerActionType::UPDATE_SPEED_LIMIT);
  EXPECT_DOUBLE_EQ(output.planner_action.max_vx, 0.8);

  // Sequence unchanged
  EXPECT_EQ(mgr.activeSequence(), 1u);

  // Mode and points unchanged
  NavigationTask stored{};
  mgr.copyActiveTask(stored);
  EXPECT_EQ(stored.mode, TaskMode::NORMAL_AVOID);
  EXPECT_EQ(stored.points.size(), 3u);
  EXPECT_DOUBLE_EQ(stored.points[0].x, 1.0);
  EXPECT_DOUBLE_EQ(stored.points[1].x, 3.0);
  EXPECT_DOUBLE_EQ(stored.points[2].x, 5.0);
  EXPECT_DOUBLE_EQ(stored.max_vx, 0.8);
}

// =============================================================================
// 8.15 UpdateMaxVxClampsHigh
// =============================================================================

TEST(TaskManagerTest, UpdateMaxVxClampsHigh)
{
  TaskConfig config;
  config.min_max_vx = 0.15;
  config.max_max_vx = 1.0;

  TaskManager mgr(config);

  NavigationEvent start = makeValidStartEvent();
  start.task.max_vx = 0.5;
  mgr.handleEvent(start);

  NavigationEvent update{};
  update.type = NavigationEventType::UPDATE_MAX_VX;
  update.max_vx = 2.0;

  TaskManagerOutput output = mgr.handleEvent(update);

  EXPECT_EQ(output.result, TaskHandleResult::MAX_VX_UPDATED);
  EXPECT_DOUBLE_EQ(output.planner_action.max_vx, 1.0);

  NavigationTask stored{};
  mgr.copyActiveTask(stored);
  EXPECT_DOUBLE_EQ(stored.max_vx, 1.0);
  // Points unchanged
  EXPECT_EQ(stored.points.size(), 1u);
}

// =============================================================================
// 8.15 UpdateMaxVxClampsLow
// =============================================================================

TEST(TaskManagerTest, UpdateMaxVxClampsLow)
{
  TaskConfig config;
  config.min_max_vx = 0.15;
  config.max_max_vx = 1.0;

  TaskManager mgr(config);

  NavigationEvent start = makeValidStartEvent();
  start.task.max_vx = 0.5;
  mgr.handleEvent(start);

  NavigationEvent update{};
  update.type = NavigationEventType::UPDATE_MAX_VX;
  update.max_vx = 0.05;

  TaskManagerOutput output = mgr.handleEvent(update);

  EXPECT_EQ(output.result, TaskHandleResult::MAX_VX_UPDATED);
  EXPECT_DOUBLE_EQ(output.planner_action.max_vx, 0.15);

  NavigationTask stored{};
  mgr.copyActiveTask(stored);
  EXPECT_DOUBLE_EQ(stored.max_vx, 0.15);
}

// =============================================================================
// 8.16 UnchangedMaxVxDoesNotEmitPlannerAction
// =============================================================================

TEST(TaskManagerTest, UnchangedMaxVxDoesNotEmitPlannerAction)
{
  TaskManager mgr;

  NavigationEvent start = makeValidStartEvent();
  start.task.max_vx = 0.4;
  mgr.handleEvent(start);

  // Same speed after clamping (default max_max_vx=1.0, min_max_vx=0.15)
  NavigationEvent update{};
  update.type = NavigationEventType::UPDATE_MAX_VX;
  update.max_vx = 0.4;

  TaskManagerOutput output = mgr.handleEvent(update);

  EXPECT_EQ(output.result, TaskHandleResult::MAX_VX_UNCHANGED);
  EXPECT_EQ(output.planner_action.type, PlannerActionType::NONE);
}

// =============================================================================
// 8.17 RejectsInvalidMaxVxUpdate
// =============================================================================

TEST(TaskManagerTest, RejectsZeroMaxVxUpdate)
{
  TaskManager mgr;

  NavigationEvent start = makeValidStartEvent();
  mgr.handleEvent(start);

  NavigationEvent update{};
  update.type = NavigationEventType::UPDATE_MAX_VX;
  update.max_vx = 0.0;

  TaskManagerOutput output = mgr.handleEvent(update);

  EXPECT_EQ(output.result, TaskHandleResult::REJECTED_INVALID_MAX_VX);
  EXPECT_EQ(output.planner_action.type, PlannerActionType::NONE);

  // Original speed preserved
  NavigationTask stored{};
  mgr.copyActiveTask(stored);
  EXPECT_DOUBLE_EQ(stored.max_vx, 0.4);
}

TEST(TaskManagerTest, RejectsNegativeMaxVxUpdate)
{
  TaskManager mgr;

  NavigationEvent start = makeValidStartEvent();
  mgr.handleEvent(start);

  NavigationEvent update{};
  update.type = NavigationEventType::UPDATE_MAX_VX;
  update.max_vx = -0.5;

  TaskManagerOutput output = mgr.handleEvent(update);

  EXPECT_EQ(output.result, TaskHandleResult::REJECTED_INVALID_MAX_VX);

  NavigationTask stored{};
  mgr.copyActiveTask(stored);
  EXPECT_DOUBLE_EQ(stored.max_vx, 0.4);
}

TEST(TaskManagerTest, RejectsNaNMaxVxUpdate)
{
  TaskManager mgr;

  NavigationEvent start = makeValidStartEvent();
  mgr.handleEvent(start);

  NavigationEvent update{};
  update.type = NavigationEventType::UPDATE_MAX_VX;
  update.max_vx = std::numeric_limits<double>::quiet_NaN();

  TaskManagerOutput output = mgr.handleEvent(update);

  EXPECT_EQ(output.result, TaskHandleResult::REJECTED_INVALID_MAX_VX);

  NavigationTask stored{};
  mgr.copyActiveTask(stored);
  EXPECT_DOUBLE_EQ(stored.max_vx, 0.4);
}

// =============================================================================
// 8.18 IgnoresMaxVxUpdateWithoutActiveTask
// =============================================================================

TEST(TaskManagerTest, IgnoresMaxVxUpdateWithoutActiveTask)
{
  TaskManager mgr;

  NavigationEvent update{};
  update.type = NavigationEventType::UPDATE_MAX_VX;
  update.max_vx = 0.8;

  TaskManagerOutput output = mgr.handleEvent(update);

  EXPECT_EQ(output.result, TaskHandleResult::MAX_VX_UPDATE_IGNORED);
  EXPECT_EQ(output.planner_action.type, PlannerActionType::NONE);
}

// =============================================================================
// 8.19 UnsupportedEventDoesNotChangeTask
// =============================================================================

TEST(TaskManagerTest, UnsupportedEventDoesNotChangeTask)
{
  TaskManager mgr;

  // Start a task first
  NavigationEvent start = makeValidStartEvent();
  mgr.handleEvent(start);
  ASSERT_TRUE(mgr.hasActiveTask());

  NavigationTask before{};
  mgr.copyActiveTask(before);

  // PAUSE
  NavigationEvent pause{};
  pause.type = NavigationEventType::PAUSE;
  TaskManagerOutput out1 = mgr.handleEvent(pause);
  EXPECT_EQ(out1.result, TaskHandleResult::PAUSED);
  EXPECT_EQ(out1.planner_action.type, PlannerActionType::PAUSE);

  // RESUME
  NavigationEvent resume{};
  resume.type = NavigationEventType::RESUME;
  TaskManagerOutput out2 = mgr.handleEvent(resume);
  EXPECT_EQ(out2.result, TaskHandleResult::RESUMED);
  EXPECT_EQ(out2.planner_action.type, PlannerActionType::RESUME);

  // DYNAMIC_OBSTACLE_UPDATE
  NavigationEvent dyn{};
  dyn.type = NavigationEventType::DYNAMIC_OBSTACLE_UPDATE;
  TaskManagerOutput out3 = mgr.handleEvent(dyn);
  EXPECT_EQ(out3.result, TaskHandleResult::UNSUPPORTED_EVENT);
  EXPECT_EQ(out3.planner_action.type, PlannerActionType::NONE);

  // Task unchanged
  NavigationTask after{};
  mgr.copyActiveTask(after);
  EXPECT_EQ(before.sequence, after.sequence);
  EXPECT_DOUBLE_EQ(before.max_vx, after.max_vx);
  EXPECT_EQ(before.points.size(), after.points.size());
}

// =============================================================================
// Additional: NoneEventDoesNothing
// =============================================================================

TEST(TaskManagerTest, NoneEventDoesNothing)
{
  TaskManager mgr;

  NavigationEvent event{};
  event.type = NavigationEventType::NONE;

  TaskManagerOutput output = mgr.handleEvent(event);

  EXPECT_EQ(output.result, TaskHandleResult::NONE);
  EXPECT_EQ(output.planner_action.type, PlannerActionType::NONE);
  EXPECT_FALSE(mgr.hasActiveTask());
}

// =============================================================================
// Additional: RejectedTaskDoesNotConsumeSequence
// =============================================================================

TEST(TaskManagerTest, RejectedInvalidTaskDoesNotConsumeSequence)
{
  TaskManager mgr;

  // Send invalid task (empty route)
  NavigationEvent invalid{};
  invalid.type = NavigationEventType::START_TASK;
  invalid.task.max_vx = 0.4;
  mgr.handleEvent(invalid);

  // Send REJECTED_BUSY (lock not engaged since invalid was rejected)
  // Send valid task - should still get sequence=1
  NavigationEvent valid = makeValidStartEvent();
  TaskManagerOutput output = mgr.handleEvent(valid);
  EXPECT_EQ(output.result, TaskHandleResult::STARTED);
  EXPECT_EQ(mgr.activeSequence(), 1u);
}

// =============================================================================
// main
// =============================================================================

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
