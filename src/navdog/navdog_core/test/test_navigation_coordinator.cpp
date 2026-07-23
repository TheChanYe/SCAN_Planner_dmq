#include <gtest/gtest.h>

#include <navdog_core/navigation_coordinator.hpp>

#include <limits>

namespace navdog
{
namespace
{
NavigationEvent startEvent()
{
  NavigationEvent event{};
  event.type = NavigationEventType::START_TASK;
  event.task.max_vx = 0.4;
  event.task.points.push_back(RoutePoint{});
  RoutePoint goal{}; goal.x = 2.0;
  event.task.points.push_back(goal);
  return event;
}

CoreInput readyInput(std::uint64_t sequence, double stamp)
{
  CoreInput input{};
  input.planner.state = PlannerState::READY;
  input.planner.trajectory_id = sequence;
  input.planner.stamp_sec = stamp;
  input.planner.valid = true;
  return input;
}

CoreInput robotInput(double stamp)
{
  CoreInput input{};
  input.robot.valid = true;
  input.robot.stamp_sec = stamp;
  input.robot.x = 0.0;
  input.robot.y = 0.0;
  input.robot.yaw = 0.0;
  input.obstacles.valid = true;
  input.obstacles.stamp_sec = stamp;
  input.obstacles.front_min = input.obstacles.left_min = input.obstacles.right_min =
      std::numeric_limits<double>::infinity();
  return input;
}
}  // namespace

TEST(NavigationCoordinator, NewTaskWaitsForCorridorBeforeInitialAlignment)
{
  NavigationCoordinator coordinator;
  EXPECT_EQ(TaskHandleResult::STARTED, coordinator.handleEvent(startEvent()));
  EXPECT_EQ(NavState::PLANNING, coordinator.state());
  const CoreOutput request = coordinator.update(CoreInput{}, 1.0);
  EXPECT_EQ(PlannerActionType::SET_ROUTE, request.planner_action.type);

  coordinator.update(readyInput(coordinator.taskSession().sequence, 1.1), 1.1);
  EXPECT_EQ(NavState::START_ALIGN, coordinator.state());
  const CoreOutput aligned = coordinator.update(robotInput(1.2), 1.2);
  // The first progress sample has no map-backed corridor assessment, so
  // START_ALIGN deliberately waits rather than rotating blindly.
  EXPECT_EQ(NavState::START_ALIGN, aligned.state);
  EXPECT_DOUBLE_EQ(0.0, aligned.final_cmd.vx);
}

TEST(NavigationCoordinator, CancelClearsTaskAndRoute)
{
  NavigationCoordinator coordinator;
  ASSERT_EQ(TaskHandleResult::STARTED, coordinator.handleEvent(startEvent()));
  NavigationEvent cancel{}; cancel.type = NavigationEventType::CANCEL_TASK;
  EXPECT_EQ(TaskHandleResult::CANCELLED, coordinator.handleEvent(cancel));
  EXPECT_EQ(NavState::IDLE, coordinator.state());
  EXPECT_FALSE(coordinator.hasActiveTask());
  EXPECT_FALSE(coordinator.routeManager().hasRoute());
}

TEST(NavigationCoordinator, InvalidTimeFailsDuringPlanningHandshake)
{
  NavigationCoordinator coordinator;
  ASSERT_EQ(TaskHandleResult::STARTED, coordinator.handleEvent(startEvent()));
  coordinator.update(CoreInput{}, 1.0);
  CoreInput feedback = readyInput(coordinator.taskSession().sequence,
      std::numeric_limits<double>::infinity());
  const CoreOutput output = coordinator.update(feedback, 2.0);
  EXPECT_EQ(NavState::PLANNING, output.state);
}

}  // namespace navdog
