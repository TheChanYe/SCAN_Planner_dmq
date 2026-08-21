#include <gtest/gtest.h>

#include <navdog_core/navigation_coordinator.hpp>

#include <limits>

namespace navdog
{
class NavigationCoordinatorTestPeer
{
public:
  static void enterCompletedState(NavigationCoordinator& coordinator)
  {
    const std::uint64_t sequence =
        coordinator.task_manager_.session().sequence;
    ASSERT_TRUE(coordinator.task_manager_.complete(sequence));
    coordinator.state_ = NavState::SUCCEEDED;
  }

  static VelocityCommand executeMode(
      NavigationCoordinator& coordinator,
      const NavigationTask& task,
      const RobotState& robot,
      const RouteProgress& progress,
      const NavigationModeStatus& mode_status,
      bool corridor_available,
      double now_sec)
  {
    coordinator.state_ = NavState::TRACKING;
    return coordinator.executeMode(
        task, robot, progress, mode_status, ObstacleSummary{},
        RouteCorridorAssessment{}, corridor_available,
        coordinator.task_manager_.session().max_vx, now_sec);
  }

  static VelocityCommand executeRouteFollow(
      NavigationCoordinator& coordinator,
      const NavigationTask& task,
      const RobotState& robot,
      const RouteProgress& progress,
      const NavigationModeStatus& mode_status,
      double max_vx,
      double now_sec)
  {
    return coordinator.executeRouteFollow(
        task, robot, progress, mode_status, max_vx, now_sec);
  }
};

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

TEST(NavigationCoordinator, CancelAcknowledgesCompletedTaskAndReturnsIdle)
{
  NavigationCoordinator coordinator;
  ASSERT_EQ(TaskHandleResult::STARTED, coordinator.handleEvent(startEvent()));
  NavigationCoordinatorTestPeer::enterCompletedState(coordinator);

  ASSERT_FALSE(coordinator.hasActiveTask());
  ASSERT_EQ(NavState::SUCCEEDED, coordinator.state());
  ASSERT_TRUE(coordinator.routeManager().hasRoute());

  NavigationEvent cancel{};
  cancel.type = NavigationEventType::CANCEL_TASK;
  EXPECT_EQ(TaskHandleResult::CANCELLED, coordinator.handleEvent(cancel));
  EXPECT_EQ(NavState::IDLE, coordinator.state());
  EXPECT_FALSE(coordinator.hasActiveTask());
  EXPECT_FALSE(coordinator.routeManager().hasRoute());
}

TEST(NavigationCoordinator, LocalAvoidGoalReachedCompletesBeforeCorridorGate)
{
  NavigationCoordinator coordinator;
  ASSERT_EQ(TaskHandleResult::STARTED, coordinator.handleEvent(startEvent()));
  const auto task = coordinator.routeManager().taskView();
  const std::uint64_t sequence = coordinator.taskSession().sequence;

  RobotState robot = robotInput(1.0).robot;
  robot.x = 2.0;
  robot.y = 0.0;

  RouteProgress progress{};
  progress.valid = true;
  progress.task_sequence = sequence;
  progress.segment_index = 0;
  progress.segment_ratio = 1.0;
  progress.arc_length_m = 2.0;
  progress.total_length_m = 2.0;
  progress.remaining_distance_m = 0.0;
  progress.route_yaw = 0.0;

  NavigationModeStatus mode{};
  mode.mode = NavigationMode::LOCAL_AVOID;
  mode.task_sequence = sequence;

  const auto command = NavigationCoordinatorTestPeer::executeMode(
      coordinator, task, robot, progress, mode, false, 1.0);

  EXPECT_TRUE(command.valid);
  EXPECT_EQ(NavState::SUCCEEDED, coordinator.state());
  EXPECT_FALSE(coordinator.hasActiveTask());
}

TEST(NavigationCoordinator, PendingBlockContinuesRouteAtHandoffSpeed)
{
  NavdogConfig config{};
  config.navigation_mode.handoff_linear_speed_mps = 0.30;
  NavigationCoordinator coordinator(config);
  NavigationTask task = startEvent().task;
  task.sequence = 1;
  RobotState robot = robotInput(1.0).robot;
  RouteProgress progress{};
  progress.valid = true;
  progress.task_sequence = task.sequence;
  progress.arc_length_m = 0.0;
  progress.remaining_distance_m = 2.0;
  progress.total_length_m = 2.0;

  NavigationModeStatus mode{};
  mode.mode = NavigationMode::ROUTE_FOLLOW;
  mode.route_blocked_near = true;
  mode.avoidance_allowed = true;

  auto command = NavigationCoordinatorTestPeer::executeRouteFollow(
      coordinator, task, robot, progress, mode, 0.60, 1.0);
  EXPECT_NE(command.source, CommandSource::TRACKING_STOP);
  EXPECT_GT(command.vx, 0.0);
  EXPECT_LE(std::hypot(command.vx, command.vy), 0.30 + 1e-9);

  command = NavigationCoordinatorTestPeer::executeRouteFollow(
      coordinator, task, robot, progress, mode, 0.15, 1.0);
  EXPECT_GT(command.vx, 0.0);
  EXPECT_LE(std::hypot(command.vx, command.vy), 0.15 + 1e-9);
}

TEST(NavigationCoordinator, RouteOnlyBlockedStillStops)
{
  NavigationCoordinator coordinator;
  NavigationTask task = startEvent().task;
  task.sequence = 1;
  task.mode = TaskMode::ROUTE_ONLY;
  RobotState robot = robotInput(1.0).robot;
  RouteProgress progress{};
  progress.valid = true;
  progress.task_sequence = task.sequence;
  progress.remaining_distance_m = 2.0;

  NavigationModeStatus mode{};
  mode.mode = NavigationMode::ROUTE_FOLLOW;
  mode.route_blocked_near = true;
  mode.reason = NavigationModeReason::ROUTE_ONLY_BLOCKED;

  const auto command = NavigationCoordinatorTestPeer::executeRouteFollow(
      coordinator, task, robot, progress, mode, 0.60, 1.0);
  EXPECT_EQ(command.source, CommandSource::TRACKING_STOP);
  EXPECT_DOUBLE_EQ(command.vx, 0.0);
}

TEST(NavigationCoordinator, MaxVxUpdateKeepsSequenceRouteAndMode)
{
  NavigationCoordinator coordinator;
  NavigationEvent start = startEvent();
  start.task.mode = TaskMode::NORMAL_AVOID;
  ASSERT_EQ(TaskHandleResult::STARTED, coordinator.handleEvent(start));
  const std::uint64_t sequence = coordinator.taskSession().sequence;
  const std::size_t route_size = coordinator.routeManager().route().size();

  NavigationEvent update{};
  update.type = NavigationEventType::UPDATE_MAX_VX;
  update.max_vx = 0.2;
  EXPECT_EQ(TaskHandleResult::MAX_VX_UPDATED,
      coordinator.handleEvent(update));

  EXPECT_EQ(sequence, coordinator.taskSession().sequence);
  EXPECT_EQ(TaskMode::NORMAL_AVOID, coordinator.taskSession().mode);
  EXPECT_DOUBLE_EQ(0.2, coordinator.taskSession().max_vx);
  EXPECT_EQ(route_size, coordinator.routeManager().route().size());
  const CoreOutput output = coordinator.update(CoreInput{}, 1.0);
  EXPECT_EQ(PlannerActionType::SET_ROUTE, output.planner_action.type);
  const CoreOutput speed_output = coordinator.update(CoreInput{}, 1.1);
  EXPECT_EQ(PlannerActionType::UPDATE_SPEED_LIMIT,
      speed_output.planner_action.type);
  EXPECT_EQ(sequence, speed_output.planner_action.task.sequence);
  EXPECT_DOUBLE_EQ(0.2, speed_output.planner_action.max_vx);
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

TEST(NavigationCoordinator, FailActiveTaskEntersFailedAndClosesCoreSession)
{
  NavigationCoordinator coordinator;
  ASSERT_EQ(TaskHandleResult::STARTED, coordinator.handleEvent(startEvent()));
  const std::uint64_t sequence = coordinator.taskSession().sequence;
  ASSERT_TRUE(coordinator.hasActiveTask());

  EXPECT_TRUE(coordinator.failActiveTask());
  EXPECT_EQ(NavState::FAILED, coordinator.state());
  EXPECT_FALSE(coordinator.hasActiveTask());
  EXPECT_EQ(sequence, coordinator.taskSession().sequence);

  EXPECT_FALSE(coordinator.failActiveTask());
}

TEST(NavigationCoordinator, PlannerFailureClosesCoreSession)
{
  NavigationCoordinator coordinator;
  ASSERT_EQ(TaskHandleResult::STARTED, coordinator.handleEvent(startEvent()));
  const std::uint64_t sequence = coordinator.taskSession().sequence;
  coordinator.update(CoreInput{}, 1.0);

  CoreInput feedback{};
  feedback.planner.state = PlannerState::FAILED;
  feedback.planner.trajectory_id = sequence;
  feedback.planner.stamp_sec = 1.1;
  feedback.planner.valid = true;
  const CoreOutput output = coordinator.update(feedback, 1.1);

  EXPECT_EQ(NavState::FAILED, output.state);
  EXPECT_FALSE(coordinator.hasActiveTask());
  EXPECT_EQ(sequence, coordinator.taskSession().sequence);
}

}  // namespace navdog
