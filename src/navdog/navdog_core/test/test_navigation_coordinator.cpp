#include <gtest/gtest.h>

#include <navdog_core/navigation_coordinator.hpp>

#include <limits>
#include <utility>

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

  static bool obstacleFinished(const NavigationCoordinator& coordinator)
  {
    return coordinator.obstacle_finished_;
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

TEST(NavigationCoordinator, NearGoalDirectIgnoresStaleRouteProgress)
{
  NavigationCoordinator coordinator;
  NavigationTask task{};
  task.sequence = 1;
  RoutePoint middle{};
  middle.x = 1.0;
  middle.y = 1.0;
  RoutePoint goal{};
  goal.x = 2.0;
  task.points = {RoutePoint{}, middle, goal};

  RobotState robot = robotInput(1.0).robot;
  robot.x = 1.50;
  robot.y = 0.20;
  RouteProgress progress{};
  progress.valid = true;
  progress.task_sequence = task.sequence;
  progress.arc_length_m = 0.0;
  progress.total_length_m = 2.8;
  progress.remaining_distance_m = 1.37;

  NavigationModeStatus mode{};
  mode.mode = NavigationMode::ROUTE_FOLLOW;
  const auto command = NavigationCoordinatorTestPeer::executeRouteFollow(
      coordinator, task, robot, progress, mode, 0.60, 1.0);

  EXPECT_TRUE(command.valid);
  EXPECT_GT(command.vx, 0.0);
  EXPECT_DOUBLE_EQ(command.vy, 0.0);
  EXPECT_LT(command.yaw_rate, 0.0);
}

TEST(GoalController, TimeoutIsFailureSignalNotSuccess)
{
  GoalControllerConfig config{};
  config.finish_dist = 0.20;
  config.finish_yaw_tolerance_rad = 0.10;
  config.goal_align_timeout_sec = 8.0;
  GoalController controller(config);
  NavigationTask task = startEvent().task;
  RobotState robot = robotInput(1.0).robot;
  robot.x = 2.0;
  robot.yaw = 1.0;
  RouteProgress progress{};
  progress.valid = true;
  progress.route_yaw = 0.0;

  const auto started = controller.update(
      task, robot, progress, 0.4, 0.65, 1.0);
  EXPECT_FALSE(started.finished);
  EXPECT_FALSE(started.timed_out);

  const auto after_three_sec = controller.update(
      task, robot, progress, 0.4, 0.65, 4.1);
  EXPECT_FALSE(after_three_sec.finished);
  EXPECT_FALSE(after_three_sec.timed_out);

  const auto timed_out = controller.update(
      task, robot, progress, 0.4, 0.65, 9.1);
  EXPECT_FALSE(timed_out.finished);
  EXPECT_TRUE(timed_out.timed_out);
  EXPECT_TRUE(timed_out.position_reached);
  EXPECT_FALSE(timed_out.yaw_reached);
  EXPECT_DOUBLE_EQ(0.0, timed_out.command.yaw_rate);
}

TEST(GoalController, PositionAndYawReachedSucceedsImmediately)
{
  GoalControllerConfig config{};
  config.finish_dist = 0.20;
  config.finish_yaw_tolerance_rad = 0.10;
  GoalController controller(config);
  NavigationTask task = startEvent().task;
  RobotState robot = robotInput(1.0).robot;
  robot.x = 2.0;
  robot.yaw = 0.05;
  RouteProgress progress{};
  progress.valid = true;
  progress.route_yaw = 0.0;

  const auto result = controller.update(
      task, robot, progress, 0.4, 0.65, 1.0);
  EXPECT_TRUE(result.finished);
  EXPECT_FALSE(result.timed_out);
  EXPECT_TRUE(result.position_reached);
  EXPECT_TRUE(result.yaw_reached);
  EXPECT_DOUBLE_EQ(0.0, result.command.vx);
  EXPECT_DOUBLE_EQ(0.0, result.command.vy);
  EXPECT_DOUBLE_EQ(0.0, result.command.yaw_rate);
}

TEST(NavigationCoordinator, GoalAlignTimeoutEntersFailed)
{
  NavdogConfig config{};
  config.goal_controller.goal_align_timeout_sec = 8.0;
  NavigationCoordinator coordinator(config);
  ASSERT_EQ(TaskHandleResult::STARTED, coordinator.handleEvent(startEvent()));
  const NavigationTask task = coordinator.routeManager().taskView();

  RobotState robot = robotInput(1.0).robot;
  robot.x = 2.0;
  robot.yaw = 1.0;
  RouteProgress progress{};
  progress.valid = true;
  progress.task_sequence = coordinator.taskSession().sequence;
  progress.arc_length_m = 2.0;
  progress.total_length_m = 2.0;
  progress.remaining_distance_m = 0.0;
  progress.route_yaw = 0.0;
  NavigationModeStatus mode{};
  mode.mode = NavigationMode::ROUTE_FOLLOW;

  NavigationCoordinatorTestPeer::executeMode(
      coordinator, task, robot, progress, mode, true, 1.0);
  ASSERT_EQ(NavState::GOAL_ALIGN, coordinator.state());

  CoreInput input{};
  input.robot = robot;
  input.robot.stamp_sec = 9.1;
  const CoreOutput output = coordinator.update(input, 9.1);
  EXPECT_EQ(NavState::FAILED, output.state);
  EXPECT_EQ(CommandSource::FAILED_STOP, output.final_cmd.source);
  EXPECT_FALSE(coordinator.hasActiveTask());
}

TEST(NavigationCoordinator, BlockedGoalSuccessRequiresClosePositionAndYaw)
{
  NavdogConfig config{};
  config.goal_controller.finish_dist = 0.20;
  config.goal_controller.goal_align_reacquire_dist = 0.30;
  config.goal_controller.obstacle_finish_timeout_sec = 6.0;
  config.goal_controller.goal_align_timeout_sec = 8.0;

  const auto run_blocked = [&config](double goal_distance,
                                     double robot_yaw,
                                     double final_time) {
    NavigationCoordinator coordinator(config);
    EXPECT_EQ(TaskHandleResult::STARTED,
        coordinator.handleEvent(startEvent()));
    const NavigationTask task = coordinator.routeManager().taskView();
    RobotState robot = robotInput(1.0).robot;
    robot.x = 2.0 - goal_distance;
    robot.yaw = robot_yaw;
    RouteProgress progress{};
    progress.valid = true;
    progress.task_sequence = coordinator.taskSession().sequence;
    progress.arc_length_m = robot.x;
    progress.total_length_m = 2.0;
    progress.remaining_distance_m = goal_distance;
    progress.route_yaw = 0.0;
    NavigationModeStatus mode{};
    mode.mode = NavigationMode::ROUTE_FOLLOW;
    mode.route_blocked_near = true;

    NavigationCoordinatorTestPeer::executeMode(
        coordinator, task, robot, progress, mode, true, 1.0);
    NavigationCoordinatorTestPeer::executeMode(
        coordinator, task, robot, progress, mode, true, final_time);
    return std::make_pair(
        coordinator.state(),
        NavigationCoordinatorTestPeer::obstacleFinished(coordinator));
  };

  const auto too_far = run_blocked(0.60, 0.0, 11.0);
  EXPECT_NE(NavState::SUCCEEDED, too_far.first);
  EXPECT_FALSE(too_far.second);

  const auto yaw_mismatch = run_blocked(0.25, 1.0, 11.0);
  EXPECT_NE(NavState::SUCCEEDED, yaw_mismatch.first);
  EXPECT_FALSE(yaw_mismatch.second);

  const auto occupied_and_aligned = run_blocked(0.25, 0.0, 7.1);
  EXPECT_EQ(NavState::SUCCEEDED, occupied_and_aligned.first);
  EXPECT_TRUE(occupied_and_aligned.second);
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
